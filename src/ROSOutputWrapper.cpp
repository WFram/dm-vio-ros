/**
* ROS driver for DM-VIO written by Lukas von Stumberg (http://vision.in.tum.de/dm-vio).
*
* Copyright (c) 2022 Lukas von Stumberg <lukas dot stumberg at tum dot de>.
* for more information see <http://vision.in.tum.de/dm-vio>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DM-VIO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DM-VIO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DM-VIO. If not, see <http://www.gnu.org/licenses/>.
*/

#include "ROSOutputWrapper.h"
#include "dmvio_ros/DMVIOPoseMsg.h"
#include <FullSystem/HessianBlocks.h>
#include <util/FrameShell.h>

using namespace dmvio;

PoseTransformation::PoseType ROSOutputWrapper::transformPoseFixedScale(const PoseTransformation::PoseType &pose)
{
    // WF
    Sophus::Sim3d scaledT_w_cam = T_FS_DSO * Sophus::Sim3d(pose).inverse() * T_FS_DSO.inverse();// in fixed metric scale.
    Sophus::SE3d T_metricW_cam;

    // Is additional rotation needed?
    T_metricW_cam = Sophus::SE3d(scaledT_w_cam.matrix());
    PoseTransformation::PoseType returning = T_metricW_cam.matrix();

    return returning;
}

PoseTransformation::PoseType ROSOutputWrapper::transformPointFixedScale(const PoseTransformation::PoseType &pose,
                                                                        Eigen::Vector3d &point_cam)
{
    // WF
    Sophus::Sim3d T_w_cam = T_FS_DSO * Sophus::Sim3d(pose).inverse() * T_FS_DSO.inverse();// in fixed metric scale.
    Sophus::Vector3d t_cam_point(point_cam);
    Sophus::SO3d R_cam_point;
    Sophus::SE3d T_W_point;

    Sophus::SO3d R_dsoW_metricW = transformDSOToIMU->getR_dsoW_metricW();

    // Is additional rotation needed?
    T_W_point = Sophus::SE3d(T_w_cam.matrix()) * Sophus::SE3d(R_cam_point, t_cam_point);

    PoseTransformation::PoseType returning = T_W_point.matrix();

    return returning;
}

dmvio::ROSOutputWrapper::ROSOutputWrapper()
    : nh("dmvio"), global_cloud(new pcl::PointCloud<pcl::PointXYZ>)
{
    systemStatePublisher = nh.advertise<std_msgs::Int32>("system_status", 10);
    dmvioPosePublisher = nh.advertise<dmvio_ros::DMVIOPoseMsg>("frame_tracked", 10);
    unscaledPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("unscaled_pose", 10);
    // While we publish the metric pose for convenience we don't recommend using it.
    // The reason is that the scale used for generating it might change over time.
    // Usually it is better to save the trajectory and multiply all of it with the newest scale.
    metricPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("metric_pose", 10);

    dmvioOdomHighFreqPublisher = nh.advertise<nav_msgs::Odometry>("pose_hf", 10);
    dmvioOdomLowFreqPublisher = nh.advertise<nav_msgs::Odometry>("pose_lf", 10);
    dmvioLocalPointCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("local_point_cloud", 1);
    dmvioGlobalPointCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("global_point_cloud", 1);

    dmvioImagePublisher = nh.advertise<sensor_msgs::Image>("/dmvio/image_undistort", 10);

    ros::param::get("radiusSearch", radiusSearch);
    ros::param::get("minNeighborsInRadius", minNeighborsInRadius);
    std::cout << "radiusSearch: " << radiusSearch << std::endl;
    std::cout << "minNeighborsInRadius: " << minNeighborsInRadius << std::endl;

    poseBuf.clear();
    localPointsBuf.clear();
    globalPointsBuf.clear();

    check_existed.clear();
    m_timestamps.clear();
}

void ROSOutputWrapper::publishTransformDSOToIMU(const TransformDSOToIMU &transformDSOToIMUPassed)
{
    std::unique_lock<std::mutex> lk(mutex);
    transformDSOToIMU = std::make_unique<dmvio::TransformDSOToIMU>(transformDSOToIMUPassed,
                                                                   std::make_shared<bool>(false),
                                                                   std::make_shared<bool>(false),
                                                                   std::make_shared<bool>(false));
    scaleAvailable = lastSystemStatus == SystemStatus::VISUAL_INERTIAL;
    // You could also publish the new scale (and potentially gravity direction) here already if you want to use it as
    // soon as possible. For this simple ROS wrapper I decided to publish it bundled with the newest tracked pose as
    // this is when it is usually needed.
}

void ROSOutputWrapper::publishSystemStatus(dmvio::SystemStatus systemStatus)
{
    std_msgs::Int32 msg;
    msg.data = static_cast<int>(systemStatus);
    systemStatePublisher.publish(msg);
    lastSystemStatus = systemStatus;
}

void setMsgFromSE3(geometry_msgs::Pose &poseMsg, const Sophus::SE3d &pose)
{
    poseMsg.position.x = pose.translation()[0];
    poseMsg.position.y = pose.translation()[1];
    poseMsg.position.z = pose.translation()[2];
    poseMsg.orientation.x = pose.so3().unit_quaternion().x();
    poseMsg.orientation.y = pose.so3().unit_quaternion().y();
    poseMsg.orientation.z = pose.so3().unit_quaternion().z();
    poseMsg.orientation.w = pose.so3().unit_quaternion().w();
}

void setTfFromSE3(geometry_msgs::Transform &tfMsg, const Sophus::SE3d &pose)
{
    tfMsg.translation.x = pose.translation()[0];
    tfMsg.translation.y = pose.translation()[1];
    tfMsg.translation.z = pose.translation()[2];
    tfMsg.rotation.x = pose.so3().unit_quaternion().x();
    tfMsg.rotation.y = pose.so3().unit_quaternion().y();
    tfMsg.rotation.z = pose.so3().unit_quaternion().z();
    tfMsg.rotation.w = pose.so3().unit_quaternion().w();
}

void ROSOutputWrapper::publishOutput()
{
    if (poseBuf.empty() || localPointsBuf.empty() || globalPointsBuf.empty())
    {
        //        ROS_WARN("Empty buffer: \n\t Pose Buf: %d \t Local PCL Buf: %d \t Global PCL Buf: %d \t ",
        //                 poseBuf.empty(),
        //                 localPointsBuf.empty(),
        //                 globalPointsBuf.empty());
        return;
    }

    // TODO: how the mutex affects FILLING the buffers?
    //    ROS_WARN("Good buffer");
    std::unique_ptr<nav_msgs::Odometry> dmvio_pose = nullptr;
    poseMutex.lock();
    while (poseBuf.size() > 1)
        poseBuf.pop_front();
    dmvio_pose = std::make_unique<nav_msgs::Odometry>(poseBuf.front());
    //    ROS_WARN("Pose Timestamp: %f", dmvio_pose.header.stamp.toSec());
    dmvioOdomLowFreqPublisher.publish(*dmvio_pose);
    poseBuf.clear();
    poseMutex.unlock();

    if (dmvio_pose == nullptr)
    {
        ROS_WARN("Locked the pose. Nothing to publish");
        return;
    }

    sensor_msgs::PointCloud2 dmvio_local_cloud, dmvio_global_cloud;
    pclMutex.lock();

    while (localPointsBuf.size() > 1)
        localPointsBuf.pop_front();
    dmvio_local_cloud = localPointsBuf.front();
    dmvio_local_cloud.header.stamp = dmvio_pose->header.stamp;
    //    ROS_WARN("Local PCL Timestamp: %f", dmvio_local_cloud.header.stamp.toSec());
    dmvioLocalPointCloudPublisher.publish(dmvio_local_cloud);
    localPointsBuf.clear();

    while (globalPointsBuf.size() > 1)
        globalPointsBuf.pop_front();
    dmvio_global_cloud = globalPointsBuf.front();
    dmvio_global_cloud.header.stamp = dmvio_pose->header.stamp;
    //    ROS_WARN("Global PCL Timestamp: %f", dmvio_global_cloud.header.stamp.toSec());
    dmvioGlobalPointCloudPublisher.publish(dmvio_global_cloud);
    globalPointsBuf.clear();

    pclMutex.unlock();
}

void ROSOutputWrapper::publishCamPose(dso::FrameShell *frame, dso::CalibHessian *HCalib)
{
    dmvio_ros::DMVIOPoseMsg msg;

    // The frames are published in ORDERED way
    ros::Time ros_ts;
    //    ROS_WARN("frame->id: %d \n", frame->id);
    ros_ts.fromSec(frame->timestamp);
    msg.header.stamp = ros_ts;
    msg.header.frame_id = "map";

    auto &camToWorld = frame->camToWorld;

    geometry_msgs::Pose &poseMsg = msg.pose;
    setMsgFromSE3(poseMsg, camToWorld);

    geometry_msgs::PoseStamped unscaledMsg;
    unscaledMsg.header = msg.header;
    unscaledMsg.pose = poseMsg;
    unscaledPosePublisher.publish(unscaledMsg);

    {
        std::unique_lock<std::mutex> lk(mutex);
        if (transformDSOToIMU && scaleAvailable)
        {
            if (firstScaleAvailable)
            {
                fixed_scale = transformDSOToIMU->getScale();
                setFixedScale();
                ROS_INFO("Start publishing");
                firstScaleAvailable = false;
            }
            msg.scale = transformDSOToIMU->getScale();

            // Publish scaled pose.
            geometry_msgs::PoseStamped scaledMsg;
            scaledMsg.header = msg.header;

            nav_msgs::Odometry metricOdomMsg;
            metricOdomMsg.header = msg.header;

            // Transform to metric imu to world. Note that we need to use the inverse as transformDSOToIMU expects
            // worldToCam as an input!
            // The input for transformPose() is Eigen matrix (original SE3 pose was converted to it)
            Sophus::SE3d imuToWorld(transformDSOToIMU->transformPose(camToWorld.inverse().matrix()));
            Sophus::SE3d fixedScaleImuToWorld(transformPoseFixedScale(camToWorld.inverse().matrix()));
            setMsgFromSE3(scaledMsg.pose, imuToWorld);
            setMsgFromSE3(metricOdomMsg.pose.pose, fixedScaleImuToWorld);

            geometry_msgs::TransformStamped tf;
            tf.header = msg.header;
            tf.child_frame_id = "camera";
            setTfFromSE3(tf.transform, fixedScaleImuToWorld);

            metricPosePublisher.publish(scaledMsg);
            {
                std::unique_lock<std::mutex> mtx(poseMutex);
                poseBuf.push_back(metricOdomMsg);
            }
            dmvioOdomHighFreqPublisher.publish(metricOdomMsg);
            //            dmvioWcamBr.sendTransform(tf);
        }
        else
        {
            msg.scale = std::numeric_limits<double>::quiet_NaN();
            if (transformDSOToIMU) assert(transformDSOToIMU->getScale() == 1.0);
        }

        if (transformDSOToIMU)
        {
            Sophus::SO3d gravityDirection = transformDSOToIMU->getR_dsoW_metricW();
            msg.rotationMetricToDSO.x = gravityDirection.unit_quaternion().x();
            msg.rotationMetricToDSO.y = gravityDirection.unit_quaternion().y();
            msg.rotationMetricToDSO.z = gravityDirection.unit_quaternion().z();
            msg.rotationMetricToDSO.w = gravityDirection.unit_quaternion().w();
            setMsgFromSE3(msg.imuToCam, transformDSOToIMU->getT_cam_imu());
        }
    }

    dmvioPosePublisher.publish(msg);
}

void ROSOutputWrapper::pushLiveFrame(dso::FrameHessian *image)
{
    cv_bridge::CvImage cvImage;
    cvImage.encoding = "mono8";

    int cols = 848;
    int rows = 800;
    unsigned char *img_data = new unsigned char[cols * rows];
    for (int i = 0; i < cols * rows; i++)
        img_data[i] = image->dI[i][0] * 0.8 > 255.0f ? 255.0 : image->dI[i][0] * 0.8;
    cv::Mat cv_mat_image(rows, cols, CV_8UC1, &img_data[0]);
    cvImage.image = cv_mat_image;

    sensor_msgs::Image imageMsg;
    cvImage.toImageMsg(imageMsg);
    delete[] img_data;

    ros::Time ros_ts;
    ros_ts.fromSec(image->shell->timestamp);
    imageMsg.header.stamp = ros_ts;
    dmvioImagePublisher.publish(imageMsg);
}

// TODO: check if we can allFrameHistory unmappedTrackedFrames
void ROSOutputWrapper::publishKeyframes(std::vector<dso::FrameHessian *> &frames,
                                        bool final,
                                        dso::CalibHessian *HCalib)
{
    if (!scaleAvailable)
        return;


    float fx = HCalib->fxl();
    float fy = HCalib->fyl();
    float cx = HCalib->cxl();
    float cy = HCalib->cyl();

    float fxi = 1 / fx;
    float fyi = 1 / fy;
    float cxi = -cx / fx;
    float cyi = -cy / fy;

    sensor_msgs::PointCloud2 msg_local_cloud, msg_global_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    long int npoints = 0;
    double timestamp;

    {
        std::unique_lock<std::mutex> lk(mutex);
        // Frames in sliding window
        for (dso::FrameHessian *fh: frames)
        {
            //        dso::FrameHessian *fh = frames.back();
            npoints += fh->pointHessiansMarginalized.size();

            // TODO: Anyways, the frames inserted are unordered
            //        ROS_WARN("fh->shell->id TO CHECK: %d", fh->shell->id);
            //        if (m_timestamps.count(fh->idx) != 0)
            //            return;

            // TODO: 1) remove outliers
            //       2) filter out bad points
            //       3) amount [Pose] == amount [PCL2]

            // TODO: don't add same ts; or don't add earlier
            for (dso::PointHessian *p: fh->pointHessiansMarginalized)
            {
                Eigen::Vector3d pos_cam, pos_world;

                // [sx, sy, s]
                float idpeth = p->idepth_scaled;
                float idepth_hessian = p->idepth_hessian;
                float relObsBaseline = p->maxRelBaseline;

                if (idpeth < 0) continue;

                float depth = (1.0f / idpeth);
                float depth4 = depth * depth;
                depth4 *= depth4;
                float var = (1.0f / (idepth_hessian + 0.01));

                if (var * depth4 > scaledTH)
                    continue;

                if (var > absTH)
                    continue;

                if (relObsBaseline < minRelBS)
                    continue;

                pos_cam[0] = (p->u * fxi + cxi) * depth * fixed_scale;
                pos_cam[1] = (p->v * fyi + cyi) * depth * fixed_scale;
                pos_cam[2] = depth * fixed_scale * (1 + 2 * fxi * (rand() / (float) RAND_MAX - 0.5f));

                auto &camToWorld = fh->shell->camToWorld;
                Sophus::SE3d fixedScalePointToWorld(transformPointFixedScale(camToWorld.inverse().matrix(),
                                                                             pos_cam));

                Eigen::Matrix<double, 4, 4> e_fixedScalePointToWorld = fixedScalePointToWorld.matrix();

                for (int i = 0; i < 3; i++)
                    pos_world[i] = e_fixedScalePointToWorld(i, 3);

                pcl::PointXYZ point_cam(pos_cam(0), pos_cam(1), pos_cam(2));
                pcl::PointXYZ point_world(pos_world(0), pos_world(1), pos_world(2));

                local_cloud->push_back(point_world);

                // TODO: 1) exclude repeating frames ( but it may affect point quality? ) --> test how it works ( with nvblox? )
                //                     2)
                //                if (check_existed.count(p->idx) == 0)
                //                {
                //                    global_cloud.push_back(point_world);
                //                    check_existed[p->idx] = true;
                //                }
            }
            timestamp = fh->shell->timestamp;

            //        m_timestamps[fh->idx] = true;

            //        ROS_WARN("timestamp: %f", timestamp);
            //        ROS_WARN("fh->idx: %f \n", fh->idx);
        }
        //        ROS_WARN("Finish sliding window \n");
    }

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_local_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    outrem.setInputCloud(local_cloud);
    // TODO: set as ROS params
    outrem.setRadiusSearch(radiusSearch);
    outrem.setMinNeighborsInRadius(minNeighborsInRadius);
    outrem.setKeepOrganized(true);
    outrem.filter(*filtered_local_cloud);

    pcl::toROSMsg(*filtered_local_cloud, msg_local_cloud);
    ros::Time ros_ts;
    ros_ts.fromSec(timestamp);
    msg_local_cloud.header.stamp = ros_ts;
    msg_local_cloud.header.frame_id = "map";
    //    msg_local_cloud.header.frame_id = "camera";

    *global_cloud += *filtered_local_cloud;
    pcl::toROSMsg(*global_cloud, msg_global_cloud);
    msg_global_cloud.header.stamp = msg_local_cloud.header.stamp;
    msg_global_cloud.header.frame_id = "map";

    {
        std::unique_lock<std::mutex> mtx(pclMutex);
        localPointsBuf.push_back(msg_local_cloud);
        globalPointsBuf.push_back(msg_global_cloud);
    }
    //    dmvioLocalPointCloudPublisher.publish(msg_local_cloud);
    //    dmvioGlobalPointCloudPublisher.publish(msg_global_cloud);
}

// TODO: from one side we need to publish poses according to their ids order
//       but we can't from that place, because the frame doesn't contain any points yet
//       Anyways, we need to publish after the points are available.
//void ROSOutputWrapper::publishPoseAndPoints(dso::FrameHessian *fh, dso::CalibHessian *HCalib)
//{
//    dmvio_ros::DMVIOPoseMsg msg;
//
//    // The frames are published in ORDERED way
//    ros::Time ros_ts;
//    ROS_WARN("PUBLISH POSE AND POINTS : fh->shell->id: %d \n", fh->shell->id);
//    ros_ts.fromSec(fh->shell->timestamp);
//    msg.header.stamp = ros_ts;
//    msg.header.frame_id = "map";
//
//    auto &camToWorld = fh->shell->camToWorld;
//
//    geometry_msgs::Pose &poseMsg = msg.pose;
//    setMsgFromSE3(poseMsg, camToWorld);
//
//    geometry_msgs::PoseStamped unscaledMsg;
//    unscaledMsg.header = msg.header;
//    unscaledMsg.pose = poseMsg;
//    unscaledPosePublisher.publish(unscaledMsg);
//
//    {
//        std::unique_lock<std::mutex> lk(mutex);
//        if (transformDSOToIMU && scaleAvailable)
//        {
//            if (firstScaleAvailable)
//            {
//                fixed_scale = transformDSOToIMU->getScale();
//                setFixedScale();
//                firstScaleAvailable = false;
//            }
//            msg.scale = transformDSOToIMU->getScale();
//
//            float fx = HCalib->fxl();
//            float fy = HCalib->fyl();
//            float cx = HCalib->cxl();
//            float cy = HCalib->cyl();
//
//            float fxi = 1 / fx;
//            float fyi = 1 / fy;
//            float cxi = -cx / fx;
//            float cyi = -cy / fy;
//
//            // Publish scaled pose.
//            geometry_msgs::PoseStamped scaledMsg;
//            scaledMsg.header = msg.header;
//
//            nav_msgs::Odometry metricOdomMsg;
//            metricOdomMsg.header = msg.header;
//
//            sensor_msgs::PointCloud2 msg_local_cloud, msg_global_cloud;
//            pcl::PointCloud<pcl::PointXYZ> local_cloud;
//            long int npoints = fh->pointHessiansMarginalized.size();
//            ROS_WARN("npoints: %ld \n", npoints);
//
//            // Transform to metric imu to world. Note that we need to use the inverse as transformDSOToIMU expects
//            // worldToCam as an input!
//            // The input for transformPose() is Eigen matrix (original SE3 pose was converted to it)
//            Sophus::SE3d imuToWorld(transformDSOToIMU->transformPose(camToWorld.inverse().matrix()));
//            Sophus::SE3d fixedScaleImuToWorld(transformPoseFixedScale(camToWorld.inverse().matrix()));
//            setMsgFromSE3(scaledMsg.pose, imuToWorld);
//            setMsgFromSE3(metricOdomMsg.pose.pose, fixedScaleImuToWorld);
//
//            geometry_msgs::TransformStamped tf;
//            tf.header = msg.header;
//            tf.child_frame_id = "camera";
//            setTfFromSE3(tf.transform, fixedScaleImuToWorld);
//
//            // Select points to publish
//            for (dso::PointHessian *p: fh->pointHessiansMarginalized)
//            {
//                Eigen::Vector3d pos_cam, pos_world;
//
//                // [sx, sy, s]
//                float idpeth = p->idepth_scaled;
//                float idepth_hessian = p->idepth_hessian;
//                float relObsBaseline = p->maxRelBaseline;
//
//                if (idpeth < 0) continue;
//
//                float depth = (1.0f / idpeth);
//                float depth4 = depth * depth;
//                depth4 *= depth4;
//                float var = (1.0f / (idepth_hessian + 0.01));
//
//                if (var * depth4 > scaledTH)
//                    continue;
//
//                if (var > absTH)
//                    continue;
//
//                if (relObsBaseline < minRelBS)
//                    continue;
//
//                pos_cam[0] = (p->u * fxi + cxi) * depth * fixed_scale;
//                pos_cam[1] = (p->v * fyi + cyi) * depth * fixed_scale;
//                pos_cam[2] = depth * fixed_scale * (1 + 2 * fxi * (rand() / (float) RAND_MAX - 0.5f));
//
//                auto &camToWorld = fh->shell->camToWorld;
//                Sophus::SE3d fixedScalePointToWorld(transformPointFixedScale(camToWorld.inverse().matrix(),
//                                                                             pos_cam));
//
//                Eigen::Matrix<double, 4, 4> e_fixedScalePointToWorld = fixedScalePointToWorld.matrix();
//
//                for (int i = 0; i < 3; i++)
//                    pos_world[i] = e_fixedScalePointToWorld(i, 3);
//
//                // TODO
//                pcl::PointXYZ point_world(pos_world(0), pos_world(1), pos_world(2));
//
//                local_cloud.push_back(point_world);
//
//                // TODO: don't add point unordered_map<int id, PointXYZ point>
//                //       (!) check sizes
//
//                // TODO: 1) exclude repeating frames ( but it may affect point quality? ) --> test how it works ( with nvblox? )
//                //                     2)
//                if (check_existed.count(p->idx) == 0)
//                {
//                    global_cloud.push_back(point_world);
//                    check_existed[p->idx] = true;
//                }
//            }
//
//            pcl::toROSMsg(local_cloud, msg_local_cloud);
//            msg_local_cloud.header.stamp = ros_ts;
//            msg_local_cloud.header.frame_id = "map";
//
//            pcl::toROSMsg(global_cloud, msg_global_cloud);
//            msg_global_cloud.header.stamp = msg_local_cloud.header.stamp;
//            msg_global_cloud.header.frame_id = "map";
//
//            metricPosePublisher.publish(scaledMsg);
//            dmvioOdomPublisher.publish(metricOdomMsg);
//            dmvioWcamBr.sendTransform(tf);
//            dmvioLocalPointCloudPublisher.publish(msg_local_cloud);
//            dmvioGlobalPointCloudPublisher.publish(msg_global_cloud);
//        }
//        else
//        {
//            msg.scale = std::numeric_limits<double>::quiet_NaN();
//            if (transformDSOToIMU) assert(transformDSOToIMU->getScale() == 1.0);
//        }
//
//        if (transformDSOToIMU)
//        {
//            Sophus::SO3d gravityDirection = transformDSOToIMU->getR_dsoW_metricW();
//            msg.rotationMetricToDSO.x = gravityDirection.unit_quaternion().x();
//            msg.rotationMetricToDSO.y = gravityDirection.unit_quaternion().y();
//            msg.rotationMetricToDSO.z = gravityDirection.unit_quaternion().z();
//            msg.rotationMetricToDSO.w = gravityDirection.unit_quaternion().w();
//            setMsgFromSE3(msg.imuToCam, transformDSOToIMU->getT_cam_imu());
//        }
//    }
//
//    dmvioPosePublisher.publish(msg);
//}