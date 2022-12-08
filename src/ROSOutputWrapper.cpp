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

PoseTransformation::PoseType ROSOutputWrapper::invTransformPointFixedScale(const PoseTransformation::PoseType &pose,
                                                                           Eigen::Vector3d &point_world)
{
    // WF
    Sophus::Sim3d T_w_cam = T_FS_DSO * Sophus::Sim3d(pose).inverse() * T_FS_DSO.inverse();// in fixed metric scale.
    Sophus::Vector3d t_world_point(point_world);
    Sophus::SO3d R_world_point;
    Sophus::SE3d T_cam_point;

    Sophus::SO3d R_dsoW_metricW = transformDSOToIMU->getR_dsoW_metricW();

    // Is additional rotation needed?
    T_cam_point = Sophus::SE3d(T_w_cam.matrix().inverse()) * Sophus::SE3d(R_world_point, t_world_point);

    PoseTransformation::PoseType returning = T_cam_point.matrix();

    return returning;
}

PoseTransformation::PoseType ROSOutputWrapper::transformPointFakeCam(Eigen::Vector3d &point_cam)
{
    Sophus::Vector3d t_cam_point(point_cam);
    Sophus::SO3d R_cam_point;
    Sophus::SE3d T_fake_point;

    // Is additional rotation needed?
    T_fake_point = Sophus::SE3d(T_fake_cam.matrix()) * Sophus::SE3d(R_cam_point, t_cam_point);

    PoseTransformation::PoseType returning = T_fake_point.matrix();

    return returning;
}

dmvio::ROSOutputWrapper::ROSOutputWrapper()
    : nh("dmvio"),
      global_cloud(new PointCloudXYZINormal),
      reference_cloud(new PointCloudXYZINormal),
      lastTimestamp(0.0)
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
    dmvioReferencePointCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("reference_point_cloud", 1);

    dmvioImagePublisher = nh.advertise<sensor_msgs::Image>("/dmvio/image_undistort", 10);

    ros::param::get("distanceThreshold", distanceThreshold);
    ros::param::get("probability", probability);
    ros::param::get("maxIterations", maxIterations);
    std::cout << "distanceThreshold: " << distanceThreshold << std::endl;
    std::cout << "probability: " << probability << std::endl;
    std::cout << "maxIterations: " << maxIterations << std::endl;

    ros::param::get("meanK", meanK);
    ros::param::get("stddevMulThresh", stddevMulThresh);
    std::cout << "meanK: " << meanK << std::endl;
    std::cout << "stddevMulThresh: " << stddevMulThresh << std::endl;

    ros::param::get("activeRadiusSearch", activeRadiusSearch);
    ros::param::get("activeMinNeighborsInRadius", activeMinNeighborsInRadius);
    std::cout << "activeRadiusSearch: " << activeRadiusSearch << std::endl;
    std::cout << "activeMinNeighborsInRadius: " << activeMinNeighborsInRadius << std::endl;

    ros::param::get("marginRadiusSearch", marginRadiusSearch);
    ros::param::get("marginMinNeighborsInRadius", marginMinNeighborsInRadius);
    std::cout << "marginRadiusSearch: " << marginRadiusSearch << std::endl;
    std::cout << "marginMinNeighborsInRadius: " << marginMinNeighborsInRadius << std::endl;

    ros::param::get("clusterTolerance", clusterTolerance);
    ros::param::get("minClusterSize", minClusterSize);
    ros::param::get("maxClusterSize", maxClusterSize);
    std::cout << "clusterTolerance: " << clusterTolerance << std::endl;
    std::cout << "minClusterSize: " << minClusterSize << std::endl;
    std::cout << "maxClusterSize: " << maxClusterSize << std::endl;

    ros::param::get("minNumPointsToSend", minNumPointsToSend);
    ros::param::get("useRANSAC", useRANSAC);

    std::cout << "minNumPointsToSend: " << minNumPointsToSend << std::endl;
    std::cout << "useRANSAC: " << useRANSAC << std::endl;

    poseBuf.clear();
    localPointsBuf.clear();
    globalPointsBuf.clear();

    margin_cloud_window.clear();

    referencePointsBuf.clear();

    Eigen::Matrix4d e_T_fake_cam;
    e_T_fake_cam << 0, -1, 0, 0,
            0, 0, -1, 0,
            1, 0, 0, 0,
            0, 0, 0, 1;
    T_fake_cam = Sophus::SE3d(e_T_fake_cam);

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
    // TODO: back after publishing in camera frame
    if (poseBuf.empty() || localPointsBuf.empty())
    {
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

    sensor_msgs::PointCloud2 dmvio_local_cloud, dmvio_global_cloud, dmvio_reference_cloud;
    pclMutex.lock();

    while (localPointsBuf.size() > 1)
        localPointsBuf.pop_front();
    dmvio_local_cloud = localPointsBuf.front();
    dmvio_local_cloud.header.stamp = dmvio_pose->header.stamp;
    //    ROS_WARN("Local PCL Timestamp: %f", dmvio_local_cloud.header.stamp.toSec());
    dmvioLocalPointCloudPublisher.publish(dmvio_local_cloud);
    localPointsBuf.clear();

    // TODO: back after publishing in camera frame
//    while (referencePointsBuf.size() > 1)
//        referencePointsBuf.pop_front();
//    dmvio_reference_cloud = referencePointsBuf.front();
//    dmvio_reference_cloud.header.stamp = dmvio_pose->header.stamp;
//    //    ROS_WARN("Global PCL Timestamp: %f", dmvio_global_cloud.header.stamp.toSec());
//    dmvioReferencePointCloudPublisher.publish(dmvio_reference_cloud);
//    referencePointsBuf.clear();
//
//    while (globalPointsBuf.size() > 1)
//        globalPointsBuf.pop_front();
//    dmvio_global_cloud = globalPointsBuf.front();
//    dmvio_global_cloud.header.stamp = dmvio_pose->header.stamp;
//    //    ROS_WARN("Global PCL Timestamp: %f", dmvio_global_cloud.header.stamp.toSec());
//    dmvioGlobalPointCloudPublisher.publish(dmvio_global_cloud);
//    globalPointsBuf.clear();

    pclMutex.unlock();
}

void ROSOutputWrapper::publishCamPose(dso::FrameShell *frame, dso::CalibHessian *HCalib)
{
    dmvio_ros::DMVIOPoseMsg msg;

    // The frames are published in ORDERED way
    ros::Time ros_ts;
    //    ROS_WARN("frame->id: %d \n", frame->id);
    ros_ts.fromSec(frame->timestamp);
    lastTimestamp = frame->timestamp;
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

            Sophus::SE3d fixedScaleFakeToWorld = fixedScaleImuToWorld * T_fake_cam;

            setMsgFromSE3(scaledMsg.pose, imuToWorld);
            setMsgFromSE3(metricOdomMsg.pose.pose, fixedScaleImuToWorld);

            geometry_msgs::TransformStamped tf_cam;
            tf_cam.header = msg.header;
            tf_cam.child_frame_id = "camera";
            setTfFromSE3(tf_cam.transform, fixedScaleImuToWorld);

            geometry_msgs::TransformStamped tf_fake;
            tf_fake.header = msg.header;
            tf_fake.child_frame_id = "fake_camera";
            setTfFromSE3(tf_fake.transform, fixedScaleFakeToWorld);

            metricPosePublisher.publish(scaledMsg);
            {
                std::unique_lock<std::mutex> mtx(poseMutex);
                poseBuf.push_back(metricOdomMsg);
            }
            dmvioOdomHighFreqPublisher.publish(metricOdomMsg);
            dmvioWcamBr.sendTransform(tf_cam);
            //            dmvioFakeCamBr.sendTransform(tf_fake);
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

    sensor_msgs::PointCloud2 msg_local_cloud, msg_global_cloud, msg_reference_cloud;
    PointCloudXYZINormal::Ptr local_cloud_cam(new PointCloudXYZINormal);
    PointCloudXYZINormal::Ptr local_cloud_world(new PointCloudXYZINormal);
    PointCloudXYZINormal::Ptr active_local_cloud_cam(new PointCloudXYZINormal);
    PointCloudXYZINormal::Ptr active_local_cloud_world(new PointCloudXYZINormal);
    PointCloudXYZINormal::Ptr margin_local_cloud(new PointCloudXYZINormal);

    long int npointsHessians = 0;
    long int npointsHessiansMarginalized = 0;
    double delay = 25.0;
    double delayedTimestamp = lastTimestamp - delay;
    double timestamp = 0.0;
    //    ROS_WARN("delayedTimestamp: %f", delayedTimestamp);

    {
        std::unique_lock<std::mutex> lk(mutex);
        for (dso::FrameHessian *fh: frames)
        {
            npointsHessians += fh->pointHessians.size();
            npointsHessiansMarginalized += fh->pointHessiansMarginalized.size();

            for (dso::PointHessian *ph: fh->pointHessians)
            {
                Eigen::Vector3d pos_cam, pos_world;

                // [sx, sy, s]
                float idpeth = ph->idepth_scaled;
                float idepth_hessian = ph->idepth_hessian;
                float relObsBaseline = ph->maxRelBaseline;

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

                pos_cam[0] = (ph->u * fxi + cxi) * depth * fixed_scale;
                pos_cam[1] = (ph->v * fyi + cyi) * depth * fixed_scale;
                pos_cam[2] = depth * fixed_scale * (1 + 2 * fxi * (rand() / (float) RAND_MAX - 0.5f));

                auto &camToWorld = fh->shell->camToWorld;
                Sophus::SE3d fixedScalePointToWorld(transformPointFixedScale(camToWorld.inverse().matrix(),
                                                                             pos_cam));
                //                Sophus::SE3d pointToFakeCamera(transformPointFakeCam(pos_cam));

                Eigen::Matrix<double, 4, 4> e_fixedScalePointToWorld = fixedScalePointToWorld.matrix();
                //                Eigen::Matrix<double, 4, 4> e_pointToFakeCamera = pointToFakeCamera.matrix();

                for (int i = 0; i < 3; i++)
                {
                    pos_world[i] = e_fixedScalePointToWorld(i, 3);
                    //                    pos_fake_cam[i] = e_pointToFakeCamera(i, 3);
                }

                //                Sophus::SE3d fixedScalePointToCam(invTransformPointFixedScale(camToWorld.inverse().matrix(),
                //                                                                              pos_world));
                //                Eigen::Matrix<double, 4, 4> e_fixedScalePointToCam = fixedScalePointToCam.matrix();
                //                for (int i = 0; i < 3; i++)
                //                {
                //                    pos_metric_cam[i] = e_fixedScalePointToCam(i, 3);
                //            }

                pcl::PointXYZINormal point_world;
                point_world.x = pos_world(0);
                point_world.y = pos_world(1);
                point_world.z = pos_world(2);

                active_local_cloud_world->push_back(point_world);
            }

            for (dso::PointHessian *phm: fh->pointHessiansMarginalized)
            {
                Eigen::Vector3d pos_cam, pos_world;

                // [sx, sy, s]
                float idpeth = phm->idepth_scaled;
                float idepth_hessian = phm->idepth_hessian;
                float relObsBaseline = phm->maxRelBaseline;

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

                pos_cam[0] = (phm->u * fxi + cxi) * depth * fixed_scale;
                pos_cam[1] = (phm->v * fyi + cyi) * depth * fixed_scale;
                pos_cam[2] = depth * fixed_scale * (1 + 2 * fxi * (rand() / (float) RAND_MAX - 0.5f));

                auto &camToWorld = fh->shell->camToWorld;
                Sophus::SE3d fixedScalePointToWorld(transformPointFixedScale(camToWorld.inverse().matrix(),
                                                                             pos_cam));

                Eigen::Matrix<double, 4, 4> e_fixedScalePointToWorld = fixedScalePointToWorld.matrix();

                for (int i = 0; i < 3; i++)
                    pos_world[i] = e_fixedScalePointToWorld(i, 3);

                pcl::PointXYZINormal point_world;
                point_world.x = pos_world(0);
                point_world.y = pos_world(1);
                point_world.z = pos_world(2);
                point_world.curvature = fh->shell->timestamp;

                margin_local_cloud->push_back(point_world);
            }

            timestamp = fh->shell->timestamp;

            //            if (fh->shell->timestamp < delayedTimestamp)
            //                ROS_WARN("Too much delay for KF!!! (%f)", fh->shell->timestamp);
        }
    }

    if (active_local_cloud_world->size() < 1)
        return;

    PointCloudXYZINormal::Ptr filtered_active_local_cloud_cam(new PointCloudXYZINormal);
    PointCloudXYZINormal::Ptr filtered_active_local_cloud_world(new PointCloudXYZINormal);
    PointCloudXYZINormal::Ptr filtered_margin_local_cloud(new PointCloudXYZINormal);

    // todo: 1) find the optimal parameters for filtering the local cloud (because now it's margin)
    //       2) make sure it contains enough points (we need to choose normal keyframes, it's a timestamp issue)
    //       3) find clusters???

    // Sliding window of clouds
    // WF: 03:30
//    PointCloudXYZINormal::Ptr concatenated_margin_local_cloud(new PointCloudXYZINormal);
//    PointCloudXYZINormal::Ptr filtered_concatenated_margin_local_cloud(new PointCloudXYZINormal);
//    int window_size = 5;
//    margin_cloud_window.push_back(margin_local_cloud);
//    if (margin_cloud_window.size() == window_size + 1)
//        margin_cloud_window.pop_front();
//    for (auto &margin_cloud: margin_cloud_window)
//        *concatenated_margin_local_cloud += *margin_cloud;

    // TODO: concatenate all the clouds inside the window into a single one

    //    if (useRANSAC)
    //    {
    //        std::vector<int> inliers_l;
    //        PointCloudXYZINormal::Ptr filtered_concatenated_margin_cloud_l(new PointCloudXYZINormal);
    //        pcl::SampleConsensusModelLine<pcl::PointXYZINormal>::Ptr model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZINormal>(active_local_cloud));// change
    //        pcl::RandomSampleConsensus<pcl::PointXYZINormal> ransac_l(model_l);
    //        ransac_l.setDistanceThreshold(distanceThreshold);
    //        ransac_l.computeModel();
    //        ransac_l.getInliers(inliers_l);
    //        pcl::copyPointCloud(*active_local_cloud, inliers_l, *filtered_concatenated_margin_cloud_l);// change
    //
    //        //        std::vector<int> inliers_per_p;
    //        //        PointCloudXYZINormal::Ptr filtered_concatenated_margin_cloud_per_p(new PointCloudXYZINormal);
    //        //        pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZINormal>::Ptr model_per_p(new pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZINormal>(active_local_cloud)); // change
    //        //        pcl::RandomSampleConsensus<pcl::PointXYZINormal> ransac_per_p(model_per_p);
    //        //        ransac_per_p.setDistanceThreshold(distanceThreshold);
    //        //        ransac_per_p.computeModel();
    //        //        ransac_per_p.getInliers(inliers_per_p);
    //        //        pcl::copyPointCloud(*active_local_cloud, inliers_per_p, *filtered_concatenated_margin_cloud_per_p); // change
    //
    //        //        outrem.setInputCloud(filtered_local_cloud_per_p);
    //        //        outrem.setRadiusSearch(activeRadiusSearch);
    //        //        outrem.setMinNeighborsInRadius(activeMinNeighborsInRadius);
    //        //        outrem.setKeepOrganized(true);
    //        //        outrem.filter(*filtered_local_cloud_per_p);
    //
    //        *filtered_active_local_cloud = *filtered_concatenated_margin_cloud_l;
    //        //        *filtered_active_local_cloud = *filtered_concatenated_margin_cloud_l + *filtered_concatenated_margin_cloud_per_p; // change
    //    }
    //    else
    //    {
    //    outrem.setInputCloud(active_local_cloud_cam);
    //    outrem.setRadiusSearch(activeRadiusSearch);
    //    outrem.setMinNeighborsInRadius(activeMinNeighborsInRadius);
    //    outrem.setKeepOrganized(true);
    //    outrem.filter(*filtered_active_local_cloud_cam);

    outrem.setInputCloud(active_local_cloud_world);
    outrem.setRadiusSearch(activeRadiusSearch);
    outrem.setMinNeighborsInRadius(activeMinNeighborsInRadius);
    outrem.setKeepOrganized(true);
    outrem.filter(*filtered_active_local_cloud_world);

    outrem.setInputCloud(margin_local_cloud);
    outrem.setRadiusSearch(marginRadiusSearch);
    outrem.setMinNeighborsInRadius(marginMinNeighborsInRadius);
    outrem.setKeepOrganized(true);
    outrem.filter(*filtered_margin_local_cloud);
    //    }


    if (filtered_active_local_cloud_world->size() < minNumPointsToSend)
    {
        ROS_WARN("Not enough points");
        return;
    }

    ros::Time ros_ts;
    ros_ts.fromSec(timestamp);

    // Get local_cloud

    *local_cloud_world = *filtered_active_local_cloud_world + *filtered_margin_local_cloud;

    pcl::toROSMsg(*local_cloud_world, msg_local_cloud);
    msg_local_cloud.header.stamp = ros_ts;
    msg_local_cloud.header.frame_id = "map";

    // TODO: back after publishing in camera frame
//    *reference_cloud = *concatenated_margin_local_cloud;
//    pcl::toROSMsg(*reference_cloud, msg_reference_cloud);
//    msg_reference_cloud.header.stamp = ros_ts;
//    msg_reference_cloud.header.frame_id = "map";

//    *global_cloud += *local_cloud_world;
//    pcl::toROSMsg(*global_cloud, msg_global_cloud);
//    msg_global_cloud.header.stamp = msg_local_cloud.header.stamp;
//    msg_global_cloud.header.frame_id = "map";

    // TODO: find clusters

    //    ROS_WARN("pub_idx: %ld", pub_idx);
    //    if (pub_idx % 20 == 0)
    //    {
    //        ROS_WARN("Clustering");
    //        pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZINormal>);
    //        tree->setInputCloud(reference_cloud);
    //        std::vector<pcl::PointIndices> cluster_indices;
    //        pcl::EuclideanClusterExtraction<pcl::PointXYZINormal> ec;
    //        ec.setClusterTolerance(clusterTolerance);
    //        ec.setMinClusterSize(minClusterSize);
    //        ec.setMaxClusterSize(maxClusterSize);
    //        ec.setSearchMethod(tree);
    //        ec.setInputCloud(reference_cloud);
    //        ec.extract(cluster_indices);
    //
    //        int j = 0;
    //        pcl::PCDWriter writer;
    //        for (const auto &cluster: cluster_indices)
    //        {
    //            PointCloudXYZINormal::Ptr cloud_cluster(new PointCloudXYZINormal);
    //            for (const auto &idx: cluster.indices)
    //                cloud_cluster->push_back((*reference_cloud)[idx]);
    //            cloud_cluster->width = cloud_cluster->size();
    //            cloud_cluster->height = 1;
    //            cloud_cluster->is_dense = true;
    //
    //            std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
    //            std::stringstream ss_clusters, ss_cloud;
    //            ss_clusters << "cloud_" << pub_idx << "_cluster_" << j << ".pcd";
    //            writer.write<pcl::PointXYZINormal>(ss_clusters.str(), *cloud_cluster, false);
    //            ss_cloud << "cloud_" << pub_idx << ".pcd";
    //            writer.write<pcl::PointXYZINormal>(ss_cloud.str(), *reference_cloud, false);
    //            j++;
    //        }
    //    }
    //    pub_idx++;


    {
        std::unique_lock<std::mutex> mtx(pclMutex);
        localPointsBuf.push_back(msg_local_cloud);
        // TODO: back after publishing in camera frame
//        referencePointsBuf.push_back(msg_reference_cloud);
//        globalPointsBuf.push_back(msg_global_cloud);
    }
    //    dmvioLocalPointCloudPublisher.publish(msg_local_cloud);
    //    dmvioGlobalPointCloudPublisher.publish(msg_global_cloud);


    // Margin in FOV + Active []

    //    ROS_WARN("lastTimestamp: %f", lastTimestamp);
    //    double delayedTimestamp = lastTimestamp - 25.0;
    //    ROS_WARN("delayedTimestamp: %f \n", delayedTimestamp);
    //    pcl::ConditionAnd<pcl::PointXYZINormal>::Ptr time_cond(new pcl::ConditionAnd<pcl::PointXYZINormal>());
    //    time_cond->addComparison(pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZINormal>("curvature",
    //                                                                                                                                 pcl::ComparisonOps::GT,
    //                                                                                                                                 delayedTimestamp)));
    //    pcl::ConditionalRemoval<pcl::PointXYZINormal> condrem;
    //    condrem.setCondition(time_cond);
    //    condrem.setInputCloud(local_cloud);
    //    condrem.setKeepOrganized(true);
    //    condrem.filter(*filtered_local_cloud);
}
