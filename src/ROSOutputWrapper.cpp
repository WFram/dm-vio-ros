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
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32.h>
#include <util/FrameShell.h>

using namespace dmvio;

PoseTransformation::PoseType ROSOutputWrapper::transformPoseFixedScale(const PoseTransformation::PoseType &pose)
{
    Sophus::Sim3d scaledT_w_cam = T_FS_DSO * Sophus::Sim3d(pose).inverse() * T_FS_DSO.inverse();// in fixed metric scale.
    assert(std::abs(scaledT_w_cam.scale() - 1.0) < 0.0001);
    Sophus::SE3d T_metricW_cam;

    //    Sophus::SO3d R_dsoW_metricW = transformDSOToIMU->getR_dsoW_metricW();
    //    Sophus::SE3d T_cam_imu = transformDSOToIMU->getT_cam_imu();
    //    T_metricW_imu = Sophus::SE3d(scaledT_w_cam.matrix()) * T_cam_imu;
    //    T_metricW_imu = Sophus::SE3d(R_dsoW_metricW.inverse(), Sophus::Vector3d::Zero()) *
    //                    Sophus::SE3d(scaledT_w_cam.matrix()) * T_cam_imu;

    T_metricW_cam = Sophus::SE3d(scaledT_w_cam.matrix());
    PoseTransformation::PoseType returning = T_metricW_cam.matrix();

    return returning;
}

dmvio::ROSOutputWrapper::ROSOutputWrapper()
    : nh("dmvio")
{
    systemStatePublisher = nh.advertise<std_msgs::Int32>("system_status", 10);
    dmvioPosePublisher = nh.advertise<dmvio_ros::DMVIOPoseMsg>("frame_tracked", 10);
    unscaledPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("unscaled_pose", 10);
    // While we publish the metric pose for convenience we don't recommend using it.
    // The reason is that the scale used for generating it might change over time.
    // Usually it is better to save the trajectory and multiply all of it with the newest scale.
    metricPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("metric_pose", 10);

    dmvioOdomPublisher = nh.advertise<nav_msgs::Odometry>("pose", 10);
    dmvioPointCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("local_point_cloud", 1);
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

void ROSOutputWrapper::publishCamPose(dso::FrameShell *frame, dso::CalibHessian *HCalib)
{
    dmvio_ros::DMVIOPoseMsg msg;
    msg.header.stamp = ros::Time::now();
    //    msg.header.stamp = ros::Time(frame->timestamp);
    msg.header.frame_id = "world";

    auto &camToWorld = frame->camToWorld;

    geometry_msgs::Pose &poseMsg = msg.pose;
    setMsgFromSE3(poseMsg, camToWorld);

    // Also publish unscaled pose on its own (e.g. for visualization in Rviz).
    geometry_msgs::PoseStamped unscaledMsg;
    unscaledMsg.header = msg.header;
    unscaledMsg.pose = poseMsg;
    unscaledPosePublisher.publish(unscaledMsg);

    {
        std::unique_lock<std::mutex> lk(mutex);
        // TODO: we can just set the best scale, fix it and publish the pose with stable scale
        if (transformDSOToIMU && scaleAvailable)
        {
            if (firstScaleAvailable)
            {
                fixed_scale = transformDSOToIMU->getScale();
                setFixedScale();
                firstScaleAvailable = false;
            }
            msg.scale = transformDSOToIMU->getScale();

            // Publish scaled pose.
            geometry_msgs::PoseStamped scaledMsg;
            scaledMsg.header = msg.header;

            // TODO: probably we need convert this pose to ROS pose
            nav_msgs::Odometry metricOdomMsg;
            metricOdomMsg.header = msg.header;

            // Transform to metric imu to world. Note that we need to use the inverse as transformDSOToIMU expects
            // worldToCam as an input!
            // The input for transformPose() is Eigen matrix (original SE3 pose was converted to it)
            Sophus::SE3d imuToWorld(transformDSOToIMU->transformPose(camToWorld.inverse().matrix()));
            Sophus::SE3d fixedScaleImuToWorld(transformPoseFixedScale(camToWorld.inverse().matrix()));
            // TODO: transform with fixed here
            setMsgFromSE3(scaledMsg.pose, imuToWorld);
            setMsgFromSE3(metricOdomMsg.pose.pose, fixedScaleImuToWorld);

            geometry_msgs::TransformStamped tf;
            tf.header = msg.header;
            tf.child_frame_id = "camera";
            setTfFromSE3(tf.transform, fixedScaleImuToWorld);

            metricPosePublisher.publish(scaledMsg);
            dmvioOdomPublisher.publish(metricOdomMsg);
            dmvioWcamBr.sendTransform(tf);
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

void ROSOutputWrapper::publishKeyframes(std::vector<dso::FrameHessian *> &frames,
                                        bool final,
                                        dso::CalibHessian *HCalib)
{
    float fx = HCalib->fxl();
    float fy = HCalib->fyl();
    float cx = HCalib->cxl();
    float cy = HCalib->cyl();

    float fxi = 1 / fx;
    float fyi = 1 / fy;
    float cxi = -cx / fx;
    float cyi = -cy / fy;

    sensor_msgs::PointCloud2 msg;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    long int npoints = 0;

    {
        std::unique_lock<std::mutex> lk(mutex);
        for (dso::FrameHessian *fh: frames)
        {
            // add all traces, inlier and outlier points.
            npoints += fh->pointHessians.size();

            for (dso::PointHessian *p: fh->pointHessians)
            {
                // TODO: solve about coordinate frames
                Eigen::Vector3f pos_cam;

                // [sx, sy, s]
                // TODO: consider dx, dy
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

                pos_cam[0] = (p->u * fxi + cxi) * depth;
                pos_cam[1] = (p->v * fyi + cyi) * depth;
                pos_cam[2] = depth * (1 + 2 * fxi * (rand() / (float) RAND_MAX - 0.5f));

                pcl::PointXYZ point(pos_cam(0), pos_cam(1), pos_cam(2));

                cloud.push_back(point);
            }
        }
    }
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "camera";
    ROS_INFO("Publish PCL");
    dmvioPointCloudPublisher.publish(msg);
}
