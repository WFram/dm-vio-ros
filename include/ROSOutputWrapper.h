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

#ifndef DMVIO_ROS_ROSOUTPUTWRAPPER_H
#define DMVIO_ROS_ROSOUTPUTWRAPPER_H

#include <IOWrapper/Output3DWrapper.h>
#include <TicToc.h>
#include <mutex>

#include "cv_bridge/cv_bridge.h"

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32.h>

#include <GTSAMIntegration/PoseTransformation.h>
#include <GTSAMIntegration/PoseTransformationIMU.h>

namespace dmvio
{
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
    typedef pcl::PointCloud<pcl::PointXYZINormal> PointCloudXYZINormal;

    // We publish 3 topics by default:
    // dmvio/frame_tracked: DMVIOPoseMsg
    // dmvio/unscaled_pose: PoseStamped
    // dmvio/metric_poses: PoseStamped
    // For more details on these see the README.md file.
    class ROSOutputWrapper : public dso::IOWrap::Output3DWrapper
    {
    public:
        ROSOutputWrapper();

        /*
     * Usage:
     * Called once after each keyframe is optimized and passes the new transformation from DSO frame (worldToCam in
     * DSO scale) to metric frame (imuToWorld in metric scale).
     * Use transformPose of the passed object to transform poses between the frames.
     * Note that the object should not be used any more after the method returns.
     * The caller can create copy however , preferable with the following constructor (as otherwise the shared_ptrs will be kept).
     * TransformDSOToIMU(TransformDSOToIMU& other, std::shared_ptr<bool> optScale, std::shared_ptr<bool> optGravity, std::shared_ptr<bool> optT_cam_imu);
     */
        virtual void publishTransformDSOToIMU(const dmvio::TransformDSOToIMU &transformDSOToIMU) override;


        /*
     * Usage:
     * Called every time the status of the system changes.
     */
        virtual void publishSystemStatus(dmvio::SystemStatus systemStatus) override;


        /* Usage:
     * Called once for each tracked frame, with the real-time, low-delay frame pose.
     *
     * Calling:
     * Always called, no overhead if not used.
     */

        virtual void publishCamPose(dso::FrameShell *frame, dso::CalibHessian *HCalib) override;

        // In case you want to additionally publish pointclouds or keyframe poses you need to override Output3DWrapper::publishKeyframes

        virtual void publishKeyframes(std::vector<dso::FrameHessian *> &frames,
                                      bool final,
                                      dso::CalibHessian *HCalib) override;

        virtual void pushLiveFrame(dso::FrameHessian *image) override;

        //        virtual void publishPoseAndPoints(dso::FrameHessian *fh, dso::CalibHessian *HCalib) override;

        void publishOutput();

        void undistortCallback(const sensor_msgs::ImageConstPtr img);

        void setFixedScale() { T_FS_DSO.setScale(fixed_scale); }

        PoseTransformation::PoseType transformPoseFixedScale(const PoseTransformation::PoseType &pose);

        PoseTransformation::PoseType transformPointFixedScale(const PoseTransformation::PoseType &pose,
                                                              Eigen::Vector3d &point_cam);

        PoseTransformation::PoseType invTransformPointFixedScale(const PoseTransformation::PoseType &pose,
                                                                 Eigen::Vector3d &point_world);

        PoseTransformation::PoseType transformPointFakeCam(Eigen::Vector3d &point_cam);

    private:
        ros::NodeHandle nh;
        ros::Publisher dmvioPosePublisher, systemStatePublisher, unscaledPosePublisher, metricPosePublisher;
        ros::Publisher dmvioOdomHighFreqPublisher, dmvioOdomLowFreqPublisher;
        ros::Publisher dmvioLocalPointCloudPublisher, dmvioGlobalPointCloudPublisher, dmvioReferencePointCloudPublisher;

        ros::Publisher dmvioImagePublisher;
        ros::Subscriber dmvioImageSubscriber;

        pcl::RadiusOutlierRemoval<pcl::PointXYZINormal> outrem;
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

        PointCloudXYZINormal::Ptr global_cloud;
        PointCloudXYZINormal::Ptr reference_cloud;
        std::deque<PointCloudXYZINormal::Ptr> margin_cloud_window;
        double lastTimestamp;
        int minNumPointsToSend;
        bool useRANSAC;

        // RANSAC parameters
        double distanceThreshold;
        double probability;
        int maxIterations;

        // RadiusOutlierRemoval parameters
        double activeRadiusSearch, marginRadiusSearch;
        int activeMinNeighborsInRadius, marginMinNeighborsInRadius;

        // StatisticalOutlierRemoval parameters
        int meanK;
        double stddevMulThresh;

        // Clustering parameters
        double clusterTolerance;
        int minClusterSize;
        int maxClusterSize;

        long int pub_idx = 0;

        std::map<int, bool> check_existed;
        std::map<double, bool> m_timestamps;

        std::deque<nav_msgs::Odometry> poseBuf;
        std::deque<sensor_msgs::PointCloud2> localPointsBuf, globalPointsBuf;
        std::deque<sensor_msgs::PointCloud2> referencePointsBuf;

        tf2_ros::TransformBroadcaster dmvioWcamBr, dmvioFakeCamBr;

        Sophus::SE3d T_fake_cam;

        double fixed_scale = 1.0;
        Sophus::Sim3d T_FS_DSO;

        float scaledTH = 1e10;
        float absTH = 1e10;
        float minRelBS = 0;

        // Protects transformDSOToIMU.
        std::mutex mutex;

        // Protects queues
        std::mutex poseMutex, pclMutex;

        std::unique_ptr<dmvio::TransformDSOToIMU> transformDSOToIMU;
        bool scaleAvailable = false;// True iff transformDSOToIMU contains a valid scale.
        bool firstScaleAvailable = true;
        std::atomic<dmvio::SystemStatus> lastSystemStatus;
    };


}// namespace dmvio
#endif//DMVIO_ROS_ROSOUTPUTWRAPPER_H
