#ifndef __LASER_ODOMETRY_H                                                                          
#define __LASER_ODOMETRY_H

#include<vector>
#include<utility>

#include"util.h"

#include"ros/node_handle.h"
#include"pcl_ros/transforms.h"
#include"sensor_msgs/PointCloud2.h"
#include"nav_msgs/Odometry.h"
#include"pcl/point_cloud.h"
#include"pcl/point_types.h"
#include"tf/transform_broadcaster.h"
#include"tf/transform_listener.h"
#include"geometry_msgs/Pose.h"
#include"pcl_conversions/pcl_conversions.h"
#include"pcl/segmentation/extract_clusters.h"
#include"pcl/registration/icp.h"
#include "tf_conversions/tf_eigen.h"

namespace semloam{

    class GT_LaserOdometry{

        public:
            GT_LaserOdometry();

            bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);

            void spin();



        private:

            tf::TransformListener listener;
            tf::StampedTransform map_to_gt;

            nav_msgs::Odometry odom_data;
            nav_msgs::Odometry last_odom_data;

            ros::Subscriber _subCentroid;
            ros::Subscriber _subEdge;
            ros::Subscriber _subVelodynePoints;
            ros::Subscriber _subOdometry;

            ros::Publisher _pubLaserOdomToInit;
            ros::Publisher _pubCentroidPointLast;
            ros::Publisher _pubEdgePointLast;
            ros::Publisher _pubVelodynePoints3;







    };

}

#endif
