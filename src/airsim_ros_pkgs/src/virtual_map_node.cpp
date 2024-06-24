#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <octomap/OcTreeKey.h>
#include <octomap/octomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap_server/OctomapServer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Eigen>
#include <iostream>
#include <pcl/search/impl/kdtree.hpp>
#include <random>

#include "ros/ros.h"
#define map_f 1
using namespace octomap;
using namespace octomap_msgs;
using namespace octomap_server;
using namespace std;
ros::Publisher virtual_map_pub;
sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;
// octree
OctomapServer* server_drone;
bool use_octree;
bool use_wall;
bool use_circle;
int main(int argc, char** argv) {
    ros::init(argc, argv, "virtual_map_node");
    ros::NodeHandle nh("~");
    ros::NodeHandle private_nh("~");
    std::string host_ip;
    double resolution;
    std::string world_frameid;

    nh.param("resolution", resolution, 0.1);
    nh.param("world_frame_id", world_frameid, std::string("world_enu"));
    nh.param("use_octree", use_octree, false);
    nh.param("use_wall", use_wall, true);
    nh.param("use_circle", use_circle, false);

    if (use_octree)
        server_drone = new OctomapServer(private_nh, nh, world_frameid);
    virtual_map_pub =
        nh.advertise<sensor_msgs::PointCloud2>("/virtual_global_map", 1);
    vector<string> objects_list;
    ros::Rate rate(map_f);
    int count = 0;
    while (ros::ok()) {
        ros::spinOnce();
        if (count <= 10) {
            cloudMap.points.clear();
            if (use_octree) server_drone->m_octree->clear();
            // 人为设置  objects_list
            
            Eigen::Vector3d wall_position(2.0, -1.5, 1);
            Eigen::Vector3d wall_scale(0.1, 20, 6);
            Eigen::Quaterniond wall_q(1, 0, 0, 0);

            // Eigen::AngleAxisd rollAngle(0.5 * M_PI, Eigen::Vector3d::UnitX());
            // wall_q = rollAngle * wall_q;
            Eigen::Matrix3d wall_body2world = wall_q.toRotationMatrix();
            
            if (use_wall) {
                double hole_size = 0.6;
                for (double lx = -wall_scale.x() / 2;
                    lx < wall_scale.x() / 2 + resolution; lx += resolution) {
                    for (double ly = -wall_scale.y() / 2;
                        ly < wall_scale.y() / 2 + resolution; ly += resolution) {
                        if (ly > -hole_size / 2 && ly < hole_size / 2) {
                            continue;
                        }
                        for (double lz = -wall_scale.z() / 2;
                            lz < wall_scale.z() / 2 + resolution;
                            lz += resolution) {
                            Eigen::Vector3d obs_body(lx, ly, lz);
                            Eigen::Vector3d obs_world =
                                wall_body2world * obs_body + wall_position;
                            pcl::PointXYZ pt(obs_world[0], obs_world[1],
                                            obs_world[2]);
                            cloudMap.points.push_back(pt);

                            if (use_octree) {
                                server_drone->m_octree->updateNode(
                                    point3d(pt.x + 1e-5, pt.y + 1e-5, pt.z + 1e-5),
                                    true);
                            }
                        }
                    }
                }
            }


            // 是否使用转圈模式
            if (use_circle) {
                            // 生成圆圈的点云
                auto generateCircle = [&](double cx, double cy, double cz,
                                        double radius, double thickness,
                                        const Eigen::Quaterniond& orientation) {
                    for (double angle = 0; angle < 2 * M_PI; angle += resolution) {
                        for (double r = radius - thickness / 2;
                            r < radius + thickness / 2; r += resolution) {
                            Eigen::Vector3d circle_point(r * cos(angle), 0,
                                                        r * sin(angle));
                            circle_point = orientation * circle_point +
                                        Eigen::Vector3d(cx, cy, cz);
                            pcl::PointXYZ pt(circle_point[0], circle_point[1],
                                            circle_point[2]);
                            cloudMap.points.push_back(pt);

                            if (use_octree) {
                                server_drone->m_octree->updateNode(
                                    point3d(pt.x + 1e-5, pt.y + 1e-5, pt.z + 1e-5),
                                    true);
                            }
                        }
                    }
                };

                Eigen::Quaterniond q_circle1(
                    Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitZ()));
                // Circle 1
                generateCircle(3, -2.0, 1, 0.5, 0.1,
                            q_circle1);
                // Circle 2
                generateCircle(6, 0, 1, 0.5, 0.1, Eigen::Quaterniond::Identity());
                // Circle 3
                generateCircle(3, 1.5, 1, 0.5, 0.1, q_circle1);
                // 根据每个gate的位置来设置转圈的点云数据 circle的半径为0.5m
                // 厚度为0.1m circle 1  x: 3m -1.5m 1m circle 2  x: 6m 0m 1m
                // 方向绕z轴逆时针旋转90度 citcle 3  x: 3m 1,5m 1m target x  x: 0m
                // 0m 1m
    
            }
 

            if (use_octree) server_drone->publishAll();
            count++;
            cloudMap.width = cloudMap.points.size();
            cloudMap.height = 1;
            cloudMap.is_dense = true;
            pcl::toROSMsg(cloudMap, globalMap_pcd);
            globalMap_pcd.header.frame_id = world_frameid;
            virtual_map_pub.publish(globalMap_pcd);
            ROS_INFO("send global map");
        }
        rate.sleep();
    }
    return 0;
}