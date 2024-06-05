#include "ros/ros.h"
#include <ros/spinner.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <math.h>
#include <random>
#include<geometry_msgs/Point32.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <octomap_server/OctomapServer.h>
#define map_f 1
using namespace octomap;
using namespace octomap_msgs;
using namespace octomap_server;
using namespace std;
ros::Publisher  virtual_map_pub;
sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;
//octree
OctomapServer* server_drone;
bool use_octree;
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "virtual_map_node");
    ros::NodeHandle nh("~");
    ros::NodeHandle private_nh("~");
    std::string host_ip;
    double resolution;
    std::string world_frameid;

    nh.param("resolution",resolution,0.1);
    nh.param("world_frame_id",world_frameid,std::string("world_enu"));
    nh.param("use_octree",use_octree,false);
    if(use_octree)
      server_drone = new OctomapServer(private_nh, nh,world_frameid);
    virtual_map_pub   = nh.advertise<sensor_msgs::PointCloud2>("/virtual_global_map", 1);  
    vector<string> objects_list;
    ros::Rate rate(map_f);
    int count=0;
    while(ros::ok()){
        ros::spinOnce();
        if(count<=10){
          cloudMap.points.clear();
          if(use_octree)
            server_drone->m_octree->clear();
          // 人为设置  objects_list 
          Eigen::Vector3d wall_position;
          Eigen::Vector3d wall_scale;
          Eigen::Quaterniond wall_q;
          Eigen::Matrix3d wall_body2world;
          wall_position << 10, 0, 1;
          wall_scale << 1, 20, 6;
          wall_q = Eigen::Quaterniond(1,0,0,0);
          //wall_q逆时针旋转90度
          Eigen::AngleAxisd rollAngle(0.5*M_PI, Eigen::Vector3d::UnitX());
          wall_q = rollAngle*wall_q;
          wall_body2world = wall_q.toRotationMatrix();
          double lx,ly,lz;
          double hole_size = 0.8;
          //中间开个洞 
          for(lx = -wall_scale.x()/2; lx<wall_scale.x()/2+resolution;lx+=resolution){
            for(ly = -wall_scale.y()/2; ly<wall_scale.y()/2+resolution;ly+=resolution){
                if (ly > -hole_size/2 && ly < hole_size/2 ) {
                    continue;
                }
                for(lz = -wall_scale.z()/2; lz<wall_scale.z()/2+resolution;lz+=resolution){
                    Eigen::Vector3d obs_body;
                    obs_body << lx,ly,lz;
                    Eigen::Vector3d obs_world;
                    obs_world = obs_body+wall_position;
                    pcl::PointXYZ pt;
                    pt.x = obs_world[0];
                    pt.y = obs_world[1];
                    pt.z = obs_world[2];
                    cloudMap.points.push_back(pt); 
                    geometry_msgs::Point32 cpt;
                    cpt.x = pt.x;
                    cpt.y = pt.y;
                    cpt.z = pt.z;
                    if(use_octree)
                      server_drone->m_octree->updateNode(point3d(pt.x+1e-5,pt.y+1e-5,pt.z+1e-5), true);
                }
            }
          }

          if(use_octree)
            server_drone->publishAll();
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