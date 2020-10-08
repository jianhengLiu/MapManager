//
// Created by chrisliu on 2019/9/25.
//
#include "MapManager/MapManager.h"

#include "ros/ros.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace std;
using namespace Eigen;

// 0.3 表示了对障碍物的膨胀半径（m）
MapManager _MapManager(0.3);

geometry_msgs::PoseStamped currentPos;

ros::Publisher _obs_vis_pub,_grid_path_vis_pub;

//  路径可视化函数
void visGridPath( std::vector<Eigen::Vector3d> grid_path);

void callback_map(const nav_msgs::OccupancyGrid::ConstPtr &map_msg)
{
    //  初始化地图
    _MapManager.initMap(map_msg);
    if(_MapManager.is_Map)
    {
        _obs_vis_pub.publish(_MapManager.get_visOstacle());
    }
}

// 实际"/odom"节点数据类型可能不是这个，请自行更改
void callback_odom(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg)
{
    currentPos.header = pose_msg->header;
    currentPos.pose = pose_msg->pose.pose;
}

void callback_goal(const geometry_msgs::PoseStamped::ConstPtr &goal_msg)
{
    if(_MapManager.is_Map)
    {
        // 若没有odom想离线测试可将start_pt设为地图原点，如下
//         Eigen::Vector3d start_pt = Eigen::Vector3d(0,0,0);

        Eigen::Vector3d start_pt = Eigen::Vector3d(currentPos.pose.position.x,currentPos.pose.position.y,currentPos.pose.position.z);
        Eigen::Vector3d target_pt(goal_msg->pose.position.x,goal_msg->pose.position.y,goal_msg->pose.position.z);

        /**
         * 自行实现路径搜索算法
         */
        /*
        std::vector<Eigen::Vector3d> grid_path = AstarGraphSearch(start_pt,target_pt);

        //Visualize the result
        visGridPath(grid_path);
        */
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "MapManager");//初始化ROS节点
    ros::NodeHandle n;
    ros::Rate rate(100);

    _obs_vis_pub = n.advertise<visualization_msgs::Marker>("vis_obstacle", 1);
    _grid_path_vis_pub = n.advertise<visualization_msgs::Marker>("grid_path_vis", 1);

    ros::Subscriber sub_map = n.subscribe("/map",1,callback_map);
    ros::Subscriber sub_goal = n.subscribe("/move_base_simple/goal",1,callback_goal);
    ros::Subscriber sub_odom = n.subscribe("/odom",1,callback_odom);

    while (ros::ok()) {

        ros::spinOnce();
        rate.sleep();

    }
    return 0;
}

void visGridPath( std::vector<Eigen::Vector3d> grid_path)
{
    visualization_msgs::Marker path_vis;
    path_vis.header.frame_id = "map";
    path_vis.header.stamp = ros::Time::now();

    path_vis.ns = "path";

    path_vis.type = visualization_msgs::Marker::CUBE_LIST;
    path_vis.action = visualization_msgs::Marker::ADD;
    path_vis.id = 0;

    path_vis.pose.orientation.x = 0.0;
    path_vis.pose.orientation.y = 0.0;
    path_vis.pose.orientation.z = 0.0;
    path_vis.pose.orientation.w = 1.0;

    path_vis.color.a = 1.0;
    path_vis.color.r = 0.0;
    path_vis.color.g = 1.0;
    path_vis.color.b = 0.0;


    path_vis.scale.x = _MapManager.resolution;
    path_vis.scale.y = _MapManager.resolution;
    path_vis.scale.z = _MapManager.resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(grid_path.size()); i++)
    {
        Eigen::Vector3d coord = grid_path[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        path_vis.points.push_back(pt);
    }

    _grid_path_vis_pub.publish(path_vis);
}
