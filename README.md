# MapManager

## 下载编译

```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/jianhengLiu/MapManager.git
cd ..
catkin_make
```

## 教程

使用样例请看 `./App/main.cpp`. 

### MapManager 函数及相关信息

```cpp
bool is_Map = false;// 表示是否有地图信息,默认为false
double resolution;//	表示了地图的分辨率,单位为(m/index)

geometry_msgs::Pose origin;//	表示了地图索引(index)为(0,0,0)的实际位置(meter)

int GLX_SIZE, GLY_SIZE, GLZ_SIZE; //	表示了地图按索引的长宽高的大小
int GLXYZ_SIZE,GLXY_SIZE, GLYZ_SIZE;//	分别表示了:长x宽x高(表示了栅格地图的大小),长x宽,宽x高
```



**初始化函数**

```cpp
MapManager _MapManager(0.3);	// 0.3 表示将障碍物往外膨胀 0.3m
```

**地图初始化函数**

在订阅了格式为`nav_msgs::OccupancyGrid`的地图节点后,在相应回调函数中使用以下函数实现地图的初始化.

完成后相应变量`is_Map = true`

```cpp
void initMap(nav_msgs::OccupancyGrid::ConstPtr map_msg);
```

**实际位置（meter）和栅格地图索引（index）转换**

```cpp
// 将地图索引转化为实际位置(unit m)
Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index);
// 将实际位置(unit m)转化为地图索引
Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt)
```

从`odom`获取机器人的位置`(x,y,z)`(单位为米),使用上述函数转化为地图索引(index)坐标(为非负整数),即可在栅格地图上进行路径搜索算法

**获取地图占用信息**

利用下列函数即可查询对应地图索引(index)的障碍物情况

```cpp
bool isOccupied(const Eigen::Vector3i & index);//	查询是否有障碍物
bool isFree(const Eigen::Vector3i & index);//	查询是否无障碍物
```

**获取可视化信息**

```cpp
visualization_msgs::Marker get_visOstacle();
```

能够返回格式为`visualization_msgs::Marker`的可视化数据,将相关数据利用节点发布即可在`rviz`中添加相关信息查看地图障碍物情况(具体可看`./App/main.cpp`文件)



## 样例讲解

```cpp
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
//	全局变量暂存实时机器人位置（meter）
geometry_msgs::PoseStamped currentPos;
//	定义了ROS相关的发布者
ros::Publisher _obs_vis_pub,_grid_path_vis_pub;

//  路径可视化函，grid_path为搜索得到的以地图索引（index）存储的路径
void visGridPath( std::vector<Eigen::Vector3d> grid_path);

//	地图订阅的回调函数
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

//	目标位置（meter）的回调函数，由rviz中的插件2D_Nav_Goal触发
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
```



## Download and install

```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/jianhengLiu/MapManager.git
cd ..
catkin_make
```

## Tutorials

The demo node is in `./App/main.cpp`. 

### Map information

```cpp
MapManager _MapManager(0.3);	// 0.3 represent inflation distance 0.3m
```

#### Transform between coordinate and grid index 

```cpp
// Map the map coordinates(unit m) from the grid map index
Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index);
// Map the grid map index from the map coordinates(unit m)
Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt)
```

You can get the robots position from `/odom` which is in unit of meter. The you need to switch the 3D coordinates `(x,y,z)` in the form of  `Eigen::Vector3d` to the grid map index in the form of `Eigen::Vector3i`, so that you can apply your search algorithm with the index, which is nonnegative and integers.

#### Get the size of the map

```cpp
int GLX_SIZE, GLY_SIZE, GLZ_SIZE; //	represent MAP's sizes in XYZ dimension respectively
int GLXYZ_SIZE,GLXY_SIZE, GLYZ_SIZE;
```

