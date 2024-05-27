#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include "Astar.h"

using namespace cv;
using namespace std;
MapParamNode MapParam;
Mat Maptest;

class MapNode {
public:
    int x;
    int y;
    int cost_f;
    int cost_g;
    int cost_h;
    std::vector<int> parent;

    MapNode() : x(0), y(0), cost_f(0), cost_g(0), cost_h(0), parent({0, 0}) {}
};

class AStar {
public:
    AStar(cv::Mat& map, cv::Point& start, cv::Point& goal, MapParamNode& MapParam) {
        this->map = map;
        this->state_map = std::vector<std::vector<int>>((MapParam.height, MapParam.width), std::vector<int>(MapParam.width, 0));
        this->open_list = std::vector<MapNode>();
        this->close_list = std::vector<MapNode>();
        this->path = std::vector<std::vector<int>>();
        this->start = start;
        this->goal = goal;
        this->if_reach = false;
    
            // this->map = map;
            // this->state_map = std::vector<std::vector<int>>(this->map.size(), std::vector<int>(this->map[0].size(), 0));
            // this->open_list = std::vector<MapNode>();
            // this->close_list = std::vector<MapNode>();
            // this->path = std::vector<std::vector<int>>();
            // this->start = start;
            // this->goal = goal;
            // this->if_reach = false;
    //     this->map = extendMap(map);
    //     this->state_map = std::vector<std::vector<int>>(map.size() + 2, std::vector<int>(map[0].size() + 2, 0));
    //     this->start = start;
    //     this->goal = goal;
    //     this->open_list = {};
    //     this->close_list = {};
    //     this->path = {};
    //     this->if_reach = false;
        }
    // 返回path的函数
    void getPath(vector <Point>& point_path) {
        // vector<Point> point_path;
        for (auto& p : this->path) {
            point_path.push_back(Point(p[0], p[1]));
        }
        // return point_path;
    
        // return this->path;
    }
    void startFind() {
        if (this->map.at<uchar>(this->start.x,this->start.y) != 255) {
            std::cout << "[E]: Please set a valid start point" << std::endl;
            std::cout << "value = " << this->map.at<uchar>(this->start.x,this->start.y) << std::endl;
            return;
        }
        if (this->map.at<uchar>(this->goal.x,this->goal.y) != 255) {
            std::cout << "[E]: Please set a valid goal point" << std::endl;
            return;
        }
        
        appendAroundOpen(this->start, 0);
        

        // 把起始节点加到close_list
        MapNode temp;
        temp.x = this->start.x;
        temp.y = this->start.y;
        appendClose(temp);
        
        while (true) {
            auto [min_cost, index_min_cost] = findMinCostF();
            // cout<<index_min_cost<<endl;
            // printf(open_list.size());
            MapNode current_node = this->open_list[index_min_cost];
            // cout<<"Begin here"<<endl;
            // printf("current_node.x: %d, current_node.y: %d\n", current_node.x, current_node.y);

            if (current_node.x == this->goal.x && current_node.y == this->goal.y) {
                appendPath(current_node);
                break;
            }

            appendAroundOpen({current_node.x, current_node.y}, current_node.cost_g);
            appendClose(current_node);
            this->open_list.erase(this->open_list.begin() + index_min_cost);
        }
    }

private:
    // std::vector<std::vector<int>> map;
    // std::vector<std::vector<int>> state_map;
    // std::vector<MapNode> open_list;
    // std::vector<MapNode> close_list;
    // std::vector<std::vector<int>> path;
    // std::vector<int> start;
    // std::vector<int> goal;
    // bool if_reach;
    cv::Mat map;
    std::vector<std::vector<int>> state_map;
    std::vector<MapNode> open_list;
    std::vector<MapNode> close_list;
    std::vector<std::vector<int>> path;
    cv::Point start;
    cv::Point goal;
    bool if_reach;


    void appendAroundOpen(cv::Point coordinate, int cost_g) {
        // Implement the function to append nodes around the current node to the open list
        // Your code here
        int dx[] = {-1, 0, 1};
        int dy[] = {-1, 0, 1};

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                if (dx[i] == 0 && dy[j] == 0) {
                    continue;
                }

                int new_x = coordinate.x + dx[i];
                int new_y = coordinate.y + dy[j];

                if (this->map.at<uchar>(new_x,new_y) == 255 && this->state_map[new_x][new_y] != 1) {
                    MapNode temp;
                    // printf("new_x: %d, new_y: %d\n", new_x, new_y);
                    temp.cost_g = 1 + cost_g;
                    temp.cost_h = sqrt((goal.x - new_x)*(goal.x - new_x) + (goal.y - new_y)*(goal.y - new_y));
                    temp.cost_f = temp.cost_g + temp.cost_h;
                    temp.x = new_x;
                    temp.y = new_y;
                    temp.parent[0] = coordinate.x;
                    temp.parent[1] = coordinate.y;

                    if (this->state_map[new_x][new_y] == 2) {
                        int current_index = findIndex(new_x, new_y);
                        if (this->open_list[current_index].cost_f > temp.cost_f) {
                            this->open_list[current_index] = temp;
                        }
                    } else {
                        this->state_map[new_x][new_y] = 2;
                        this->open_list.push_back(temp);
                    }
                    // printf("append node (%d, %d) to open_list\n", new_x, new_y);
                }
            }
        }
        // printf("open_list size: %d\n", open_list.size());
    }

    void appendPath(MapNode node) {
        // Implement the function to append the shortest path to the path list
        // Your code here
        while (true) {
            this->path.push_back({node.x, node.y});
            if (node.x == this->start.x && node.y == this->start.y) {
                break;
            }
            int current_index = findCloseIndex(node.parent[0], node.parent[1]);
            node = this->close_list[current_index];
        }
        // this->path.push_back({node.x, node.y});
        // while (node.parent[0] != 0 || node.parent[1] != 0) {
        //     node = this->close_list[findIndex(node.parent[0], node.parent[1])];
        //     this->path.push_back({node.x, node.y});
        // }
        // std::reverse(this->path.begin(), this->path.end());
        // this->if_reach = true;
        
    }

    std::pair<int, int> findMinCostF() {
        // Implement the function to find the node with the minimum cost f in the open list
        // Your code here
        int min_cost = 1000000000;
        int index_min_cost = -1;
        for (int i = 0; i < this->open_list.size(); i++) {
            if (this->open_list[i].cost_f < min_cost) {
                min_cost = this->open_list[i].cost_f;
                index_min_cost = i;
            }
        }
        // printf("this->open_list.size(): %zu\n", this->open_list.size());
        // printf("min_cost: %d, index_min_cost: %d\n", min_cost, index_min_cost);
        return {min_cost, index_min_cost};

        // return {0, 0};  // Placeholder return
    }

    int findCloseIndex(int x, int y) {
        // Implement the function to find the index of a node in the close list
        // Your code here
        for (int i = 0; i < this->close_list.size(); i++) {
            if (this->close_list[i].x == x && this->close_list[i].y == y) {
                return i;
            }
        }
        
        // return 0;  // Placeholder return
    }

    int findIndex(int x, int y) {
        // Implement the function to find the index of a node in the open list
        // Your code here
        for (int i = 0; i < this->open_list.size(); i++) {
            if (this->open_list[i].x == x && this->open_list[i].y == y) {
                return i;
            }
        }

        // return 0;  // Placeholder return
    }

    void appendClose(MapNode node) {
        // Implement the function to append a node to the close list
        // Your code here
        this->close_list.push_back(node);

        this->state_map[node.x][node.y] = 1;

        
    }
};

void World2MapGrid(MapParamNode& MapParam, Point2d& src_point, Point& dst_point)
{
    Mat P_src = Mat(Vec2d(src_point.x, src_point.y), CV_64FC1);
    Mat P_dst = MapParam.Rotation.inv() * (P_src - MapParam.Translation);

    dst_point.x = round(P_dst.at<double>(0, 0));
    dst_point.y = MapParam.height - 1 - round(P_dst.at<double>(1, 0));
}
void MapGrid2world(MapParamNode& MapParam, Point& src_point, Point2d& dst_point)
{
    Mat P_src = Mat(Vec2d(src_point.x, MapParam.height - 1 - src_point.y), CV_64FC1);

    Mat P_dst = MapParam.Rotation * P_src + MapParam.Translation;

    dst_point.x = P_dst.at<double>(0, 0);
    dst_point.y = P_dst.at<double>(1, 0);
}

void MapCallback(const nav_msgs::OccupancyGrid& msg)
{

    // Get the parameters of map
    MapParam.resolution = msg.info.resolution;
    MapParam.height = msg.info.height;
    MapParam.width = msg.info.width;
    // The origin of the MapGrid is on the bottom left corner of the map
    MapParam.x = msg.info.origin.position.x;
    MapParam.y = msg.info.origin.position.y;


    // Calculate the pose of map with respect to the world of rviz
    double roll, pitch, yaw;
    geometry_msgs::Quaternion q = msg.info.origin.orientation;
    tf::Quaternion quat(q.x, q.y, q.z, q.w); // x, y, z, w
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    double theta = yaw;

    //从rviz上所给定的起点和终点坐标是真实世界坐标系下的位置，需要转化为地图坐标下的表示
    //MapParam.Rotation MapParam.Translation 用于该变换
    MapParam.Rotation = Mat::zeros(2,2, CV_64FC1);
    MapParam.Rotation.at<double>(0, 0) = MapParam.resolution * cos(theta);
    MapParam.Rotation.at<double>(0, 1) = MapParam.resolution * sin(-theta);
    MapParam.Rotation.at<double>(1, 0) = MapParam.resolution * sin(theta);
    MapParam.Rotation.at<double>(1, 1) = MapParam.resolution * cos(theta);
    MapParam.Translation = Mat(Vec2d(MapParam.x, MapParam.y), CV_64FC1);

    cout<<"Map:"<<endl;
    cout<<"MapParam.height:"<<MapParam.height<<endl;
    cout<<"MapParam.width:"<<MapParam.width<<endl;

    Mat Map(MapParam.height, MapParam.width, CV_8UC1);
    Maptest= Map;
    int GridFlag;
    for(int i = 0; i < MapParam.height; i++)
    {
        for(int j = 0; j < MapParam.width; j++)
        {
            GridFlag = msg.data[i * MapParam.width + j];
            GridFlag = (GridFlag < 0) ? 100 : GridFlag; // set Unknown to 0
            Map.at<uchar>(j,MapParam.height-i-1) = 255 - round(GridFlag * 255.0 / 100.0);
        }
    }
}

void StartPointCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    Point2d src_point = Point2d(msg.pose.pose.position.x, msg.pose.pose.position.y);
    World2MapGrid(MapParam,src_point, MapParam.StartPoint);
    cout<<"StartPoint:"<<MapParam.StartPoint<<endl;
}

void TargetPointtCallback(const geometry_msgs::PoseStamped& msg)
{
    Point2d src_point = Point2d(msg.pose.position.x, msg.pose.position.y);
    World2MapGrid(MapParam,src_point, MapParam.TargetPoint);
    int p =Maptest.at<uchar>(MapParam.TargetPoint.x, MapParam.TargetPoint.y);
    cout<<"flag:"<<p<<endl;
    MapGrid2world(MapParam,MapParam.TargetPoint,src_point);
    cout<<"TargetPoint world:"<<src_point<<endl;
    cout<<"TargetPoint:"<<MapParam.TargetPoint<<endl;
}
void PathGrid2world(MapParamNode& MapParam, vector<Point>& PathList, nav_msgs::Path& plan_path)
{
    plan_path.header.stamp = ros::Time::now();
    plan_path.header.frame_id = "map";
    plan_path.poses.clear();
    for(int i=0;i<PathList.size();i++)
    {
        
        Point2d dst_point;
        MapGrid2world(MapParam,PathList[i], dst_point);
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = dst_point.x;
        pose_stamped.pose.position.y = dst_point.y;
        pose_stamped.pose.position.z = 0;
        pose_stamped.pose.orientation.w = 1.0;
        plan_path.poses.push_back(pose_stamped);
    }
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "astar");
    ros::NodeHandle n;


    geometry_msgs::PointStamped astar_step;

    // Subscribe topics
    ros::Subscriber Map_sub = n.subscribe("map", 10, MapCallback);
    ros::Subscriber StarPoint_sub = n.subscribe("initialpose", 10, StartPointCallback);
    ros::Subscriber TargetPoint_sub = n.subscribe("move_base_simple/goal", 10, TargetPointtCallback);

    // Publisher topics
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("move_base/NavfnROS/nav_path", 10);
    ros::Rate loop_rate(20);
    nav_msgs::Path plan_path;

//    vector<Point>& PathList;
    MapParamNode MapParam_temp;
    while(ros::ok())
    {
        vector <Point> point_path;
        if(MapParam.StartPoint.x!= MapParam_temp.StartPoint.x || MapParam.StartPoint.y != MapParam_temp.StartPoint.y || MapParam.TargetPoint.x != MapParam_temp.TargetPoint.x || MapParam.TargetPoint.y != MapParam_temp.TargetPoint.y)
        {
            cout<<"StartPoints:"<<MapParam.StartPoint<<endl;
            cout<<"TargetPoints:"<<MapParam.TargetPoint<<endl;
            AStar astar(Maptest, MapParam.StartPoint, MapParam.TargetPoint, MapParam);
            
            astar.startFind();
            
            
            astar.getPath(point_path);

            PathGrid2world(MapParam, point_path,plan_path);
            path_pub.publish(plan_path);
            cout<<"PathList:"<<endl;
            // for(int i=0;i<point_path.size();i++)
            // {
            //     cout<<point_path[i]<<endl;
            // }
            cout<<"PathList size:"<<point_path.size()<<endl;
            MapParam_temp = MapParam;
            // break;
            // vector <Point> point_path;
            // vector<Point> PathList;
            // astar.getPath(PathList);
            // PathGrid2world(MapParam, PathList,plan_path);
            // path_pub.publish(plan_path);
            // cout<<"PathList:"<<endl;
            // for(int i=0;i<PathList.size();i++)
            // {
            //     cout<<PathList[i]<<endl;
            // }
            // cout<<"PathList size:"<<PathList.size()<<endl;
            // break;
        }
        
        // PathGrid2world(MapParam, point_path,plan_path);
        // path_pub.publish(plan_path);
        
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
