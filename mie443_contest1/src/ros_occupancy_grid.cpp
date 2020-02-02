/*
ros Occupancy Grid Map functions
call service static_map to get OGM in a ROS node
contains two main structures:
1. MapMetaData – metdata of the map, contains:
	• resolution – map resolution in m/cell
	• width – number of cells in the y axis
	• height – number of cells in the x axis
2. int8[] data – the map’s data

Documentation: http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html
Or can get the OGM in terminal by the command: rostopic echo /map
*/


#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <vector>
using namespace std;
// grid map
int rows;
int cols;
double mapResolution;
vector<vector<bool> > grid;
bool requestMap(ros::NodeHandle &nh);
void readMap(const nav_msgs::OccupancyGrid& msg);
void printGrid();

int main(int argc, char** argv)
{
 ros::init(argc, argv, "load_ogm");
 ros::NodeHandle nh;
 if (!requestMap(nh))
 exit(-1);
 printGrid();
 return 0;
}

bool requestMap(ros::NodeHandle &nh)
{
 nav_msgs::GetMap::Request req;
 nav_msgs::GetMap::Response res;
 while (!ros::service::waitForService("static_map", ros::Duration(3.0))) {
 ROS_INFO("Waiting for service static_map to become available");
 }
 ROS_INFO("Requesting the map...");
 ros::ServiceClient mapClient =
nh.serviceClient<nav_msgs::GetMap>("static_map");
 if (mapClient.call(req, res)) {
 readMap(res.map);
 return true;
 }
 else {
 ROS_ERROR("Failed to call map service");
 return false;
 }
}

void readMap(const nav_msgs::OccupancyGrid& map)
{
 ROS_INFO("Received a %d X %d map @ %.3f m/px\n",
 map.info.width,
 map.info.height,
 map.info.resolution);
 rows = map.info.height;
 cols = map.info.width;
 mapResolution = map.info.resolution;
 // Dynamically resize the grid
 grid.resize(rows);
 for (int i = 0; i < rows; i++) {
 grid[i].resize(cols);
 }
 int currCell = 0;
 for (int i = 0; i < rows; i++) {
 for (int j = 0; j < cols; j++) {
 if (map.data[currCell] == 0) // unoccupied cell
 grid[i][j] = false;
 else
 grid[i][j] = true; // occupied (100) or unknown cell (-1)
 currCell++;
 }
 }
}

void printGrid()
{
 printf("Grid map:\n");
 int freeCells = 0;
 for (int i = 0; i < rows; i++)
 {
 printf("Row no. %d\n", i);
 for (int j = 0; j < cols; j++)
 {
 printf("%d ", grid[i][j] ? 1 : 0);
 }
 printf("\n");
 }
}