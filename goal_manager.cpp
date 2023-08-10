#include <ros/ros.h>

#include <tf/tf.h>
#include <nav_msgs/Path.h>
#include <string>
#include "std_msgs/Int8.h"
//#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <turtlesim/Spawn.h>
#include <math.h>
#include <iostream>
#include <sstream> // for ostringstream
#include <string>
#include <cmath>
#include <fstream>
#include <termios.h>
#define RtoD(x) x*180/M_PI
using std_msgs::Empty;

int getch()
{
	int ch;
	struct termios oldt;
	struct termios newt;

	// Store old settings, and copy to new settings
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;

	// Make required changes and apply the settings
	newt.c_lflag &= ~(ICANON | ECHO);
	newt.c_iflag |= IGNBRK;
	newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
	newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
	newt.c_cc[VMIN] = 0;
	newt.c_cc[VTIME] = 1;
	tcsetattr(fileno(stdin), TCSANOW, &newt);

	// Get the current character
	ch = getchar();

	// Reapply old settings
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

	return ch;
}

void load_path(std::string path_file, std::vector<geometry_msgs::PoseStamped>  & poses)
{
	//std::vector<geometry_msgs::PoseStamped> poses;
	std::ifstream fin(path_file);
	
	float px, py, pz, ox,oy,oz,ow;


	int seq = 0;
	ros::Time begin = ros::Time::now();
	while(fin >> px >> py >> pz >> ox >> oy >> oz>>ow)
	{
		//std::cout << px << " " << py << " " << py << std::endl;
		geometry_msgs::PoseStamped pose;	
		pose.header.seq = seq;
		seq += 1;
		pose.header.stamp = begin;
		//pose.header.frame_id =  "slamware_map";
		pose.header.frame_id =  "map";
		pose.pose.position.x = px;
		pose.pose.position.y = py;
		pose.pose.position.z = pz;
		pose.pose.orientation.z = ox;
		pose.pose.orientation.y = oy;
		pose.pose.orientation.z = oz;
		pose.pose.orientation.w = ow;
		poses.push_back(pose);
	}
}

void check_dist(std::vector<geometry_msgs::PoseStamped> poses, geometry_msgs::TransformStamped curr, float & path_dist, float & goal_dist)
{
	// check the relative distance
	
	// step 1, find the closest point on the path
	//
	float min_dist = 1000;
	int min_dist_id = -1;
	for(int i = 0; i < poses.size(); ++i)
	{
		float dx = poses[i].pose.position.x - curr.transform.translation.x;
		float dy = poses[i].pose.position.y - curr.transform.translation.y;
			
		float dist = sqrt((dx* dx) + (dy * dy));
		if(dist < min_dist)
		{
			min_dist = dist;
			min_dist_id = i;
		}

	}
	path_dist = min_dist;
	// step 2, find the geodesic distance to the end
	float geodesic_dist = min_dist;
	for(int j = min_dist_id; j < poses.size() - 1; ++j)
	{
		float dx = poses[j].pose.position.x - poses[j + 1].pose.position.x;
		float dy = poses[j].pose.position.y - poses[j + 1].pose.position.y;
		float dist = sqrt((dx* dx) + (dy * dy));
		geodesic_dist += dist;
	}
	goal_dist = geodesic_dist;
}

int waypoints_per_meter = 20;
void interpolatePath(std::vector<geometry_msgs::PoseStamped> & poses)
{
  std::vector<geometry_msgs::PoseStamped> temp_path;
  for (int i = 0; i < static_cast<int>(poses.size()-1); i++)
  {
    // calculate distance between two consecutive waypoints
    double x1 = poses[i].pose.position.x;
    double y1 = poses[i].pose.position.y;
    double x2 = poses[i+1].pose.position.x;
    double y2 = poses[i+1].pose.position.y;
    double dist =  hypot(x1-x2, y1-y2);
    int num_wpts = dist * waypoints_per_meter;

    temp_path.push_back(poses[i]);
    geometry_msgs::PoseStamped p = poses[i];
    for (int j = 0; j < num_wpts - 2; j++)
    {
      p.pose.position.x = x1 + static_cast<double>(j) / num_wpts * (x2 - x1);
      p.pose.position.y = y1 + static_cast<double>(j) / num_wpts * (y2 - y1);
      temp_path.push_back(p);
    }
  }

  // update sequence of poses
  for (size_t i = 0; i < temp_path.size(); i++)
    temp_path[i].header.seq = static_cast<int>(i);

  temp_path.push_back(poses.back());
  poses = temp_path;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "goal_manager");

  ros::NodeHandle node;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  //tf::TransformListener listener;

  ros::Publisher goal_pub;
  goal_pub = node.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 100);

  int curr_time = ros::Time::now().toSec();


  // (TODO) read this root path from ros param later on
  //std::string root_path = argv[1];
  std::string root_path;
  node.getParam("/move_base/WaypointGlobalPlanner/path_folder", root_path);
  
  //std::cout << "root path = " << root_path << std::endl;
  root_path = "/home/unitree/.ros/gala_goals/";
  std::cout << "root path = " << root_path << std::endl;
  std::ifstream fin(root_path + "/goal_db.txt");

  int path_id;
  std::vector<std::vector<std::vector<geometry_msgs::PoseStamped>>> path_db;
  std::vector<std::vector<int>> path_id_db;

  int n_goals;
  fin >> n_goals;
  path_db.resize(n_goals); // (TODO) a hack here, assume a maximum of 20 goals
  path_id_db.resize(n_goals);

  double x, y, z, ox, oy, oz, ow;

   std::vector<geometry_msgs::PoseStamped> goal_list;
   int seq = 0;
ros::Time begin = ros::Time::now();
  while(fin >> x >> y >> z >> ox >> oy >> oz >> ow)
  {

	  geometry_msgs::PoseStamped pose;	
	  pose.header.seq = seq;
	  seq += 1;
	  pose.header.stamp = begin;
	  //pose.header.frame_id =  "slamware_map";
	  pose.header.frame_id =  "map";
	  pose.pose.position.x = x;
	  pose.pose.position.y = y;
	  pose.pose.position.z = z;
	  pose.pose.orientation.z = ox;
	  pose.pose.orientation.y = oy;
	  pose.pose.orientation.z = oz;
	  pose.pose.orientation.w = ow;
	  goal_list.push_back(pose);
  }
  ros::Rate rate(20);
  while (node.ok()){
    int ch = 0;
    ch = getch();
    //printf("ch = %d\n\n", ch);
    if(ch == 'q') break;
    //tf2::StampedTransform transform;
    // currently only support < 10 goals    
    if(ch >= '0' and ch <= '9')
    {
   	int goal_id = int(ch - '0'); 
	std::cout << " you select target goal " << goal_id << std::endl;
	auto msg = goal_list[goal_id];

	goal_pub.publish(msg);
	     ros::spinOnce();

	     rate.sleep();
    }

  }

  return 0;
};
