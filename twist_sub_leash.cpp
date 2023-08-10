#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <chrono>
#include <pthread.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>

using namespace UNITREE_LEGGED_SDK;
class Custom
{
public:
    UDP low_udp;
    UDP high_udp;

    HighCmd high_cmd = {0};
    HighState high_state = {0};

    LowCmd low_cmd = {0};
    LowState low_state = {0};

public:
    Custom()
        : low_udp(LOWLEVEL),
          high_udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState))
    {
        high_udp.InitCmdData(high_cmd);
        low_udp.InitCmdData(low_cmd);
    }

    void highUdpSend()
    {
        // printf("high udp send is running\n");

        high_udp.SetSend(high_cmd);
        high_udp.Send();
    }

    void lowUdpSend()
    {

        low_udp.SetSend(low_cmd);
        low_udp.Send();
    }

    void lowUdpRecv()
    {

        low_udp.Recv();
        low_udp.GetRecv(low_state);
    }

    void highUdpRecv()
    {
        // printf("high udp recv is running\n");

        high_udp.Recv();
        high_udp.GetRecv(high_state);
    }
};

Custom custom;

ros::Subscriber sub_cmd_vel;
ros::Subscriber sub_joy;
ros::Subscriber sub_ultrasonic;
ros::Subscriber sub_leash;
ros::Publisher pub_high;

long cmd_vel_count = 0;

//int joy_stop = 0;
int joy_stop = 1;
int ultrasonic_stop = 0;

float max_vel = 0.7;

void stand()
{

    UNITREE_LEGGED_SDK::HighCmd cmd;

    cmd.head[0] = 0xFE;
    cmd.head[1] = 0xEF;
    cmd.levelFlag = UNITREE_LEGGED_SDK::HIGHLEVEL;
    cmd.mode = 7;
    cmd.gaitType = 0;
    cmd.speedLevel = 0;
    cmd.footRaiseHeight = 0;
    cmd.bodyHeight = 0;
    cmd.euler[0] = 0;
    cmd.euler[1] = 0;
    cmd.euler[2] = 0;
    cmd.velocity[0] = 0.0f;
    cmd.velocity[1] = 0.0f;
    cmd.yawSpeed = 0.0f;
    cmd.reserve = 0;

    //cmd.velocity[0] = msg->linear.x;
    //cmd.velocity[1] = msg->linear.y;
    //cmd.yawSpeed = msg->angular.z;
    cmd.mode = 7;
    cmd.gaitType = 1;
    custom.high_cmd = cmd;

    ros::Duration(1).sleep(); 
    cmd.mode = 5;
    custom.high_cmd = cmd;
    ros::Duration(1).sleep(); 
    cmd.mode = 6;
    custom.high_cmd = cmd;

    ros::Duration(1).sleep(); 
    cmd.mode = 1;
    custom.high_cmd = cmd;

    ros::Duration(1).sleep(); 
    cmd.mode = 2;
    custom.high_cmd = cmd;
    ros::Duration(1).sleep(); 
}
void sit()
{
    UNITREE_LEGGED_SDK::HighCmd cmd;

    cmd.head[0] = 0xFE;
    cmd.head[1] = 0xEF;
    cmd.levelFlag = UNITREE_LEGGED_SDK::HIGHLEVEL;
    cmd.mode = 2;
    cmd.gaitType = 0;
    cmd.speedLevel = 0;
    cmd.footRaiseHeight = 0;
    cmd.bodyHeight = 0;
    cmd.euler[0] = 0;
    cmd.euler[1] = 0;
    cmd.euler[2] = 0;
    cmd.velocity[0] = 0.0f;
    cmd.velocity[1] = 0.0f;
    cmd.yawSpeed = 0.0f;
    cmd.reserve = 0;

    cmd.mode = 2;
    cmd.gaitType = 1;
    custom.high_cmd = cmd;

    ros::Duration(0.2).sleep(); 
    cmd.mode = 1;
    custom.high_cmd = cmd;
    ros::Duration(1).sleep(); 
    cmd.mode = 6;
    custom.high_cmd = cmd;

    ros::Duration(1).sleep(); 
    cmd.mode = 5;
    custom.high_cmd = cmd;

    ros::Duration(1).sleep(); 
    cmd.mode = 7;
    custom.high_cmd = cmd;
    ros::Duration(1).sleep(); 
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    printf("cmdVelCallback is running!\t%ld\n", cmd_vel_count);

    custom.high_cmd = rosMsg2Cmd(msg);
    //custom.high_cmd.velocity[0] = 0.2;
    custom.high_cmd.yawSpeed *= 1.5;

    if(custom.high_cmd.velocity[0] > max_vel)
	    custom.high_cmd.velocity[0] = max_vel;

    //custom.high_cmd.velocity[1] = -custom.high_cmd.velocity[1];
    //cout << "shaojun modification!!!!" << std::endl;
    printf("shaojun modification");
    //if(joy_stop == 1)
    //
    if(joy_stop == 1 || ultrasonic_stop == 1)
    {
   	//msg.linear.x = 0; 
   	//msg.linear.y = 0; 
   	//msg.linear.z = 0; 
   	//msg.angular.x = 0; 
   	//msg.angular.y = 0; 
   	//msg.angular.z = 0; 
	custom.high_cmd.velocity[0] = 0;
	custom.high_cmd.velocity[1] = 0;
	custom.high_cmd.yawSpeed = 0;
    }

    printf("cmd_x_vel = %f\n", custom.high_cmd.velocity[0]);
    printf("cmd_y_vel = %f\n", custom.high_cmd.velocity[1]);
    printf("cmd_yaw_vel = %f\n", custom.high_cmd.yawSpeed);

    unitree_legged_msgs::HighState high_state_ros;

    high_state_ros = state2rosMsg(custom.high_state);

    pub_high.publish(high_state_ros);

    printf("cmdVelCallback ending!\t%ld\n\n", cmd_vel_count++);
}

void cmdVelCallbackSoftLeash(const geometry_msgs::Twist::ConstPtr &msg)
{
    printf("cmdVelCallback is running!\t%ld\n", cmd_vel_count);

    custom.high_cmd = rosMsg2Cmd(msg);
    //custom.high_cmd.velocity[0] = 0.2;
    custom.high_cmd.yawSpeed *= 1.5;
    //custom.high_cmd.velocity[1] = -custom.high_cmd.velocity[1];
    //cout << "shaojun modification!!!!" << std::endl;
    printf("shaojun modification");
    //if(joy_stop == 1)

    printf("cmd_x_vel = %f\n", custom.high_cmd.velocity[0]);
    printf("cmd_y_vel = %f\n", custom.high_cmd.velocity[1]);
    printf("cmd_yaw_vel = %f\n", custom.high_cmd.yawSpeed);

    unitree_legged_msgs::HighState high_state_ros;

    high_state_ros = state2rosMsg(custom.high_state);

    pub_high.publish(high_state_ros);

    printf("cmdVelCallback ending!\t%ld\n\n", cmd_vel_count++);
}


void joyCallback(const sensor_msgs::Joy::ConstPtr &Joy) {
//    cout << "axes("<<Joy->axes.size()<<"): [";
//    for (int i = 0; i < Joy->axes.size(); i++) {
//        cout << Joy->axes.at(i) << " ,";
//    }
//    cout << "]" << endl << "buttons("<<Joy->buttons.size()<<"): [";
//    for (int i = 0; i < Joy->buttons.size(); i++) {
//        cout << Joy->buttons.at(i) << " ,";
//    }
//    cout << "]" << endl;
    //geometry_msgs::Twist twist;
    //twist.linear.x = Joy->axes[1] * linear_speed_limit;
    //twist.angular.z = Joy->axes[3] * angular_speed_limit;
    //ROS_INFO("linear: %.3f angular: %.3f", twist.linear.x, twist.angular.z);
    if(Joy->buttons[0] == 1)
    {
   	printf("stop button pressed\n");	
	joy_stop = 1;
    }
    else if (Joy->buttons[1] == 1)
    {
	joy_stop = 0;
   	printf("go button pressed\n"); 
    }
    else
    {
   //printf("other button pressed\n");	 
    }
}


void leashCallback(const std_msgs::String::ConstPtr & msg) {
    const std::string Joy = msg->data; 
    //std::cout << Joy << std::endl;
    //std::string text = "Let me split this into words";
    std::istringstream iss(Joy);
    std::vector<std::string> results((std::istream_iterator<std::string>(iss)),
		                                     std::istream_iterator<std::string>());
    //std::cout << "splited results " << std::endl;
    //for(int i = 0; i < results.size(); ++i)
    //{
    //        std::cout << results[i] << " "; 
    //}
    std::cout << endl;

    static float on_duration = 0;
    //std::cout << "current mode = " << custom.high_state.mode << std::endl;
    unitree_legged_msgs::HighState high_state_ros;
    high_state_ros = state2rosMsg(custom.high_state);
    //std::cout << "current mode = " << high_state_ros.mode << std::endl;
    std::cout << "current joy stop = " << joy_stop << std::endl;
    if(high_state_ros.mode == 1 || high_state_ros.mode == 0)
    {
   	std::cout << "standing!!!!" << std::endl; 
    }
    if(high_state_ros.mode == 7)
    {
   	std::cout << "sitting!!!!" << std::endl; 
    }
    if(results[3] == "ON")
    {
   	on_duration += 1; 
    }
    else if (results[3] == "OFF")
    {
	if(on_duration >= 10) // short push triggered
	{
	
		std::cout << "short push triggered" << std::endl;	
		joy_stop = 1 - joy_stop;
   		on_duration = 0; 
		return;
	}

   	on_duration = 0; 
    }
    std::cout << "joy stop = " << joy_stop << std::endl;
	if(on_duration >= 100)  // long push triggered
	{
		std::cout << "long push triggered" << std::endl;	
		if(high_state_ros.mode == 7)
		{
			stand();
		}
		else if(high_state_ros.mode == 1 || high_state_ros.mode == 0)
		{
			sit();	
		}
   		on_duration = 0; 
		return;
	}

    std::cout << "last command = " << results[3] << std::endl;

    //if (results[4] == "ON")
    //{
    //    	std::cout << "microswitch triggered triggered" << std::endl;	
    //    	if(high_state_ros.mode == 1 || high_state_ros.mode == 0)
    //    	{
    //    		sit();	
    //    		return;
    //    	}
    //}
    //else if(results[4] == "OFF")
    //{
    //    	std::cout << "microswitch triggered released" << std::endl;	
    //    	if(high_state_ros.mode == 7)
    //    	{
    //    		stand();
    //    		return;
    //    	}
    //
    //}
    //std::cout << "on duration = " << on_duration << std::endl;

    geometry_msgs::Twist  twist;
    //geometry_msgs::Twist::ConstPtr twist_ptr(twist);
    if(results[1] == "BKWD")
    {
        twist.linear.x = -0.2;
    }
    else if(results[1] == "FWD")
    {
        twist.linear.x = 0.2;
    }
    if(results[2] == "LEFT")
    {
    	twist.angular.z = 0.6;
    }
    else if(results[2] == "RIGHT")
    {
        twist.angular.z = -0.6;
    }
	

    //if(joy_stop == 0 && twist.linear.x == 0 && twist.angular.z == 0)
    if(joy_stop == 0)
    {

    	if(results[1] == "BKWD")
    	{
    	    max_vel -= 0.01;
    	}
    	else if(results[1] == "FWD")
    	{
    	    max_vel += 0.01;
    	}

	if(max_vel > 1.0) max_vel = 1.0;
	if(max_vel < 0.3) max_vel = 0.3;

	printf("max vel = %f\n", max_vel);
   	return; 
    }

    UNITREE_LEGGED_SDK::HighCmd cmd;

    cmd.head[0] = 0xFE;
    cmd.head[1] = 0xEF;
    cmd.levelFlag = UNITREE_LEGGED_SDK::HIGHLEVEL;
    cmd.mode = 0;
    cmd.gaitType = 0;
    cmd.speedLevel = 0;
    cmd.footRaiseHeight = 0;
    cmd.bodyHeight = 0;
    cmd.euler[0] = 0;
    cmd.euler[1] = 0;
    cmd.euler[2] = 0;
    cmd.velocity[0] = 0.0f;
    cmd.velocity[1] = 0.0f;
    cmd.yawSpeed = 0.0f;
    cmd.reserve = 0;

    //cmd.velocity[0] = msg->linear.x;
    //cmd.velocity[1] = msg->linear.y;
    //cmd.yawSpeed = msg->angular.z;

    cmd.mode = 2;
    cmd.gaitType = 1;

    custom.high_cmd = cmd;
    //custom.high_cmd.velocity[0] = 0.2;
    //custom.high_cmd.velocity[1] = -custom.high_cmd.velocity[1];
    //cout << "shaojun modification!!!!" << std::endl;
    //printf("shaojun modification");
    custom.high_cmd.velocity[0] = twist.linear.x;;
    custom.high_cmd.yawSpeed = twist.angular.z;;
    printf("cmd_x_vel = %f\n", custom.high_cmd.velocity[0]);
    printf("cmd_y_vel = %f\n", custom.high_cmd.velocity[1]);
    printf("cmd_yaw_vel = %f\n", custom.high_cmd.yawSpeed);

    //unitree_legged_msgs::HighState high_state_ros;

    high_state_ros = state2rosMsg(custom.high_state);
    high_state_ros.velocity[0] = twist.linear.x;
    high_state_ros.velocity[1] = twist.angular.z;

    pub_high.publish(high_state_ros);


    // handle the cmd_vel part

    // need to split the string
    //if(Joy[0] == 'F')
    //{
    //    printf("forward button pressed\n");	
    //    joy_stop = 0;
    //}
    //else if (Joy[0] == 'B')
    //{
    //    printf("forward button pressed\n");	
    //    joy_stop = 1;
    //}
    //else
    //{
    //  printf("other button pressed\n");	 
    //}
}

//void ultrasonicCallback(const sensor_msgs::Range::ConstPtr & msg) {
//	printf("ultrasonic range = ", msg->range);
//}
//
//
void ultrasonicCallback(const sensor_msgs::Range::ConstPtr & msg) {
	printf("ultrasonic range = %f\n", msg->range);
	float range = msg->range;
	if(range == 10.0)
	{
		return;	
	}
	if(range < 0.7)
	{
		//joy_stop = 1;	
		ultrasonic_stop = 1;	
	}
	else
	{
		//joy_stop = 0;	
		ultrasonic_stop = 0;	
	}
	printf("ultrasonic stop = %d\n", ultrasonic_stop);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "twist_sub");

    ros::NodeHandle nh;

    pub_high = nh.advertise<unitree_legged_msgs::HighState>("high_state", 1);

    sub_cmd_vel = nh.subscribe("cmd_vel", 1, cmdVelCallback);
    //sub_cmd_vel = nh.subscribe("cmd_vel_soft_leash", 1, cmdVelCallbackSoftLeash);
    //sub_joy = nh.subscribe("cmd_vel", 1, joyCallback);
    sub_joy = nh.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);
    //sub_joy = nh.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);
    sub_leash = nh.subscribe<std_msgs::String>("chatter", 10, leashCallback);

    //sub_ultrasonic = nh.subscribe<sensor_msgs::Range>("/range_ultrasonic_face", 10, ultrasonicCallback);
    //sub_ultrasonic = nh.subscribe<sensor_msgs::Range>("/range_ultrasonic_face", 10, ultrasonicCallback);
    sub_ultrasonic = nh.subscribe<sensor_msgs::Range>("/ultrasonic_front", 10, ultrasonicCallback);

    LoopFunc loop_udpSend("high_udp_send", 0.002, 3, boost::bind(&Custom::highUdpSend, &custom));
    LoopFunc loop_udpRecv("high_udp_recv", 0.002, 3, boost::bind(&Custom::highUdpRecv, &custom));


    loop_udpSend.start();
    loop_udpRecv.start();

    //stand();
    //sit();


    ros::spin();

    return 0;
}
