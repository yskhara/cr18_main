/*
 * tr_main.cpp
 *
 *  Created on: Jan 4, 2018
 *      Author: yusaku
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <string>
/*
static constexpr int ButtonA = 0;
static constexpr int ButtonB = 1;
static constexpr int ButtonX = 2;
static constexpr int ButtonY = 3;
static constexpr int ButtonLB = 4;
static constexpr int ButtonRB = 5;
static constexpr int ButtonSelect = 6;
static constexpr int ButtonStart = 7;
static constexpr int ButtonLeftThumb = 8;
static constexpr int ButtonRightThumb = 9;

static constexpr int AxisDPadX = 6;
static constexpr int AxisDPadY = 7;
*/

enum class CarrierStatus : uint16_t
{
	shutdown			= 0x0000,
	reset				= 0x0001,

	/*
	operational			= 0x0002,

	chuck0_chucked		= 0x0010,
	chuck1_chucked		= 0x0020,
	chuck2_chucked		= 0x0040,
	chuck3_chucked		= 0x0080,
	 */
};

enum class CarrierCommands : uint16_t
{
	shutdown_cmd		= 0x0000,
	reset_cmd			= 0x0001,
	/*
	operational			= 0x0002,
	 */

	chuck_cmd			= 0x0100,
	unchuck_cmd			= 0x0200,

	chuck0				= 0x0010,
	chuck1				= 0x0020,
	chuck2				= 0x0040,
	chuck3				= 0x0080,
};

class CrMain
{
public:
	CrMain(void);

private:
	void shutdownCallback(const std_msgs::Bool::ConstPtr& msg);
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

	ros::NodeHandle nh_;

	//int linear_, angular_;
	ros::Subscriber joy_sub;
	ros::Subscriber shutdown_sub;
	ros::Publisher hand_cmd_pub;
	//ros::Publisher hand_unchuck_thres_pub;

	std_msgs::UInt16 hand_cmd_msg;
	//std_msgs::UInt16 hand_unchuck_thres_msg;

	bool _shutdown = false;

	static int ButtonA;
	static int ButtonB;
	static int ButtonX;
	static int ButtonY;
	static int ButtonLB;
	static int ButtonRB;
	static int ButtonSelect;
	static int ButtonStart;
	static int ButtonLeftThumb;
	static int ButtonRightThumb;

	static int AxisDPadX;
	static int AxisDPadY;
};

int CrMain::ButtonA = 0;
int CrMain::ButtonB = 1;
int CrMain::ButtonX = 2;
int CrMain::ButtonY = 3;
int CrMain::ButtonLB = 4;
int CrMain::ButtonRB = 5;
int CrMain::ButtonSelect = 6;
int CrMain::ButtonStart = 7;
int CrMain::ButtonLeftThumb = 8;
int CrMain::ButtonRightThumb = 9;

int CrMain::AxisDPadX = 6;
int CrMain::AxisDPadY = 7;

CrMain::CrMain(void)
{
	joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &CrMain::joyCallback, this);
	shutdown_sub = nh_.subscribe<std_msgs::Bool>("shutdown", 10, &CrMain::shutdownCallback, this);

	this->hand_cmd_pub = nh_.advertise<std_msgs::UInt16>("hand/cmd", 1);
	//this->hand_unchuck_thres_pub = nh_.advertise<std_msgs::UInt16>("hand/unchuck_thres", 1);

	auto nh_priv = ros::NodeHandle("~");
	//this->hand_thresholds = {0x0130, 0x0130, 0x0090};
	//std::vector<int> tmp;
	//nh_priv.getParam("unchuck_thres", tmp);
	//if(tmp.size() == 3)
	//{
	//	this->hand_thresholds = tmp;
	//}
	//ROS_INFO("thresholds: %d, %d, %d", this->hand_thresholds[0], this->hand_thresholds[1], this->hand_thresholds[2]);

	nh_.getParam("ButtonA", ButtonA);
	nh_.getParam("ButtonB", ButtonB);
	nh_.getParam("ButtonX", ButtonX);
	nh_.getParam("ButtonY", ButtonY);
	nh_.getParam("ButtonLB", ButtonLB);
	nh_.getParam("ButtonRB", ButtonRB);
	nh_.getParam("ButtonSelect", ButtonSelect);
	nh_.getParam("ButtonStart", ButtonStart);
	nh_.getParam("ButtonLeftThumb", ButtonLeftThumb);
	nh_.getParam("ButtonRightThumb", ButtonRightThumb);

	nh_.getParam("AxisDPadX", AxisDPadX);
	nh_.getParam("AxisDPadY", AxisDPadY);
}

void CrMain::shutdownCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if(msg->data)
	{
		if(!this->_shutdown)
		{
			this->_shutdown = true;

			ROS_INFO("aborting.");
		}

		hand_cmd_msg.data = (uint16_t)CarrierCommands::shutdown_cmd;
		hand_cmd_pub.publish(hand_cmd_msg);
	}
	else
	{
		if(this->_shutdown)
		{
			hand_cmd_msg.data = (uint16_t) CarrierCommands::reset_cmd;
			hand_cmd_pub.publish(hand_cmd_msg);

			this->_shutdown = false;
		}
	}
}

void CrMain::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	static bool last_a = false;
	static bool last_b = false;
	static bool last_x = false;
	static bool last_y = false;
	//static int last_dpadXCmd = 0;

	bool _a = joy->buttons[ButtonA];
	bool _b = joy->buttons[ButtonB];
	bool _x = joy->buttons[ButtonX];
	bool _y = joy->buttons[ButtonY];

	if(!this->_shutdown)
	{
		if(_a && !last_a)
		{
			// chuck all
			hand_cmd_msg.data = (uint16_t)CarrierCommands::chuck_cmd
					| (uint16_t)CarrierCommands::chuck0
					| (uint16_t)CarrierCommands::chuck1
					| (uint16_t)CarrierCommands::chuck2
					| (uint16_t)CarrierCommands::chuck3;
			hand_cmd_pub.publish(hand_cmd_msg);
		}
		else if(_b && !last_b)
		{
			// unchuck all
			hand_cmd_msg.data = (uint16_t)CarrierCommands::unchuck_cmd
					| (uint16_t)CarrierCommands::chuck0
					| (uint16_t)CarrierCommands::chuck1
					| (uint16_t)CarrierCommands::chuck2
					| (uint16_t)CarrierCommands::chuck3;
			hand_cmd_pub.publish(hand_cmd_msg);
		}
		else if(_x && !last_x)
		{
			// unchuck 2
			hand_cmd_msg.data = (uint16_t)CarrierCommands::unchuck_cmd
					| (uint16_t)CarrierCommands::chuck2 ;
			hand_cmd_pub.publish(hand_cmd_msg);
		}
		else if(_y && !last_y)
		{
			// unchuck 0
			hand_cmd_msg.data = (uint16_t)CarrierCommands::unchuck_cmd
					| (uint16_t)CarrierCommands::chuck0 ;
			hand_cmd_pub.publish(hand_cmd_msg);
		}
	}

	last_a = _a;
	last_b = _b;
	last_x = _x;
	last_y = _y;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "cr_main_joy");

	CrMain *crMain = new CrMain();
	ROS_INFO("cr_main_joy node has started.");

	ros::spin();
	ROS_INFO("cr_main_joy node has been terminated.");
}



