/*
 * tr_main.cpp
 *
 *  Created on: Jan 4, 2018
 *      Author: yusaku
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt16.h>
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

class CrMain
{
public:
	CrMain(void);

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

	ros::NodeHandle nh_;

	//int linear_, angular_;
	ros::Subscriber joy_sub_;
	ros::Publisher hand_cmd_pub;
	//ros::Publisher hand_unchuck_thres_pub;

	std_msgs::UInt16 hand_cmd_msg;
	//std_msgs::UInt16 hand_unchuck_thres_msg;

	//std::vector<int> hand_thresholds;
	//int hand_currentThresholdIndex = 0;
	int hand_currentIndex = 1;

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

	static constexpr uint16_t null_cmd			= 0x0000;
	static constexpr uint16_t close_cmd		= 0x0010;
	static constexpr uint16_t open_cmd			= 0x0020;
	static constexpr uint16_t rotate_cmd		= 0x0030;


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
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &CrMain::joyCallback, this);

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

void CrMain::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	static bool last_a = false;
	static bool last_b = false;
	static bool last_x = false;
	static bool last_y = false;
	static int last_dpadXCmd = 0;

	bool _a = joy->buttons[ButtonA];
	bool _b = joy->buttons[ButtonB];
	bool _x = joy->buttons[ButtonX];
	bool _y = joy->buttons[ButtonY];

	//hand_cmd_msg.data = null_cmd;
	if(_a && (_a != last_a))
	{
		// disarm
		hand_cmd_msg.data = open_cmd | 0x0001;
		hand_cmd_pub.publish(hand_cmd_msg);
	}
	else if(_b && (_b != last_b))
	{
		// arm
		hand_cmd_msg.data = close_cmd | 0x0001;
		hand_cmd_pub.publish(hand_cmd_msg);
	}
	else if(_x && (_x != last_x))
	{
		// launch
		hand_cmd_msg.data = rotate_cmd | 0x0001;
		hand_cmd_pub.publish(hand_cmd_msg);
	}

	last_a = _a;
	last_b = _b;
	last_x = _x;
	last_y = _y;

	/*

	double dpadX = -joy->axes[AxisDPadX];
	int dpadXCmd = 0;

	dpadXCmd = (dpadX > 0.5) ? 1 : (dpadX > -0.5) ? 0 : -1;

	/*
	if(dpadX > 0.5)
	{
		// right for next
		dpadXCmd = 1;
	}
	else if(dpadX > -0.5)
	{
		// neutral
		dpadXCmd = 0;
	}
	else
	{
		// left for prev
		dpadXCmd = -1;
	}
	*

	if(dpadXCmd != 0 && dpadXCmd != last_dpadXCmd)
	{
		if(dpadXCmd == 1)
		{
			this->hand_currentThresholdIndex++;
			if(hand_currentThresholdIndex > 2)
			{
				hand_currentThresholdIndex = 2;
			}
		}
		else if(dpadXCmd == -1)
		{
			this->hand_currentThresholdIndex--;
			if(hand_currentThresholdIndex < 0)
			{
				hand_currentThresholdIndex = 0;
			}
		}

		//this->hand_unchuck_thres_msg.data = this->hand_thresholds[this->hand_currentThresholdIndex];
		//this->hand_unchuck_thres_pub.publish(this->hand_unchuck_thres_msg);
	}

	last_dpadXCmd = dpadXCmd;

	*/
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "cr_main_joy");

	CrMain *crMain = new CrMain();
	ROS_INFO("cr_main_joy node has started.");

	ros::spin();
	ROS_INFO("cr_main_joy node has been terminated.");
}



