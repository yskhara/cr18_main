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

enum class LauncherCommands : uint16_t
{
	shutdown_cmd		= 0x0000,
	reset_cmd			= 0x0001,

	disarm_cmd			= 0x0010,
	receive_cmd			= 0x0011,
	launch_cmd			= 0x0014,

	set_thres_cmd		= 0x4000,
	set_thres_mask		= 0xcfff,
};

class TrMain
{
public:
	TrMain(void);

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

	ros::NodeHandle nh_;

	//int linear_, angular_;
	ros::Subscriber joy_sub_;
	ros::Publisher launcher_cmd_pub;
	ros::Publisher launcher_unchuck_thres_pub;

	std_msgs::UInt16 launcher_cmd_msg;
	std_msgs::UInt16 launcher_unchuck_thres_msg;

	std::vector<int> launcher_thresholds;
	int launcher_currentThresholdIndex = 0;


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
	static int AxisDPadY;\
};

int TrMain::ButtonA = 0;
int TrMain::ButtonB = 1;
int TrMain::ButtonX = 2;
int TrMain::ButtonY = 3;
int TrMain::ButtonLB = 4;
int TrMain::ButtonRB = 5;
int TrMain::ButtonSelect = 6;
int TrMain::ButtonStart = 7;
int TrMain::ButtonLeftThumb = 8;
int TrMain::ButtonRightThumb = 9;

int TrMain::AxisDPadX = 6;
int TrMain::AxisDPadY = 7;

TrMain::TrMain(void)
{
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TrMain::joyCallback, this);

	this->launcher_cmd_pub = nh_.advertise<std_msgs::UInt16>("launcher/cmd", 1);
	this->launcher_unchuck_thres_pub = nh_.advertise<std_msgs::UInt16>("launcher/unchuck_thres", 1);

	auto nh_priv = ros::NodeHandle("~");
	this->launcher_thresholds = {0x0130, 0x0130, 0x0090};
	std::vector<int> tmp;
	nh_priv.getParam("unchuck_thres", tmp);
	if(tmp.size() == 3)
	{
		this->launcher_thresholds = tmp;
	}
	ROS_INFO("thresholds: %d, %d, %d", this->launcher_thresholds[0], this->launcher_thresholds[1], this->launcher_thresholds[2]);

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

void TrMain::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
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

	//launcher_cmd_msg.data = null_cmd;
	if(_a && (_a != last_a))
	{
		// disarm
		launcher_cmd_msg.data = (uint16_t)LauncherCommands::disarm_cmd;
		launcher_cmd_pub.publish(launcher_cmd_msg);
	}
	else if(_b && (_b != last_b))
	{
		// arm
		launcher_cmd_msg.data = (uint16_t)LauncherCommands::receive_cmd;
		launcher_cmd_pub.publish(launcher_cmd_msg);
	}
	else if(_x && (_x != last_x))
	{
		// launch
		launcher_cmd_msg.data = (uint16_t)LauncherCommands::launch_cmd;
		launcher_cmd_pub.publish(launcher_cmd_msg);
	}

	last_a = _a;
	last_b = _b;
	last_x = _x;
	last_y = _y;

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
	*/

	if(dpadXCmd != 0 && dpadXCmd != last_dpadXCmd)
	{
		if(dpadXCmd == 1)
		{
			this->launcher_currentThresholdIndex++;
			if(launcher_currentThresholdIndex > 2)
			{
				launcher_currentThresholdIndex = 2;
			}
		}
		else if(dpadXCmd == -1)
		{
			this->launcher_currentThresholdIndex--;
			if(launcher_currentThresholdIndex < 0)
			{
				launcher_currentThresholdIndex = 0;
			}
		}

		this->launcher_unchuck_thres_msg.data = this->launcher_thresholds[this->launcher_currentThresholdIndex];
		this->launcher_unchuck_thres_pub.publish(this->launcher_unchuck_thres_msg);
	}

	last_dpadXCmd = dpadXCmd;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "tr_main");

	TrMain *trMain = new TrMain();
	ROS_INFO("tr_main node has started.");

	ros::spin();
	ROS_INFO("tr_main node has been terminated.");
}



