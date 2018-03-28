/*
 * tr_main.cpp
 *
 *  Created on: Jan 4, 2018
 *      Author: yusaku
 */

#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>
#include <vector>
#include <string>

#include "Coordinates.hpp"

enum class ControllerStatus : uint16_t
{
	init				= 0x0000,
	shutdown			= 0x0001,

	standby = 0x0010,				// standby at start zone
	sz_to_dp1,						// moving from SZ  to DP1
	dp_delivery,					// receiving a shuttle at a delivery point
	dp1_to_tz1,						// moving from DP1 to TZ1
	tz1_throwing,					// throwing at TZ1
	tz1_to_dp1,						// moving from TZ1 to DP1
	dp1_to_tz2,						// moving from DP1 to TZ2
	tz2_to_dp2,						// moving from TZ2_to_DP2
};

enum class ControllerCommands : uint16_t
{
	shutdown,						// shutdown

	standby,						// stand-by at SZ

	dp_receive,						// receive a shuttle at a DP
	tz_throw,						// throw at a TZ

	set_tz1,
	set_tz2,						// set threshold
	set_tz3,

	sz_to_dp1,						// move from SZ  to DP1
	sz_to_dp2,						// move from SZ  to DP2

	tz1_to_dp1,						// move from TZ1 to DP1
	tz2_to_dp2,						// move from TZ2 to DP2
	tz3_to_dp2,						// move from TZ3 to DP2

	dp1_to_tz1,						// move from DP1 to TZ1
	dp1_to_tz2,						// move from DP1 to TZ2

	dp2_to_tz2,						// move from DP2 to TZ2
	dp2_to_tz3,						// move from DP2 to TZ3

	disarm,
	delay,
};

enum class LauncherStatus : uint16_t
{
	shutdown			= 0x0000,
	reset				= 0x0001,

	disarmed			= 0x0010,
	receiving			= 0x0011,
	armed				= 0x0012,
	launching			= 0x0013,
	launch_cplt			= 0x0014,
};

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

enum class BaseStatus : uint16_t
{
	shutdown			= 0x0000,
	reset				= 0x0001,
	operational			= 0x0010,
};

enum class BaseCommands : uint16_t
{
	shutdown_cmd		= 0x0000,
	reset_cmd			= 0x0001,
	operational_cmd		= 0x0010,
};

class TrLauncherUnitTest
{
public:
	TrLauncherUnitTest(void);

private:
	void launcherStatusCallback(const std_msgs::UInt16::ConstPtr& msg);
	void shutdownCallback(const std_msgs::Bool::ConstPtr& msg);

	void control_timer_callback(const ros::TimerEvent& event);

	ros::NodeHandle nh_;

	ros::Subscriber launcher_status_sub;
	ros::Subscriber shutdown_sub;

	ros::Publisher launcher_cmd_pub;
	std_msgs::UInt16 launcher_cmd_msg;

	ros::Publisher base_cmd_pub;
	std_msgs::UInt16 base_cmd_msg;

	int currentCommandIndex = -1;
	std::vector<ControllerCommands> command_list;


	//uint16_t launcher_threshold = 0x0100;
	std::vector<int> launcher_thresholds;

	int _delay_s;

	ros::Timer control_timer;

	LauncherStatus last_status = LauncherStatus::shutdown;
	ros::Time last_status_time;


	void set_thres(uint16_t threshold);
	void disarm(void);
	void receive_shuttle(void);
	void arm_ready(void);
	void launch(void);

	void set_delay(double delay_s);

	// flags
	bool _start_pressed = false;
	bool _launch_completed = false;
	bool _shuttle_received = false;
	bool _is_moving = false;
	bool _is_throwing = false;
	bool _is_receiving = false;

	inline void clear_flags(void)
	{
		//_start_pressed = false;
		_launch_completed = false;
		_shuttle_received = false;
		_is_moving = false;
		_is_throwing = false;
		_is_receiving = false;
	}
};

TrLauncherUnitTest::TrLauncherUnitTest(void)
{
	this->launcher_status_sub = nh_.subscribe<std_msgs::UInt16>("launcher/status", 10, &TrLauncherUnitTest::launcherStatusCallback, this);
	this->shutdown_sub = nh_.subscribe<std_msgs::Bool>("shutdown", 10, &TrLauncherUnitTest::shutdownCallback, this);

	this->launcher_cmd_pub = nh_.advertise<std_msgs::UInt16>("launcher/cmd", 1);
	this->base_cmd_pub = nh_.advertise<std_msgs::UInt16>("base/cmd", 1);

	auto nh_priv = ros::NodeHandle("~");

	this->launcher_thresholds = {0x0130*4, 0x0130, 0x0090};
	std::vector<int> tmp;
	nh_priv.getParam("unchuck_thres", tmp);
	if(tmp.size() == 3)
	{
		this->launcher_thresholds = tmp;
	}
	ROS_INFO("thresholds: %d, %d, %d", this->launcher_thresholds[0], this->launcher_thresholds[1], this->launcher_thresholds[2]);

	this->command_list.push_back(ControllerCommands::standby);

	// receive at dp1
	//this->command_list.push_back(ControllerCommands::sz_to_dp1);
	this->command_list.push_back(ControllerCommands::dp_receive);
	this->command_list.push_back(ControllerCommands::delay);

	// throw at tz1
	//this->command_list.push_back(ControllerCommands::dp1_to_tz1);
	this->command_list.push_back(ControllerCommands::set_tz1);
	this->command_list.push_back(ControllerCommands::tz_throw);
	this->command_list.push_back(ControllerCommands::delay);
	this->command_list.push_back(ControllerCommands::disarm);

	// timer starts immediately
	control_timer = nh_.createTimer(ros::Duration(0.1), &TrLauncherUnitTest::control_timer_callback, this);
}

void TrLauncherUnitTest::launcherStatusCallback(const std_msgs::UInt16::ConstPtr& msg)
{
	LauncherStatus status = (LauncherStatus)msg->data;

	switch(status)
	{
	case LauncherStatus::disarmed:
		break;

	case LauncherStatus::receiving:
		this->_shuttle_received = false;
		break;

	case LauncherStatus::armed:
		this->_shuttle_received = true;
		break;

	case LauncherStatus::launch_cplt:
		this->_launch_completed = true;
		break;

	default:
		break;
	}

	last_status = status;
	last_status_time = ros::Time::now();
}

void TrLauncherUnitTest::shutdownCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if(msg->data)
	{
		if(this->currentCommandIndex != 0)
		{
			ROS_INFO("aborting.");

			this->currentCommandIndex = 0;

			//this->disarm();
		}

		launcher_cmd_msg.data = (uint16_t)LauncherCommands::shutdown_cmd;
		launcher_cmd_pub.publish(launcher_cmd_msg);

		base_cmd_msg.data = (uint16_t)BaseCommands::shutdown_cmd;
		base_cmd_pub.publish(base_cmd_msg);
	}
	else if(this->currentCommandIndex == 0)
	{
		launcher_cmd_msg.data = (uint16_t)LauncherCommands::reset_cmd;
		launcher_cmd_pub.publish(launcher_cmd_msg);

		base_cmd_msg.data = (uint16_t)BaseCommands::reset_cmd;
		base_cmd_pub.publish(base_cmd_msg);

		base_cmd_msg.data = (uint16_t)BaseCommands::operational_cmd;
		base_cmd_pub.publish(base_cmd_msg);

		this->disarm();


		this->clear_flags();

		this->_start_pressed = true;
	}
}

void TrLauncherUnitTest::control_timer_callback(const ros::TimerEvent& event)
{
	if(this->command_list.size() <= this->currentCommandIndex)
	{
		// TODO: do something
		currentCommandIndex = 1;

		return;
	}

	ControllerCommands currentCommand = this->command_list.at(this->currentCommandIndex);


	if(currentCommand == ControllerCommands::standby)
	{
		clear_flags();

		if(this->_start_pressed)
		{
			this->_start_pressed = false;
			this->currentCommandIndex++;
			ROS_INFO("starting.");
		}
	}
	else if(currentCommand == ControllerCommands::dp_receive)
	{
		if(this->_is_receiving)
		{
			if(this->_shuttle_received)
			{
				set_delay(5.0);

				this->_is_receiving = false;
				this->_shuttle_received = false;
				this->currentCommandIndex++;
				ROS_INFO("shuttle received.");
			}
		}
		else
		{
			clear_flags();
			this->receive_shuttle();
			this->_is_receiving = true;
			ROS_INFO("waiting for shuttle.");
		}
	}
	else if(currentCommand == ControllerCommands::tz_throw)
	{
		if(this->_is_throwing)
		{
			if(this->_launch_completed)
			{
				//this->disarm();

				set_delay(5.0);

				this->_is_throwing = false;
				this->_launch_completed = false;
				this->currentCommandIndex++;
				ROS_INFO("launch completed.");
			}
		}
		else
		{
			this->launch();
			this->_is_throwing = true;

			ROS_INFO("launching.");
		}
	}
	else if(currentCommand == ControllerCommands::set_tz1)
	{
		this->set_thres(this->launcher_thresholds[0]);
		this->currentCommandIndex++;
	}
	else if(currentCommand == ControllerCommands::delay)
	{
		if(this->_delay_s == 0)
		{
			return;
		}

		if(this->_delay_s < ros::Time::now().toSec())
		{
			this->_delay_s = 0;
			this->currentCommandIndex++;
		}
	}
	else if(currentCommand == ControllerCommands::disarm)
	{
		this->disarm();
		set_delay(3.0);
		this->currentCommandIndex++;
	}
	else
	{

	}
}

void TrLauncherUnitTest::set_thres(uint16_t threshold)
{
	// set threshold
	launcher_cmd_msg.data = (uint16_t)LauncherCommands::set_thres_cmd | (threshold & (uint16_t)LauncherCommands::set_thres_mask);
	launcher_cmd_pub.publish(launcher_cmd_msg);
}

void TrLauncherUnitTest::disarm(void)
{
	// disarm
	launcher_cmd_msg.data = (uint16_t)LauncherCommands::disarm_cmd;
	launcher_cmd_pub.publish(launcher_cmd_msg);
}

void TrLauncherUnitTest::receive_shuttle(void)
{
	// arm
	launcher_cmd_msg.data = (uint16_t)LauncherCommands::receive_cmd;
	launcher_cmd_pub.publish(launcher_cmd_msg);
}

void TrLauncherUnitTest::launch(void)
{
	// launch
	launcher_cmd_msg.data = (uint16_t)LauncherCommands::launch_cmd;
	launcher_cmd_pub.publish(launcher_cmd_msg);
}

void TrLauncherUnitTest::set_delay(double delay_s)
{
	//if(this->_delay_s != 0)
	//{
	//	return;
	//}

	this->_delay_s = ros::Time::now().toSec() + delay_s;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tr_main");

	TrLauncherUnitTest *instance = new TrLauncherUnitTest();
	ROS_INFO("TR launcher unit test node has started.");

	ros::spin();
	ROS_INFO("TR launcher unit test node has been terminated.");
}



