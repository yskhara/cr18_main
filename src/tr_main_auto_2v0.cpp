/*
 * tr_main_auto_2v0.cpp
 *
 *  Created on: Mar 22, 2018
 *      Author: yusaku
 */

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
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

	sz_to_tz1,						// move from SZ  to TZ1
	sz_to_dp2,						// move from SZ  to DP2

	tz1_to_tz2,						// move from TZ1 to TZ2

	dp2_to_tz2,						// move from DP2 to TZ2
	dp2_to_tz3,						// move from DP2 to TZ3

	tz2_to_dp2,						// move from TZ2 to DP2
	tz3_to_dp2,						// move from TZ3 to DP2

	set_delay_250ms,
	set_delay_500ms,
	set_delay_1s,

	disarm,
	delay,

	segno,
	dal_segno,
};

enum class OpMode : uint8_t
{
	def		= 0b000,
	tz1_op	= 0b001,		// sz -> tz1 -> tz2 -> tz3
	tz2_op	= 0b010,		// sz -> tz2 -> tz3
	tz3_op	= 0b011,		// sz -> tz3
	tz1_ut	= 0b101,
	tz2_ut	= 0b110,
	tz3_ut	= 0b111,
};

enum class LauncherStatus : uint16_t
{
	shutdown			= 0x0000,
	reset				= 0x0001,

	disarmed			= 0x0010,
	receiving			= 0x0011,
	received			= 0x0012,
	arming				= 0x0013,
	armed				= 0x0014,
	launching			= 0x0015,
	launch_cplt			= 0x0016,

	sensing				= 0x0020,
};

enum class LauncherCommands : uint16_t
{
	shutdown_cmd		= 0x0000,
	reset_cmd			= 0x0001,

	disarm_cmd			= 0x0010,
	receive_cmd			= 0x0011,
//	arm_cmd				= 0x0013,
	launch_cmd			= 0x0014,

	sense_cmd			= 0x0020,

	set_thres_cmd		= 0x4000,
	set_thres_mask		= 0x3fff,
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

class TrMain
{
public:
	TrMain(void);

private:
	void baseStatusCallback(const std_msgs::UInt16::ConstPtr& msg);
	void launcherStatusCallback(const std_msgs::UInt16::ConstPtr& msg);
	//void shutdownCallback(const std_msgs::Bool::ConstPtr& msg);

	void baseConfCallback(const std_msgs::UInt8::ConstPtr& msg);

	void goalReachedCallback(const std_msgs::Bool::ConstPtr& msg);

	void control_timer_callback(const ros::TimerEvent& event);

	ros::NodeHandle nh_;

	ros::Subscriber launcher_status_sub;
	//ros::Subscriber shutdown_sub;

	ros::Publisher launcher_cmd_pub;
	std_msgs::UInt16 launcher_cmd_msg;

	ros::Subscriber base_status_sub;
	ros::Publisher base_cmd_pub;
	ros::Subscriber base_conf_sub;
	std_msgs::UInt16 base_cmd_msg;


	ros::Subscriber goal_reached_sub;

	ros::Publisher target_pub;
	ros::Publisher abort_pub;

	//nav_msgs::Path target_msg;
	std_msgs::Bool abort_msg;

	ros::Publisher initialpose_pub;
	//geometry_msgs::PoseWithCovarianceStamped initialpose_msg;

	int currentCommandIndex = -1;
	const std::vector<ControllerCommands> *command_list;

	static const std::vector<ControllerCommands> tz1_op_commands;
	static const std::vector<ControllerCommands> tz2_op_commands;
	static const std::vector<ControllerCommands> tz3_op_commands;

	static const std::vector<ControllerCommands> tz1_ut_commands;
	static const std::vector<ControllerCommands> tz2_ut_commands;
	static const std::vector<ControllerCommands> tz3_ut_commands;

	static const std::vector<ControllerCommands> default_commands;

	//uint16_t launcher_threshold = 0x0100;
	std::vector<int> launcher_thresholds;
	OpMode _op_mode;

	double _receive_delay_s;

	int _delay_s = 0;

	ros::Timer control_timer;

	LauncherStatus launcher_last_status = LauncherStatus::shutdown;
	ros::Time launcher_last_status_time;

	BaseStatus base_last_status = BaseStatus::shutdown;
	ros::Time base_last_status_time;

	void shutdown(void);
	void restart(void);

	void set_thres(uint16_t threshold);
	void disarm(void);
	void receive_shuttle(void);
	void arm_ready(void);
	void launch(void);
	void sense(void);

	void set_pose(geometry_msgs::Pose pose);
	void publish_path(nav_msgs::Path path);
	void publish_path(geometry_msgs::Pose from, geometry_msgs::Pose to);
	void publish_path(geometry_msgs::Pose from, geometry_msgs::Pose via, geometry_msgs::Pose to);

	void set_delay(double delay_s);

	// flags
	bool _start_pressed = false;
	bool _launch_completed = false;
	bool _shuttle_received = false;
	bool _armed = false;
	bool _is_moving = false;
	bool _is_throwing = false;
	bool _is_receiving = false;
	bool _goal_reached = false;
	bool _has_base_restarted = false;

	inline void clear_flags(void)
	{
		//_start_pressed = false;
		_launch_completed = false;
		_shuttle_received = false;
		_armed = false;
		_is_moving = false;
		_is_throwing = false;
		_is_receiving = false;
		_goal_reached = false;
		_has_base_restarted = false;
	}
};

const std::vector<ControllerCommands> TrMain::tz1_op_commands
(
	{
		ControllerCommands::standby,

		// receive at tz1
		ControllerCommands::sz_to_tz1,
		ControllerCommands::dp_receive,
		//ControllerCommands::set_delay_500ms,
		//ControllerCommands::delay,
		//ControllerCommands::set_delay_500ms,
		//ControllerCommands::delay,

		// throw at tz1
		ControllerCommands::set_tz1,
		ControllerCommands::tz_throw,
		ControllerCommands::disarm,

		// receive at tz1
		ControllerCommands::dp_receive,
		ControllerCommands::set_delay_500ms,
		ControllerCommands::delay,
		ControllerCommands::set_delay_500ms,
		ControllerCommands::delay,
		ControllerCommands::set_delay_500ms,
		ControllerCommands::delay,

		// throw at tz2
		ControllerCommands::tz1_to_tz2,
		ControllerCommands::set_tz2,
		ControllerCommands::tz_throw,
		ControllerCommands::disarm,

		ControllerCommands::tz2_to_dp2,

		// repeat from here
		ControllerCommands::segno,

		// receive at dp2
		ControllerCommands::dp_receive,
		ControllerCommands::set_delay_500ms,
		ControllerCommands::delay,
		ControllerCommands::set_delay_500ms,
		ControllerCommands::delay,

		// throw at tz3
		ControllerCommands::dp2_to_tz3,
		ControllerCommands::set_tz3,
		ControllerCommands::tz_throw,
		ControllerCommands::disarm,

		ControllerCommands::tz3_to_dp2,

		// repeat forever
		ControllerCommands::dal_segno
	}
);

const std::vector<ControllerCommands> TrMain::tz2_op_commands
(
	{
		ControllerCommands::standby,

		// receive at tz2
		ControllerCommands::sz_to_dp2,
		ControllerCommands::dp_receive,
		ControllerCommands::set_delay_500ms,
		ControllerCommands::delay,
		ControllerCommands::set_delay_500ms,
		ControllerCommands::delay,

		// throw at tz2
		ControllerCommands::dp2_to_tz2,
		ControllerCommands::set_tz2,
		ControllerCommands::tz_throw,
		ControllerCommands::disarm,

		ControllerCommands::tz2_to_dp2,

		// repeat from here
		ControllerCommands::segno,

		// receive at dp2
		ControllerCommands::dp_receive,
		ControllerCommands::set_delay_500ms,
		ControllerCommands::delay,
		ControllerCommands::set_delay_500ms,
		ControllerCommands::delay,

		// throw at tz3
		ControllerCommands::dp2_to_tz3,
		ControllerCommands::set_tz3,
		ControllerCommands::tz_throw,
		ControllerCommands::disarm,

		ControllerCommands::tz3_to_dp2,

		// repeat forever
		ControllerCommands::dal_segno
	}
);

const std::vector<ControllerCommands> TrMain::tz3_op_commands
(
	{
		ControllerCommands::standby,

		// receive at tz2
		ControllerCommands::sz_to_dp2,

		// repeat from here
		ControllerCommands::segno,

		// receive at dp2
		ControllerCommands::dp_receive,
		ControllerCommands::set_delay_500ms,
		ControllerCommands::delay,
		ControllerCommands::set_delay_500ms,
		ControllerCommands::delay,

		// throw at tz3
		ControllerCommands::dp2_to_tz3,
		ControllerCommands::set_tz3,
		ControllerCommands::tz_throw,
		ControllerCommands::disarm,

		ControllerCommands::tz3_to_dp2,

		// repeat forever
		ControllerCommands::dal_segno
	}
);

const std::vector<ControllerCommands> TrMain::tz1_ut_commands
(
	{
		ControllerCommands::standby,

		ControllerCommands::segno,

		ControllerCommands::dp_receive,
		ControllerCommands::delay,
		ControllerCommands::set_tz1,
		ControllerCommands::tz_throw,
		ControllerCommands::disarm,

		// repeat forever
		ControllerCommands::dal_segno
	}
);

const std::vector<ControllerCommands> TrMain::tz2_ut_commands
(
	{
		ControllerCommands::standby,

		ControllerCommands::segno,

		ControllerCommands::dp_receive,
		ControllerCommands::delay,
		ControllerCommands::set_tz2,
		ControllerCommands::tz_throw,
		ControllerCommands::disarm,

		// repeat forever
		ControllerCommands::dal_segno
	}
);

const std::vector<ControllerCommands> TrMain::tz3_ut_commands
(
	{
		ControllerCommands::standby,

		ControllerCommands::segno,

		ControllerCommands::dp_receive,
		ControllerCommands::delay,
		ControllerCommands::set_tz3,
		ControllerCommands::tz_throw,
		ControllerCommands::disarm,

		// repeat forever
		ControllerCommands::dal_segno
	}
);

const std::vector<ControllerCommands> TrMain::default_commands
(
	{
		ControllerCommands::shutdown
	}
);

TrMain::TrMain(void)
{
	this->launcher_status_sub = nh_.subscribe<std_msgs::UInt16>("launcher/status", 10, &TrMain::launcherStatusCallback, this);
	this->base_status_sub = nh_.subscribe<std_msgs::UInt16>("base/status", 10, &TrMain::baseStatusCallback, this);
	//this->shutdown_sub = nh_.subscribe<std_msgs::Bool>("shutdown", 10, &TrMain::shutdownCallback, this);

	this->launcher_cmd_pub = nh_.advertise<std_msgs::UInt16>("launcher/cmd", 1);
	this->base_cmd_pub = nh_.advertise<std_msgs::UInt16>("base/cmd", 1);

	this->base_conf_sub = nh_.subscribe<std_msgs::UInt8>("base/conf", 10, &TrMain::baseConfCallback, this);

	this->goal_reached_sub = nh_.subscribe<std_msgs::Bool>("goal_reached", 10, &TrMain::goalReachedCallback, this);
	this->target_pub = nh_.advertise<nav_msgs::Path>("target_path", 1);
	this->abort_pub = nh_.advertise<std_msgs::Bool>("abort", 1);

	this->initialpose_pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

	auto nh_priv = ros::NodeHandle("~");

	this->launcher_thresholds = {450, 450, 430};
	std::vector<int> tmp;
	nh_priv.getParam("unchuck_thres", tmp);
	if(tmp.size() == 3)
	{
		this->launcher_thresholds = tmp;
	}
	ROS_INFO("thresholds: %d, %d, %d", this->launcher_thresholds[0], this->launcher_thresholds[1], this->launcher_thresholds[2]);

	if(!nh_priv.getParam("receive_delay", _receive_delay_s))
	{
		_receive_delay_s = 1.25;
	}

	this->_op_mode = OpMode::def;
	//this->command_list = nullptr;
	this->command_list = &TrMain::default_commands;

	// timer starts immediately
	this->control_timer = nh_.createTimer(ros::Duration(0.05), &TrMain::control_timer_callback, this);
}

void TrMain::baseStatusCallback(const std_msgs::UInt16::ConstPtr& msg)
{
	BaseStatus status = (BaseStatus)msg->data;

	switch(status)
	{
	case BaseStatus::shutdown:
		if(this->base_last_status != BaseStatus::shutdown)
		{
			this->shutdown();
		}
		break;

	case BaseStatus::reset:
		if(this->base_last_status == BaseStatus::shutdown)
		{
			this->restart();
			this->_has_base_restarted = true;
		}
		break;

	default:
		break;
	}

	base_last_status = status;
	base_last_status_time = ros::Time::now();
}

void TrMain::launcherStatusCallback(const std_msgs::UInt16::ConstPtr& msg)
{
	LauncherStatus status = (LauncherStatus)msg->data;

	switch(status)
	{
	case LauncherStatus::disarmed:
		if(launcher_last_status == LauncherStatus::sensing)
		{
			this->_start_pressed = true;

			//this->restart();
		}
		break;

	case LauncherStatus::receiving:
		this->_shuttle_received = false;
		break;

	case LauncherStatus::received:
		// you can't get this

		this->_shuttle_received = true;
		break;

	case LauncherStatus::arming:
		this->_shuttle_received = true;
		this->_armed = false;
		break;

	case LauncherStatus::armed:
		this->_armed = true;
		break;

	case LauncherStatus::launching:
		this->_armed = false;
		this->_launch_completed = false;
		break;

	case LauncherStatus::launch_cplt:
		this->_launch_completed = true;
		break;

	default:
		break;
	}

	launcher_last_status = status;
	launcher_last_status_time = ros::Time::now();
}

void TrMain::baseConfCallback(const std_msgs::UInt8::ConstPtr& msg)
{
	if(this->currentCommandIndex != -1 && this->currentCommandIndex != 0)
	{
		return;
	}

	if(this->_op_mode != (OpMode)msg->data)
	{
		this->_op_mode = (OpMode)msg->data;

		if(this->_op_mode == OpMode::tz1_op)
		{
			this->command_list = &TrMain::tz1_op_commands;
			ROS_INFO("operation mode set to tz1_op.");
		}
		else if(this->_op_mode == OpMode::tz2_op)
		{
			this->command_list = &TrMain::tz2_op_commands;
			ROS_INFO("operation mode set to tz2_op.");
		}
		else if(this->_op_mode == OpMode::tz3_op)
		{
			this->command_list = &TrMain::tz3_op_commands;
			ROS_INFO("operation mode set to tz3_op.");
		}
		else if(this->_op_mode == OpMode::tz1_ut)
		{
			this->command_list = &TrMain::tz1_ut_commands;
			ROS_INFO("operation mode set to tz1_ut.");
		}
		else if(this->_op_mode == OpMode::tz2_ut)
		{
			this->command_list = &TrMain::tz2_ut_commands;
			ROS_INFO("operation mode set to tz2_ut.");
		}
		else if(this->_op_mode == OpMode::tz3_ut)
		{
			this->command_list = &TrMain::tz3_ut_commands;
			ROS_INFO("operation mode set to tz3_ut.");
		}
	}
}

void TrMain::goalReachedCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if(!msg->data)
	{
		// not reached yet
		return;
	}

	//ROS_INFO("goal reached.");

	abort_msg.data = true;
	this->abort_pub.publish(abort_msg);

	this->_goal_reached = true;
}

/*
void TrMain::shutdownCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if(msg->data)
	{
		this->shutdown();
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
*/

void TrMain::shutdown(void)
{
	if(this->currentCommandIndex != -1)
	{
		ROS_INFO("aborting.");

		this->currentCommandIndex = -1;

		//this->disarm();
	}

	abort_msg.data = true;
	this->abort_pub.publish(abort_msg);

	launcher_cmd_msg.data = (uint16_t)LauncherCommands::shutdown_cmd;
	launcher_cmd_pub.publish(launcher_cmd_msg);

	base_cmd_msg.data = (uint16_t)BaseCommands::shutdown_cmd;
	base_cmd_pub.publish(base_cmd_msg);
}

void TrMain::restart(void)
{
	this->currentCommandIndex = 0;
	ROS_INFO("restarting.");

	launcher_cmd_msg.data = (uint16_t)LauncherCommands::reset_cmd;
	launcher_cmd_pub.publish(launcher_cmd_msg);

	base_cmd_msg.data = (uint16_t)BaseCommands::reset_cmd;
	base_cmd_pub.publish(base_cmd_msg);



	if(this->_op_mode == OpMode::tz1_op ||
			this->_op_mode == OpMode::tz2_op ||
			this->_op_mode == OpMode::tz3_op)
	{
		base_cmd_msg.data = (uint16_t)BaseCommands::operational_cmd;
		base_cmd_pub.publish(base_cmd_msg);
	}

	//this->disarm();

	this->clear_flags();
}

void TrMain::set_pose(geometry_msgs::Pose pose)
{
	geometry_msgs::PoseWithCovarianceStamped initialpose_msg;
	// assuming that the robot is at start position
	initialpose_msg.header.frame_id = "map";
	initialpose_msg.header.stamp = ros::Time::now();
	initialpose_msg.pose.pose = pose;
	initialpose_msg.pose.covariance =
	{
		0.025, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.025, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.006853891945200942
	};

	this->initialpose_pub.publish(initialpose_msg);
}

void TrMain::publish_path(nav_msgs::Path path)
{
	this->set_pose(path.poses.at(0).pose);

	this->target_pub.publish(path);
}

void TrMain::publish_path(geometry_msgs::Pose from, geometry_msgs::Pose to)
{
	nav_msgs::Path path_msg;
	path_msg.poses.clear();

	geometry_msgs::PoseStamped _pose;

	path_msg.header.frame_id = "map";
	path_msg.header.stamp = ros::Time::now();

	_pose.header.frame_id = "map";
	_pose.header.stamp = ros::Time::now();
	_pose.pose = from;
	path_msg.poses.push_back(_pose);

	_pose.pose = to;
	path_msg.poses.push_back(_pose);

	this->publish_path(path_msg);
}

void TrMain::publish_path(geometry_msgs::Pose from, geometry_msgs::Pose via, geometry_msgs::Pose to)
{
	nav_msgs::Path path_msg;
	path_msg.poses.clear();

	geometry_msgs::PoseStamped _pose;

	path_msg.header.frame_id = "map";
	path_msg.header.stamp = ros::Time::now();

	_pose.header.frame_id = "map";
	_pose.header.stamp = ros::Time::now();
	_pose.pose = from;
	path_msg.poses.push_back(_pose);

	_pose.pose = via;
	path_msg.poses.push_back(_pose);

	_pose.pose = to;
	path_msg.poses.push_back(_pose);

	this->publish_path(path_msg);
}

void TrMain::control_timer_callback(const ros::TimerEvent& event)
{
	if(this->command_list->size() <= this->currentCommandIndex)
	{
		this->shutdown();

		return;
	}

	if(this->currentCommandIndex == -1)
	{
		this->currentCommandIndex = 0;
	}

	ControllerCommands currentCommand = this->command_list->at(this->currentCommandIndex);


	if(currentCommand == ControllerCommands::shutdown)
	{
		this->shutdown();
	}
	else if(currentCommand == ControllerCommands::standby)
	{
		//clear_flags();

		if(!this->_has_base_restarted)
		{
			return;
		}

		if(this->_is_receiving)
		{
			if(this->_start_pressed)
			{
				//set_delay(1.25);
				set_delay(this->_receive_delay_s);

				clear_flags();
				this->_is_receiving = false;
				this->_start_pressed = false;
				this->_has_base_restarted = false;
				this->currentCommandIndex++;
				ROS_INFO("starting.");
			}
		}
		else
		{
			//clear_flags();

			this->disarm();

			set_pose(Coordinates::GetInstance()->get_tr_sz());

			this->_start_pressed = false;
			this->_is_receiving = true;

			this->sense();
			ROS_INFO("standing by.");
		}
	}
	else if(currentCommand == ControllerCommands::sz_to_tz1)
	{
		if(this->_is_moving)
		{
			if(this->_goal_reached)
			{
				this->_is_moving = false;
				this->_goal_reached = false;
				this->currentCommandIndex++;
				ROS_INFO("goal reached : dp1");
			}
		}
		else
		{
			this->publish_path(Coordinates::GetInstance()->get_tr_sz(), Coordinates::GetInstance()->get_tr_wp1(), Coordinates::GetInstance()->get_tr_tz1());

			this->_goal_reached = false;
			this->_is_moving = true;

			ROS_INFO("moving : sz -> tz1");
		}
	}
	else if(currentCommand == ControllerCommands::sz_to_dp2)
	{
		if(this->_is_moving)
		{
			if(this->_goal_reached)
			{
				this->_is_moving = false;
				this->_goal_reached = false;
				this->currentCommandIndex++;
				ROS_INFO("goal reached : dp2");
			}
		}
		else
		{
			this->publish_path(Coordinates::GetInstance()->get_tr_sz(), Coordinates::GetInstance()->get_tr_wp2_2(), Coordinates::GetInstance()->get_tr_dp2());

			this->_goal_reached = false;
			this->_is_moving = true;

			ROS_INFO("moving : sz -> dp2");
		}
	}
	else if(currentCommand == ControllerCommands::tz1_to_tz2)
	{
		if(this->_is_moving)
		{
			if(this->_goal_reached)
			{
				this->_is_moving = false;
				this->_goal_reached = false;
				this->currentCommandIndex++;
				ROS_INFO("goal reached : tz2");
			}
		}
		else
		{
			nav_msgs::Path path_msg;
			path_msg.poses.clear();

			geometry_msgs::PoseStamped _pose;

			path_msg.header.frame_id = "map";
			path_msg.header.stamp = ros::Time::now();

			_pose.header.frame_id = "map";
			_pose.header.stamp = ros::Time::now();
			_pose.pose = Coordinates::GetInstance()->get_tr_tz1();
			path_msg.poses.push_back(_pose);

			_pose.pose = Coordinates::GetInstance()->get_tr_wp2_1();
			path_msg.poses.push_back(_pose);

			_pose.pose = Coordinates::GetInstance()->get_tr_wp2_2();
			path_msg.poses.push_back(_pose);

			_pose.pose = Coordinates::GetInstance()->get_tr_tz2();
			path_msg.poses.push_back(_pose);

			this->publish_path(path_msg);

			this->_goal_reached = false;
			this->_is_moving = true;
		}
	}
	else if(currentCommand == ControllerCommands::dp2_to_tz2)
	{
		if(this->_is_moving)
		{
			if(this->_goal_reached)
			{
				this->_is_moving = false;
				this->_goal_reached = false;
				this->currentCommandIndex++;
				ROS_INFO("goal reached : tz2");
			}
		}
		else
		{
			this->publish_path(Coordinates::GetInstance()->get_tr_dp2(), Coordinates::GetInstance()->get_tr_tz2());

			this->_goal_reached = false;
			this->_is_moving = true;

			ROS_INFO("moving : dp2 -> tz2");
		}
	}
	else if(currentCommand == ControllerCommands::dp2_to_tz3)
	{
		if(this->_is_moving)
		{
			if(this->_goal_reached)
			{
				this->_is_moving = false;
				this->_goal_reached = false;
				this->currentCommandIndex++;
				ROS_INFO("goal reached : tz3");
			}
		}
		else
		{
			this->publish_path(Coordinates::GetInstance()->get_tr_dp2(), Coordinates::GetInstance()->get_tr_wp3_1(), Coordinates::GetInstance()->get_tr_tz3());

			this->_goal_reached = false;
			this->_is_moving = true;

			ROS_INFO("moving : dp2 -> tz3");
		}
	}
	else if(currentCommand == ControllerCommands::tz2_to_dp2)
	{
		if(this->_is_moving)
		{
			if(this->_goal_reached)
			{
				this->_is_moving = false;
				this->_goal_reached = false;
				this->currentCommandIndex++;
				ROS_INFO("goal reached : dp2");
			}
		}
		else
		{
			this->publish_path(Coordinates::GetInstance()->get_tr_tz2(), Coordinates::GetInstance()->get_tr_dp2());

			this->_goal_reached = false;
			this->_is_moving = true;

			ROS_INFO("moving : tz2 -> dp2");
		}
	}
	else if(currentCommand == ControllerCommands::tz3_to_dp2)
	{
		if(this->_is_moving)
		{
			if(this->_goal_reached)
			{
				this->_is_moving = false;
				this->_goal_reached = false;
				this->currentCommandIndex++;
				ROS_INFO("goal reached : dp2");
			}
		}
		else
		{
			this->publish_path(Coordinates::GetInstance()->get_tr_tz3(), Coordinates::GetInstance()->get_tr_wp3_2(), Coordinates::GetInstance()->get_tr_dp2());

			this->_goal_reached = false;
			this->_is_moving = true;

			ROS_INFO("moving : tz3 -> dp2");
		}
	}
	else if(currentCommand == ControllerCommands::dp_receive)
	{
		if(this->_is_receiving)
		{
			if(this->_shuttle_received)
			{
				//set_delay(1.25);
				set_delay(this->_receive_delay_s);

				this->_is_receiving = false;
				this->_shuttle_received = false;
				this->currentCommandIndex++;
				ROS_INFO("shuttle received.");
			}
		}
		else
		{
			this->disarm();
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

				set_delay(0.7);

				this->_is_throwing = false;
				this->_launch_completed = false;
				this->currentCommandIndex++;
				ROS_INFO("launch completed.");
			}
		}
		else
		{
			if(this->_armed)
			{
				this->launch();
				this->_is_throwing = true;

				ROS_INFO("launching.");
			}
		}
	}
	else if(currentCommand == ControllerCommands::set_tz1)
	{
		if(this->_armed)
		{
			this->set_thres(this->launcher_thresholds[0]);
			this->currentCommandIndex++;
		}
	}
	else if(currentCommand == ControllerCommands::set_tz2)
	{
		if(this->_armed)
		{
			this->set_thres(this->launcher_thresholds[1]);
			this->currentCommandIndex++;
		}
	}
	else if(currentCommand == ControllerCommands::set_tz3)
	{
		if(this->_armed)
		{
			this->set_thres(this->launcher_thresholds[2]);
			this->currentCommandIndex++;
		}
	}
	else if(currentCommand == ControllerCommands::set_delay_250ms)
	{
		set_delay(0.250);
		this->currentCommandIndex++;
	}
	else if(currentCommand == ControllerCommands::set_delay_500ms)
	{
		set_delay(0.500);
		this->currentCommandIndex++;
	}
	else if(currentCommand == ControllerCommands::set_delay_1s)
	{
		set_delay(1.000);
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
	else if(currentCommand == ControllerCommands::segno)
	{
		this->currentCommandIndex++;
	}
	else if(currentCommand == ControllerCommands::dal_segno)
	{
		auto segno_iter = std::find(this->command_list->begin(), this->command_list->end(), ControllerCommands::segno);
		if(segno_iter == this->command_list->end())
		{
			// abort on error
			this->shutdown();
		}
		auto segno_index = std::distance(this->command_list->begin(), segno_iter);
		this->currentCommandIndex = segno_index;
	}
}


void TrMain::set_thres(uint16_t threshold)
{
	// set threshold
	launcher_cmd_msg.data = (uint16_t)LauncherCommands::set_thres_cmd | (threshold & (uint16_t)LauncherCommands::set_thres_mask);
	launcher_cmd_pub.publish(launcher_cmd_msg);
}

void TrMain::disarm(void)
{
	// disarm
	launcher_cmd_msg.data = (uint16_t)LauncherCommands::disarm_cmd;
	launcher_cmd_pub.publish(launcher_cmd_msg);
}

void TrMain::receive_shuttle(void)
{
	// arm
	launcher_cmd_msg.data = (uint16_t)LauncherCommands::receive_cmd;
	launcher_cmd_pub.publish(launcher_cmd_msg);
}

void TrMain::launch(void)
{
	// launch
	launcher_cmd_msg.data = (uint16_t)LauncherCommands::launch_cmd;
	launcher_cmd_pub.publish(launcher_cmd_msg);
}

void TrMain::sense(void)
{
	// sense
	launcher_cmd_msg.data = (uint16_t)LauncherCommands::sense_cmd;
	launcher_cmd_pub.publish(launcher_cmd_msg);
}

void TrMain::set_delay(double delay_s)
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

	TrMain *instance = new TrMain();
	ROS_INFO("TR launcher unit test node has started.");

	ros::spin();
	ROS_INFO("TR launcher unit test node has been terminated.");
}



