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
#include <nav_msgs/Path.h>
#include <vector>
#include <string>

class Command
{

};

class TrMain
{
public:
	TrMain(void);

private:
	void goalReachedCallback(const std_msgs::Bool::ConstPtr& msg);
	void launchCompletedCallback(const std_msgs::Bool::ConstPtr& msg);
	void shutdownCallback(const std_msgs::Bool::ConstPtr& msg);

	ros::NodeHandle nh_;

	//int linear_, angular_;
	ros::Subscriber goal_reached_sub;
	ros::Subscriber launch_completed_sub;
	ros::Subscriber shutdown_sub;

	ros::Publisher launcher_cmd_pub;
	ros::Publisher launcher_unchuck_thres_pub;
	ros::Publisher target_pub;
	ros::Publisher abort_pub;

	std_msgs::UInt16 launcher_cmd_msg;
	std_msgs::UInt16 launcher_unchuck_thres_msg;

	geometry_msgs::Pose2D target_msg;

	std_msgs::Bool abort_msg;

	std::vector<int> launcher_thresholds;

	int currentCommandIndex = 0;

	int tr;
	bool move_base;

	static constexpr uint16_t null_cmd			= 0x0000;
	static constexpr uint16_t disarm_cmd		= 0x0001;
	static constexpr uint16_t arm_cmd			= 0x0002;
	static constexpr uint16_t launch_cmd		= 0x0004;

	void move(double x, double y, double theta);

	void disarm(void);
	void arm(void);
	void launch(void);
};

TrMain::TrMain(void)
{
	goal_reached_sub = nh_.subscribe<std_msgs::Bool>("goal_reached", 10, &TrMain::goalReachedCallback, this);
	launch_completed_sub = nh_.subscribe<std_msgs::Bool>("launcher/launch_completed", 10, &TrMain::launchCompletedCallback, this);
	shutdown_sub = nh_.subscribe<std_msgs::Bool>("shutdown", 10, &TrMain::shutdownCallback, this);

	target_pub = nh_.advertise<nav_msgs::Path>("nav/target_path", 1);
	this->abort_pub = nh_.advertise<std_msgs::Bool>("nav/abort", 1);

	this->launcher_cmd_pub = nh_.advertise<std_msgs::UInt16>("launcher/cmd", 1);
	this->launcher_unchuck_thres_pub = nh_.advertise<std_msgs::UInt16>("launcher/unchuck_thres", 1);

	auto nh_priv = ros::NodeHandle("~");

	this->tr = 1;
	int tr_tmp;
	nh_priv.getParam("tr", tr_tmp);
	if(tr_tmp == 1 || tr_tmp == 2 || tr_tmp == 3)
	{
		this->tr = tr_tmp;
	}

	this->launcher_thresholds = {0x0130, 0x0130, 0x0090};
	std::vector<int> tmp;
	nh_priv.getParam("unchuck_thres", tmp);
	if(tmp.size() == 3)
	{
		this->launcher_thresholds = tmp;
	}
	ROS_INFO("thresholds: %d, %d, %d", this->launcher_thresholds[0], this->launcher_thresholds[1], this->launcher_thresholds[2]);
}

void TrMain::goalReachedCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if(!msg->data)
	{
		// not reached yet
		return;
	}

	if(this->currentCommandIndex == 1)
	{
		//ros::Duration(0.5).sleep(); // sleep for half a second

		this->launcher_unchuck_thres_msg.data = this->launcher_thresholds[(this->tr - 1)];
		this->launcher_unchuck_thres_pub.publish(this->launcher_unchuck_thres_msg);

		if(tr == 1)
		{
			this->move(1.2, -0.2, M_PI/2);
		}
		else if(tr == 2)
		{
			this->move(1.2, 0, M_PI/2);
		}
		else if(tr == 3)
		{
			this->move((1.2 + 3.27), 0, M_PI/2);
		}

		this->currentCommandIndex = 2;
		ROS_INFO("moving: 2");
	}
	else if(this->currentCommandIndex == 2)
	{
		abort_msg.data = true;
		this->abort_pub.publish(abort_msg);

		ros::Duration(1.5).sleep(); // sleep for half a second

		this->launch();

		//this->move(2.7, -3.0, M_PI/2);

		this->currentCommandIndex = 3;
		ROS_INFO("moving: 3; launching.");
	}
	/*
	else if(this->currentCommandIndex == 3)
	{
		ros::Duration(0.5).sleep(); // sleep for half a second

		this->launch();

		this->currentCommandIndex = 4;
		ROS_INFO("moving: 4; launching.");
	}
	*/
}

void TrMain::launchCompletedCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if(msg->data)
	{
		ros::Duration(0.5).sleep(); // sleep for half a second

		this->disarm();
	}
}

void TrMain::shutdownCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if(msg->data)
	{
		if(this->currentCommandIndex != 0)
		{
			ROS_INFO("aborting.");
		}
		this->currentCommandIndex = 0;

		this->disarm();

		abort_msg.data = true;
		this->abort_pub.publish(abort_msg);
	}
#if 0
	else if(this->currentCommandIndex == 0)
	{
		this->launcher_unchuck_thres_msg.data = this->launcher_thresholds[(this->tr - 1)];
		this->launcher_unchuck_thres_pub.publish(this->launcher_unchuck_thres_msg);

		//ros::Duration(0.1).sleep(); // sleep for half a second
		this->disarm();

		ros::Duration(2.0).sleep();
		this->arm();

		ros::Duration(5.0).sleep(); // sleep for half a second
		this->launch();

		ros::Duration(1.0).sleep();
		//this->move(2.7, -3.0, M_PI/2);

		std_msgs::Bool _abort_msg;
		_abort_msg.data = true;
		this->abort_pub.publish(_abort_msg);

		this->currentCommandIndex = 3;
		ROS_INFO("launching.");
	}
#else
	else if(this->currentCommandIndex == 0)
	{
		ros::Duration(0.5).sleep(); // sleep for half a second
		this->disarm();

		ros::Duration(1.5).sleep();
		this->arm();

		ros::Duration(0.5).sleep(); // sleep for half a second

		this->move(0.5, 0, 0);

		this->currentCommandIndex = 1;
		ROS_INFO("moving: 1");
	}
#endif
}

void TrMain::move(double x, double y, double theta)
{
	geometry_msgs::Pose2D _target_msg;

	_target_msg.x = x;
	_target_msg.y = y;
	_target_msg.theta = theta;

	this->target_pub.publish(_target_msg);
}

void TrMain::disarm(void)
{
	// disarm
	launcher_cmd_msg.data = disarm_cmd;
	launcher_cmd_pub.publish(launcher_cmd_msg);
}

void TrMain::arm(void)
{
	// arm
	launcher_cmd_msg.data = arm_cmd;
	launcher_cmd_pub.publish(launcher_cmd_msg);
}

void TrMain::launch(void)
{
	// launch
	launcher_cmd_msg.data = launch_cmd;
	launcher_cmd_pub.publish(launcher_cmd_msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tr_main");

	TrMain *trMain = new TrMain();
	ROS_INFO("tr_main node has started.");

	ros::spin();
	ROS_INFO("tr_main node has been terminated.");
}



