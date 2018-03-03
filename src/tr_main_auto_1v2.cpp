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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
//#include <tf/transform_broadcaster.h>
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
	ros::Publisher set_pose_pub;

	std_msgs::UInt16 launcher_cmd_msg;
	std_msgs::UInt16 launcher_unchuck_thres_msg;

	nav_msgs::Path target_msg;

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

	this->set_pose_pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/nav/set_pose", 1);

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

	ROS_INFO("goal reached.");

	if(this->currentCommandIndex == 1)
	{
		//ros::Duration(2.0).sleep(); // sleep for half a second

		//this->disarm();

		this->currentCommandIndex = 2;

		abort_msg.data = true;
		this->abort_pub.publish(abort_msg);

		ROS_INFO("done: 2");
	}
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
	else if(this->currentCommandIndex == 0)
	{
		geometry_msgs::PoseWithCovarianceStamped _set_pose;
		_set_pose.header.frame_id = "odom";
		_set_pose.header.stamp = ros::Time::now();
		this->set_pose_pub.publish(_set_pose);

		ros::Duration(0.5).sleep(); // sleep for half a second

		//this->move(0.53, -4.0, 0);

		this->target_msg.poses.clear();

		geometry_msgs::PoseStamped _pose;

		this->target_msg.header.frame_id = "map";
		this->target_msg.header.stamp = ros::Time::now();

		_pose.header.frame_id = "map";
		_pose.header.stamp = ros::Time::now();
		_pose.pose.position.x = 0.0;
		_pose.pose.position.y = 0.0;
		_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
		this->target_msg.poses.push_back(_pose);

		_pose.pose.position.x = 2.0;
		_pose.pose.position.y = 0.0;
		_pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI / 2.0);
		this->target_msg.poses.push_back(_pose);

		this->target_pub.publish(target_msg);

		this->currentCommandIndex = 1;
		ROS_INFO("moving: 1");
	}
}

void TrMain::move(double x, double y, double theta)
{
	this->target_msg.poses.clear();

	geometry_msgs::PoseStamped _pose;

	_pose.header.frame_id = "map";


	_pose.header.stamp = ros::Time::now();
	_pose.pose.position.x = x;
	_pose.pose.position.y = y;
	_pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

	this->target_msg.header.frame_id = "map";
	this->target_msg.header.stamp = ros::Time::now();
	this->target_msg.poses.push_back(_pose);

	this->target_pub.publish(target_msg);
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



