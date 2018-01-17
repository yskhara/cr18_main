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


class TrLauncherUnitTest
{
public:
	TrLauncherUnitTest(void);

private:
	void unchuckThresholdCallback(const std_msgs::UInt16::ConstPtr& msg);
	void launchCompletedCallback(const std_msgs::Bool::ConstPtr& msg);
	void shutdownCallback(const std_msgs::Bool::ConstPtr& msg);

	ros::NodeHandle nh_;

	ros::Subscriber launch_completed_sub;
	ros::Subscriber shutdown_sub;
	ros::Subscriber unchuck_thres_sub;

	ros::Publisher launcher_cmd_pub;
	ros::Publisher launcher_unchuck_thres_pub;

	std_msgs::UInt16 launcher_cmd_msg;
	std_msgs::UInt16 launcher_unchuck_thres_msg;

	int launcher_threshold = 400;

	int currentCommandIndex = 0;

	static constexpr uint16_t null_cmd			= 0x0000;
	static constexpr uint16_t disarm_cmd		= 0x0001;
	static constexpr uint16_t arm_cmd			= 0x0002;
	static constexpr uint16_t launch_cmd		= 0x0004;

	void disarm(void);
	void arm(void);
	void launch(void);
};

TrLauncherUnitTest::TrLauncherUnitTest(void)
{
	this->launch_completed_sub = nh_.subscribe<std_msgs::Bool>("launcher/launch_completed", 10, &TrLauncherUnitTest::launchCompletedCallback, this);
	this->shutdown_sub = nh_.subscribe<std_msgs::Bool>("shutdown", 10, &TrLauncherUnitTest::shutdownCallback, this);
	this->unchuck_thres_sub = nh_.subscribe<std_msgs::UInt16>("unchuck_thres", 10, &TrLauncherUnitTest::unchuckThresholdCallback, this);


	this->launcher_cmd_pub = nh_.advertise<std_msgs::UInt16>("launcher/cmd", 1);
	this->launcher_unchuck_thres_pub = nh_.advertise<std_msgs::UInt16>("launcher/unchuck_thres", 1);

	auto nh_priv = ros::NodeHandle("~");
}

void TrLauncherUnitTest::unchuckThresholdCallback(const std_msgs::UInt16::ConstPtr& msg)
{
	this->launcher_threshold = static_cast<int>(msg->data);

	ROS_INFO("unchuck threshold was set: %d", this->launcher_threshold);
}

void TrLauncherUnitTest::launchCompletedCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if(msg->data)
	{
		ros::Duration(0.5).sleep(); // sleep for half a second

		this->disarm();
	}
}

void TrLauncherUnitTest::shutdownCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if(msg->data)
	{
		if(this->currentCommandIndex != 0)
		{
			ROS_INFO("aborting.");
		}
		this->currentCommandIndex = 0;

		this->disarm();
	}
	else if(this->currentCommandIndex == 0)
	{
		this->launcher_unchuck_thres_msg.data = this->launcher_threshold;
		this->launcher_unchuck_thres_pub.publish(this->launcher_unchuck_thres_msg);

		this->disarm();

		ros::Duration(2.0).sleep();
		this->arm();

		ros::Duration(3.0).sleep(); // sleep for half a second
		this->launch();

		this->currentCommandIndex = -1;
		ROS_INFO("launching.");
	}
}

void TrLauncherUnitTest::disarm(void)
{
	// disarm
	launcher_cmd_msg.data = disarm_cmd;
	launcher_cmd_pub.publish(launcher_cmd_msg);
}

void TrLauncherUnitTest::arm(void)
{
	// arm
	launcher_cmd_msg.data = arm_cmd;
	launcher_cmd_pub.publish(launcher_cmd_msg);
}

void TrLauncherUnitTest::launch(void)
{
	// launch
	launcher_cmd_msg.data = launch_cmd;
	launcher_cmd_pub.publish(launcher_cmd_msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tr_main");

	TrLauncherUnitTest *instance = new TrLauncherUnitTest();
	ROS_INFO("TR launcher unit test node has started.");

	ros::spin();
	ROS_INFO("TR launcher unit test node has been terminated.");
}



