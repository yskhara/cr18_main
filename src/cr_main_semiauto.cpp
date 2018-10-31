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
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
//#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>
#include <vector>
#include <string>

#include "RedCoordinates.hpp"

enum class OpMode
    : uint8_t
    {
        route1_pp1_op = 0b000,        // sz -> pp1 -> pp2 -> pp3 -> pp4
    route1_pp2_op = 0b001,        // sz -> pp2 -> pp3 -> pp4
    route1_pp3_op = 0b010,        // sz -> pp3 -> pp4
    route1_pp4_op = 0b011,        // sz -> pp4

    route2_pp1_op = 0b100,        // sz -> pp1 -> pp2 -> pp3 -> pp4
    route2_pp2_op = 0b101,        // sz -> pp2 -> pp3 -> pp4
    route2_pp3_op = 0b110,        // sz -> pp3 -> pp4
    route2_pp4_op = 0b111,        // sz -> pp4
};

enum class ControllerStatus
    : uint16_t
    {
        shutdown = 0x0000,
    reset = 0x0001,

    standby = 0x0010,				// standing by at start zone
    moving,							// moving around
    motion_cplt,					// motion completed; transitioning

    pp_pickingup,					// picking up shuttles at a PP
    dp_delivering,				// delivering a shuttle at a DP
};

enum class ControllerCommands
    : uint16_t
    {
        shutdown,						// shutdown

    standby,						// stand-by at SZ

    pp_pickup,						// pick up shuttles at a PP
    dp_deliver,					    // deliver shuttles at a DP

    move_to_pp1,                    // move to PP1
    move_to_pp2,                    // move to PP2
    move_to_pp3,                    // move to PP3
    move_to_pp4,                    // move to PP4
    move_to_pp5,                    // move to PP4

    move_to_dp1,					// move to DP1
    move_to_dp2,                    // move to DP2
    move_to_dp3,                    // move to DP3
    move_to_dp4,                    // move to DP4

    rtb,                            // return to base

    set_lift_p,                     // set chuck height for pick-up
    set_lift_1,                     // set chuck height to ground floor
    set_lift_2,                     // set chuck height to 2nd floor
    set_lift_3,                     // set chuck height to 3rd floor

    //disarm,
    delay,

    segno,
    dal_segno,

    checkpoint_pp1,
    checkpoint_pp2,
    checkpoint_pp3,
    checkpoint_pp4,
};

class CrMain
{
public:
    CrMain(void);

private:
    //void baseStatusCallback(const std_msgs::UInt16::ConstPtr& msg);
    //void handStatusCallback(const std_msgs::UInt16::ConstPtr& msg);
    //void shutdownCallback(const std_msgs::Bool::ConstPtr& msg);

    void shutdownInputCallback(const std_msgs::Bool::ConstPtr& msg);
    void startInputCallback(const std_msgs::Bool::ConstPtr& msg);
    //void baseConfCallback(const std_msgs::UInt8::ConstPtr& msg);

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    void goalReachedCallback(const std_msgs::Bool::ConstPtr& msg);

    void control_timer_callback(const ros::TimerEvent& event);

    ros::NodeHandle nh_;

    ros::Subscriber goal_reached_sub;

    ros::Subscriber joy_sub;

    ros::Publisher target_pub;
    ros::Publisher abort_pub;

    //nav_msgs::Path target_msg;
    std_msgs::Bool abort_msg;
    tf::TransformListener _tflistener;

    ros::Publisher initialpose_pub;
    //geometry_msgs::PoseWithCovarianceStamped initialpose_msg;

    ros::Subscriber shutdown_input_sub;
    ros::Subscriber start_input_sub;

    ros::Publisher lift_position_pub;
    ros::Publisher hand_cylinder_pub;
    ros::Publisher act_enable_pub;

    std_msgs::Int32 lift_position_msg;
    std_msgs::Bool hand_cylinder_msg;
    std_msgs::Bool act_enable_msg;

    double steps_per_mm = 16 * 200 * 3 / 40;

    std::vector<int> lift_position = { 0, 0, 0, 0, 0 };
    int lift_position_index = 0;
    //      {0, -40 * steps_per_mm;
    //static constexpr int lift_position_first = -40 * steps_per_mm;
    //static constexpr int lift_position_second = lift_position_first - (248 * steps_per_mm);
    //static constexpr int lift_position_third = lift_position_second - (248 * steps_per_mm);

    OpMode _op_mode = OpMode::route1_pp1_op;

    int currentCommandIndex = -1;
    const std::vector<ControllerCommands> *command_list;
    static const std::vector<ControllerCommands> route1_pp1_op_commands;
    static const std::vector<ControllerCommands> route1_pp2_op_commands;
    static const std::vector<ControllerCommands> route1_pp3_op_commands;
    static const std::vector<ControllerCommands> route1_pp4_op_commands;
    static const std::vector<ControllerCommands> route2_pp1_op_commands;
    static const std::vector<ControllerCommands> route2_pp2_op_commands;
    static const std::vector<ControllerCommands> route2_pp3_op_commands;
    static const std::vector<ControllerCommands> route2_pp4_op_commands;
    static const std::vector<ControllerCommands> default_commands;

    //std::vector<ControllerCommands> command_list;
    ControllerStatus _status;

    double _amt_coeff;
    double _target_x = 0.0;
    double _target_y = 0.0;
    double _target_yaw = M_PI / 2.0;
    bool _rush = false;

    //double _receive_delay_s;

    double _delay_s = 0.0;

    ros::Timer control_timer;

    void shutdown(void);
    //void restart(void);

    void amt(void);
    void chuck(void);
    void unchuck(void);
    void move_lift(int index);
    void enable_actuators(void);
    //void disable_actuators(void);
    //void arm_ready(void);

    void set_pose(const geometry_msgs::Pose &pose);
    void publish_path(const nav_msgs::Path &path);
    void publish_path_to(const geometry_msgs::Pose &to);
    void publish_path_to(const geometry_msgs::Pose &to, const geometry_msgs::Pose &wp1, const geometry_msgs::Pose &wp2);
    void publish_path_to_relative(const double dx, const double dy);

    void set_delay(double delay_s);

    // flags
    //bool _start_pressed = false;
    bool _next_pressed = false;
    bool _abort_pressed = false;
    //bool _is_moving = false;
    //bool _is_throwing = false;
    //bool _is_receiving = false;
    bool _is_manual_enabled = false;
    bool _goal_reached = false;
    bool _has_base_restarted = false;

    inline void clear_flags(void)
    {
        //_start_pressed = false;
        _next_pressed = false;
        _abort_pressed = false;
        //_is_moving = false;
        //_is_throwing = false;
        //_is_receiving = false;
        _is_manual_enabled = false;
        _goal_reached = false;
        _has_base_restarted = false;
    }

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
    static int AxisLeftThumbX;
    static int AxisLeftThumbY;
    static int AxisRightThumbX;
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
int CrMain::AxisLeftThumbX = 0;
int CrMain::AxisLeftThumbY = 1;
int CrMain::AxisRightThumbX = 2;

#include "ControllerCommands.hpp"

CrMain::CrMain(void)
{
    //this->hand_status_sub = nh_.subscribe<std_msgs::UInt16>("hand/status", 10, &CrMain::handStatusCallback, this);
    //this->base_status_sub = nh_.subscribe<std_msgs::UInt16>("base/status", 10, &CrMain::baseStatusCallback, this);
    joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &CrMain::joyCallback, this);

    shutdown_input_sub = nh_.subscribe<std_msgs::Bool>("shutdown_input", 10, &CrMain::shutdownInputCallback, this);
    start_input_sub = nh_.subscribe<std_msgs::Bool>("start_input", 10, &CrMain::startInputCallback, this);

    this->lift_position_pub = nh_.advertise<std_msgs::Int32>("lift_position", 1);
    this->hand_cylinder_pub = nh_.advertise<std_msgs::Bool>("hand_cylinder", 1);
    this->act_enable_pub = nh_.advertise<std_msgs::Bool>("act_enable", 1);

    //this->hand_cmd_pub = nh_.advertise<std_msgs::UInt16>("hand/cmd", 10);
    //this->base_cmd_pub = nh_.advertise<std_msgs::UInt16>("base/cmd", 10);

    this->goal_reached_sub = nh_.subscribe<std_msgs::Bool>("goal_reached", 10, &CrMain::goalReachedCallback, this);
    this->target_pub = nh_.advertise<nav_msgs::Path>("target_path", 10);
    this->abort_pub = nh_.advertise<std_msgs::Bool>("abort", 10);

    this->initialpose_pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);

    auto private_nh = ros::NodeHandle("~");
    private_nh.param("amt_coeff", this->_amt_coeff, 0.25);

    this->lift_position =
    {   0, 51, 51 + 248, 51 + (2 * 248)};
    std::vector<int> tmp;
    private_nh.getParam("lift_position", tmp);
    if (tmp.size() == 4)
    {
        this->lift_position = tmp;
    }
    for (int& pos : this->lift_position)
    {
        pos *= (-steps_per_mm);
    }
    ROS_INFO("thresholds: %d, %d, %d, %d", this->lift_position[0], this->lift_position[1], this->lift_position[2],
            this->lift_position[3]);

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
    nh_.getParam("AxisLeftThumbX", AxisLeftThumbX);
    nh_.getParam("AxisLeftThumbY", AxisLeftThumbY);
    nh_.getParam("AxisRightThumbX", AxisRightThumbX);

    //command_list.reserve(64);
    command_list = &CrMain::route1_pp1_op_commands;
    ROS_INFO("operation mode set to route1_pp1_op. command size: %ld", this->command_list->size());

    this->_status = ControllerStatus::shutdown;

    // timer starts immediately
    control_timer = nh_.createTimer(ros::Duration(0.1), &CrMain::control_timer_callback, this);
}

#if 0
void CrMain::baseStatusCallback(const std_msgs::UInt16::ConstPtr& msg)
{
    BaseStatus status = (BaseStatus) msg->data;

    switch (status)
    {
        case BaseStatus::shutdown:
        if (this->base_last_status != BaseStatus::shutdown)
        {
            this->shutdown();
        }
        break;

        case BaseStatus::reset:
        if (this->base_last_status == BaseStatus::shutdown)
        {
            //this->restart();
            this->_status = CRControllerStatus::reset;
            this->currentCommandIndex = 0;
            ROS_INFO("restarting.");

            this->_has_base_restarted = true;
        }
        break;

        default:
        break;
    }

    base_last_status = status;
    base_last_status_time = ros::Time::now();
}

void CrMain::handStatusCallback(const std_msgs::UInt16::ConstPtr& msg)
{
    CarrierStatus status = (CarrierStatus) msg->data;

    switch (status)
    {
        case CarrierStatus::shutdown:
        break;

        case CarrierStatus::reset:
        break;
        default:
        break;
    }

    hand_last_status = status;
    hand_last_status_time = ros::Time::now();
}
#endif

void CrMain::shutdownInputCallback(const std_msgs::Bool::ConstPtr& msg)
{
    /*
    if (this->_status != ControllerStatus::shutdown)
    {
        this->_status = ControllerStatus::shutdown;
        this->currentCommandIndex = -1;

        ROS_INFO("Aborting.");
    }*/
    if(!msg->data)
    {
        return;
    }

    ROS_INFO("base reported a shutdown input.");
    shutdown();

    // reset this:
    // this->CurrentCommandIndex = -1;
    act_enable_msg.data = false;
    act_enable_pub.publish(act_enable_msg);

    lift_position_index = 0;
}

void CrMain::startInputCallback(const std_msgs::Bool::ConstPtr& msg)
{
    // bring the robot back operational

    if(!msg->data)
    {
        return;
    }

    if (this->_status == ControllerStatus::shutdown)
    {
        act_enable_msg.data = true;
        act_enable_pub.publish(act_enable_msg);

        this->_status = ControllerStatus::reset;
        this->currentCommandIndex = 0;
        ROS_INFO("Restarting on start signal.");

        this->_has_base_restarted = true;
    }
    else
    {
        ROS_WARN("An illegal start command received while the controller is not in shutdown state. Aborting.");

        this->_status = ControllerStatus::shutdown;
        this->currentCommandIndex = -1;
    }
}

void CrMain::goalReachedCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (!msg->data)
    {
        // not reached yet
        return;
    }

    //ROS_INFO("goal reached.");

    abort_msg.data = true;
    this->abort_pub.publish(abort_msg);

    this->_goal_reached = true;
}

void CrMain::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    static bool last_a = false;
    static bool last_b = false;
    static bool last_x = false;
    static bool last_y = false;

    bool _a = joy->buttons[ButtonA];
    bool _b = joy->buttons[ButtonB];
    bool _x = joy->buttons[ButtonX];
    bool _y = joy->buttons[ButtonY];

    bool _start = joy->buttons[ButtonStart];

    if (_start)
    {
        ROS_INFO("shutdown command from controller.");
        this->shutdown();
    }

    if (_a && !last_a)
    {
        this->_next_pressed = true;
    }
    else if (_b && !last_b)
    {
        this->_abort_pressed = true;
    }
    else if (_x && !last_x)
    {

    }
    else if (_y && !last_y)
    {

    }

    if(this->_status == ControllerStatus::shutdown)
    {
        bool rb = joy->buttons[ButtonRB];
        bool lb = joy->buttons[ButtonLB];
        int dx = -joy->axes[AxisDPadX];
        int dy = joy->axes[AxisDPadY];

        if(lb && !rb)
        {
            // route 1
            if(dx == 0 && dy > 0)
            {
                // from pp1
                command_list = &CrMain::route1_pp1_op_commands;
                ROS_INFO("operation mode set to route1_pp1_op. command size: %ld", this->command_list->size());
            }
            else if(dx > 0 && dy == 0)
            {
                // from pp2
                command_list = &CrMain::route1_pp2_op_commands;
                ROS_INFO("operation mode set to route1_pp2_op. command size: %ld", this->command_list->size());
            }
            else if(dx == 0 && dy < 0)
            {
                // from pp3
                command_list = &CrMain::route1_pp3_op_commands;
                ROS_INFO("operation mode set to route1_pp3_op. command size: %ld", this->command_list->size());
            }
            else if(dx < 0 && dy == 0)
            {
                // from pp4
                command_list = &CrMain::route1_pp4_op_commands;
                ROS_INFO("operation mode set to route1_pp4_op. command size: %ld", this->command_list->size());
            }
        }
        else if(rb && !lb)
        {
            // route 2
            if(dx == 0 && dy > 0)
            {
                // from pp1
                command_list = &CrMain::route2_pp1_op_commands;
                ROS_INFO("operation mode set to route2_pp1_op. command size: %ld", this->command_list->size());
            }
            else if(dx > 0 && dy == 0)
            {
                // from pp2
                command_list = &CrMain::route2_pp2_op_commands;
                ROS_INFO("operation mode set to route2_pp2_op. command size: %ld", this->command_list->size());
            }
            else if(dx == 0 && dy < 0)
            {
                // from pp3
                command_list = &CrMain::route2_pp3_op_commands;
                ROS_INFO("operation mode set to route2_pp3_op. command size: %ld", this->command_list->size());
            }
            else if(dx < 0 && dy == 0)
            {
                // from pp4
                command_list = &CrMain::route2_pp4_op_commands;
                ROS_INFO("operation mode set to route2_pp4_op. command size: %ld", this->command_list->size());
            }
        }
    }

    last_a = _a;
    last_b = _b;
    last_x = _x;
    last_y = _y;

    if (this->_is_manual_enabled)
    {
        //swap x and y
        this->_target_x = joy->axes[AxisLeftThumbY];
        this->_target_y = joy->axes[AxisLeftThumbX];
        this->_rush = false;//(joy->buttons[ButtonRB] != 0);
    }
    else
    {
        this->_target_x = 0.0;
        this->_target_y = 0.0;
        //this->_target_yaw = 0.0;
        this->_rush = false;
    }
}

void CrMain::shutdown(void)
{
    if (this->currentCommandIndex != -1)
    {
        ROS_INFO("Aborting.");

        this->currentCommandIndex = -1;

        //this->unchuck_all();
    }

    abort_msg.data = true;
    this->abort_pub.publish(abort_msg);

    act_enable_msg.data = false;
    act_enable_pub.publish(act_enable_msg);

    clear_flags();

    this->_status = ControllerStatus::shutdown;
}

void CrMain::enable_actuators(void)
{
    act_enable_msg.data = true;
    act_enable_pub.publish(act_enable_msg);

    //this->unchuck_all();

    //this->clear_flags();
}

void CrMain::set_pose(const geometry_msgs::Pose &pose)
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

void CrMain::publish_path(const nav_msgs::Path &path)
{
    //this->set_pose(path.poses.at(0).pose);

    this->target_pub.publish(path);

    this->_target_yaw = tf::getYaw(path.poses.back().pose.orientation);
}

void CrMain::publish_path_to(const geometry_msgs::Pose &to)
{
    nav_msgs::Path path_msg;
    path_msg.poses.clear();

    geometry_msgs::PoseStamped _pose;

    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();

    _pose.header.frame_id = "map";
    _pose.header.stamp = ros::Time::now();

    tf::StampedTransform base_link;
    try
    {
        this->_tflistener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(0.1));
        this->_tflistener.lookupTransform("/map", "/base_link", ros::Time(0), base_link);
    }
    catch (...)
    {
        return;
    }
    _pose.pose.position.x = base_link.getOrigin().x();
    _pose.pose.position.y = base_link.getOrigin().y();
    // TODO: alert
    tf::quaternionTFToMsg(base_link.getRotation(), _pose.pose.orientation);
    path_msg.poses.push_back(_pose);

    _pose.pose = to;
    path_msg.poses.push_back(_pose);

    this->publish_path(path_msg);
}

void CrMain::publish_path_to(const geometry_msgs::Pose &to, const geometry_msgs::Pose &wp1,
        const geometry_msgs::Pose &wp2)
{
    nav_msgs::Path path_msg;
    path_msg.poses.clear();

    geometry_msgs::PoseStamped _pose;

    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();

    _pose.header.frame_id = "map";
    _pose.header.stamp = ros::Time::now();

    tf::StampedTransform base_link;
    try
    {
        this->_tflistener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(0.1));
        this->_tflistener.lookupTransform("/map", "/base_link", ros::Time(0), base_link);
    }
    catch (...)
    {
        return;
    }
    _pose.pose.position.x = base_link.getOrigin().x();
    _pose.pose.position.y = base_link.getOrigin().y();
    // TODO: alert
    tf::quaternionTFToMsg(base_link.getRotation(), _pose.pose.orientation);
    path_msg.poses.push_back(_pose);

    _pose.pose = wp2;
    path_msg.poses.push_back(_pose);

    _pose.pose = wp1;
    path_msg.poses.push_back(_pose);

    _pose.pose = to;
    path_msg.poses.push_back(_pose);

    this->publish_path(path_msg);
}

void CrMain::publish_path_to_relative(const double dx, const double dy)
{
    nav_msgs::Path path_msg;
    path_msg.poses.clear();

    geometry_msgs::PoseStamped _pose;

    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();

    _pose.header.frame_id = "map";
    _pose.header.stamp = ros::Time::now();

    tf::StampedTransform base_link;
    try
    {
        this->_tflistener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(0.1));
        this->_tflistener.lookupTransform("/map", "/base_link", ros::Time(0), base_link);
    }
    catch (...)
    {
        return;
    }
    _pose.pose.position.x = base_link.getOrigin().x();
    _pose.pose.position.y = base_link.getOrigin().y();
    // TODO: alert
    tf::quaternionTFToMsg(base_link.getRotation(), _pose.pose.orientation);
    path_msg.poses.push_back(_pose);

    _pose.pose.position.x = base_link.getOrigin().x() + dx;
    _pose.pose.position.y = base_link.getOrigin().y() + dy;
    // TODO: alert
    tf::quaternionTFToMsg(base_link.getRotation(), _pose.pose.orientation);
    path_msg.poses.push_back(_pose);

    this->publish_path(path_msg);
}

void CrMain::control_timer_callback(const ros::TimerEvent& event)
{
    if ((long)this->command_list->size() <= (long)this->currentCommandIndex)
    {
        ROS_INFO("shutting down on an error: current command index is invalid.");
        ROS_INFO("size of command list: %ld", this->command_list->size());
        ROS_INFO("current index: %d", this->currentCommandIndex);
        this->shutdown();

        return;
    }

    if (this->_status == ControllerStatus::shutdown)
    {
        return;
    }

    if (this->currentCommandIndex == -1)
    {
        this->currentCommandIndex = 0;
    }

    ControllerCommands currentCommand = this->command_list->at(this->currentCommandIndex);

    if (currentCommand == ControllerCommands::shutdown)
    {
        ROS_INFO("shutting down on command.");
        this->shutdown();
    }
    else if (currentCommand == ControllerCommands::standby)
    {
        if (!this->_has_base_restarted)
        {
            return;
        }

        if (this->_status == ControllerStatus::standby)
        {
            // TODO: this is INSANE
            if (this->_next_pressed)// || true)
            {
                this->enable_actuators();
                //this->restart();

                //this->unchuck_all();
                this->unchuck();

                clear_flags();
                this->_status = ControllerStatus::motion_cplt;

                this->currentCommandIndex++;
                ROS_INFO("starting.");
            }
        }
        else
        {
            //this->unchuck_all();

            ROS_INFO("setting initial position.");
            set_pose(RedCoordinates::GetInstance()->get_cr_sz());

            this->_next_pressed = false;
            //clear_flags();
            this->_status = ControllerStatus::standby;

            //this->sense();
            ROS_INFO("standing by.");
        }
    }
    else if (currentCommand == ControllerCommands::move_to_pp1)
    {
        static bool _wp = true;

        if (this->_status == ControllerStatus::moving)
        {
            if (this->_abort_pressed || this->_goal_reached)
            {
                if (_wp)
                {
                    clear_flags();
                    this->_status = ControllerStatus::motion_cplt;

                    //this->currentCommandIndex++;
                    ROS_INFO("goal reached : pp1_wp");
                    _wp = false;
                }
                else
                {
                    clear_flags();
                    this->_status = ControllerStatus::motion_cplt;

                    this->currentCommandIndex++;
                    ROS_INFO("goal reached : pp1");
                    _wp = true;
                }
            }
        }
        else
        {
            if (_wp)
            {
                this->publish_path_to(RedCoordinates::GetInstance()->get_cr_pp1_wp2());
            }
            else
            {
                this->publish_path_to(RedCoordinates::GetInstance()->get_cr_pp1(),
                        RedCoordinates::GetInstance()->get_cr_pp1_wp1(), RedCoordinates::GetInstance()->get_cr_pp1_wp2());
            }

            clear_flags();
            this->_status = ControllerStatus::moving;
        }
    }
    else if (currentCommand == ControllerCommands::move_to_pp2)
    {
        static bool _wp = true;

        if (this->_status == ControllerStatus::moving)
        {
            if (this->_abort_pressed || this->_goal_reached)
            {
                if (_wp)
                {
                    clear_flags();
                    this->_status = ControllerStatus::motion_cplt;

                    //this->currentCommandIndex++;
                    ROS_INFO("goal reached : pp2_wp");
                    _wp = false;
                }
                else
                {
                    clear_flags();
                    this->_status = ControllerStatus::motion_cplt;

                    this->currentCommandIndex++;
                    ROS_INFO("goal reached : pp2");
                    _wp = true;
                }
            }
        }
        else
        {
            if (_wp)
            {
                this->publish_path_to(RedCoordinates::GetInstance()->get_cr_pp2_wp2());
            }
            else
            {
                this->publish_path_to(RedCoordinates::GetInstance()->get_cr_pp2(),
                        RedCoordinates::GetInstance()->get_cr_pp2_wp1(), RedCoordinates::GetInstance()->get_cr_pp2_wp2());
            }

            clear_flags();
            this->_status = ControllerStatus::moving;
        }
    }
    else if (currentCommand == ControllerCommands::move_to_pp3)
    {
        static bool _wp = true;

        if (this->_status == ControllerStatus::moving)
        {
            if (this->_abort_pressed || this->_goal_reached)
            {
                if (_wp)
                {
                    clear_flags();
                    this->_status = ControllerStatus::motion_cplt;

                    //this->currentCommandIndex++;
                    ROS_INFO("goal reached : pp3_wp");
                    _wp = false;
                }
                else
                {
                    clear_flags();
                    this->_status = ControllerStatus::motion_cplt;

                    this->currentCommandIndex++;
                    ROS_INFO("goal reached : pp3");
                    _wp = true;
                }
            }
        }
        else
        {
            if (_wp)
            {
                this->publish_path_to(RedCoordinates::GetInstance()->get_cr_pp3_wp2());
            }
            else
            {
                this->publish_path_to(RedCoordinates::GetInstance()->get_cr_pp3(),
                        RedCoordinates::GetInstance()->get_cr_pp3_wp1(), RedCoordinates::GetInstance()->get_cr_pp3_wp2());
            }

            clear_flags();
            this->_status = ControllerStatus::moving;
        }
    }
    else if (currentCommand == ControllerCommands::move_to_pp4)
    {
        static bool _wp = true;

        if (this->_status == ControllerStatus::moving)
        {
            if (this->_abort_pressed || this->_goal_reached)
            {
                if (_wp)
                {
                    clear_flags();
                    this->_status = ControllerStatus::motion_cplt;

                    //this->currentCommandIndex++;
                    ROS_INFO("goal reached : pp4_wp");
                    _wp = false;
                }
                else
                {
                    clear_flags();
                    this->_status = ControllerStatus::motion_cplt;

                    this->currentCommandIndex++;
                    ROS_INFO("goal reached : pp4");
                    _wp = true;
                }
            }
        }
        else
        {
            if (_wp)
            {
                this->publish_path_to(RedCoordinates::GetInstance()->get_cr_pp4_wp2());
            }
            else
            {
                this->publish_path_to(RedCoordinates::GetInstance()->get_cr_pp4(),
                        RedCoordinates::GetInstance()->get_cr_pp4_wp1(), RedCoordinates::GetInstance()->get_cr_pp4_wp2());
            }

            clear_flags();
            this->_status = ControllerStatus::moving;
        }
    }
    else if (currentCommand == ControllerCommands::move_to_dp1)
    {
        static bool _wp = true;

        if (this->_status == ControllerStatus::moving)
        {
            if (this->_abort_pressed || this->_goal_reached)
            {
                if (_wp)
                {
                    clear_flags();
                    this->_status = ControllerStatus::motion_cplt;

                    //this->currentCommandIndex++;
                    ROS_INFO("goal reached : dp1_wp");
                    _wp = false;
                }
                else
                {
                    clear_flags();
                    this->_status = ControllerStatus::motion_cplt;

                    this->currentCommandIndex++;
                    ROS_INFO("goal reached : dp1");
                    _wp = true;
                }
            }
        }
        else
        {
            if (_wp)
            {
                this->publish_path_to(RedCoordinates::GetInstance()->get_cr_dp1_wp2());
            }
            else
            {
                this->publish_path_to(RedCoordinates::GetInstance()->get_cr_dp1(),
                        RedCoordinates::GetInstance()->get_cr_dp1_wp1(), RedCoordinates::GetInstance()->get_cr_dp1_wp2());
            }

            clear_flags();
            this->_status = ControllerStatus::moving;
        }
    }
    else if (currentCommand == ControllerCommands::move_to_dp2)
    {
        static bool _wp = true;

        if (this->_status == ControllerStatus::moving)
        {
            if (this->_abort_pressed || this->_goal_reached)
            {
                if (_wp)
                {
                    clear_flags();
                    this->_status = ControllerStatus::motion_cplt;

                    //this->currentCommandIndex++;
                    ROS_INFO("goal reached : dp2_wp");
                    _wp = false;
                }
                else
                {
                    clear_flags();
                    this->_status = ControllerStatus::motion_cplt;

                    this->currentCommandIndex++;
                    ROS_INFO("goal reached : dp2");
                    _wp = true;
                }
            }
        }
        else
        {
            if (_wp)
            {
                this->publish_path_to(RedCoordinates::GetInstance()->get_cr_dp2_wp2());
            }
            else
            {
                this->publish_path_to(RedCoordinates::GetInstance()->get_cr_dp2(),
                        RedCoordinates::GetInstance()->get_cr_dp2_wp1(), RedCoordinates::GetInstance()->get_cr_dp2_wp2());
            }

            clear_flags();
            this->_status = ControllerStatus::moving;
        }
    }
    else if (currentCommand == ControllerCommands::move_to_dp3)
    {
        static bool _wp = true;

        if (this->_status == ControllerStatus::moving)
        {
            if (this->_abort_pressed || this->_goal_reached)
            {
                if (_wp)
                {
                    clear_flags();
                    this->_status = ControllerStatus::motion_cplt;

                    //this->currentCommandIndex++;
                    ROS_INFO("goal reached : dp3_wp");
                    _wp = false;
                }
                else
                {
                    clear_flags();
                    this->_status = ControllerStatus::motion_cplt;

                    this->currentCommandIndex++;
                    ROS_INFO("goal reached : dp3");
                    _wp = true;
                }
            }
        }
        else
        {
            if (_wp)
            {
                this->publish_path_to(RedCoordinates::GetInstance()->get_cr_dp3_wp2());
            }
            else
            {
                this->publish_path_to(RedCoordinates::GetInstance()->get_cr_dp3(),
                        RedCoordinates::GetInstance()->get_cr_dp3_wp1(), RedCoordinates::GetInstance()->get_cr_dp3_wp2());
            }

            clear_flags();
            this->_status = ControllerStatus::moving;
        }
    }
    else if (currentCommand == ControllerCommands::move_to_dp4)
    {
        static bool _wp = true;

        if (this->_status == ControllerStatus::moving)
        {
            if (this->_abort_pressed || this->_goal_reached)
            {
                if (_wp)
                {
                    clear_flags();
                    this->_status = ControllerStatus::motion_cplt;

                    //this->currentCommandIndex++;
                    ROS_INFO("goal reached : dp4_wp");
                    _wp = false;
                }
                else
                {
                    clear_flags();
                    this->_status = ControllerStatus::motion_cplt;

                    this->currentCommandIndex++;
                    ROS_INFO("goal reached : dp4");
                    _wp = true;
                }
            }
        }
        else
        {
            if (_wp)
            {
                this->publish_path_to(RedCoordinates::GetInstance()->get_cr_dp4_wp2());
            }
            else
            {
                this->publish_path_to(RedCoordinates::GetInstance()->get_cr_dp4(),
                        RedCoordinates::GetInstance()->get_cr_dp4_wp1(), RedCoordinates::GetInstance()->get_cr_dp4_wp2());
            }

            clear_flags();
            this->_status = ControllerStatus::moving;
        }
    }
    else if (currentCommand == ControllerCommands::rtb)
    {
        if (this->_status == ControllerStatus::moving)
        {
            if (this->_abort_pressed || this->_goal_reached)
            {
                clear_flags();
                this->_status = ControllerStatus::motion_cplt;

                this->currentCommandIndex++;
                ROS_INFO("goal reached : sz");
            }
        }
        else
        {
            this->publish_path_to(RedCoordinates::GetInstance()->get_cr_sz());

            clear_flags();
            this->_status = ControllerStatus::moving;
        }
    }
    else if (currentCommand == ControllerCommands::pp_pickup)
    {
        if (this->_status == ControllerStatus::pp_pickingup)
        {
            // TODO: this is INSANE
            if (this->_next_pressed)    // || true)
            {
                clear_flags();

                this->chuck();
                this->move_lift(0);

                // move y +0.4
                this->publish_path_to_relative(0.000, 0.200);

                this->_status = ControllerStatus::moving;

                // do not increment the command index

                ROS_INFO("picked up workpieces and backing up now.");
            }
        }
        else if (this->_status == ControllerStatus::moving)
        {
            if (this->_goal_reached)
            {
                clear_flags();
                this->_status = ControllerStatus::motion_cplt;

                this->currentCommandIndex++;
                ROS_INFO("goal reached : ppx_wpy");
            }
        }
        else
        {
            clear_flags();
            this->_is_manual_enabled = true;
            this->_status = ControllerStatus::pp_pickingup;
        }
        this->amt();
    }
    else if (currentCommand == ControllerCommands::dp_deliver)
    {
        if (this->_status == ControllerStatus::dp_delivering)
        {
            if (this->_next_pressed)
            {
                clear_flags();

                this->unchuck();

                // move y -0.4
                this->publish_path_to_relative(0.000, -0.200);

                this->_status = ControllerStatus::moving;

                // do not increment the command index

                ROS_INFO("delivered a workpiece and backing up now.");
            }
        }
        else if (this->_status == ControllerStatus::moving)
        {
            if (this->_goal_reached)
            {
                clear_flags();
                this->_status = ControllerStatus::motion_cplt;

                this->currentCommandIndex++;
                ROS_INFO("goal reached : dpx_wpy");
            }
        }
        else
        {
            clear_flags();
            this->_is_manual_enabled = true;
            this->_status = ControllerStatus::dp_delivering;
        }
        this->amt();
    }
    else if (currentCommand == ControllerCommands::set_lift_p)
    {
        clear_flags();

        move_lift(0);

        this->_status = ControllerStatus::motion_cplt;

        this->currentCommandIndex++;
    }
    else if (currentCommand == ControllerCommands::set_lift_1)
    {
        clear_flags();

        move_lift(1);

        this->_status = ControllerStatus::motion_cplt;

        this->currentCommandIndex++;
    }
    else if (currentCommand == ControllerCommands::set_lift_2)
    {
        clear_flags();
        move_lift(2);
        this->_status = ControllerStatus::motion_cplt;
        this->currentCommandIndex++;
        ROS_INFO("lift position set: 2");
    }
    else if (currentCommand == ControllerCommands::set_lift_3)
    {
        clear_flags();

        move_lift(3);

        this->_status = ControllerStatus::motion_cplt;

        this->currentCommandIndex++;
    }
    else if (currentCommand == ControllerCommands::delay)
    {
        if (this->_delay_s == 0)
        {
            return;
        }

        if (this->_delay_s < ros::Time::now().toSec())
        {
            this->_delay_s = 0;
            this->currentCommandIndex++;
        }
    }
    else if (currentCommand == ControllerCommands::segno)
    {
        this->currentCommandIndex++;
    }
    else if (currentCommand == ControllerCommands::dal_segno)
    {
        auto segno_iter = std::find(this->command_list->begin(), this->command_list->end(), ControllerCommands::segno);
        if (segno_iter == this->command_list->end())
        {
            // abort on error
            ROS_INFO("segno was not found.");
            this->shutdown();
        }
        auto segno_index = std::distance(this->command_list->begin(), segno_iter);
        this->currentCommandIndex = segno_index;
    }
    else if (currentCommand == ControllerCommands::checkpoint_pp1)
    {
        this->currentCommandIndex++;
    }
    else if (currentCommand == ControllerCommands::checkpoint_pp2)
    {
        this->currentCommandIndex++;
    }
    else if (currentCommand == ControllerCommands::checkpoint_pp3)
    {
        this->currentCommandIndex++;
    }
    else if (currentCommand == ControllerCommands::checkpoint_pp4)
    {
        this->currentCommandIndex++;
    }
    else
    {
        ROS_INFO("shutting down on an error: an unknown command detected.");

        this->shutdown();
    }
}

void CrMain::amt(void)
{
    if (!this->_is_manual_enabled)
    {
        return;
    }

    geometry_msgs::Pose moving_target;
    tf::StampedTransform base_link;

    try
    {
        this->_tflistener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(0.1));
        this->_tflistener.lookupTransform("/map", "/base_link", ros::Time(0), base_link);
    }
    catch (...)
    {
        return;
    }

    double coeff = 0.0;
    if (this->_rush)
    {
        // rushing mode
        coeff = this->_amt_coeff * 10;
    }
    else
    {
        // precision mode
        coeff = this->_amt_coeff;
    }
    moving_target.position.x = base_link.getOrigin().x() + (this->_target_x * coeff);
    moving_target.position.y = base_link.getOrigin().y() + (this->_target_y * coeff);
    moving_target.orientation = tf::createQuaternionMsgFromYaw(this->_target_yaw);

    this->publish_path_to(moving_target);
}

void CrMain::chuck(void)
{
    // chuck
    hand_cylinder_msg.data = true;
    hand_cylinder_pub.publish(hand_cylinder_msg);
}

void CrMain::unchuck(void)
{
    // unchuck
    hand_cylinder_msg.data = false;
    hand_cylinder_pub.publish(hand_cylinder_msg);
}

void CrMain::move_lift(int index)
{
    // move lift
    lift_position_msg.data = lift_position[index];
    lift_position_pub.publish(lift_position_msg);
}

void CrMain::set_delay(double delay_s)
{
    //if(this->_delay_s != 0)
    //{
    //	return;
    //}

    this->_delay_s = ros::Time::now().toSec() + delay_s;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cr_main");

    CrMain *instance = new CrMain();
    ROS_INFO("CR main node has started.");

    ros::spin();
    ROS_INFO("CR main node has been terminated.");
}

