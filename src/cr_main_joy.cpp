/*
 * tr_main.cpp
 *
 *  Created on: Jan 4, 2018
 *      Author: yusaku
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
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

enum class CarrierStatus
    : uint16_t
    {
        shutdown = 0x0000,
    reset = 0x0001,

/*
 operational			= 0x0002,

 chuck0_chucked		= 0x0010,
 chuck1_chucked		= 0x0020,
 chuck2_chucked		= 0x0040,
 chuck3_chucked		= 0x0080,
 */
};

enum class CarrierCommands
    : uint16_t
    {
        shutdown_cmd = 0x0000,
    reset_cmd = 0x0001,
    /*
     operational			= 0x0002,
     */

    chuck_cmd = 0x0100,
    unchuck_cmd = 0x0200,

    chuck0 = 0x0010,
    chuck1 = 0x0020,
    chuck2 = 0x0040,
    chuck3 = 0x0080,
};

class CrMain
{
public:
    CrMain(void);

private:
    void shutdownInputCallback(const std_msgs::Empty::ConstPtr& msg);
    void startInputCallback(const std_msgs::Empty::ConstPtr& msg);
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;

    //int linear_, angular_;
    ros::Subscriber joy_sub;
    ros::Subscriber shutdown_input_sub;
    ros::Subscriber start_input_sub;

    ros::Publisher lift_position_pub;
    ros::Publisher hand_cylinder_pub;
    ros::Publisher act_enable_pub;

    std_msgs::Int32 lift_position_msg;
    std_msgs::Bool hand_cylinder_msg;
    std_msgs::Bool act_enable_msg;

    double steps_per_mm = 16 * 200 * 3 / 40;

    std::vector<int> lift_position = { 0, 0, 0, 0 };
    int lift_position_index = 0;
    //		{0, -40 * steps_per_mm;
    //static constexpr int lift_position_first = -40 * steps_per_mm;
    //static constexpr int lift_position_second = lift_position_first - (248 * steps_per_mm);
    //static constexpr int lift_position_third = lift_position_second - (248 * steps_per_mm);

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
    shutdown_input_sub = nh_.subscribe<std_msgs::Empty>("shutdown_input", 10, &CrMain::shutdownInputCallback, this);
    start_input_sub = nh_.subscribe<std_msgs::Empty>("start_input", 10, &CrMain::startInputCallback, this);

    this->lift_position_pub = nh_.advertise<std_msgs::Int32>("lift_position", 1);
    this->hand_cylinder_pub = nh_.advertise<std_msgs::Bool>("hand_cylinder", 1);
    this->act_enable_pub = nh_.advertise<std_msgs::Bool>("act_enable", 1);
    //this->hand_unchuck_thres_pub = nh_.advertise<std_msgs::UInt16>("hand/unchuck_thres", 1);

    auto nh_priv = ros::NodeHandle("~");

    nh_priv.getParam("lift_step_per_mm", this->steps_per_mm);

    std::vector<int> tmp;
    nh_priv.getParam("lift_position", tmp);
    if (tmp.size() == 4)
    {
        this->lift_position = tmp;
    }

    for (int& pos : this->lift_position)
    {
        pos *= (-steps_per_mm);
    }

    ROS_INFO("lift_pos: %d, %d, %d, %d", this->lift_position[0], this->lift_position[1], this->lift_position[2],
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
}

void CrMain::shutdownInputCallback(const std_msgs::Empty::ConstPtr& msg)
{
    if (!this->_shutdown)
    {
        this->_shutdown = true;

        ROS_INFO("aborting.");
    }

    // reset this:
    // this->CurrentCommandIndex = -1;
    lift_position_index = 0;
}

void CrMain::startInputCallback(const std_msgs::Empty::ConstPtr& msg)
{
    // bring the robot back operational

    ROS_INFO("starting.");

    act_enable_msg.data = true;
    act_enable_pub.publish(act_enable_msg);

    this->_shutdown = false;
}

void CrMain::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    static bool last_a = false;
    static bool last_b = false;
    static bool last_x = false;
    static bool last_y = false;
    static bool last_lb = false;
    static bool last_rb = false;
    static bool last_start = false;
    static bool last_select = false;
    //static int last_dpadXCmd = 0;

    bool _a = joy->buttons[ButtonA];
    bool _b = joy->buttons[ButtonB];
    bool _x = joy->buttons[ButtonX];
    bool _y = joy->buttons[ButtonY];
    bool _lb = joy->buttons[ButtonLB];
    bool _rb = joy->buttons[ButtonRB];
    bool _start = joy->buttons[ButtonStart];
    bool _select = joy->buttons[ButtonSelect];

    if ((_start && !last_start) || (_select && !last_select))
    {
        if (!this->_shutdown)
        {
            this->_shutdown = true;

            ROS_INFO("aborting.");
        }

        act_enable_msg.data = false;
        act_enable_pub.publish(act_enable_msg);
    }

    if (!this->_shutdown)
    {
        if (_a && !last_a)
        {
            // chuck
            hand_cylinder_msg.data = true;
            hand_cylinder_pub.publish(hand_cylinder_msg);
        }
        else if (_b && !last_b)
        {
            // unchuck
            hand_cylinder_msg.data = false;
            hand_cylinder_pub.publish(hand_cylinder_msg);
        }
        else if (_x && !last_x)
        {

        }
        else if (_y && !last_y)
        {

        }
        else if (_lb && !last_lb)
        {
            // lower the lift
            /*
            lift_position_index--;
            if (lift_position_index < 0)
            {
                lift_position_index = 0;
            }
            lift_position_msg.data = lift_position[lift_position_index];
            lift_position_pub.publish(lift_position_msg);

            //ROS_INFO("lift: %d");

             */
        }
        else if (_rb && !last_rb)
        {
            // raise the lift
            /*
            lift_position_index++;
            if (lift_position_index >= 5)
            {
                lift_position_index = 4;
            }
            lift_position_msg.data = lift_position[lift_position_index];
            lift_position_pub.publish(lift_position_msg);
            */
        }

        int dx = -joy->axes[AxisDPadX];
        int dy = joy->axes[AxisDPadY];

        // route 1
        if (dx == 0 && dy > 0)
        {
            // from pp1
            lift_position_msg.data = lift_position[0];
            lift_position_pub.publish(lift_position_msg);
        }
        else if (dx > 0 && dy == 0)
        {
            // from pp2
            lift_position_msg.data = lift_position[1];
            lift_position_pub.publish(lift_position_msg);
        }
        else if (dx == 0 && dy < 0)
        {
            // from pp3
            lift_position_msg.data = lift_position[2];
            lift_position_pub.publish(lift_position_msg);
        }
        else if (dx < 0 && dy == 0)
        {
            // from pp4
            lift_position_msg.data = lift_position[3];
            lift_position_pub.publish(lift_position_msg);
        }
    }

    last_a = _a;
    last_b = _b;
    last_x = _x;
    last_y = _y;
    last_lb = _lb;
    last_rb = _rb;
    last_start = _start;
    last_select = _select;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cr_main_joy");

    CrMain *crMain = new CrMain();
    ROS_INFO("cr_main_joy node has started.");

    ros::spin();
    ROS_INFO("cr_main_joy node has been terminated.");
}

