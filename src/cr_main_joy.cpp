/*
 * tr_main.cpp
 *
 *  Created on: Jan 4, 2018
 *      Author: yusaku
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <string>

class CrMain
{
public:
    CrMain(void);

private:
    void shutdownInputCallback(const std_msgs::Empty::ConstPtr& msg);
    void startInputCallback(const std_msgs::Empty::ConstPtr& msg);
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    void chuckLeft(void);
    void unchuckLeft(void);
    void chuckRight(void);
    void unchuckRight(void);

    void extendChucks(void);
    void halfExtendChucks(void);
    void retractChucks(void);
    void chucksRetractionTimerCallback(const ros::TimerEvent& event);
    void chucksExtensionTimerCallback(const ros::TimerEvent& event);

    ros::NodeHandle nh_;

    ros::Timer chucks_retraction_timer;
    ros::Timer chucks_extension_timer;

    ros::Subscriber joy_sub;
    ros::Subscriber shutdown_input_sub;
    ros::Subscriber start_input_sub;
    ros::Publisher act_enable_pub;

    ros::Publisher extension_angle_pub;
    ros::Publisher chuck_left_chucked_pub;
    ros::Publisher chuck_right_chucked_pub;

    std_msgs::Bool act_enable_msg;
    std_msgs::Bool chuck_left_chucked_msg;
    std_msgs::Bool chuck_right_chucked_msg;
    std_msgs::Int32 extension_angle_msg;

    static constexpr double extension_steps_per_rev = (160.0 / 32 * 50 / 16) * 16 * 200;
    double extension_steps_per_deg = extension_steps_per_rev / 360.0;

    double delay_before_retraction = 3.0;
    double delay_after_extension = 3.0;

    double extension_angle_extended = 0.0;
    double extension_angle_half_extended = 10.0;
    double extension_angle_retracted = 90.0;
    double extension_angle_initial = 90.0;

    bool _shutdown = false;

    bool chuck_left_chucked = false;
    bool chuck_right_chucked = false;
    bool chucks_releasing = false;
    bool chucks_capturing = false;

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

    this->chuck_left_chucked_pub = nh_.advertise<std_msgs::Bool>("chuck_left_chucked", 1);
    this->chuck_right_chucked_pub = nh_.advertise<std_msgs::Bool>("chuck_right_chucked", 1);

    this->extension_angle_pub = nh_.advertise<std_msgs::Int32>("extension_angle", 1);
    this->act_enable_pub = nh_.advertise<std_msgs::Bool>("act_enable", 1);

    auto nh_priv = ros::NodeHandle("~");

    nh_priv.getParam("delay_before_retraction", delay_before_retraction);
    nh_priv.getParam("delay_after_extension", delay_after_extension);

    nh_priv.getParam("extension_steps_per_deg", this->extension_steps_per_deg);

    nh_priv.getParam("extension_angle_extended", this->extension_angle_extended);
    nh_priv.getParam("extension_angle_half_extended", this->extension_angle_half_extended);
    nh_priv.getParam("extension_angle_retracted", this->extension_angle_retracted);
    nh_priv.getParam("extension_angle_initial", this->extension_angle_initial);
    this->extension_angle_extended *= extension_steps_per_deg;
    this->extension_angle_half_extended *= extension_steps_per_deg;
    this->extension_angle_retracted *= extension_steps_per_deg;
    this->extension_angle_initial *= extension_steps_per_deg;

    chucks_extension_timer = nh_.createTimer(delay_after_extension, &CrMain::chucksExtensionTimerCallback, this, true, false);
    chucks_retraction_timer = nh_.createTimer(delay_before_retraction, &CrMain::chucksRetractionTimerCallback, this, true, false);

    ROS_INFO("delay_before_retraction: %f", this->delay_before_retraction);
    ROS_INFO("delay_after_extension: %f", this->delay_after_extension);

    ROS_INFO("extension_angle_extended: %lf", this->extension_angle_extended);
    ROS_INFO("extension_angle_half_extended: %lf", this->extension_angle_half_extended);
    ROS_INFO("extension_angle_retracted: %lf", this->extension_angle_retracted);
    ROS_INFO("extension_angle_initial: %lf", this->extension_angle_initial);

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

        ROS_INFO("Aborting. (Source: EMS)");
    }

    // Reset internal state:
    chuck_left_chucked = false;
    chuck_right_chucked = false;
    chucks_releasing = false;
    chucks_capturing = false;
}

void CrMain::startInputCallback(const std_msgs::Empty::ConstPtr& msg)
{
    // bring the robot back operational
    ROS_INFO("Starting.");

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

            ROS_INFO("Aborting. (Source: user input)");
        }

        act_enable_msg.data = false;
        act_enable_pub.publish(act_enable_msg);
    }

    if (!this->_shutdown)
    {
        if (_a && !last_a)
        {
            if(!chucks_capturing && !chucks_releasing)
            {
                // Toggle left chuck
                if(chuck_left_chucked)
                {
                    unchuckLeft();
                }
                else
                {
                    chuckLeft();
                }
            }
        }

        if (_b && !last_b)
        {
            if(!chucks_capturing && !chucks_releasing)
            {
                // Toggle right chuck
                if(chuck_right_chucked)
                {
                    unchuckRight();
                }
                else
                {
                    chuckRight();
                }
            }
        }

        if (_x && !last_x)
        {
            // Extend the extension and then unchuck both
            if(!chucks_capturing && !chucks_releasing)
            {
                chucks_releasing = true;
                extendChucks();
                chucks_extension_timer.setPeriod(ros::Duration(delay_after_extension), true);
                chucks_extension_timer.start();
            }
        }

        if (_y && !last_y)
        {
            if(!chucks_capturing && !chucks_releasing)
            {
                // Chuck both and then retract the extension
                chucks_capturing = true;
                chuckLeft();
                chuckRight();
                chucks_retraction_timer.setPeriod(ros::Duration(delay_before_retraction), true);
                chucks_retraction_timer.start();
            }
        }

        if (_lb && !last_lb)
        {
            if(!chucks_capturing && !chucks_releasing)
            {
                // simply extend extension
                extendChucks();
            }
        }

        if (_rb && !last_rb)
        {
            if(!chucks_capturing && !chucks_releasing)
            {
                // simply retract extension
                retractChucks();
            }
        }

        int dx = -joy->axes[AxisDPadX];
        int dy = joy->axes[AxisDPadY];

        if (dx == 0 && dy > 0)
        {
            // Up: Extend all the way
            if(!chucks_capturing && !chucks_releasing)
            {
                extendChucks();
            }
        }
        else if (dx > 0 && dy == 0)
        {
            // Right: Do nothing
        }
        else if (dx == 0 && dy < 0)
        {
            // Down: Retract all the way
            if(!chucks_capturing && !chucks_releasing)
            {
                retractChucks();
            }
        }
        else if (dx < 0 && dy == 0)
        {
            // Left: Extend half way
            if(!chucks_capturing && !chucks_releasing)
            {
                halfExtendChucks();
            }
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

void CrMain::chuckLeft(void)
{
    ROS_DEBUG("Chucking LEFT...");
    chuck_left_chucked_msg.data = true;
    chuck_left_chucked_pub.publish(chuck_left_chucked_msg);
    chuck_left_chucked = true;
}

void CrMain::unchuckLeft(void)
{
    ROS_DEBUG("Unchucking LEFT...");
    chuck_left_chucked_msg.data = false;
    chuck_left_chucked_pub.publish(chuck_left_chucked_msg);
    chuck_left_chucked = false;
}

void CrMain::chuckRight(void)
{
    ROS_DEBUG("Chucking RIGHT...");
    chuck_right_chucked_msg.data = true;
    chuck_right_chucked_pub.publish(chuck_right_chucked_msg);
    chuck_right_chucked = true;
}

void CrMain::unchuckRight(void)
{
    ROS_DEBUG("Unchucking RIGHT...");
    chuck_right_chucked_msg.data = false;
    chuck_right_chucked_pub.publish(chuck_right_chucked_msg);
    chuck_right_chucked = false;
}

void CrMain::extendChucks(void)
{
    extension_angle_msg.data = extension_angle_extended - extension_angle_initial;
    ROS_DEBUG("Extending the Extension: %d", extension_angle_msg.data);
    extension_angle_pub.publish(extension_angle_msg);
}

void CrMain::halfExtendChucks(void)
{
    extension_angle_msg.data = extension_angle_half_extended - extension_angle_initial;
    ROS_DEBUG("Half-extending the Extension: %d", extension_angle_msg.data);
    extension_angle_pub.publish(extension_angle_msg);
}

void CrMain::retractChucks(void)
{
    extension_angle_msg.data = extension_angle_retracted - extension_angle_initial;
    ROS_DEBUG("Retracting the Extension: %d", extension_angle_msg.data);
    extension_angle_pub.publish(extension_angle_msg);
}

void CrMain::chucksRetractionTimerCallback(const ros::TimerEvent& event)
{
    // Both chucks are chucked
    retractChucks();
    chucks_capturing = false;
}

void CrMain::chucksExtensionTimerCallback(const ros::TimerEvent& event)
{
    // Chucks are extended. Unchuck both.
    unchuckLeft();
    unchuckRight();
    chucks_releasing = false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cr_main_joy");

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    CrMain *crMain = new CrMain();
    ROS_INFO("cr_main_joy node has started.");

    ros::spin();
    ROS_INFO("cr_main_joy node has been terminated.");
}

