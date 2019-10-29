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
//#include <std_msgs/Empty.h>
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
    void shutdownInputCallback(const std_msgs::Bool::ConstPtr& msg);
    void startInputCallback(const std_msgs::Bool::ConstPtr& msg);
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    void driveLauncherWheel(void);
    void stopLauncherWheel(void);
    void extendLoader(void);
    void retractLoader(void);
    void LoaderRetractTimerCallback(const ros::TimerEvent& event);

    void drivePitch(void);

    void extendPicker(void);
    void retractPicker(void);
    void startPickerFan(void);
    void stopPickerFan(void);
    void pickerRetractTimerCallback(const ros::TimerEvent& event);
    void pickerExtendTimerCallback(const ros::TimerEvent& event);

    ros::NodeHandle nh_;

    ros::Timer picker_retract_timer;
    ros::Timer picker_extend_timer;
    ros::Timer loader_timer;

    //int linear_, angular_;
    ros::Subscriber joy_sub;
    ros::Subscriber shutdown_input_sub;
    ros::Subscriber start_input_sub;

    ros::Publisher launcher_esc_pub;
    ros::Publisher loader_servo_pub;
    ros::Publisher pitch_angle_pub;
    ros::Publisher act_enable_pub;

    ros::Publisher picker_esc_pub;
    ros::Publisher picker_servo_pub;

    std_msgs::Int16 launcher_esc_msg;
    std_msgs::Int16 loader_servo_msg;
    std_msgs::Int16 pitch_angle_msg;
    //std_msgs::Bool hand_cylinder_msg;
    std_msgs::Bool act_enable_msg;

    std_msgs::Int16 picker_esc_msg;
    std_msgs::Int16 picker_servo_msg;

    double steps_per_mm = 16 * 200 * 3 / 40;

    static constexpr int servo_neutral = 1520;

    int picker_esc_on = 2000;       // fully on
    int picker_esc_off = servo_neutral;      // neutral
    int picker_servo_extend = servo_neutral;
    int picker_servo_retract = servo_neutral;
    int loader_servo_extend = servo_neutral;
    int loader_servo_retract = servo_neutral;

    double loader_retract_delay = 3.0;
    double picker_retract_delay = 3.0;
    double picker_extend_delay = 3.0;

    std::vector<int> pitch_angle_list = { 0, 0, 0, 0 };
    std::vector<int> launcher_esc_list = { 0, 0, 0, 0 };
    int launcher_target_index = 0;
    //		{0, -40 * steps_per_mm;
    //static constexpr int lift_position_first = -40 * steps_per_mm;
    //static constexpr int lift_position_second = lift_position_first - (248 * steps_per_mm);
    //static constexpr int lift_position_third = lift_position_second - (248 * steps_per_mm);

    bool _shutdown = false;

    bool picker_extended = false;
    bool picker_retracted = false;
    bool launcher_wheel_running = false;
    bool launcher_wheel_stop_scheduled = false;
    bool loader_loading = false;

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
    shutdown_input_sub = nh_.subscribe<std_msgs::Bool>("shutdown_input", 10, &CrMain::shutdownInputCallback, this);
    start_input_sub = nh_.subscribe<std_msgs::Bool>("start_input", 10, &CrMain::startInputCallback, this);

    this->launcher_esc_pub = nh_.advertise<std_msgs::Int16>("launcher_esc", 1);
    this->loader_servo_pub = nh_.advertise<std_msgs::Int16>("loader_servo", 1);

    this->pitch_angle_pub = nh_.advertise<std_msgs::Int16>("pitch_angle", 1);
    this->act_enable_pub = nh_.advertise<std_msgs::Bool>("act_enable", 1);

    this->picker_esc_pub = nh_.advertise<std_msgs::Int16>("picker_esc", 1);
    this->picker_servo_pub = nh_.advertise<std_msgs::Int16>("picker_servo", 1);

    auto nh_priv = ros::NodeHandle("~");

    nh_priv.getParam("lift_step_per_mm", this->steps_per_mm);

    nh_priv.getParam("picker_esc_on", this->picker_esc_on);
    nh_priv.getParam("picker_esc_off", this->picker_esc_off);
    nh_priv.getParam("picker_servo_extend", this->picker_servo_extend);
    nh_priv.getParam("picker_servo_retract", this->picker_servo_retract);
    nh_priv.getParam("loader_servo_extend", this->loader_servo_extend);
    nh_priv.getParam("loader_servo_retract", this->loader_servo_retract);

    nh_priv.getParam("loader_retract_delay", loader_retract_delay);
    loader_timer = nh_.createTimer(loader_retract_delay, &CrMain::LoaderRetractTimerCallback, this, true, false);
    nh_priv.getParam("picker_retract_delay", picker_retract_delay);
    nh_priv.getParam("picker_extend_delay", picker_extend_delay);

    // create picker timer
    picker_retract_timer = nh_.createTimer(picker_retract_delay, &CrMain::pickerRetractTimerCallback, this, true,
            false);
    picker_extend_timer = nh_.createTimer(picker_extend_delay, &CrMain::pickerExtendTimerCallback, this, true, false);

    ROS_INFO("picker_esc_on: %d", this->picker_esc_on);
    ROS_INFO("picker_esc_off: %d", this->picker_esc_off);
    ROS_INFO("picker_servo_extend: %d", this->picker_servo_extend);
    ROS_INFO("picker_servo_retract: %d", this->picker_servo_retract);
    ROS_INFO("loader_servo_extend: %d", this->loader_servo_extend);
    ROS_INFO("loader_servo_retract: %d", this->loader_servo_retract);

    std::vector<int> tmp;
    nh_priv.getParam("pitch_angle", tmp);
    if (tmp.size() == 4)
    {
        this->pitch_angle_list = tmp;
    }

    /*
     for (int& pos : this->pitch_angle_list)
     {
     pos *= (-steps_per_mm);
     }
     */

    ROS_INFO("lift_pos: %d, %d, %d, %d", this->pitch_angle_list[0], this->pitch_angle_list[1],
            this->pitch_angle_list[2], this->pitch_angle_list[3]);

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

void CrMain::shutdownInputCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (!msg->data)
    {
        return;
    }

    if (!this->_shutdown)
    {
        this->_shutdown = true;

        ROS_INFO("aborting.");
    }

    // reset this:
    // this->CurrentCommandIndex = -1;
    launcher_target_index = 0;
}

void CrMain::startInputCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (!msg->data)
    {
        return;
    }

    // bring the robot back operational

    ROS_INFO("starting.");

    act_enable_msg.data = true;
    act_enable_pub.publish(act_enable_msg);

    extendPicker();
    stopPickerFan();

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

        }

        if (_b && !last_b)
        {

        }

        if (_x && !last_x)
        {
            // retract picker and then turn off the fan
            retractPicker();
            picker_retract_timer.setPeriod(ros::Duration(picker_retract_delay), true);
            picker_retract_timer.start();
        }

        if (_y && !last_y)
        {
            picker_retract_timer.stop();
            extendPicker();
            startPickerFan();
        }

        if (_lb && !last_lb)
        {
            // keep the wheels rolling
            if (!launcher_wheel_running)
            {
                driveLauncherWheel();
                launcher_wheel_running = true;
            }
        }

        if(!_lb && last_lb)
        {
            if (loader_loading)
            {
                // don't stop the wheels (yet
                launcher_wheel_stop_scheduled = true;
            }
            else
            {
                stopLauncherWheel();
                launcher_wheel_running = false;
                launcher_wheel_stop_scheduled = false;
            }
        }

        if (_rb && !last_rb)
        {
            if (!loader_loading)
            {
                // load projectile into the launcher
                loader_loading = true;
                extendLoader();
                loader_timer.setPeriod(ros::Duration(loader_retract_delay), true);
                loader_timer.start();
            }
        }

        int dx = -joy->axes[AxisDPadX];
        int dy = joy->axes[AxisDPadY];

        // route 1
        if (dx == 0 && dy > 0)
        {
            // down?
            // middle(center) target
            launcher_target_index = 1;
            driveLauncherWheel();
            drivePitch();
        }
        else if (dx > 0 && dy == 0)
        {
            // right?
            // right-most target
            launcher_target_index = 2;
            driveLauncherWheel();
            drivePitch();
        }
        else if (dx == 0 && dy < 0)
        {
            // left?
            // left-most target
            launcher_target_index = 3;
            driveLauncherWheel();
            drivePitch();
        }
        else if (dx < 0 && dy == 0)
        {
            // up?
            // v goal
            launcher_target_index = 4;
            driveLauncherWheel();
            drivePitch();
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

void CrMain::driveLauncherWheel(void)
{
    ROS_DEBUG("driving the launcher...");
    launcher_esc_msg.data = launcher_esc_list[launcher_target_index];
    launcher_esc_pub.publish(launcher_esc_msg);
}

void CrMain::stopLauncherWheel(void)
{
    ROS_DEBUG("killing the launcher...");
    launcher_esc_msg.data = launcher_esc_list[0];
    launcher_esc_pub.publish(launcher_esc_msg);
}

void CrMain::extendLoader(void)
{
    ROS_DEBUG("loading projectile into the chamber...");
    loader_servo_msg.data = loader_servo_extend;
    loader_servo_pub.publish(loader_servo_msg);
    loader_loading = true;
}

void CrMain::retractLoader(void)
{
    ROS_DEBUG("retracting the loader...");
    loader_servo_msg.data = loader_servo_retract;
    loader_servo_pub.publish(loader_servo_msg);
    loader_loading = false;
}

void CrMain::LoaderRetractTimerCallback(const ros::TimerEvent& event)
{
    retractLoader();

    if (launcher_wheel_stop_scheduled)
    {
        stopLauncherWheel();
        launcher_wheel_running = false;
        launcher_wheel_stop_scheduled = false;
    }
}

void CrMain::drivePitch(void)
{
    ROS_DEBUG("driving the pitchCtrl...");
    pitch_angle_msg.data = pitch_angle_list[launcher_target_index];
    pitch_angle_pub.publish(pitch_angle_msg);
}

void CrMain::pickerExtendTimerCallback(const ros::TimerEvent& event)
{
    // picker already retracted and the fan turned off, so extend again and start the fan
    extendPicker();

    // don't start the fan just yet
    //startPickerFan();
}

void CrMain::pickerRetractTimerCallback(const ros::TimerEvent& event)
{
    // picker already retracted (so we assume), turn the fan off
    stopPickerFan();
    picker_extend_timer.setPeriod(ros::Duration(picker_extend_delay), true);
    picker_extend_timer.start();
}

void CrMain::extendPicker(void)
{
    ROS_DEBUG("extending picker...");
    picker_servo_msg.data = picker_servo_extend;
    picker_servo_pub.publish(picker_servo_msg);
}

void CrMain::retractPicker(void)
{
    ROS_DEBUG("picking up...");
    picker_servo_msg.data = picker_servo_retract;
    picker_servo_pub.publish(picker_servo_msg);
    picker_extended = false;
}

void CrMain::startPickerFan(void)
{
    ROS_DEBUG("turning the fan on...");
    picker_esc_msg.data = picker_esc_on;
    picker_esc_pub.publish(picker_esc_msg);
}

void CrMain::stopPickerFan(void)
{
    ROS_DEBUG("turning the fan off...");
    picker_esc_msg.data = picker_esc_off;
    picker_esc_pub.publish(picker_esc_msg);
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

