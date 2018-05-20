/*
 * Coordinates.hpp
 *
 *  Created on: Mar 21, 2018
 *      Author: yusaku
 */

#pragma once


#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

class Coordinates
{
private:
	geometry_msgs::Pose _tr_sz;
	geometry_msgs::Pose _tr_wp1;
	geometry_msgs::Pose _tr_wp2_1;
	geometry_msgs::Pose _tr_wp2_2;
	geometry_msgs::Pose _tr_wp3_1;
	geometry_msgs::Pose _tr_wp3_2;
	geometry_msgs::Pose _tr_dp1;
	geometry_msgs::Pose _tr_dp2;
	geometry_msgs::Pose _tr_tz1;
	geometry_msgs::Pose _tr_tz2;
	geometry_msgs::Pose _tr_tz3;

	geometry_msgs::Pose _cr_sz;
	geometry_msgs::Pose _cr_pp1;
	geometry_msgs::Pose _cr_pp2;
	geometry_msgs::Pose _cr_dp1;
	geometry_msgs::Pose _cr_dp2;
	geometry_msgs::Pose _cr_wp1_1;
	geometry_msgs::Pose _cr_wp1_2;
	geometry_msgs::Pose _cr_wp1_3;
	geometry_msgs::Pose _cr_wp2_1;
	geometry_msgs::Pose _cr_wp2_2;
	geometry_msgs::Pose _cr_wp3_1;
	geometry_msgs::Pose _cr_wp3_2;
	geometry_msgs::Pose _cr_wp3_3;

	static const Coordinates *instance;

public:
	Coordinates(void);

	static inline const Coordinates * const GetInstance(void)
	{
		return instance;
	}

	inline geometry_msgs::Pose get_tr_sz(void) const
	{
		return this->_tr_sz;
	}
	inline geometry_msgs::Pose get_tr_wp1(void) const
	{
		return this->_tr_wp1;
	}
	inline geometry_msgs::Pose get_tr_wp2_1(void) const
	{
		return this->_tr_wp2_1;
	}
	inline geometry_msgs::Pose get_tr_wp2_2(void) const
	{
		return this->_tr_wp2_2;
	}
	inline geometry_msgs::Pose get_tr_wp3_1(void) const
	{
		return this->_tr_wp3_1;
	}
	inline geometry_msgs::Pose get_tr_wp3_2(void) const
	{
		return this->_tr_wp3_2;
	}
	/*
	inline geometry_msgs::Pose get_tr_dp1(void) const
	{
		return this->_tr_dp1;
	}
	*/
	inline geometry_msgs::Pose get_tr_dp2(void) const
	{
		return this->_tr_dp2;
	}
	inline geometry_msgs::Pose get_tr_tz1(void) const
	{
		return this->_tr_tz1;
	}
	inline geometry_msgs::Pose get_tr_tz2(void) const
	{
		return this->_tr_tz2;
	}
	inline geometry_msgs::Pose get_tr_tz3(void) const
	{
		return this->_tr_tz3;
	}

	inline geometry_msgs::Pose get_cr_sz(void) const
	{
		return this->_cr_sz;
	}
	inline geometry_msgs::Pose get_cr_pp1(void) const
	{
		return this->_cr_pp1;
	}
	inline geometry_msgs::Pose get_cr_pp2(void) const
	{
		return this->_cr_pp2;
	}
	inline geometry_msgs::Pose get_cr_dp1(void) const
	{
		return this->_cr_dp1;
	}
	inline geometry_msgs::Pose get_cr_dp2(void) const
	{
		return this->_cr_dp2;
	}
	inline geometry_msgs::Pose get_cr_wp1_1(void) const
	{
		return this->_cr_wp1_1;
	}
	inline geometry_msgs::Pose get_cr_wp1_2(void) const
	{
		return this->_cr_wp1_2;
	}
	inline geometry_msgs::Pose get_cr_wp1_3(void) const
	{
		return this->_cr_wp1_3;
	}
	inline geometry_msgs::Pose get_cr_wp2_1(void) const
	{
		return this->_cr_wp2_1;
	}
	inline geometry_msgs::Pose get_cr_wp2_2(void) const
	{
		return this->_cr_wp2_2;
	}
	inline geometry_msgs::Pose get_cr_wp3_1(void) const
	{
		return this->_cr_wp3_1;
	}
	inline geometry_msgs::Pose get_cr_wp3_2(void) const
	{
		return this->_cr_wp3_2;
	}
	inline geometry_msgs::Pose get_cr_wp3_3(void) const
	{
		return this->_cr_wp3_3;
	}
};

const Coordinates *Coordinates::instance = new Coordinates();

Coordinates::Coordinates(void)
{
	/*
	 * Field Coordinates for Throwing Robot (TR)
	 */

	//this->_tr_sz.position.x = 0.500;
	//this->_tr_sz.position.y = 7.550;
	//this->_tr_sz.orientation = tf::createQuaternionMsgFromYaw(-M_PI/4);
	this->_tr_sz.position.x = 0.600;
	this->_tr_sz.position.y = 7.450;
	this->_tr_sz.position.z = 0.000;
	this->_tr_sz.orientation = tf::createQuaternionMsgFromYaw(M_PI/4);

	// SZ -> TZ1
	this->_tr_wp1.position.x = 1.250;
	this->_tr_wp1.position.y = 3.500;
	this->_tr_wp1.position.z = 0.000;
	this->_tr_wp1.orientation = tf::createQuaternionMsgFromYaw(M_PI * 105 / 256);

	//this->_tr_dp1.position.x = 1.065;
	//this->_tr_dp1.position.y = 2.985;
	//this->_tr_dp1.orientation = tf::createQuaternionMsgFromYaw(-M_PI/2);
	//this->_tr_dp1.position.x = 1.900;
	this->_tr_dp1.position.x = 2.100;
	this->_tr_dp1.position.y = 2.750;
	this->_tr_dp1.position.z = 0.000;
	this->_tr_dp1.orientation = tf::createQuaternionMsgFromYaw(0.0);

	// DP1 -> TZ2
	this->_tr_wp2_1.position.x = 1.500;
	this->_tr_wp2_1.position.y = 2.500;
	this->_tr_wp2_1.position.z = 0.000;
	this->_tr_wp2_1.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);

	this->_tr_wp2_2.position.x = 1.500;
	this->_tr_wp2_2.position.y = 1.250;
	this->_tr_wp2_2.position.z = 0.000;
	this->_tr_wp2_2.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);

	this->_tr_wp3_1.position.x = 5.500;
	this->_tr_wp3_1.position.y = 1.000;
	this->_tr_wp3_1.position.z = 0.000;
	this->_tr_wp3_1.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);

	this->_tr_wp3_2.position.x = 4.500;
	this->_tr_wp3_2.position.y = 1.000;
	this->_tr_wp3_2.position.z = 0.000;
	this->_tr_wp3_2.orientation = tf::createQuaternionMsgFromYaw(M_PI * 110 / 256);

	//this->_tr_dp2.position.x = 1.065;
	//this->_tr_dp2.position.y = 0.985;
	//this->_tr_dp2.orientation = tf::createQuaternionMsgFromYaw(-M_PI/2);
	this->_tr_dp2.position.x = 2.700;
	this->_tr_dp2.position.y = 1.300;
	this->_tr_dp2.position.z = 0.000;
	this->_tr_dp2.orientation = tf::createQuaternionMsgFromYaw(M_PI * 110 / 256);

	//this->_tr_tz1.position.x = 3.775;
	//this->_tr_tz1.position.y = 2.985;
	//this->_tr_tz1.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);
	this->_tr_tz1.position.x = 2.700;
	this->_tr_tz1.position.y = 3.150;
	this->_tr_tz1.position.z = 0.000;
	//this->_tr_tz1.orientation = tf::createQuaternionMsgFromYaw(M_PI * 105 / 256);
	this->_tr_tz1.orientation = tf::createQuaternionMsgFromYaw(M_PI * 110 / 256);

	//this->_tr_tz2.position.x = 3.775;
	//this->_tr_tz2.position.y = 0.985;
	//this->_tr_tz2.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);
	this->_tr_tz2.position.x = 2.800;
	this->_tr_tz2.position.y = 1.300;
	this->_tr_tz2.position.z = 0.000;
	this->_tr_tz2.orientation = tf::createQuaternionMsgFromYaw(M_PI * 57 / 128);

	//this->_tr_tz3.position.x = 7.035;
	//this->_tr_tz3.position.y = 0.985;
	//this->_tr_tz3.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);
	this->_tr_tz3.position.x = 7.150;
	this->_tr_tz3.position.y = 0.985;
	this->_tr_tz3.position.z = 0.000;
	this->_tr_tz3.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);


	/*
	 * Field Coordinates for Carrying Robot (CR)
	 */

	//this->_cr_sz.position.x = 0.500;
	//this->_cr_sz.position.y = 9.350;
	//this->_cr_sz.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);
	this->_cr_sz.position.x = 0.600;
	this->_cr_sz.position.y = 9.350;
	this->_cr_sz.position.z = 0.000;
	this->_cr_sz.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);

	this->_cr_pp1.position.x =  0.480 + 0.150;
	this->_cr_pp1.position.y = 13.550;
	this->_cr_pp1.position.z =  0.000;
	this->_cr_pp1.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);

	this->_cr_pp2.position.x =  0.630;
	this->_cr_pp2.position.y = 13.550;
	this->_cr_pp2.position.z =  0.000;
	this->_cr_pp2.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);

	this->_cr_dp1.position.x = 0.600;
	this->_cr_dp1.position.y = 2.750 + 0.075;
	this->_cr_dp1.position.z = 0.000;
	this->_cr_dp1.orientation = tf::createQuaternionMsgFromYaw(0.0);

	this->_cr_dp2.position.x = 0.600;
	this->_cr_dp2.position.y = 0.900 + 0.075;
	this->_cr_dp2.position.z = 0.000;
	this->_cr_dp2.orientation = tf::createQuaternionMsgFromYaw(0.0);

	// waypoint 1
	// for LZ (PP1) -> DP1
	this->_cr_wp1_1.position.x =  0.480 + 0.150;
	this->_cr_wp1_1.position.y = 12.500;
	this->_cr_wp1_1.position.z =  0.000;
	this->_cr_wp1_1.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);

	this->_cr_wp1_2.position.x =  1.000;
	this->_cr_wp1_2.position.y = 10.000;
	this->_cr_wp1_2.position.z =  0.000;
	this->_cr_wp1_2.orientation = tf::createQuaternionMsgFromYaw(0.0);

	this->_cr_wp1_3.position.x = 0.600;
	this->_cr_wp1_3.position.y = 3.500;
	this->_cr_wp1_3.position.z = 0.000;
	this->_cr_wp1_3.orientation = tf::createQuaternionMsgFromYaw(0.0);

	// waypoint 2
	// for DP1 -> LZ (PP2)
	this->_cr_wp2_1.position.x =  1.000;
	this->_cr_wp2_1.position.y = 10.000;
	this->_cr_wp2_1.position.z =  0.000;
	this->_cr_wp2_1.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);

	//this->_cr_wp2_2.position.x =  0.480;
	//this->_cr_wp2_2.position.y = 12.500;
	this->_cr_wp2_2.position.x =  0.700;
	this->_cr_wp2_2.position.y = 12.500;
	this->_cr_wp2_2.position.z =  0.000;
	this->_cr_wp2_2.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);

	// waypoint 3
	// for LZ (PP2) -> DP2
	this->_cr_wp3_1.position.x =  0.480;
	this->_cr_wp3_1.position.y = 12.500;
	this->_cr_wp3_1.position.z =  0.000;
	this->_cr_wp3_1.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);

	this->_cr_wp3_2.position.x =  1.000;
	this->_cr_wp3_2.position.y = 10.000;
	this->_cr_wp3_2.position.z =  0.000;
	this->_cr_wp3_2.orientation = tf::createQuaternionMsgFromYaw(0.0);

	this->_cr_wp3_3.position.x = 0.650;
	this->_cr_wp3_3.position.y = 1.500;
	this->_cr_wp3_3.position.z = 0.000;
	this->_cr_wp3_3.orientation = tf::createQuaternionMsgFromYaw(0.0);
}


