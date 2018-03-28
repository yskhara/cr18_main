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
	geometry_msgs::Pose _tr_vp1;
	geometry_msgs::Pose _tr_vp2;
	geometry_msgs::Pose _tr_dp1;
	geometry_msgs::Pose _tr_dp2;
	geometry_msgs::Pose _tr_tz1;
	geometry_msgs::Pose _tr_tz2;
	geometry_msgs::Pose _tr_tz3;

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
	inline geometry_msgs::Pose get_tr_vp1(void) const
	{
		return this->_tr_vp1;
	}
	inline geometry_msgs::Pose get_tr_vp2(void) const
	{
		return this->_tr_vp2;
	}
	inline geometry_msgs::Pose get_tr_dp1(void) const
	{
		return this->_tr_dp1;
	}
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
};

const Coordinates *Coordinates::instance = new Coordinates();

Coordinates::Coordinates(void)
{
	//this->_tr_sz.position.x = 0.500;
	//this->_tr_sz.position.y = 7.550;
	//this->_tr_sz.orientation = tf::createQuaternionMsgFromYaw(-M_PI/4);
	this->_tr_sz.position.x = 0.600;
	this->_tr_sz.position.y = 7.450;
	this->_tr_sz.position.z = 0.000;
	this->_tr_sz.orientation = tf::createQuaternionMsgFromYaw(-M_PI/4);

	this->_tr_vp1.position.x = 1.500;
	this->_tr_vp1.position.y = 2.750;
	this->_tr_vp1.position.z = 0.000;
	this->_tr_vp1.orientation = tf::createQuaternionMsgFromYaw(-M_PI/2);

	//this->_tr_dp1.position.x = 1.065;
	//this->_tr_dp1.position.y = 2.985;
	//this->_tr_dp1.orientation = tf::createQuaternionMsgFromYaw(-M_PI/2);
	this->_tr_dp1.position.x = 1.900;
	this->_tr_dp1.position.y = 2.750;
	this->_tr_dp1.position.z = 0.000;
	this->_tr_dp1.orientation = tf::createQuaternionMsgFromYaw(0.0);

	this->_tr_vp2.position.x = 1.300;
	this->_tr_vp2.position.y = 1.500;
	this->_tr_vp2.position.z = 0.000;
	this->_tr_vp2.orientation = tf::createQuaternionMsgFromYaw(-M_PI/2);

	//this->_tr_dp2.position.x = 1.065;
	//this->_tr_dp2.position.y = 0.985;
	//this->_tr_dp2.orientation = tf::createQuaternionMsgFromYaw(-M_PI/2);
	this->_tr_dp2.position.x = 2.250;
	this->_tr_dp2.position.y = 0.900;
	this->_tr_dp2.position.z = 0.000;
	this->_tr_dp2.orientation = tf::createQuaternionMsgFromYaw(0.0);

	//this->_tr_tz1.position.x = 3.775;
	//this->_tr_tz1.position.y = 2.985;
	//this->_tr_tz1.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);
	this->_tr_tz1.position.x = 2.8;
	this->_tr_tz1.position.y = 2.985;
	this->_tr_tz1.position.z = 0.000;
	this->_tr_tz1.orientation = tf::createQuaternionMsgFromYaw(M_PI * 113 / 256);

	//this->_tr_tz2.position.x = 3.775;
	//this->_tr_tz2.position.y = 0.985;
	//this->_tr_tz2.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);
	this->_tr_tz2.position.x = 2.800;
	this->_tr_tz2.position.y = 1.400;
	this->_tr_tz2.position.z = 0.000;
	this->_tr_tz2.orientation = tf::createQuaternionMsgFromYaw(M_PI * 29 / 64);

	this->_tr_tz3.position.x = 7.035;
	this->_tr_tz3.position.y = 0.985;
	this->_tr_tz3.position.z = 0.000;
	this->_tr_tz3.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);
}


