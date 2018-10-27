/*
 * Coordinates.hpp
 *
 *  Created on: Mar 21, 2018
 *      Author: yusaku
 */

#pragma once


#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>

class Coordinates
{
private:
	geometry_msgs::Pose _cr_sz;

    geometry_msgs::Pose _cr_pp1;
    geometry_msgs::Pose _cr_pp1_wp1;
    geometry_msgs::Pose _cr_pp1_wp2;

    geometry_msgs::Pose _cr_pp2;
    geometry_msgs::Pose _cr_pp2_wp1;
    geometry_msgs::Pose _cr_pp2_wp2;

    geometry_msgs::Pose _cr_pp3;
    geometry_msgs::Pose _cr_pp3_wp1;
    geometry_msgs::Pose _cr_pp3_wp2;

    geometry_msgs::Pose _cr_pp4;
    geometry_msgs::Pose _cr_pp4_wp1;
    geometry_msgs::Pose _cr_pp4_wp2;

    geometry_msgs::Pose _cr_dp1;
    geometry_msgs::Pose _cr_dp1_wp1;
    geometry_msgs::Pose _cr_dp1_wp2;

    geometry_msgs::Pose _cr_dp2;
    geometry_msgs::Pose _cr_dp2_wp1;
    geometry_msgs::Pose _cr_dp2_wp2;

    geometry_msgs::Pose _cr_dp3;
    geometry_msgs::Pose _cr_dp3_wp1;
    geometry_msgs::Pose _cr_dp3_wp2;

    geometry_msgs::Pose _cr_dp4;
    geometry_msgs::Pose _cr_dp4_wp1;
    geometry_msgs::Pose _cr_dp4_wp2;

	static const Coordinates *instance;

public:
	Coordinates(void);

	static inline const Coordinates * const GetInstance(void)
	{
		return instance;
	}

	// SZ
	inline geometry_msgs::Pose get_cr_sz(void) const
	{
		return this->_cr_sz;
	}

	// PP1
    inline geometry_msgs::Pose get_cr_pp1(void) const
    {
        return this->_cr_pp1;
    }
    inline geometry_msgs::Pose get_cr_pp1_wp1(void) const
    {
        return this->_cr_pp1_wp1;
    }
    inline geometry_msgs::Pose get_cr_pp1_wp2(void) const
    {
        return this->_cr_pp1_wp2;
    }

    // PP2
    inline geometry_msgs::Pose get_cr_pp2(void) const
    {
        return this->_cr_pp2;
    }
    inline geometry_msgs::Pose get_cr_pp2_wp1(void) const
    {
        return this->_cr_pp2_wp1;
    }
    inline geometry_msgs::Pose get_cr_pp2_wp2(void) const
    {
        return this->_cr_pp2_wp2;
    }

    // DP1
    inline geometry_msgs::Pose get_cr_dp1(void) const
    {
        return this->_cr_dp1;
    }
    inline geometry_msgs::Pose get_cr_dp1_wp1(void) const
    {
        return this->_cr_dp1_wp1;
    }
    inline geometry_msgs::Pose get_cr_dp1_wp2(void) const
    {
        return this->_cr_dp1_wp2;
    }
};

const Coordinates *Coordinates::instance = new Coordinates();

Coordinates::Coordinates(void)
{
	/*
	 * Field Coordinates for Carrying Robot (CR)
	 *
	 * Start Zone
	 */
	this->_cr_sz.position.x = 0.330;
	this->_cr_sz.position.y = 1.100;
	this->_cr_sz.position.z = 0.000;
	this->_cr_sz.orientation = tf::createQuaternionMsgFromYaw(-M_PI/2);

    /*
     * Pickup Point 1
     */
    this->_cr_pp1_wp2.position.x    = 0.495;
    this->_cr_pp1_wp2.position.y    = 0.900;
    this->_cr_pp1_wp2.orientation   = tf::createQuaternionMsgFromYaw(-M_PI/2);

    this->_cr_pp1_wp1.position.x    = 0.495;
    this->_cr_pp1_wp1.position.y    = 0.700;
    this->_cr_pp1_wp1.orientation   = tf::createQuaternionMsgFromYaw(-M_PI/2);

    this->_cr_pp1.position.x        = 0.495;
    this->_cr_pp1.position.y        = 0.350;
    this->_cr_pp1.orientation       = tf::createQuaternionMsgFromYaw(-M_PI/2);

    /*
     * Pickup Point 2
     */
    this->_cr_pp2_wp2.position.x    = 0.825;
    this->_cr_pp2_wp2.position.y    = 0.900;
    this->_cr_pp2_wp2.orientation   = tf::createQuaternionMsgFromYaw(-M_PI/2);

    this->_cr_pp2_wp1.position.x    = 0.825;
    this->_cr_pp2_wp1.position.y    = 0.700;
    this->_cr_pp2_wp1.orientation   = tf::createQuaternionMsgFromYaw(-M_PI/2);

    this->_cr_pp2.position.x        = 0.825;
    this->_cr_pp2.position.y        = 0.350;
    this->_cr_pp2.orientation       = tf::createQuaternionMsgFromYaw(-M_PI/2);

    /*
     * Delivery Point 1
     */
    this->_cr_dp1.position.x        = 0.938;
    this->_cr_dp1.position.y        = 2.150;
    this->_cr_dp1.orientation       = tf::createQuaternionMsgFromYaw(M_PI/2);

    this->_cr_dp1_wp1.position.x    = 0.938;
    this->_cr_dp1_wp1.position.y    = 1.900;
    this->_cr_dp1_wp1.orientation   = tf::createQuaternionMsgFromYaw(M_PI/2);

    this->_cr_dp1_wp2.position.x    = 0.938;
    this->_cr_dp1_wp2.position.y    = 1.700;
    this->_cr_dp1_wp2.orientation   = tf::createQuaternionMsgFromYaw(M_PI/2);

    /*
     * Delivery Point 2
     * TODO: wrong parameters
     */
    this->_cr_dp2_wp2.position.x    = 0.825;
    this->_cr_dp2_wp2.position.y    = 0.900;
    this->_cr_dp2_wp2.orientation   = tf::createQuaternionMsgFromYaw(M_PI/2);

    this->_cr_dp2_wp1.position.x    = 0.825;
    this->_cr_dp2_wp1.position.y    = 0.700;
    this->_cr_dp2_wp1.orientation   = tf::createQuaternionMsgFromYaw(M_PI/2);

    this->_cr_dp2.position.x        = 0.825;
    this->_cr_dp2.position.y        = 0.350;
    this->_cr_dp2.orientation       = tf::createQuaternionMsgFromYaw(M_PI/2);

}


