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
public:
    virtual ~Coordinates(){};

	// SZ
	virtual geometry_msgs::Pose get_cr_sz(void) const = 0;

	// PP1
    virtual geometry_msgs::Pose get_cr_pp1(void) const = 0;
    virtual geometry_msgs::Pose get_cr_pp1_wp1(void) const = 0;
    virtual geometry_msgs::Pose get_cr_pp1_wp2(void) const = 0;

    // PP2
    virtual geometry_msgs::Pose get_cr_pp2(void) const = 0;
    virtual geometry_msgs::Pose get_cr_pp2_wp1(void) const = 0;
    virtual geometry_msgs::Pose get_cr_pp2_wp2(void) const = 0;

    // PP3
    virtual geometry_msgs::Pose get_cr_pp3(void) const = 0;
    virtual geometry_msgs::Pose get_cr_pp3_wp1(void) const = 0;
    virtual geometry_msgs::Pose get_cr_pp3_wp2(void) const = 0;

    // PP4
    virtual geometry_msgs::Pose get_cr_pp4(void) const = 0;
    virtual geometry_msgs::Pose get_cr_pp4_wp1(void) const = 0;
    virtual geometry_msgs::Pose get_cr_pp4_wp2(void) const = 0;

    // DP1
    virtual geometry_msgs::Pose get_cr_dp1(void) const = 0;
    virtual geometry_msgs::Pose get_cr_dp1_wp1(void) const = 0;
    virtual geometry_msgs::Pose get_cr_dp1_wp2(void) const = 0;

    // DP2
    virtual geometry_msgs::Pose get_cr_dp2(void) const = 0;
    virtual geometry_msgs::Pose get_cr_dp2_wp1(void) const = 0;
    virtual geometry_msgs::Pose get_cr_dp2_wp2(void) const = 0;

    // DP3
    virtual geometry_msgs::Pose get_cr_dp3(void) const = 0;
    virtual geometry_msgs::Pose get_cr_dp3_wp1(void) const = 0;
    virtual geometry_msgs::Pose get_cr_dp3_wp2(void) const = 0;

    // DP4
    virtual geometry_msgs::Pose get_cr_dp4(void) const = 0;
    virtual geometry_msgs::Pose get_cr_dp4_wp1(void) const = 0;
    virtual geometry_msgs::Pose get_cr_dp4_wp2(void) const = 0;
};


