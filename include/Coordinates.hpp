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
protected:
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

    // PP3
    inline geometry_msgs::Pose get_cr_pp3(void) const
    {
        return this->_cr_pp3;
    }
    inline geometry_msgs::Pose get_cr_pp3_wp1(void) const
    {
        return this->_cr_pp3_wp1;
    }
    inline geometry_msgs::Pose get_cr_pp3_wp2(void) const
    {
        return this->_cr_pp3_wp2;
    }

    // PP4
    inline geometry_msgs::Pose get_cr_pp4(void) const
    {
        return this->_cr_pp4;
    }
    inline geometry_msgs::Pose get_cr_pp4_wp1(void) const
    {
        return this->_cr_pp4_wp1;
    }
    inline geometry_msgs::Pose get_cr_pp4_wp2(void) const
    {
        return this->_cr_pp4_wp2;
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

    // DP2
    inline geometry_msgs::Pose get_cr_dp2(void) const
    {
        return this->_cr_dp2;
    }
    inline geometry_msgs::Pose get_cr_dp2_wp1(void) const
    {
        return this->_cr_dp2_wp1;
    }
    inline geometry_msgs::Pose get_cr_dp2_wp2(void) const
    {
        return this->_cr_dp2_wp2;
    }

    // DP3
    inline geometry_msgs::Pose get_cr_dp3(void) const
    {
        return this->_cr_dp3;
    }
    inline geometry_msgs::Pose get_cr_dp3_wp1(void) const
    {
        return this->_cr_dp3_wp1;
    }
    inline geometry_msgs::Pose get_cr_dp3_wp2(void) const
    {
        return this->_cr_dp3_wp2;
    }

    // DP4
    inline geometry_msgs::Pose get_cr_dp4(void) const
    {
        return this->_cr_dp4;
    }
    inline geometry_msgs::Pose get_cr_dp4_wp1(void) const
    {
        return this->_cr_dp4_wp1;
    }
    inline geometry_msgs::Pose get_cr_dp4_wp2(void) const
    {
        return this->_cr_dp4_wp2;
    }
};

const Coordinates *Coordinates::instance = new Coordinates();

Coordinates::Coordinates(void)
{

}


