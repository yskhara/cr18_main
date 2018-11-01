/*
 * Coordinates.hpp
 *
 *  Created on: Mar 21, 2018
 *      Author: yusaku
 */

#pragma once

#include "Coordinates.hpp"

class RedCoordinates : public Coordinates
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

    static const RedCoordinates *instance;

public:
    RedCoordinates(void);

    static inline const RedCoordinates * const GetInstance(void)
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

const RedCoordinates *RedCoordinates::instance = new RedCoordinates();

RedCoordinates::RedCoordinates(void)
{
    /*
     * Field Coordinates for Carrying Robot (CR)
     *
     * Start Zone
     */
    this->_cr_sz.position.x = 4 - 0.357614;
    this->_cr_sz.position.y = 0.997017;
    this->_cr_sz.orientation = tf::createQuaternionMsgFromYaw(-M_PI / 2);

    geometry_msgs::Quaternion pp_orientation = tf::createQuaternionMsgFromYaw(-M_PI / 2);
    geometry_msgs::Quaternion dp_orientation = tf::createQuaternionMsgFromYaw(M_PI / 2);

    /*
     * Pickup Point 1
     */
    this->_cr_pp1.position.x = 4 - 0.495;
    this->_cr_pp1.position.y = 0.337017;
    this->_cr_pp1.orientation = pp_orientation;

    this->_cr_pp1_wp1.position.x = this->_cr_pp1.position.x;
    this->_cr_pp1_wp1.position.y = 0.500;
    this->_cr_pp1_wp1.orientation = pp_orientation;

    this->_cr_pp1_wp2.position.x = this->_cr_pp1.position.x;
    this->_cr_pp1_wp2.position.y = 0.700;
    this->_cr_pp1_wp2.orientation = pp_orientation;

    /*
     * Pickup Point 2
     */
    this->_cr_pp2.position.x = 4 - 0.825;
    this->_cr_pp2.position.y = this->_cr_pp1.position.y;
    this->_cr_pp2.orientation = pp_orientation;

    this->_cr_pp2_wp1.position.x = this->_cr_pp2.position.x;
    this->_cr_pp2_wp1.position.y = this->_cr_pp1_wp1.position.y;
    this->_cr_pp2_wp1.orientation = pp_orientation;

    this->_cr_pp2_wp2.position.x = this->_cr_pp2.position.x;
    this->_cr_pp2_wp2.position.y = this->_cr_pp1_wp2.position.y;
    this->_cr_pp2_wp2.orientation = pp_orientation;

    /*
     * Pickup Point 3
     */
    this->_cr_pp3.position.x = 4 - 1.155;
    this->_cr_pp3.position.y = this->_cr_pp1.position.y;
    this->_cr_pp3.orientation = pp_orientation;

    this->_cr_pp3_wp1.position.x = this->_cr_pp3.position.x;
    this->_cr_pp3_wp1.position.y = this->_cr_pp1_wp1.position.y;
    this->_cr_pp3_wp1.orientation = pp_orientation;

    this->_cr_pp3_wp2.position.x = this->_cr_pp3.position.x;
    this->_cr_pp3_wp2.position.y = this->_cr_pp1_wp2.position.y;
    this->_cr_pp3_wp2.orientation = pp_orientation;

    /*
     * Pickup Point 4
     */
    this->_cr_pp4.position.x = 4 - 1.485;
    this->_cr_pp4.position.y = this->_cr_pp1.position.y;
    this->_cr_pp4.orientation = pp_orientation;

    this->_cr_pp4_wp1.position.x = this->_cr_pp4.position.x;
    this->_cr_pp4_wp1.position.y = this->_cr_pp1_wp1.position.y;
    this->_cr_pp4_wp1.orientation = pp_orientation;

    this->_cr_pp4_wp2.position.x = this->_cr_pp4.position.x;
    this->_cr_pp4_wp2.position.y = this->_cr_pp1_wp2.position.y;
    this->_cr_pp4_wp2.orientation = pp_orientation;

    /*
     * Delivery Point 1
     */
    this->_cr_dp1.position.x = 4 - 0.938;
    this->_cr_dp1.position.y = 2.050;
    this->_cr_dp1.orientation = dp_orientation;

    this->_cr_dp1_wp1.position.x = this->_cr_dp1.position.x;
    this->_cr_dp1_wp1.position.y = 1.850;
    this->_cr_dp1_wp1.orientation = dp_orientation;

    this->_cr_dp1_wp2.position.x = this->_cr_dp1.position.x;
    this->_cr_dp1_wp2.position.y = 1.700;
    //this->_cr_dp1_wp2.position.y    = 1.700;
    this->_cr_dp1_wp2.orientation = dp_orientation;

    /*
     * Delivery Point 2
     */
    this->_cr_dp2.position.x = 4 - 1.176;
    this->_cr_dp2.position.y = this->_cr_dp1.position.y;
    this->_cr_dp2.orientation = dp_orientation;

    this->_cr_dp2_wp1.position.x = this->_cr_dp2.position.x;
    this->_cr_dp2_wp1.position.y = this->_cr_dp1_wp1.position.y;
    this->_cr_dp2_wp1.orientation = dp_orientation;

    this->_cr_dp2_wp2.position.x = this->_cr_dp2.position.x;
    this->_cr_dp2_wp2.position.y = this->_cr_dp1_wp2.position.y;
    this->_cr_dp2_wp2.orientation = dp_orientation;

    /*
     * Delivery Point 3
     */
    this->_cr_dp3.position.x = 4 - 1.414;
    this->_cr_dp3.position.y = this->_cr_dp1.position.y;
    this->_cr_dp3.orientation = dp_orientation;

    this->_cr_dp3_wp1.position.x = this->_cr_dp3.position.x;
    this->_cr_dp3_wp1.position.y = this->_cr_dp1_wp1.position.y;
    this->_cr_dp3_wp1.orientation = dp_orientation;

    this->_cr_dp3_wp2.position.x = this->_cr_dp3.position.x;
    this->_cr_dp3_wp2.position.y = this->_cr_dp1_wp2.position.y;
    this->_cr_dp3_wp2.orientation = dp_orientation;

    /*
     * Delivery Point 4
     */
    this->_cr_dp4.position.x = 4 - 1.652;
    this->_cr_dp4.position.y = this->_cr_dp1.position.y;
    this->_cr_dp4.orientation = dp_orientation;

    this->_cr_dp4_wp1.position.x = this->_cr_dp4.position.x;
    this->_cr_dp4_wp1.position.y = this->_cr_dp1_wp1.position.y;
    this->_cr_dp4_wp1.orientation = dp_orientation;

    this->_cr_dp4_wp2.position.x = this->_cr_dp4.position.x;
    this->_cr_dp4_wp2.position.y = this->_cr_dp1_wp2.position.y;
    this->_cr_dp4_wp2.orientation = dp_orientation;
}

