/*
 * Copyright (C) 2020, PolyExplore, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of PolyExplore, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 
#ifndef POLYX_node_TALKER_HPP
#define POLYX_node_TALKER_HPP

#include "rclcpp/rclcpp.hpp"

#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>

#ifdef ENABLED_GEO_POSE_STAMPED
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#endif

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "polyx_convert.h"

class PolyxnodeTalker : public rclcpp::Node
{
public:
    PolyxnodeTalker();
    ~PolyxnodeTalker();
	
    static const int MaxMsgLen = 2048;

private:	
    void init();

    void polyxWheelSpeedEventCallback(const polyx_node::msg::WheelSpeedEvent::SharedPtr msg);
    void polyxStaticHeadingEventCallback(const polyx_node::msg::StaticHeadingEvent::SharedPtr stmsg);
    void polyxStaticGeoPoseEventCallback(const polyx_node::msg::StaticGeoPoseEvent::SharedPtr sgmsg);
    void execute();
    

    rclcpp::Publisher<polyx_node::msg::Kalman>::SharedPtr kalman_pub_;
    rclcpp::Publisher<polyx_node::msg::RawIMU>::SharedPtr RawIMU_pub_;
    rclcpp::Publisher<polyx_node::msg::SolutionStatus>::SharedPtr SolutionStatus_pub_;
    rclcpp::Publisher<polyx_node::msg::CompactNav>::SharedPtr compactNav_pub_;
#ifdef ENABLED_GEO_POSE_STAMPED    
    rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr geopose_pub_;
#endif
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr accel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navfix_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr attitude_imu_pub_;
    rclcpp::Publisher<polyx_node::msg::EulerAttitude>::SharedPtr EulerAttitude_pub_;
    rclcpp::Publisher<polyx_node::msg::TimeSync>::SharedPtr timeSync_pub_;
    rclcpp::Publisher<polyx_node::msg::Geoid>::SharedPtr geoid_pub_;
    rclcpp::Publisher<polyx_node::msg::CorrectedIMU>::SharedPtr CorrectedIMU_pub_;
    rclcpp::Publisher<polyx_node::msg::LeapSeconds>::SharedPtr leapSeconds_pub_;
    rclcpp::Publisher<polyx_node::msg::NmeaGGA>::SharedPtr nmeaGGA_pub_;
    rclcpp::Publisher<polyx_node::msg::Dmi>::SharedPtr dmi_pub_;

    rclcpp::Subscription<polyx_node::msg::WheelSpeedEvent>::SharedPtr wheelSpeed_sub_;
    rclcpp::Subscription<polyx_node::msg::StaticHeadingEvent>::SharedPtr  staticHeading_sub_;
    rclcpp::Subscription<polyx_node::msg::StaticGeoPoseEvent>::SharedPtr  staticGeoPose_sub_;
    
    // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    // void topic_callback(const std_msgs::msg::String::SharedPtr msg);
    rclcpp::TimerBase::SharedPtr timer_;
	
	enum DecodeStatus
    {
       _SYNC = 0,
       _HEAD,
       _MSG
    };
	
    DecodeStatus decode_status_ = _SYNC;
	
    uint8_t buf[MaxMsgLen];
    int bufpos = 0;
    int msglen = 0;

    void getTimeStamp(const double t, builtin_interfaces::msg::Time& stamp);
    void parseAttitudeImu(uint8_t* buf, sensor_msgs::msg::Imu& imu);
    void parseCompactNav(
       uint8_t*          buf, 
       polyx_node::msg::CompactNav& msg,
       polyx::ref_frame_trans_type  frame_trans);
};

#endif