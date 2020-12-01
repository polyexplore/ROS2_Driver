/*
 * Copyright (C) 2020, PolyExplore Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
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
 
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "polyx_node/msg/static_geo_pose_event.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


class StaticGeoPoseEventNode : public rclcpp::Node
{
public:
    StaticGeoPoseEventNode()
    : Node("event_gen","event_gen_ns"), count_(0)
    {
        event = polyx_node::msg::StaticGeoPoseEvent();
		this->declare_parameter<double> ("latitude",         37);	// [deg]
		this->declare_parameter<double> ("longitude",        -121);  // [deg]
		this->declare_parameter<float>  ("ellipsoidal_height", -10); // [m]
		this->declare_parameter<int16_t>("roll",             0);		// [0.01 deg]
		this->declare_parameter<int16_t>("pitch",            0); 	// [0.01 deg]
		this->declare_parameter<int16_t>("heading",          3000); 	// [0.01 deg]
		this->declare_parameter<uint16_t>("position_rms",     200);	// [0.01 m]
		this->declare_parameter<uint16_t>("zupt_rms",         10);	// [0.001 m/s]	
		this->declare_parameter<uint16_t>("heading_rms",       100);   // [0.1 deg]
		this->declare_parameter<uint16_t>("flags",            0);  	// 0

        publisher_ = this->create_publisher<polyx_node::msg::StaticGeoPoseEvent>("polyx_StaticGeoPose", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&StaticGeoPoseEventNode::timer_callback, this));
    }

private:
    void timer_callback()
    {

        this->get_parameter("latitude",             event.latitude);
        this->get_parameter("longitude",            event.longitude);
        this->get_parameter("ellipsoidal_height",   event.ellipsoidal_height);
        this->get_parameter("roll",                 event.roll);
        this->get_parameter("pitch",                event.pitch);
        this->get_parameter("heading",              event.heading);
        this->get_parameter("position_rms",         event.position_rms);
        this->get_parameter("zupt_rms",             event.zupt_rms);
        this->get_parameter("heading_rms",          event.heading_rms);
        this->get_parameter("flags",                event.flags);

        RCLCPP_INFO(this->get_logger(), "Sent a StaticGeoPose message, count = %d\n", ++count_);
        RCLCPP_INFO(this->get_logger(), "Latitude: %.9lf deg", event.latitude);
        RCLCPP_INFO(this->get_logger(), "Longitude: %.9lf deg",  event.longitude);
        RCLCPP_INFO(this->get_logger(), "EllipsoidalHeight: %.4lf m",  event.ellipsoidal_height);
        RCLCPP_INFO(this->get_logger(), "Roll: %.4lf deg",  event.roll / 100.);
        RCLCPP_INFO(this->get_logger(), "Pitch: %.4lf deg",  event.pitch / 100.);
        RCLCPP_INFO(this->get_logger(), "Heading: %.4lf deg",  event.heading / 100.);
        RCLCPP_INFO(this->get_logger(), "PositionRMS: %.4lf m",  event.position_rms / 100.);
        RCLCPP_INFO(this->get_logger(), "ZUPTRMS: %.4lf m/s",  event.zupt_rms / 1000. );
        RCLCPP_INFO(this->get_logger(), "HeadingRMS: %.4lf deg",  event.heading_rms / 10. );
        RCLCPP_INFO(this->get_logger(), "Roll & Pitch Valid, disable GNSS: %u\n",  event.flags);

        publisher_->publish(event);
    }


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<polyx_node::msg::StaticGeoPoseEvent>::SharedPtr publisher_;
    size_t count_;
    polyx_node::msg::StaticGeoPoseEvent event;
  };

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticGeoPoseEventNode>());
    rclcpp::shutdown();
    return 0;
}