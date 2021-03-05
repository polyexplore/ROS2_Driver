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
#include "rclcpp/rclcpp.hpp"
#include "polyx_node/msg/compact_nav.hpp"
#include "polyx_node/msg/wheel_speed_event.hpp"

// node pointer
rclcpp::Node::SharedPtr node = nullptr;

int main(int argc, char **argv)
{

   rclcpp::init(argc, argv);
   node = rclcpp::Node::make_shared("polyx_node_speed");

   int count = 0;

   auto WheelSpeed_pub = node->create_publisher<polyx_node::msg::WheelSpeedEvent>("polyx_WheelSpeed", 1000);

   rclcpp::Rate loop_rate(10);

   polyx_node::msg::WheelSpeedEvent msg;
   
   // Default message for testing
   double duration = 5.0; // seconds
   msg.time = 0;       // seconds
   msg.speed = 0;      // m/s
   msg.speed_rms = 0;  // mm/s
   msg.flags = 0;


   node->declare_parameter<double>("duration", 0.);
   node->declare_parameter<double>("time", 0);
   node->declare_parameter<float>("speed", 0.0f);
   node->declare_parameter<uint16_t>("speed_rms", 50);
   node->declare_parameter<uint8_t>("flags", 0);

   node->get_parameter("duration"   ,duration);
   node->get_parameter("time"       ,msg.time);
   node->get_parameter("speed"      ,msg.speed);
   node->get_parameter("speed_rms"  ,msg.speed_rms);
   node->get_parameter("flags"      ,msg.flags);
   RCLCPP_INFO(node->get_logger(), "duration %lf", duration);
   double start_time = node->get_clock()->now().seconds();
   while (rclcpp::ok())
   {

	   auto current_stamp = node->get_clock()->now(); // rclcpp::Time
      msg.header.stamp = current_stamp; // builtin_interfaces::msg::Time, convert

      if (current_stamp.seconds() - start_time > duration)
         break;

      printf("Sent a Wheel Speed Event Message, count = %d\n", ++count);
      RCLCPP_INFO(node->get_logger(), "Time = %f seconds", msg.time);
      RCLCPP_INFO(node->get_logger(), "Speed = %f m/s", msg.speed);
      RCLCPP_INFO(node->get_logger(), "Speed RMS = %d mm/s", msg.speed_rms);
      RCLCPP_INFO(node->get_logger(), "Flags = %d\n", msg.flags);

      WheelSpeed_pub->publish(msg);

      rclcpp::spin_some(node);
      loop_rate.sleep();
   }

   return 0;
}
// %EndTag(FULLTEXT)%

