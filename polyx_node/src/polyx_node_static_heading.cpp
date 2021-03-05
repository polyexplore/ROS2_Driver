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
#include "polyx_node/msg/static_heading_event.hpp"

// node pointer
rclcpp::Node::SharedPtr node = nullptr;

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	node = rclcpp::Node::make_shared("polyx_node_heading");

   int count = 0;

   auto staticHeading_pub = node->create_publisher<polyx_node::msg::StaticHeadingEvent>("polyx_StaticHeading", 1000);


   polyx_node::msg::StaticHeadingEvent msg;
   msg.heading = -11600; // -116 degrees
   msg.zupt_rms = 10;     // 1 cm/s
   msg.heading_rms = 100; // 10 degrees
   double duration = 5.0; // seconds

   // Default message for testing
   node->declare_parameter<int16_t>("heading", -11600);
   node->declare_parameter<uint16_t>("zupt_rms", 10);
   node->declare_parameter<uint8_t>("heading_rms", 100);
   node->declare_parameter<double>("duration", 5.0);

   node->get_parameter("heading", msg.heading);
   node->get_parameter("zupt_rms", msg.zupt_rms);
   node->get_parameter("heading_rms", msg.heading_rms);
   node->get_parameter("duration", duration);

   rclcpp::Rate loop_rate(100);
   double start_time = node->get_clock()->now().seconds();

   while (rclcpp::ok())
   {

	   auto current_stamp = node->get_clock()->now(); // rclcpp::Time
      msg.header.stamp = current_stamp; // builtin_interfaces::msg::Time, convert

      if (current_stamp.seconds() - start_time > duration)
         break;

      RCLCPP_INFO(node->get_logger(), "Sent a Static Heading Event Message, count = %d\n", ++count);
      RCLCPP_INFO(node->get_logger(), "Heading = %d deg", msg.heading/100);
      RCLCPP_INFO(node->get_logger(), "ZUPT RMS = %d mm/s", msg.zupt_rms);
      RCLCPP_INFO(node->get_logger(), "Heading RMS = %d deg\n", msg.heading_rms/10);

      staticHeading_pub->publish(msg);


      rclcpp::spin_some(node);
      loop_rate.sleep();
   }
   RCLCPP_INFO(node->get_logger(), "Finished!");
   rclcpp::shutdown();
   return 0;
}

