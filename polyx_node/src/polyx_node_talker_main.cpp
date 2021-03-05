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
 
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "polyx_node_talker.hpp"

using namespace std::chrono_literals;

void intHandler(int) 
{
   printf("polyx_node_talker: Ctrl-C hit, will stop!\r\n");
}

void abortHandler(int) 
{
   printf("polyx_node_talker: Asked to Abort, will stop!\r\n");
}

void segHandler(int) {
   printf("polyx_node_talker: Segment Fault\r\n");
}

void ioHandler(int) {
   printf("polyx_node_talker: Received SIGIO\r\n");
}


int main(int argc, char * argv[])
{

    struct sigaction int_act;
    int_act.sa_handler = intHandler;
    sigaction(SIGINT, &int_act, NULL);

    struct sigaction abort_act;
    abort_act.sa_handler = abortHandler;
    sigaction(SIGABRT, &abort_act, NULL);

    struct sigaction seg_act;
    seg_act.sa_handler = segHandler;
    sigaction(SIGILL, &seg_act, NULL);

    struct sigaction io_act;
    io_act.sa_handler = ioHandler;
    sigaction(SIGIO, &io_act, NULL);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PolyxnodeTalker>());
    rclcpp::shutdown();
    return 0;
}