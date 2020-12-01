/*
 * Copyright (C) 2020, PolyExplore Inc.
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