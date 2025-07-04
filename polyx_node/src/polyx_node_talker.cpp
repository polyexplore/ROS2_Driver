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

#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h> /* Error number definitions */
#include <unistd.h>
#include <string.h>
#include <termios.h> /* POSIX terminal control definitions */
#include <time.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#include "polyx_convert.h"
#include "polyx_nmea.h"

#include "polyx_node_talker.hpp"

#define MON_PORT "/dev/ttyUSB1"
#define ICD_SYNC1 0xAF
#define ICD_SYNC2 0x20
#define ICD_TYPE 5

#define SERIAL_WAIT (1000000 / 46080)
#define SERIAL_TIMEOUT 10 // seconds
#define SERIAL_BAUD 230400
#define MASK_SIG_UART 1

#define OUT_COMPACTNAV 0x01
#define OUT_POSE 0x02
#define OUT_TWIST 0x04
#define OUT_ACCEL 0x08
#define OUT_NAVSATFIX 0x10
#define OUT_IMU 0x20
#define OUT_GEOPOSE 0x40
#define OUT_EULER_ATT 0x80

#define OUT_ALL 0x7FFFFFFF

// global ethernet port
bool eth_enable = false;
int sockfd = -1;

// global serial port file descriptor
int fd_mon = -1;

std::string my_port;
int my_baud;

int my_output; // ROS2 Node class, my_output has to be global

struct polyx::origin_type myorigin;
bool is_origin_set = false;
polyx::ref_frame_trans_type msg13_frame_trans = polyx::ref_frame_trans_type::NO_FRAME_TRANS;

polyx_node::msg::Kalman kalmsg;
polyx_node::msg::GnssHmr ghmsg;
polyx_node::msg::RawIMU imsg;
polyx_node::msg::SolutionStatus smsg;
polyx_node::msg::CompactNav compactNavMsg;
polyx_node::msg::EulerAttitude qtemsg;
polyx_node::msg::TimeSync tsmsg;
polyx_node::msg::Geoid gmsg;
polyx_node::msg::CorrectedIMU cimsg;
polyx_node::msg::LeapSeconds lsmsg;
polyx_node::msg::Dmi dmi_msg;

int new_output;

using namespace polyx;
using namespace std::chrono_literals;
using std::placeholders::_1;

int write_port(int fd, void *buf, int len);
int read_port(int fd, uint8_t *buf, size_t max_len, struct timeval *tout);
int open_mon(const char *port, int baud);
int connectEthernet(const char *str_server, const char *str_port);
int read_serail(void);

PolyxnodeTalker::PolyxnodeTalker() : Node("polyx_node_talker")
{
    this->init();

    tsmsg.system_computer_time = 0;

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(5), std::bind(&PolyxnodeTalker::execute, this));
}

void PolyxnodeTalker::init()
{
    // get parameters
    //  parameters need to be declared before any attempt to get_parameter()
    // could be redundant, since default values could be set twice, see get_parameter_or()
    this->declare_parameter<bool>("eth_enable", false);
    this->declare_parameter<std::string>("eth_server", "192.168.230.97");
    this->declare_parameter<std::string>("eth_port", "2100");

    // eth_enable, eth_server, eth_port for ethernet mode, added in ROS2 NodeClass
    if (this->get_parameter("eth_enable", eth_enable))
    {
        RCLCPP_INFO(this->get_logger(), "ethernet is %s", eth_enable ? "On" : "Off");
    }
    else
        eth_enable = false;

    std::string eth_server;
    if (this->get_parameter("eth_server", eth_server))
    {
        RCLCPP_INFO(this->get_logger(), "eth_server = %s", eth_server.c_str());
    }
    else
        eth_server = "192.168.230.97";

    std::string eth_port;
    if (this->get_parameter("eth_port", eth_port))
    {
        RCLCPP_INFO(this->get_logger(), "eth_port = %s", eth_port.c_str());
    }
    else
        eth_port = "2100";

    if (eth_enable)
    {
        if (connectEthernet(eth_server.c_str(), eth_port.c_str()) < 0)
        {
            RCLCPP_FATAL(this->get_logger(), "connectEthernet Failure!\n");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "connectEthernet Success!\n");
        }
    }
    else
        RCLCPP_FATAL(this->get_logger(), "ETHERNET DISABLED\n");

    this->declare_parameter<std::string>("polyx_port", "");
    this->declare_parameter<int>("polyx_baud", 0);
    this->declare_parameter<int>("polyx_output", 0);
    if (this->get_parameter("polyx_port", my_port))
    {
        RCLCPP_INFO(this->get_logger(), "serial port=%s", my_port.c_str());
    }
    else
        my_port = MON_PORT;

    if (this->get_parameter("polyx_baud", my_baud))
    {
        RCLCPP_INFO(this->get_logger(), "serial baud=%d", my_baud);
    }
    else
        my_baud = SERIAL_BAUD;

    if (this->get_parameter("polyx_output", my_output))
    {
        RCLCPP_INFO(this->get_logger(), "output msgs=%d", my_output);
    }
    else
        my_output = OUT_COMPACTNAV | OUT_GEOPOSE | OUT_TWIST | OUT_ACCEL | OUT_NAVSATFIX | OUT_IMU | OUT_EULER_ATT | OUT_POSE;

    kalman_pub_ = this->create_publisher<polyx_node::msg::Kalman>("polyx_Kalman", 2);
    gnssHmr_pub_ = this->create_publisher<polyx_node::msg::GnssHmr>("polyx_gnssHmr", 2);
    RawIMU_pub_ = this->create_publisher<polyx_node::msg::RawIMU>("polyx_rawIMU", 2);
    SolutionStatus_pub_ = this->create_publisher<polyx_node::msg::SolutionStatus>("polyx_solutionStatus", 2);
    compactNav_pub_ = this->create_publisher<polyx_node::msg::CompactNav>("polyx_compactNav", 2);
#ifdef ENABLED_GEO_POSE_STAMPED
    geopose_pub_ = this->create_publisher<geographic_msgs::msg::GeoPoseStamped>("current_geopose", 2);
#endif
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 2);
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("current_velocity", 2);
    accel_pub_ = this->create_publisher<geometry_msgs::msg::AccelStamped>("current_acceleration", 2);
    navfix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("current_navsatfix", 2);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("current_imu", 2);
    attitude_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("attitude_imu", 2);
    EulerAttitude_pub_ = this->create_publisher<polyx_node::msg::EulerAttitude>("polyx_EulerAttitude", 2);
    timeSync_pub_ = this->create_publisher<polyx_node::msg::TimeSync>("polyx_timeSync", 2);
    geoid_pub_ = this->create_publisher<polyx_node::msg::Geoid>("polyx_Geoid", 2);
    CorrectedIMU_pub_ = this->create_publisher<polyx_node::msg::CorrectedIMU>("polyx_correctedIMU", 2);
    leapSeconds_pub_ = this->create_publisher<polyx_node::msg::LeapSeconds>("polyx_leapSeconds", 2);
    nmeaGGA_pub_ = this->create_publisher<polyx_node::msg::NmeaGGA>("polyx_nmeaGGA", 2);
    nmeaGGA2_pub_ = this->create_publisher<polyx_node::msg::NmeaGGA>("polyx_nmeaGGA2", 2);
    dmi_pub_ = this->create_publisher<polyx_node::msg::Dmi>("polyx_dmi", 2);

    // subscription_ = this->create_subscription<std_msgs::msg::String>(
    //    "topic", 10, std::bind(&PolyxnodeTalker::topic_callback, this, _1));
    RCLCPP_INFO(this->get_logger(), "Talker Initialized!");
}

void PolyxnodeTalker::execute()
{
    int num;
    uint8_t recvbuf[MaxMsgLen];

    // check output msg control changed or not
    if (this->get_parameter("polyx_output", new_output))
    {
        if (new_output != my_output)
        {
            RCLCPP_INFO(this->get_logger(), "changed otput msgs=%d", new_output);
            my_output = new_output;
        }
    }

    // first check if we need reopen the serial port
    if (eth_enable)
    {
        if (sockfd > 0)
        {
            // printf("Ethernet Connected\n");
            num = read(sockfd, recvbuf, MaxMsgLen);
            // printf("Write socket message to buff\n");
        }
    }
    else
    {
        num = read_serail();
    }

    if (num > 0)
    {
        int i;
        for (i = 0; i < num; i++)
        {
            uint8_t ch = recvbuf[i];
            switch (decode_status_)
            {
            case _SYNC:
                buf[0] = buf[1];
                buf[1] = ch;

                if ((buf[0] == ICD_SYNC1 && buf[1] == ICD_SYNC2) || (buf[0] == '$' && buf[1] == 'G'))
                {
                    decode_status_ = _HEAD;
                    bufpos = 2;
                }

                break;

            case _HEAD:

                buf[bufpos++] = ch;

                if (buf[0] == '$')
                {
                    if (ch < 32 || 126 < ch || bufpos >= MaxMsgLen)
                    {
                        decode_status_ = _SYNC;
                    }
                    else if (ch == '*')
                    {
                        decode_status_ = _MSG;
                        msglen = bufpos + 2;
                    }
                }
                else if (bufpos == 6)
                {
                    msglen = buf[5];
                    msglen = (msglen << 8) + buf[4];
                    if (msglen > MaxMsgLen)
                    {
                        printf("Invalid message length length=%d\n", msglen);
                        decode_status_ = _SYNC;
                        continue;
                    }
                    else
                        decode_status_ = _MSG;
                }
                break;
            case _MSG:

                buf[bufpos++] = ch;

                if (buf[0] == '$')
                {
                    if (bufpos >= msglen)
                    {
                        decode_status_ = _SYNC;
                        buf[bufpos] = '\0';

                        if (nmeaChecksum((char *)buf))
                        {
                            if (strncmp((char *)&buf[3], "GGA,", 4) == 0)
                            {
                                polyx_node::msg::NmeaGGA gga;

                                parseNmeaGga((char *)buf, gga);

                                nmeaGGA_pub_->publish(gga);
                            }
                            else if (strncmp((char *)&buf[3], "GGA2,", 5) == 0)
                            {
                                polyx_node::msg::NmeaGGA gga2;

                                parseNmeaGga((char *)buf, gga2);

                                nmeaGGA2_pub_->publish(gga2);
                            }
                        }
                        else
                            printf("NMEA chsecksum failure.\n");
                    }
                }
                else if (bufpos == (msglen + 8)) // got full message
                {
                    decode_status_ = _SYNC;
                    if (checkMessageType(buf) == ICD_TYPE)
                    {
                        switch (buf[3])
                        {
                        case 1:
                            parse_Kalman_message(buf, kalmsg);
                            kalman_pub_->publish(kalmsg);

                            break;
                        case 8:
                            parse_RawIMU_message(buf, imsg);
                            getTimeStamp(imsg.system_time, imsg.header.stamp);
                            RawIMU_pub_->publish(imsg);
                            break;
                        case 9:
                            parse_SolutionStatus_message(buf, smsg);
                            SolutionStatus_pub_->publish(smsg);
                            break;

                        case 12: // DMI message
                            parse_dmi_message(buf, dmi_msg);
                            getTimeStamp(dmi_msg.system_time, dmi_msg.header.stamp);
                            dmi_pub_->publish(dmi_msg);
                            break;

                        case 13:
                            parseCompactNav(buf, compactNavMsg, msg13_frame_trans);

                            if (my_output & OUT_COMPACTNAV)
                                compactNav_pub_->publish(compactNavMsg);

#ifdef ENABLED_GEO_POSE_STAMPED
                            if (my_output & OUT_GEOPOSE)
                            {
                                geographic_msgs::msg::GeoPoseStamped pmsg;
                                icd_to_GeoPoseStamped(compactNavMsg, pmsg);
                                geopose_pub_->publish(pmsg);
                            }
#endif
                            if (my_output & OUT_TWIST)
                            {
                                geometry_msgs::msg::TwistStamped tmsg;
                                icd_to_TwistStamped(compactNavMsg, tmsg);
                                twist_pub_->publish(tmsg);
                            }
                            if (my_output & OUT_ACCEL)
                            {
                                geometry_msgs::msg::AccelStamped amsg;
                                icd_to_AccelStamped(compactNavMsg, amsg);
                                accel_pub_->publish(amsg);
                            }
                            if (my_output & OUT_NAVSATFIX)
                            {
                                sensor_msgs::msg::NavSatFix nmsg;
                                icd_to_NavSatFix(compactNavMsg, nmsg);
                                navfix_pub_->publish(nmsg);
                            }

                            if (my_output & OUT_EULER_ATT)
                            {
                                if (EulerAttitude(compactNavMsg, qtemsg))
                                {
                                    EulerAttitude_pub_->publish(qtemsg);
                                }
                            }
                            if (my_output & OUT_IMU)
                            {
                                sensor_msgs::msg::Imu imsg;
                                icd_to_Imu(compactNavMsg, imsg);
                                imu_pub_->publish(imsg);
                            }
                            if (!is_origin_set)
                            {
                                SetOrigin(compactNavMsg, myorigin);
                                is_origin_set = true;
                            }

                            if (my_output & OUT_POSE)
                            {
                                geometry_msgs::msg::PoseStamped pmsg;
                                icd_to_PoseStamped(compactNavMsg, myorigin, pmsg);
                                pose_pub_->publish(pmsg);
                            }

                            break;

                        case 14:
                            parse_GnssHmr_message(buf, ghmsg);
                            gnssHmr_pub_->publish(ghmsg);
                            break;

                        case 16:
                            parse_TimeSync_message(buf, tsmsg);
                            timeSync_pub_->publish(tsmsg);
                            break;

                        case 22:
                            parse_Geoid_message(buf, gmsg);
                            geoid_pub_->publish(gmsg);
                            break;

                        case 23:
                            parse_CorrectedIMU_message(buf, cimsg);
                            CorrectedIMU_pub_->publish(cimsg);
                            break;

                        case 24:
                            parse_LeapSeconds_message(buf, lsmsg);
                            leapSeconds_pub_->publish(lsmsg);
                            break;

                        case 59: // Attitude and IMU
                            if (my_output & OUT_IMU)
                            {
                                sensor_msgs::msg::Imu imsg;
                                parseAttitudeImu(buf, imsg);
                                attitude_imu_pub_->publish(imsg);
                            }
                            break;

                        default:
                            break;
                        } // end switch (buf[3])
                    } // end if checkMessageType
                } // end if buffpos
                if (bufpos >= MaxMsgLen)
                {
                    printf("message overflow length length=%d\n", msglen);
                    decode_status_ = _SYNC;
                    continue;
                }

                break;
            default:
                decode_status_ = _SYNC;
            } // end switch(decode_status_)
        } // end for
    } // end if (num > 0)}
}

PolyxnodeTalker::~PolyxnodeTalker()
{
    if (fd_mon >= 0)
        close(fd_mon);
}

int write_port(int fd, void *buf, int len)
{
    struct timeval tout;
    // default uart read timeout
    tout.tv_sec = 0;
    tout.tv_usec = 500 * 1000;

    fd_set outputs;
    int num, ret;

#ifdef MASK_SIG_UART
    sigset_t mask;
    sigset_t orig_mask;

    sigemptyset(&mask);
    sigaddset(&mask, SIGIO);
    sigaddset(&mask, SIGINT);
    sigaddset(&mask, SIGABRT);

    if (sigprocmask(SIG_BLOCK, &mask, &orig_mask) < 0)
    {
        perror("sigprocmask");
        return -1;
    }
#endif

    num = 0;

    FD_ZERO(&outputs);
    FD_SET(fd, &outputs);

    ret = select(fd + 1, (fd_set *)NULL, &outputs, (fd_set *)NULL, &tout);
    if (ret < 0)
    {
        perror("select error!!");
    }
    else if (ret > 0)
    {
        if (FD_ISSET(fd, &outputs))
        {
            num = write(fd, buf, len);
        }
    }

#ifdef MASK_SIG_UART
    if (sigprocmask(SIG_SETMASK, &orig_mask, NULL) < 0)
    {
    }
#endif

    return num;
}

void PolyxnodeTalker::getTimeStamp(const double t, builtin_interfaces::msg::Time &stamp)
{
    if (INI_GPS_WEEK > 0 && tsmsg.system_computer_time)
    {
        double t_gps = t - tsmsg.bias_to_gps_time;
        GpsToEpoch(INI_GPS_WEEK, t_gps, stamp);
    }
    else
        stamp = this->get_clock()->now();
}

void PolyxnodeTalker::parseAttitudeImu(uint8_t *buf, sensor_msgs::msg::Imu &imu)
{
    uint16_t week;
    double time;
    float att_rms[3];
    double q[4];

    int i = 6;

    imu.header.frame_id = "imu_link_ned";

    polyx::Decode(&buf[i], time);
    i += 8;

    // Quaternion from body to NED
    for (int k = 0; k < 4; ++k, i += 8)
        polyx::Decode(&buf[i], q[k]);

    imu.orientation.x = q[0];
    imu.orientation.y = q[1];
    imu.orientation.z = q[2];
    imu.orientation.w = q[3];

    for (int k = 0; k < 3; ++k, i += 4)
        polyx::Decode(&buf[i], att_rms[k]);

    polyx::AssignDiagCov3(att_rms, imu.orientation_covariance);

    // Rotation rate in body frame (rad/s)
    for (int k = 0; k < 3; ++k, i += 8)
        polyx::Decode(&buf[i], q[k]);

    imu.angular_velocity.x = q[0];
    imu.angular_velocity.y = q[1];
    imu.angular_velocity.z = q[2];

    // Unknown angular velocity covariance
    imu.angular_velocity_covariance[0] = -1.0;

    // Acceleration (m/s^2)
    for (int k = 0; k < 3; ++k, i += 8)
        polyx::Decode(&buf[i], q[k]);

    imu.linear_acceleration.x = q[0];
    imu.linear_acceleration.y = q[1];
    imu.linear_acceleration.z = q[2];

    // Unknown acceleration covariance
    imu.linear_acceleration_covariance[0] = -1.0;

    polyx::Decode(&buf[i], week);
    i += 2;

    // Assign timestamp
    if (week == 0xffff)
        imu.header.stamp = this->get_clock()->now();
    else
        GpsToEpoch(week, time, imu.header.stamp);
}

void PolyxnodeTalker::parseCompactNav(
    uint8_t *buf,
    polyx_node::msg::CompactNav &msg,
    polyx::ref_frame_trans_type frame_trans)
{
    int p = 6, i;
    float temp;
    Decode(&buf[p], msg.gps_time_week);
    p += 8; // double
    Decode(&buf[p], msg.latitude);
    p += 8;                     // double [deg]
    msg.latitude *= DEG_TO_RAD; // double [rad]
    Decode(&buf[p], msg.longitude);
    p += 8;                      // double [deg]
    msg.longitude *= DEG_TO_RAD; // double [rad]
    Decode(&buf[p], temp);
    p += 4; // internally as float[m], but msg.longitude is double
    msg.altitude = temp;
    for (i = 0; i < 3; i++)
    {
        Decode(&buf[p], temp);
        p += 4;                     // internally float
        msg.velocity_ned[i] = temp; // float to double implicitly
    }
    for (i = 0; i < 4; i++)
    {
        Decode(&buf[p], temp);
        p += 4; // float
        msg.quaternion[i] = temp;
    }
    for (i = 0; i < 3; i++)
    {
        Decode(&buf[p], temp);
        p += 4; // float
        msg.acceleration[i] = temp;
    }
    for (i = 0; i < 3; i++)
    {
        Decode(&buf[p], temp);
        p += 4;                                   // float [deg/s]
        msg.rotation_rate[i] = temp * DEG_TO_RAD; // to double[rad/s]
    }
    for (i = 0; i < 3; i++)
    {
        Decode(&buf[p], temp);
        p += 4; // float
        msg.position_rms[i] = temp;
    }
    for (i = 0; i < 3; i++)
    {
        Decode(&buf[p], temp);
        p += 4; // float
        msg.velocity_rms[i] = temp;
    }
    for (i = 0; i < 3; i++)
    {
        Decode(&buf[p], temp);
        p += 4;                                  // float [deg]
        msg.attitude_rms[i] = temp * DEG_TO_RAD; // double [rad]
    }
    Decode(&buf[p], msg.gps_week_number);
    p += 2; // uint16
    msg.alignment = buf[p];

    if (INI_GPS_WEEK < 0 && msg.gps_week_number > 0)
        INI_GPS_WEEK = msg.gps_week_number;

    if (frame_trans == ref_frame_trans_type::WGS84_TO_NAD83)
    {
        ConvertToNAD83(msg.gps_week_number, msg.gps_time_week, msg.latitude, msg.longitude, msg.altitude);
    }

    if (msg.gps_week_number == 0xffff)
        msg.header.stamp = this->get_clock()->now();
    else
        GpsToEpoch(msg.gps_week_number, msg.gps_time_week, msg.header.stamp);
}

int read_port(int fd, uint8_t *buf, size_t max_len, struct timeval *tout)
{
    fd_set inputs;
    int num, ret;

    num = 0;

    FD_ZERO(&inputs);
    FD_SET(fd, &inputs);

    ret = select(fd + 1, &inputs, (fd_set *)NULL, (fd_set *)NULL, tout);

    if (ret < 0)
    {
        perror("select error!!");
    }
    else if (ret > 0)
    {
        if (FD_ISSET(fd, &inputs))
        {
            size_t len = 0;
            ioctl(fd, FIONREAD, &len);
            if (len == 0)
            {
                return -EIO;
            }
            num = read(fd, buf, (len < max_len) ? len : max_len);
        }
    }

    return num;
}

int open_mon(const char *port, int baud)
{
    struct termios options; // Structure with the device's options
    int fd;
    // Open device
    fd = open(port, O_RDWR | O_NONBLOCK | O_NOCTTY); // Open port
    if (fd < 0)
    {
        perror("serial open");
        return fd;
    }
    tcgetattr(fd, &options);          // Get the current options of the port
    bzero(&options, sizeof(options)); // Clear all the options
    speed_t Speed;

    switch (baud) // Set the speed (Bauds)
    {
    case 110:
        Speed = B110;
        break;
    case 300:
        Speed = B300;
        break;
    case 600:
        Speed = B600;
        break;
    case 1200:
        Speed = B1200;
        break;
    case 2400:
        Speed = B2400;
        break;
    case 4800:
        Speed = B4800;
        break;
    case 9600:
        Speed = B9600;
        break;
    case 19200:
        Speed = B19200;
        break;
    case 38400:
        Speed = B38400;
        break;
    case 57600:
        Speed = B57600;
        break;
    case 115200:
        Speed = B115200;
        break;
    case 230400:
        Speed = B230400;
        break;
    case 460800:
        Speed = B460800;
        break;
    case 921600:
        Speed = B921600;
        break;
    default:
        Speed = baud;
    }
    cfsetispeed(&options, Speed); // Set the baud rate at 115200 bauds
    cfsetospeed(&options, Speed);

    options.c_cflag |= (CLOCAL | CREAD | CS8); // Configure the device : 8 bits, no parity, no control
    options.c_iflag |= (IGNPAR | IGNBRK);
    options.c_cc[VTIME] = 0; // Timer unused
    options.c_cc[VMIN] = 0;  // At least on character before satisfy reading

    options.c_cflag |= CREAD | CLOCAL;                  // turn on READ & ignore ctrl lines
    options.c_iflag &= ~(IXON | IXOFF | IXANY);         // turn off s/w flow ctrl
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    options.c_oflag &= ~OPOST;                          // make raw

    tcsetattr(fd, TCSANOW, &options); // Activate the settings

    return fd; // successful
}

int connectEthernet(const char *str_server, const char *str_port)
{
    int portno;
    int res;
    long arg;
    fd_set myset;
    struct timeval tv;
    int valopt;
    socklen_t lon;

    struct sockaddr_in serv_addr;
    struct hostent *server;

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    // printf("sockfd is %d\n", sockfd);
    if (sockfd < 0)
    {
        printf("ERROR opening socket.\n");
        return -1;
    }

    // Set non-blocking
    arg = fcntl(sockfd, F_GETFL, NULL);
    arg |= O_NONBLOCK;
    fcntl(sockfd, F_SETFL, arg);

    server = gethostbyname(str_server);
    bzero((char *)&serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;

    portno = atoi(str_port);
    serv_addr.sin_port = htons(portno);

    bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);

    res = connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
    if (res < 0)
    {
        if (errno == EINPROGRESS)
        {
            tv.tv_sec = 10;
            tv.tv_usec = 0;
            FD_ZERO(&myset);
            FD_SET(sockfd, &myset);
            if (select(sockfd + 1, NULL, &myset, NULL, &tv) > 0)
            {
                lon = sizeof(int);
                getsockopt(sockfd, SOL_SOCKET, SO_ERROR, (void *)(&valopt), &lon);
                if (valopt)
                {
                    fprintf(stderr, "Error in connection() %d - %s\n", valopt, strerror(valopt));
                    return -2;
                }
            }
            else
            {
                fprintf(stderr, "Timeout or error() %d - %s\n", valopt, strerror(valopt));
                return -2;
            }
        }
        else
        {
            fprintf(stderr, "Error connecting %d - %s\n", errno, strerror(errno));
            return -2;
        }
    }
    // Set to blocking mode again...
    // arg = fcntl(sockfd, F_GETFL, NULL);
    // arg &= (~O_NONBLOCK);
    // fcntl(sockfd, F_SETFL, arg);

    return 0;
}

int read_serail(void)
{
    uint8_t recvbuf[PolyxnodeTalker::MaxMsgLen];

    int num = 0;
    struct timeval tout;
    // default uart read timeout
    tout.tv_sec = 1;
    tout.tv_usec = 100 * 1000;

#ifdef MASK_SIG_UART
    sigset_t mask;
    sigset_t orig_mask;

    sigemptyset(&mask);
    sigaddset(&mask, SIGIO);
    sigaddset(&mask, SIGINT);
    sigaddset(&mask, SIGABRT);

    if (sigprocmask(SIG_BLOCK, &mask, &orig_mask) < 0)
    {
        perror("sigprocmask");
        return -1;
    }
#endif

    if (fd_mon < 0)
    {
        fd_mon = open_mon(my_port.c_str(), my_baud);
        if (fd_mon < 0)
        {
            perror("open serial port error");
            usleep(200 * 1000); // delay 200ms
        }
    }
    else
    {
        num = read_port(fd_mon, recvbuf, PolyxnodeTalker::MaxMsgLen, &tout);
        if (num < 0)
        {
            if ((errno != EAGAIN) && (errno != EWOULDBLOCK))
            {
                perror("read serial port error");
                // depends on the type or error, do we need re-open the serial port?
                close(fd_mon);
                fd_mon = -1;
            }
        }
    }

#ifdef MASK_SIG_UART
    if (sigprocmask(SIG_SETMASK, &orig_mask, NULL) < 0)
    {
    }
#endif

    return num;
}
