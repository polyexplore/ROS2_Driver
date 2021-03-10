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

#ifndef _POLYX_CONVERT_H
#define _POLYX_CONVERT_H

//  ROS2: be careful with the header names
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>

// Activate definition below to pub/sub geographic_msgs::msg::GeoPoseStamped
// #define ENABLED_GEO_POSE_STAMPED 
#ifdef ENABLED_GEO_POSE_STAMPED
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#endif

// no need to explicitly include quaternion, since already in pose_stamped
// #include <geometry_msgs/msg/quaternion.hpp>
// #include <builtin_interfaces/msg/time.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "polyx_node/msg/kalman.hpp"
#include "polyx_node/msg/raw_imu.hpp"
#include "polyx_node/msg/solution_status.hpp"
#include "polyx_node/msg/compact_nav.hpp"
#include "polyx_node/msg/euler_attitude.hpp"
#include "polyx_node/msg/time_sync.hpp"
#include "polyx_node/msg/geoid.hpp"
#include "polyx_node/msg/corrected_imu.hpp"
#include "polyx_node/msg/leap_seconds.hpp"
#include "polyx_node/msg/wheel_speed_event.hpp"
#include "polyx_node/msg/static_heading_event.hpp"
#include "polyx_node/msg/static_geo_pose_event.hpp"
#include "polyx_node/msg/dmi.hpp"
#include "polyx_node/msg/nmea_gga.hpp"

#define DEG_TO_RAD (0.017453292519943295)

namespace polyx
{

enum class ref_frame_trans_type
{
   NO_FRAME_TRANS, // No transformation
   WGS84_TO_NAD83  // WGS84->NAD83
};

// moved from polydata.hpp to here
struct origin_type
{
   double r[3];      // ECEF coordinates of the origin (m)
   double Cen[3][3]; // DCM from ECEF to NED
};
//-----------------------------------------------------------------------------
// GPS time to UNIX epoch conversion
//     315964800 + week * 604800 + GPS Time of week
//     where 315964800 is the Unix Epoch of GPS start time (Jan 6, 1980), 
//
void GpsToEpoch(int gpsweek, double gpstow, builtin_interfaces::msg::Time &tm);

//-----------------------------------------------------------------------------
// UNIX epoch to GPS time conversion
void EpochToGps(const builtin_interfaces::msg::Time &tm, int& gpsweek, double& gpstow);

//-----------------------------------------------------------------------------
// Quaternion product: q3 = q1 * q2
void QuatProd(
   const geometry_msgs::msg::Quaternion& q1,
   const geometry_msgs::msg::Quaternion& q2,
   geometry_msgs::msg::Quaternion&       q3);

//-----------------------------------------------------------------------------
// Quaternion for the transformation from NED to ENU
void QuatNED2ENU(geometry_msgs::msg::Quaternion& q);

//-----------------------------------------------------------------------------
// Convert PE CompactNav message to NavSatFix message
void icd_to_NavSatFix(
   polyx_node::msg::CompactNav&       msg, 
   sensor_msgs::msg::NavSatFix& nsf);

//-----------------------------------------------------------------------------
// Convert PE CompactNav message to IMU message
void icd_to_Imu(polyx_node::msg::CompactNav& msg, sensor_msgs::msg::Imu& imu);

//-----------------------------------------------------------------------------
// Convert PE CompactNav message to GeoPoseStamped message
#ifdef ENABLED_GEO_POSE_STAMPED
void icd_to_GeoPoseStamped(
   polyx_node::msg::CompactNav&          msg,
   geographic_msgs::msg::GeoPoseStamped& ps);
#endif

//-----------------------------------------------------------------------------
// Convert PE CompactNav message to TwistStamped message
void icd_to_TwistStamped(
   polyx_node::msg::CompactNav&            msg,
   geometry_msgs::msg::TwistStamped& ts);

//-----------------------------------------------------------------------------
// Convert PE CompactNav message to AccelStamped message
void icd_to_AccelStamped(
   polyx_node::msg::CompactNav&            msg,
   geometry_msgs::msg::AccelStamped& as);

//-----------------------------------------------------------------------------
// Convert Geodetic coordinates to ECEF coordinates.
void GeodeticToECEF(
   const double& lat,
   const double& lon,
   const double& alt,
   double        r[]);

//-----------------------------------------------------------------------------
// Convert ECEF coordinates to Geodetic coordinates.
void ECEFToGeodetic(
   const double r[],
   double&      lat,
   double&      lon,
   double&      alt);

//-----------------------------------------------------------------------------
void DCM_ECEFToNED(const double& lat, const double& lon, double Cen[3][3]);

//-----------------------------------------------------------------------------
// Set origin of pose specifically for customers
void SetCustomOrigin(
   double               latitude,   // radian
   double               longitude,  // radian
   double               altitude,   // meters
   struct origin_type&  org);

//-----------------------------------------------------------------------------
// Set origin for Pose message
void SetOrigin(
   const polyx_node::msg::CompactNav&     msg,
   struct origin_type&         org);

//-----------------------------------------------------------------------------
// Convert PE CompactNav message to PoseStamped message
void icd_to_PoseStamped(
   const polyx_node::msg::CompactNav&     msg,
   const struct origin_type&   org,
   geometry_msgs::msg::PoseStamped& ps);

//-----------------------------------------------------------------------------
// Convert quaternion to Euler angles
bool EulerAttitude(
   polyx_node::msg::CompactNav&           msg, 
   polyx_node::msg::EulerAttitude& qtemsg);

//-----------------------------------------------------------------------------
void ConvertToNAD83(
   const uint16_t& week,
   const double&   tow,
   double&         lat,
   double&         lon,
   double&         alt);
   
//-----------------------------------------------------------------------------
// 3x3 Covariance
template<typename T, typename U> void AssignDiagCov3(const T& rms, U& cov)
{
   // Diagonals
   cov[0] = rms[0] * rms[0];
   cov[4] = rms[1] * rms[1];
   cov[8] = rms[2] * rms[2];

   // Off diagonals

   cov[1] = cov[2] = cov[3] = cov[5] = cov[6] = cov[7] = 0;

}

//-----------------------------------------------------------------------------
// 6x6 Covariance: to be used for pose and twist
template<typename T, typename U> void AssignDiagCov6(const T& rms1, const T& rms2, U& cov)
{
   // initialize to zero
   for (int i = 0; i < 36; ++i)
      cov[i] = 0;

   // Diagonals
   cov[0] = rms1[0] * rms1[0];
   cov[7] = rms1[1] * rms1[1];
   cov[14] = rms1[2] * rms1[2];

   cov[21] = rms2[0] * rms2[0];
   cov[28] = rms2[1] * rms2[1];
   cov[35] = rms2[2] * rms2[2];


}

//-----------------------------------------------------------------------------
// Decoding funtions for little-endian data
void Decode(const uint8_t* p, double& d);
void Decode(const uint8_t* p, int32_t& i32);
void Decode(const uint8_t* p, uint32_t& u32);
void Decode(const uint8_t* p, float& f);
void Decode(const uint8_t* p, int16_t& i16);
void Decode(const uint8_t* p, uint16_t& u16);

//-----------------------------------------------------------------------------
// Encoding funtions to little-endian data

void Encode(const double& d, uint8_t* p);
void Encode(const float& f, uint8_t* p);
void Encode(const uint32_t& u32, uint8_t* p);
void Encode(const int32_t& i32, uint8_t* p);
void Encode(const uint16_t& u16, uint8_t* p);
void Encode(const int16_t& i16, uint8_t* p);

void SetMsgHeader(uint8_t type, uint8_t sub_id, uint16_t payload_len, uint8_t* header);
uint8_t checksum(uint8_t* Buffer, uint16_t len, uint8_t& cka, uint8_t& ckb);

uint8_t checkMessageType(uint8_t *buf);
void parse_Kalman_message(uint8_t *buf, polyx_node::msg::Kalman &kalmsg);
void parse_RawIMU_message(uint8_t *buf, polyx_node::msg::RawIMU &imsg);
void parse_SolutionStatus_message(uint8_t *buf, polyx_node::msg::SolutionStatus &smsg);
void parse_TimeSync_message(uint8_t *buf, polyx_node::msg::TimeSync &tsmsg);
void parse_Geoid_message(uint8_t *buf, polyx_node::msg::Geoid &gmsg);
void parse_CorrectedIMU_message(uint8_t *buf, polyx_node::msg::CorrectedIMU &imsg);
void parse_LeapSeconds_message(uint8_t *buf, polyx_node::msg::LeapSeconds &lsmsg);
void parse_dmi_message(uint8_t *buf, polyx_node::msg::Dmi &dmi);

} //namespace polyx

extern int LEAP_SECONDS;
extern int INI_GPS_WEEK;

#endif // _POLYX_CONVERT_H