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
 
#include <cmath>       /* sin, cos, sqrt, floor, ceil */
#include <cstring>

#include "polyx_convert.h"

#define DEG_TO_RAD (0.017453292519943295)
#define RAD_TO_DEG (57.295779513082323)
#define WGS84_A    (6378137.0)
#define WGS84_E2   (6.69437999014e-3)
#define GPS_TIME_BEG 315964800.0
#define SEC_PER_WEEK 604800.0
#define MAS_TO_RAD 4.84813681109535940e-09 // milli-arc-sec to rad

int LEAP_SECONDS = 18; // GPS-UTC time offset (s)
int INI_GPS_WEEK = -1; // Initial GPS week number

namespace polyx
{
typedef struct 
{
   double trans[3];
   double rot[3];
   double scale;
} frame_trans_type;

//-----------------------------------------------------------------------------
void  GpsToEpoch(int gps_week, double gps_tow, builtin_interfaces::msg::Time &tm)
{
   double t;
   
   if (gps_week > 0)
      t = GPS_TIME_BEG + gps_week * SEC_PER_WEEK + gps_tow - LEAP_SECONDS;
   else
      t = gps_tow;

   tm.sec = floor(t);
   tm.nanosec = floor((t - tm.sec)*1.0e+9);
}

//-----------------------------------------------------------------------------
void EpochToGps(
   const builtin_interfaces::msg::Time& tm, 
   int&             gps_week, 
   double&          gps_tow)
{
   double t = tm.sec - GPS_TIME_BEG + tm.nanosec * 1.0e-9 + LEAP_SECONDS;

   gps_week = floor(t / SEC_PER_WEEK);
   gps_tow = t - (gps_week * SEC_PER_WEEK);
}

//-----------------------------------------------------------------------------
// Quaternion product: q3 = q1 * q2
void QuatProd(
   const geometry_msgs::msg::Quaternion& q1,
   const geometry_msgs::msg::Quaternion& q2,
   geometry_msgs::msg::Quaternion&       q3)
{
   q3.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
   q3.y = q1.w*q2.y + q1.y*q2.w + q1.z*q2.x - q1.x*q2.z;
   q3.z = q1.w*q2.z + q1.z*q2.w + q1.x*q2.y - q1.y*q2.x;
   q3.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
}

//-----------------------------------------------------------------------------
// Quaternion for the transformation from NED to ENU
#ifndef M_SQRT1_2
#define M_SQRT1_2 (0.70710678118654746)
#endif
void QuatNED2ENU(geometry_msgs::msg::Quaternion& q)
{
   q.x = M_SQRT1_2;
   q.y = M_SQRT1_2;
   q.z = 0;
   q.w = 0;
}

//-----------------------------------------------------------------------------
// Convert PE CompactNav message to NavSatFix message
void icd_to_NavSatFix(
   polyx_node::msg::CompactNav& msg,
   sensor_msgs::msg::NavSatFix& nsf)
{
   nsf.header.stamp = msg.header.stamp;

   nsf.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
   nsf.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS |
      sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;

   nsf.latitude = msg.latitude * RAD_TO_DEG;
   nsf.longitude = msg.longitude * RAD_TO_DEG;
   nsf.altitude = msg.altitude;

   nsf.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

   // Position RMS assignment

   AssignDiagCov3(msg.position_rms, nsf.position_covariance);

}

//-----------------------------------------------------------------------------
// Convert PE CompactNav message to IMU message
void icd_to_Imu(polyx_node::msg::CompactNav &msg, sensor_msgs::msg::Imu &imu)
{
   imu.header.stamp = msg.header.stamp;

   // Quaternion from body to NED
   imu.orientation.x = msg.quaternion[1];
   imu.orientation.y = msg.quaternion[2];
   imu.orientation.z = msg.quaternion[3];
   imu.orientation.w = msg.quaternion[0];

   // Rotation rate in body frame
   imu.angular_velocity.x = msg.rotation_rate[0];
   imu.angular_velocity.y = msg.rotation_rate[1];
   imu.angular_velocity.z = msg.rotation_rate[2];

   AssignDiagCov3(msg.attitude_rms, imu.orientation_covariance);

   imu.linear_acceleration.x = msg.acceleration[0]; // forward
   imu.linear_acceleration.y = msg.acceleration[1]; // right
   imu.linear_acceleration.z = msg.acceleration[2]; // down

  // Unknown acceleration covariance so set to zero

}

//-----------------------------------------------------------------------------
// Convert PE CompactNav message to GeoPoseStamped message
#ifdef ENABLED_GEO_POSE_STAMPED
void icd_to_GeoPoseStamped(
   polyx_node::msg::CompactNav&          msg,
   geographic_msgs::msg::GeoPoseStamped& ps)
{
   geometry_msgs::msg::Quaternion q1;
   geometry_msgs::msg::Quaternion q2;

   QuatNED2ENU(q1);

   q2.x = msg.quaternion[1];
   q2.y = msg.quaternion[2];
   q2.z = msg.quaternion[3];
   q2.w = msg.quaternion[0];

   ps.header.stamp = msg.header.stamp;

   ps.pose.position.latitude = msg.latitude * RAD_TO_DEG;   // Latitude (deg)
   ps.pose.position.longitude = msg.longitude * RAD_TO_DEG; // Longitude (deg)
   ps.pose.position.altitude = msg.altitude;  // Ellipsoidal height (m)

   // Get quaternion from the body to ENU frame

   QuatProd(q1, q2, ps.pose.orientation);

}
#endif

//-----------------------------------------------------------------------------
// Convert PE CompactNav message to TwistStamped message
void icd_to_TwistStamped(
   polyx_node::msg::CompactNav&      msg,
   geometry_msgs::msg::TwistStamped& ts)
{
   ts.header.stamp = msg.header.stamp;

   // Velocity and angular in NED

   ts.twist.linear.x = msg.velocity_ned[0];
   ts.twist.linear.y = msg.velocity_ned[1];
   ts.twist.linear.z = msg.velocity_ned[2];

   ts.twist.angular.x = msg.rotation_rate[0];
   ts.twist.angular.y = msg.rotation_rate[1];
   ts.twist.angular.z = msg.rotation_rate[2];

}

//-----------------------------------------------------------------------------
// Convert PE CompactNav message to AccelStamped message
void icd_to_AccelStamped(
   polyx_node::msg::CompactNav&      msg,
   geometry_msgs::msg::AccelStamped& as)
{
   as.header.stamp = msg.header.stamp;

   as.accel.linear.x = msg.acceleration[0];
   as.accel.linear.y = msg.acceleration[1];
   as.accel.linear.z = msg.acceleration[2];

   // Angular acceleration is not known
}
//-----------------------------------------------------------------------------
void GeodeticToECEF(
   const double& lat,
   const double& lon,
   const double& alt,
   double        r[])
{
   double clat = cos(lat);
   double clon = cos(lon);
   double slat = sin(lat);
   double slon = sin(lon);
   double Rn = WGS84_A / sqrt(1.0 - WGS84_E2 * slat * slat);
   double R = Rn + alt;

   r[0] = R * clat * clon;
   r[1] = R * clat * slon;
   r[2] = (Rn * (1.0 - WGS84_E2) + alt) * slat;
}

//-----------------------------------------------------------------------------
void ECEFToGeodetic(
   const double r[],
   double&      lat,
   double&      lon,
   double&      alt)
{
   double p, delta = 1.0e+3;
   double slat, clat, Rn, h;
   bool polar_cap;

   lon = atan2(r[1], r[0]);

   p = sqrt(r[0] * r[0] + r[1] * r[1]);
   polar_cap = p < 1.0e+5;

   alt = 0.0;
   lat = atan2(r[2], (1.0 - WGS84_E2)*p);

   while (delta > 0.001)
   {
      slat = sin(lat);
      clat = cos(lat);

      Rn = WGS84_A / sqrt(1.0 - WGS84_E2 * slat * slat);

      if (polar_cap)
      {
         double p0 = Rn*clat;
         double z0 = Rn*(1.0 - WGS84_E2)*slat;
         double dp = p - p0;
         double dz = r[2] - z0;

         h = sqrt(dp * dp + dz * dz);
         if (fabs(r[2]) < fabs(z0))
            h *= -1.0;

      }
      else
      {
         h = p / clat - Rn;
      }

      delta = fabs(h - alt);
      alt = h;
      lat = atan2(r[2], p*(1.0 - WGS84_E2 * Rn / (Rn + h)));

   } // while (delta > 0.001)

} // ECEFToGeodetic()

//-----------------------------------------------------------------------------
void DCM_ECEFToNED(const double& lat, const double& lon, double Cen[3][3])
{
   double clat = cos(lat);
   double slat = sin(lat);
   double clon = cos(lon);
   double slon = sin(lon);

   Cen[0][0] = -slat * clon; Cen[0][1] = -slat * slon; Cen[0][2] = clat;
   Cen[1][0] = -slon;        Cen[1][1] = clon;        Cen[1][2] = 0.0;
   Cen[2][0] = -clat * clon; Cen[2][1] = -clat * slon; Cen[2][2] = -slat;
}

//-----------------------------------------------------------------------------
// Set origin of pose specifically for customers
void SetCustomOrigin(
   double               latitude,   // radian
   double               longitude,  // radian
   double               altitude,   // meters
   struct origin_type&  org)
{
   GeodeticToECEF(latitude, longitude, altitude, org.r);
   DCM_ECEFToNED(latitude, longitude, org.Cen);
}

//-----------------------------------------------------------------------------
// Set origin for Pose message
void SetOrigin(
   const polyx_node::msg::CompactNav&     msg,
   struct origin_type&         org)
{
   SetCustomOrigin(msg.latitude, msg.longitude, msg.altitude, org);
}

//-----------------------------------------------------------------------------
// Convert PE CompactNav message to PoseStamped message
void icd_to_PoseStamped(
   const polyx_node::msg::CompactNav& msg,
   const struct origin_type& org,
   geometry_msgs::msg::PoseStamped& ps)
{
   double dr_e[3], dr_n[3];
   geometry_msgs::msg::Quaternion q1;
   geometry_msgs::msg::Quaternion q2;
   ps.header.stamp = msg.header.stamp;

   GeodeticToECEF(msg.latitude, msg.longitude, msg.altitude, dr_e);

   for (int i = 0; i < 3; ++i)
      dr_e[i] -= org.r[i];

   for (int i = 0; i < 3; ++i)
   {
      dr_n[i] = 0;
      for (int j = 0; j < 3; ++j)
         dr_n[i] += org.Cen[i][j] * dr_e[j];
   }

   ps.pose.position.x = dr_n[0]; // North
   ps.pose.position.y = dr_n[1]; // East 
   ps.pose.position.z = dr_n[2]; // Down

   // Get quaternion from the body to NED frame

   ps.pose.orientation.x = msg.quaternion[1];
   ps.pose.orientation.y = msg.quaternion[2];
   ps.pose.orientation.z = msg.quaternion[3];
   ps.pose.orientation.w = msg.quaternion[0];
}


// Convert quaternion to Euler angles
bool EulerAttitude(polyx_node::msg::CompactNav &msg, polyx_node::msg::EulerAttitude &qtemsg)
{


   float q0 = msg.quaternion[0] * msg.quaternion[0];
   float q1 = msg.quaternion[1] * msg.quaternion[1];
   float q2 = msg.quaternion[2] * msg.quaternion[2];
   float q3 = msg.quaternion[3] * msg.quaternion[3];


   float C31 = 2.0f * (msg.quaternion[1] * msg.quaternion[3] - msg.quaternion[0] * msg.quaternion[2]);
   float C32 = 2.0f * (msg.quaternion[2] * msg.quaternion[3] + msg.quaternion[0] * msg.quaternion[1]);
   float C33 = q0 - q1 - q2 + q3;
   qtemsg.gps_time_week = msg.gps_time_week;
   qtemsg.pitch = atan(-C31 / sqrt(C32 * C32 + C33 * C33));

   if (fabsf(C31) < 0.999f)
   {
      float C11 = q0 + q1 - q2 - q3;
      float C21 = 2.0f * (msg.quaternion[1] * msg.quaternion[2] + msg.quaternion[0] * msg.quaternion[3]);

      qtemsg.roll = atan2(C32, C33);
      qtemsg.heading = atan2(C21, C11);

      return true;
   }
   else
   {
      return false;
   }
}

//-----------------------------------------------------------------------------
void FrameTrans(
   const double            x_in[],
   const frame_trans_type& p,
   double                  x_out[])
{
   double T[3][3];

   T[0][0] = 1.0 + p.scale; T[0][1] = -p.rot[2]; T[0][2] = p.rot[1];
   T[1][0] = p.rot[2]; T[1][1] = T[0][0]; T[1][2] = -p.rot[0];
   T[2][0] = -p.rot[1]; T[2][1] = p.rot[0]; T[2][2] = T[0][0];

   for (int i = 0; i < 3; ++i)
   {
      double sum = 0.0;
      for (int j = 0; j < 3; ++j)
         sum += T[i][j] * x_in[j];

      x_out[i] = sum + p.trans[i];
   }
}

//-----------------------------------------------------------------------------
void ConvertToNAD83(
   const uint16_t& week,
   const double&   tow,
   double&         lat,
   double&         lon,
   double&         alt)
{
   double epoch = 1980.0 + (5.0 + week * 7.0 + tow / 86400.0)/365.25;
   double dy = epoch - 2000.0;
   double r[3], r_nad83[3];
   frame_trans_type p;

   p.trans[0] = 0.9958 + 0.1e-3 * dy;
   p.trans[1] = -1.9046 - 0.5e-3 * dy;
   p.trans[2] = -0.5461 - 3.2e-3 * dy;
   p.scale = 2.92e-9 + 0.09e-9 * dy;
   p.rot[0] = (25.9496 + 0.0532 * dy) * MAS_TO_RAD;
   p.rot[1] = (7.4231 - 0.7423 * dy)  * MAS_TO_RAD;
   p.rot[2] = (11.6252 - 0.0116 * dy) * MAS_TO_RAD;

   GeodeticToECEF(lat, lon, alt, r);
   FrameTrans(r, p, r_nad83);
   ECEFToGeodetic(r_nad83, lat, lon, alt);

}


//-----------------------------------------------------------------------------
// Decode 8-byte data
template<class T>
void Decode8(const uint8_t* p, T& out)
{
   int64_t* i64 = (int64_t*)&out;

   *i64 = p[7];
   for (int i = 6; i >= 0; --i)
      *i64 = ((*i64) << 8) | p[i];

}

void Decode(const uint8_t* p, double& d)
{
   Decode8(p, d);
}

//-----------------------------------------------------------------------------
// Decode 4-byte data
template<class T>
void Decode4(const uint8_t* p, T& out)
{
   int32_t* i32 = (int32_t*)& out;
   
   *i32 = p[3];

   for (int i = 2; i >= 0; --i)
      (*i32) = ((*i32) << 8) | p[i];
}

void Decode(const uint8_t* p, uint32_t& u32)
{
   Decode4(p, u32);
}

void Decode(const uint8_t* p, int32_t& i32)
{
   Decode4(p, i32);
}

void Decode(const uint8_t* p, float& f)
{
   Decode4(p, f);
}

//-----------------------------------------------------------------------------
// Decode 2-byte data
template<class T>
void Decode2(const uint8_t* p, T& out)
{
   out = p[0] | (p[1] << 8);
}

void Decode(const uint8_t* p, int16_t& i16)
{
   Decode2(p, i16);
}

void Decode(const uint8_t* p, uint16_t& u16)
{
   Decode2(p, u16);
}

//-----------------------------------------------------------------------------
// Encode 8-byte data
template<class T>
void Encode8(T dat, uint8_t* p)
{
   int64_t* i64 = (int64_t*)&dat;

   for (int i = 0; i < 8; ++i)
   {
      p[i] = (*i64) & 0xff;
      *i64 >>= 8;
   }
}

void Encode(const double& d, uint8_t* p)
{
   Encode8(d, p);
}

//-----------------------------------------------------------------------------
// Encode 4-byte data
template<class T>
void Encode4(T dat, uint8_t* p)
{
   int32_t* i32 = (int32_t*)&dat;

   for (int i = 0; i < 4; ++i)
   {
      p[i] = (*i32) & 0xff;
      *i32 >>= 8;
   }
}

void Encode(const float& f, uint8_t* p)
{
   Encode4(f, p);
}

void Encode(const uint32_t& u32, uint8_t* p)
{
   Encode4(u32, p);
}

void Encode(const int32_t& i32, uint8_t* p)
{
   Encode4(i32, p);
}

//-----------------------------------------------------------------------------
// Encode 2-byte data
template<class T>
void Encode2(T dat, uint8_t* p)
{
   int8_t* i16 = (int8_t*)&dat;

   p[0] = (*i16) & 0xff;
   p[1] = ((*i16) >> 8) & 0xff;
}

void Encode(const uint16_t& u16, uint8_t* p)
{
   Encode2(u16, p);
}

void Encode(const int16_t& i16, uint8_t* p)
{
   Encode2(i16, p);
}
void SetMsgHeader(uint8_t type, uint8_t sub_id, uint16_t payload_len, uint8_t* header)
{
   header[0] = 0xAF;
   header[1] = 0x20;
   header[2] = type;
   header[3] = sub_id;
   Encode(payload_len, &header[4]);
}

uint8_t checksum(uint8_t* Buffer, uint16_t len, uint8_t& cka, uint8_t& ckb)
{
   unsigned char CK_A = 0;
   unsigned char CK_B = 0;

   if (Buffer)
   {
      for (int i = 0; i < len; i++)
      {
         CK_A += Buffer[i];
         CK_B += CK_A;
      }
      cka = CK_A;
      ckb = CK_B;
      return 1;
   }
   else
      return 0;
}

uint8_t checkMessageType(uint8_t *buf)
{
   int len = 0;
   uint8_t c1, c2;
   uint8_t messageType;

   len = buf[5];
   len = (len << 8) + buf[4];  // payload length

   checksum(buf + 6, len, c1, c2);

   if ((c1 == buf[len + 6]) && (c2 == buf[len + 7]))
   {
      messageType = buf[2];
   }
   else 
   {
      printf("invalid message.\n");
      messageType = 0;
   }
   return messageType;
}

void parse_Kalman_message(uint8_t *buf, polyx_node::msg::Kalman &kalmsg)
{
   int p = 6;
   Decode(&buf[p], kalmsg.system_time);   p += 8; // double
   Decode(&buf[p], kalmsg.gps_time);      p += 8; // double
   Decode(&buf[p], kalmsg.latitude);      p += 8; // double [rad]
   Decode(&buf[p], kalmsg.longitude);     p += 8; // all double [rad]
   Decode(&buf[p], kalmsg.ellipsoidal_height);  p += 8; // double
   Decode(&buf[p], kalmsg.velocity_north);      p += 8; // double
   Decode(&buf[p], kalmsg.velocity_east); p += 8; // double
   Decode(&buf[p], kalmsg.velocity_down); p += 8; // double
   Decode(&buf[p], kalmsg.roll);          p += 8; // double [rad]
   Decode(&buf[p], kalmsg.pitch);         p += 8; // double [rad]
   Decode(&buf[p], kalmsg.heading);       p += 8; // double [rad]
   kalmsg.position_mode = buf[p];    ++p; // uint8
   kalmsg.velocity_mode = buf[p];    ++p; // uint8
   kalmsg.attitude_status = buf[p];       // uint8

}

void parse_RawIMU_message(uint8_t *buf, polyx_node::msg::RawIMU &imsg)
{
   int i, p = 6;

   Decode(&buf[p], imsg.system_time);  p += 8; // double
   for (i = 0; i < 3; i++)
   {
      Decode(&buf[p], imsg.acceleration[i]); p += 8; // double [m/s^2]
   }
   for (i = 0; i < 3; i++)
   {
      Decode(&buf[p], imsg.rotation_rate[i]); p += 8; // double [deg/s]
      imsg.rotation_rate[i] *= DEG_TO_RAD; // to [rad/s]
   }
}

void parse_SolutionStatus_message(uint8_t *buf, polyx_node::msg::SolutionStatus &smsg)
{
   int p = 6, i;

   Decode(&buf[p], smsg.system_time);     p += 8; // double
   smsg.number_of_svs = buf[p];   ++p; // uint8
   smsg.processing_mode = buf[p];     ++p; // uint8
   Decode(&buf[p], smsg.gps_week_number); p += 2; // uint16
   Decode(&buf[p], smsg.gps_time_week);   p += 8; // double
   for (i = 0; i < 3; i++)
   {
      Decode(&buf[p], smsg.position_rms[i]); p += 8; // double
   }
   for (i = 0; i < 3; i++)
   {
      Decode(&buf[p], smsg.velocity_rms[i]); p += 8; // double
   }
   for (i = 0; i < 3; i++)
   {
      Decode(&buf[p], smsg.attitude_rms[i]); p += 8; // double [deg/s]
      smsg.attitude_rms[i] *= DEG_TO_RAD; // double [rad/s]
   }
}

void parse_CompactNav_message(
   uint8_t*          buf, 
   polyx_node::msg::CompactNav& msg,
   polyx::ref_frame_trans_type  frame_trans)
{
   int p = 6, i;
   float temp;
   Decode(&buf[p], msg.gps_time_week); p += 8; //double
   Decode(&buf[p], msg.latitude);      p += 8; // double [deg]
   msg.latitude *= DEG_TO_RAD;  // double [rad]
   Decode(&buf[p], msg.longitude);     p += 8; // double [deg]
   msg.longitude *= DEG_TO_RAD; // double [rad]
   Decode(&buf[p], temp);  p += 4; // internally as float[m], but msg.longitude is double
   msg.altitude = temp;
   for (i = 0; i < 3; i++)
   {  
      Decode(&buf[p], temp);  p += 4; // internally float
      msg.velocity_ned[i] = temp; // float to double implicitly
   }
   for (i = 0; i < 4; i++)
   {
      Decode(&buf[p], temp);    p += 4; //float
      msg.quaternion[i] = temp;
   }
   for (i = 0; i < 3; i++)
   {
      Decode(&buf[p], temp);  p += 4; //float
      msg.acceleration[i] = temp;
   }
   for (i = 0; i < 3; i++)
   {
      Decode(&buf[p], temp); p += 4; //float [deg/s]
      msg.rotation_rate[i] = temp * DEG_TO_RAD; // to double[rad/s]
   }
   for (i = 0; i < 3; i++)
   {
      Decode(&buf[p], temp);  p += 4; //float
      msg.position_rms[i] = temp;
   }
   for (i = 0; i < 3; i++)
   {
      Decode(&buf[p], temp);  p += 4; //float
      msg.velocity_rms[i] = temp;
   }
   for (i = 0; i < 3; i++)
   {
      Decode(&buf[p], temp);  p += 4; // float [deg]
      msg.attitude_rms[i] = temp * DEG_TO_RAD; // double [rad]
   }
   Decode(&buf[p], msg.gps_week_number);  p += 2; // uint16
   msg.alignment = buf[p];

   if (INI_GPS_WEEK < 0 && msg.gps_week_number > 0)
      INI_GPS_WEEK = msg.gps_week_number;

   if (frame_trans == ref_frame_trans_type::WGS84_TO_NAD83)
   {
      ConvertToNAD83(msg.gps_week_number, msg.gps_time_week, msg.latitude, msg.longitude, msg.altitude);
   }
   // deleted header stamp here, and move to node, clock not in this scope, unless nonclass
}

void parse_TimeSync_message(uint8_t *buf, polyx_node::msg::TimeSync &tsmsg)
{
   Decode(&buf[6], tsmsg.system_computer_time);
   Decode(&buf[14], tsmsg.bias_to_gps_time);
}

void parse_Geoid_message(uint8_t *buf, polyx_node::msg::Geoid &gmsg)
{
   Decode(&buf[6], gmsg.gps_time);
   Decode(&buf[14], gmsg.geoid_height);
}

void parse_CorrectedIMU_message(uint8_t *buf, polyx_node::msg::CorrectedIMU &imsg)
{
   int p = 6, i;
   Decode(&buf[p], imsg.gps_time_week); p += 8;
   for(i = 0; i < 3; i++)
   {
      Decode(&buf[p], imsg.acceleration[i]); p += 8; //double[m/s^2]
   }
   for(i = 0; i < 3; i++)
   {
      Decode(&buf[p], imsg.rotation_rate[i]); p += 8; // double [deg/s]
      imsg.rotation_rate[i] *= DEG_TO_RAD; // double [rad/s]
   }


   Decode(&buf[p], imsg.gps_week_number);      
   
}

void parse_LeapSeconds_message(uint8_t *buf, polyx_node::msg::LeapSeconds &lsmsg)
{
   // only one byte, do nothing
   lsmsg.leap_seconds = buf[6];
   LEAP_SECONDS = lsmsg.leap_seconds;
}

void parse_dmi_message(uint8_t *buf, polyx_node::msg::Dmi &dmi)
{
   Decode(&buf[6], dmi.system_time);
   Decode(&buf[14], dmi.pulse_count);
   dmi.id = buf[18];
}

} // namespace polyx	