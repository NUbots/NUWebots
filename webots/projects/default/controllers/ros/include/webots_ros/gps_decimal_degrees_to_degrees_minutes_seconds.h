/*
 * Copyright 1996-2022 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef WEBOTS_ROS_MESSAGE_GPS_DECIMAL_DEGREES_TO_DEGREES_MINUTES_SECONDS_H
#define WEBOTS_ROS_MESSAGE_GPS_DECIMAL_DEGREES_TO_DEGREES_MINUTES_SECONDS_H

#include "ros/service_traits.h"

#include "gps_decimal_degrees_to_degrees_minutes_secondsRequest.h"
#include "gps_decimal_degrees_to_degrees_minutes_secondsResponse.h"

namespace webots_ros
{

struct gps_decimal_degrees_to_degrees_minutes_seconds
{

typedef gps_decimal_degrees_to_degrees_minutes_secondsRequest Request;
typedef gps_decimal_degrees_to_degrees_minutes_secondsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

};
} // namespace webots_ros

namespace ros
{
namespace service_traits
{

template<>
struct MD5Sum< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_seconds > {
  static const char* value()
  {
    return "2911ee9051e401397c9b1e29a01f7ead";
  }

  static const char* value(const ::webots_ros::gps_decimal_degrees_to_degrees_minutes_seconds&) { return value(); }
};

template<>
struct DataType< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_seconds > {
  static const char* value()
  {
    return "webots_ros/gps_decimal_degrees_to_degrees_minutes_seconds";
  }

  static const char* value(const ::webots_ros::gps_decimal_degrees_to_degrees_minutes_seconds&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_seconds >::value();
  }
  static const char* value(const ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_seconds >::value();
  }
  static const char* value(const ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_seconds >::value();
  }
  static const char* value(const ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_seconds >::value();
  }
  static const char* value(const ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_GPS_DECIMAL_DEGREES_TO_DEGREES_MINUTES_SECONDS_H
