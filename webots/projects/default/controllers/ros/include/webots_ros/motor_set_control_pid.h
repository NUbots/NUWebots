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

#ifndef WEBOTS_ROS_MESSAGE_MOTOR_SET_CONTROL_PID_H
#define WEBOTS_ROS_MESSAGE_MOTOR_SET_CONTROL_PID_H

#include "ros/service_traits.h"

#include "motor_set_control_pidRequest.h"
#include "motor_set_control_pidResponse.h"

namespace webots_ros
{

struct motor_set_control_pid
{

typedef motor_set_control_pidRequest Request;
typedef motor_set_control_pidResponse Response;
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
struct MD5Sum< ::webots_ros::motor_set_control_pid > {
  static const char* value()
  {
    return "712b4e401e3c9cbb098cd0435a9a13d3";
  }

  static const char* value(const ::webots_ros::motor_set_control_pid&) { return value(); }
};

template<>
struct DataType< ::webots_ros::motor_set_control_pid > {
  static const char* value()
  {
    return "webots_ros/motor_set_control_pid";
  }

  static const char* value(const ::webots_ros::motor_set_control_pid&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::motor_set_control_pidRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::motor_set_control_pid >::value();
  }
  static const char* value(const ::webots_ros::motor_set_control_pidRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::motor_set_control_pidRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::motor_set_control_pid >::value();
  }
  static const char* value(const ::webots_ros::motor_set_control_pidRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::motor_set_control_pidResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::motor_set_control_pid >::value();
  }
  static const char* value(const ::webots_ros::motor_set_control_pidResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::motor_set_control_pidResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::motor_set_control_pid >::value();
  }
  static const char* value(const ::webots_ros::motor_set_control_pidResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_MOTOR_SET_CONTROL_PID_H
