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

#ifndef WEBOTS_ROS_MESSAGE_ROBOT_SET_MODE_H
#define WEBOTS_ROS_MESSAGE_ROBOT_SET_MODE_H

#include "ros/service_traits.h"

#include "robot_set_modeRequest.h"
#include "robot_set_modeResponse.h"

namespace webots_ros
{

struct robot_set_mode
{

typedef robot_set_modeRequest Request;
typedef robot_set_modeResponse Response;
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
struct MD5Sum< ::webots_ros::robot_set_mode > {
  static const char* value()
  {
    return "ef8b4f277f1a6d92a8c4d6a68a3850de";
  }

  static const char* value(const ::webots_ros::robot_set_mode&) { return value(); }
};

template<>
struct DataType< ::webots_ros::robot_set_mode > {
  static const char* value()
  {
    return "webots_ros/robot_set_mode";
  }

  static const char* value(const ::webots_ros::robot_set_mode&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::robot_set_modeRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::robot_set_mode >::value();
  }
  static const char* value(const ::webots_ros::robot_set_modeRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::robot_set_modeRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::robot_set_mode >::value();
  }
  static const char* value(const ::webots_ros::robot_set_modeRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::robot_set_modeResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::robot_set_mode >::value();
  }
  static const char* value(const ::webots_ros::robot_set_modeResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::robot_set_modeResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::robot_set_mode >::value();
  }
  static const char* value(const ::webots_ros::robot_set_modeResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_ROBOT_SET_MODE_H
