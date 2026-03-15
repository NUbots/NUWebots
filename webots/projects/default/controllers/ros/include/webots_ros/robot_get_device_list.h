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

#ifndef WEBOTS_ROS_MESSAGE_ROBOT_GET_DEVICE_LIST_H
#define WEBOTS_ROS_MESSAGE_ROBOT_GET_DEVICE_LIST_H

#include "ros/service_traits.h"

#include "robot_get_device_listRequest.h"
#include "robot_get_device_listResponse.h"

namespace webots_ros
{

struct robot_get_device_list
{

typedef robot_get_device_listRequest Request;
typedef robot_get_device_listResponse Response;
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
struct MD5Sum< ::webots_ros::robot_get_device_list > {
  static const char* value()
  {
    return "463f6db5695c1f090f76a453a11e7381";
  }

  static const char* value(const ::webots_ros::robot_get_device_list&) { return value(); }
};

template<>
struct DataType< ::webots_ros::robot_get_device_list > {
  static const char* value()
  {
    return "webots_ros/robot_get_device_list";
  }

  static const char* value(const ::webots_ros::robot_get_device_list&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::robot_get_device_listRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::robot_get_device_list >::value();
  }
  static const char* value(const ::webots_ros::robot_get_device_listRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::robot_get_device_listRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::robot_get_device_list >::value();
  }
  static const char* value(const ::webots_ros::robot_get_device_listRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::robot_get_device_listResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::robot_get_device_list >::value();
  }
  static const char* value(const ::webots_ros::robot_get_device_listResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::robot_get_device_listResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::robot_get_device_list >::value();
  }
  static const char* value(const ::webots_ros::robot_get_device_listResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_ROBOT_GET_DEVICE_LIST_H
