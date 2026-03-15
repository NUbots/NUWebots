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

#ifndef WEBOTS_ROS_MESSAGE_GET_URDF_H
#define WEBOTS_ROS_MESSAGE_GET_URDF_H

#include "ros/service_traits.h"

#include "get_urdfRequest.h"
#include "get_urdfResponse.h"

namespace webots_ros
{

struct get_urdf
{

typedef get_urdfRequest Request;
typedef get_urdfResponse Response;
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
struct MD5Sum< ::webots_ros::get_urdf > {
  static const char* value()
  {
    return "9ae31c29f15764629e70e60492948eb1";
  }

  static const char* value(const ::webots_ros::get_urdf&) { return value(); }
};

template<>
struct DataType< ::webots_ros::get_urdf > {
  static const char* value()
  {
    return "webots_ros/get_urdf";
  }

  static const char* value(const ::webots_ros::get_urdf&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::get_urdfRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::get_urdf >::value();
  }
  static const char* value(const ::webots_ros::get_urdfRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::get_urdfRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::get_urdf >::value();
  }
  static const char* value(const ::webots_ros::get_urdfRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::get_urdfResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::get_urdf >::value();
  }
  static const char* value(const ::webots_ros::get_urdfResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::get_urdfResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::get_urdf >::value();
  }
  static const char* value(const ::webots_ros::get_urdfResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_GET_URDF_H
