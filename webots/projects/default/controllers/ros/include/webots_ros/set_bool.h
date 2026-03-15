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

#ifndef WEBOTS_ROS_MESSAGE_SET_BOOL_H
#define WEBOTS_ROS_MESSAGE_SET_BOOL_H

#include "ros/service_traits.h"

#include "set_boolRequest.h"
#include "set_boolResponse.h"

namespace webots_ros
{

struct set_bool
{

typedef set_boolRequest Request;
typedef set_boolResponse Response;
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
struct MD5Sum< ::webots_ros::set_bool > {
  static const char* value()
  {
    return "24a0f9fd764459b7e35d04a0dd83dd11";
  }

  static const char* value(const ::webots_ros::set_bool&) { return value(); }
};

template<>
struct DataType< ::webots_ros::set_bool > {
  static const char* value()
  {
    return "webots_ros/set_bool";
  }

  static const char* value(const ::webots_ros::set_bool&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::set_boolRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::set_bool >::value();
  }
  static const char* value(const ::webots_ros::set_boolRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::set_boolRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::set_bool >::value();
  }
  static const char* value(const ::webots_ros::set_boolRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::set_boolResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::set_bool >::value();
  }
  static const char* value(const ::webots_ros::set_boolResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::set_boolResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::set_bool >::value();
  }
  static const char* value(const ::webots_ros::set_boolResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_SET_BOOL_H
