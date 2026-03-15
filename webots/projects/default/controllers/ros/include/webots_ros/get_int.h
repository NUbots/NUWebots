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

#ifndef WEBOTS_ROS_MESSAGE_GET_INT_H
#define WEBOTS_ROS_MESSAGE_GET_INT_H

#include "ros/service_traits.h"

#include "get_intRequest.h"
#include "get_intResponse.h"

namespace webots_ros
{

struct get_int
{

typedef get_intRequest Request;
typedef get_intResponse Response;
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
struct MD5Sum< ::webots_ros::get_int > {
  static const char* value()
  {
    return "73dfae6ec9145dcc45d5ed973079e912";
  }

  static const char* value(const ::webots_ros::get_int&) { return value(); }
};

template<>
struct DataType< ::webots_ros::get_int > {
  static const char* value()
  {
    return "webots_ros/get_int";
  }

  static const char* value(const ::webots_ros::get_int&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::get_intRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::get_int >::value();
  }
  static const char* value(const ::webots_ros::get_intRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::get_intRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::get_int >::value();
  }
  static const char* value(const ::webots_ros::get_intRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::get_intResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::get_int >::value();
  }
  static const char* value(const ::webots_ros::get_intResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::get_intResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::get_int >::value();
  }
  static const char* value(const ::webots_ros::get_intResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_GET_INT_H
