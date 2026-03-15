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

#ifndef WEBOTS_ROS_MESSAGE_GET_STRING_H
#define WEBOTS_ROS_MESSAGE_GET_STRING_H

#include "ros/service_traits.h"

#include "get_stringRequest.h"
#include "get_stringResponse.h"

namespace webots_ros
{

struct get_string
{

typedef get_stringRequest Request;
typedef get_stringResponse Response;
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
struct MD5Sum< ::webots_ros::get_string > {
  static const char* value()
  {
    return "3bf99d9257a34f6cdd01cd192a62b3df";
  }

  static const char* value(const ::webots_ros::get_string&) { return value(); }
};

template<>
struct DataType< ::webots_ros::get_string > {
  static const char* value()
  {
    return "webots_ros/get_string";
  }

  static const char* value(const ::webots_ros::get_string&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::get_stringRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::get_string >::value();
  }
  static const char* value(const ::webots_ros::get_stringRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::get_stringRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::get_string >::value();
  }
  static const char* value(const ::webots_ros::get_stringRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::get_stringResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::get_string >::value();
  }
  static const char* value(const ::webots_ros::get_stringResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::get_stringResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::get_string >::value();
  }
  static const char* value(const ::webots_ros::get_stringResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_GET_STRING_H
