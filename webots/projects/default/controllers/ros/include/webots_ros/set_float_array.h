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

#ifndef WEBOTS_ROS_MESSAGE_SET_FLOAT_ARRAY_H
#define WEBOTS_ROS_MESSAGE_SET_FLOAT_ARRAY_H

#include "ros/service_traits.h"

#include "set_float_arrayRequest.h"
#include "set_float_arrayResponse.h"

namespace webots_ros
{

struct set_float_array
{

typedef set_float_arrayRequest Request;
typedef set_float_arrayResponse Response;
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
struct MD5Sum< ::webots_ros::set_float_array > {
  static const char* value()
  {
    return "0fcc5d2c24929b6dbbf415b9579ddce4";
  }

  static const char* value(const ::webots_ros::set_float_array&) { return value(); }
};

template<>
struct DataType< ::webots_ros::set_float_array > {
  static const char* value()
  {
    return "webots_ros/set_float_array";
  }

  static const char* value(const ::webots_ros::set_float_array&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::set_float_arrayRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::set_float_array >::value();
  }
  static const char* value(const ::webots_ros::set_float_arrayRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::set_float_arrayRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::set_float_array >::value();
  }
  static const char* value(const ::webots_ros::set_float_arrayRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::set_float_arrayResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::set_float_array >::value();
  }
  static const char* value(const ::webots_ros::set_float_arrayResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::set_float_arrayResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::set_float_array >::value();
  }
  static const char* value(const ::webots_ros::set_float_arrayResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_SET_FLOAT_ARRAY_H
