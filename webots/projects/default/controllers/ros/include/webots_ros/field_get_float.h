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

#ifndef WEBOTS_ROS_MESSAGE_FIELD_GET_FLOAT_H
#define WEBOTS_ROS_MESSAGE_FIELD_GET_FLOAT_H

#include "ros/service_traits.h"

#include "field_get_floatRequest.h"
#include "field_get_floatResponse.h"

namespace webots_ros
{

struct field_get_float
{

typedef field_get_floatRequest Request;
typedef field_get_floatResponse Response;
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
struct MD5Sum< ::webots_ros::field_get_float > {
  static const char* value()
  {
    return "c2cc517501b50e0a3d3b0ee8172ea3fb";
  }

  static const char* value(const ::webots_ros::field_get_float&) { return value(); }
};

template<>
struct DataType< ::webots_ros::field_get_float > {
  static const char* value()
  {
    return "webots_ros/field_get_float";
  }

  static const char* value(const ::webots_ros::field_get_float&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::field_get_floatRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::field_get_float >::value();
  }
  static const char* value(const ::webots_ros::field_get_floatRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::field_get_floatRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::field_get_float >::value();
  }
  static const char* value(const ::webots_ros::field_get_floatRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::field_get_floatResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::field_get_float >::value();
  }
  static const char* value(const ::webots_ros::field_get_floatResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::field_get_floatResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::field_get_float >::value();
  }
  static const char* value(const ::webots_ros::field_get_floatResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_FIELD_GET_FLOAT_H
