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

#ifndef WEBOTS_ROS_MESSAGE_FIELD_SET_INT32_H
#define WEBOTS_ROS_MESSAGE_FIELD_SET_INT32_H

#include "ros/service_traits.h"

#include "field_set_int32Request.h"
#include "field_set_int32Response.h"

namespace webots_ros
{

struct field_set_int32
{

typedef field_set_int32Request Request;
typedef field_set_int32Response Response;
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
struct MD5Sum< ::webots_ros::field_set_int32 > {
  static const char* value()
  {
    return "934ca213eb591b7ceac92091bc85f1c2";
  }

  static const char* value(const ::webots_ros::field_set_int32&) { return value(); }
};

template<>
struct DataType< ::webots_ros::field_set_int32 > {
  static const char* value()
  {
    return "webots_ros/field_set_int32";
  }

  static const char* value(const ::webots_ros::field_set_int32&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::field_set_int32Request>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::field_set_int32 >::value();
  }
  static const char* value(const ::webots_ros::field_set_int32Request&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::field_set_int32Request>
{
  static const char* value()
  {
    return DataType< ::webots_ros::field_set_int32 >::value();
  }
  static const char* value(const ::webots_ros::field_set_int32Request&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::field_set_int32Response>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::field_set_int32 >::value();
  }
  static const char* value(const ::webots_ros::field_set_int32Response&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::field_set_int32Response>
{
  static const char* value()
  {
    return DataType< ::webots_ros::field_set_int32 >::value();
  }
  static const char* value(const ::webots_ros::field_set_int32Response&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_FIELD_SET_INT32_H
