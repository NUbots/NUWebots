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

#ifndef WEBOTS_ROS_MESSAGE_FIELD_SET_STRING_H
#define WEBOTS_ROS_MESSAGE_FIELD_SET_STRING_H

#include "ros/service_traits.h"

#include "field_set_stringRequest.h"
#include "field_set_stringResponse.h"

namespace webots_ros
{

struct field_set_string
{

typedef field_set_stringRequest Request;
typedef field_set_stringResponse Response;
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
struct MD5Sum< ::webots_ros::field_set_string > {
  static const char* value()
  {
    return "a84b2f7623a3316b5dc8470fcbf631fd";
  }

  static const char* value(const ::webots_ros::field_set_string&) { return value(); }
};

template<>
struct DataType< ::webots_ros::field_set_string > {
  static const char* value()
  {
    return "webots_ros/field_set_string";
  }

  static const char* value(const ::webots_ros::field_set_string&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::field_set_stringRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::field_set_string >::value();
  }
  static const char* value(const ::webots_ros::field_set_stringRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::field_set_stringRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::field_set_string >::value();
  }
  static const char* value(const ::webots_ros::field_set_stringRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::field_set_stringResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::field_set_string >::value();
  }
  static const char* value(const ::webots_ros::field_set_stringResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::field_set_stringResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::field_set_string >::value();
  }
  static const char* value(const ::webots_ros::field_set_stringResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_FIELD_SET_STRING_H
