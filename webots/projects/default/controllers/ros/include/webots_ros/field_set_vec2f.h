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

#ifndef WEBOTS_ROS_MESSAGE_FIELD_SET_VEC2F_H
#define WEBOTS_ROS_MESSAGE_FIELD_SET_VEC2F_H

#include "ros/service_traits.h"

#include "field_set_vec2fRequest.h"
#include "field_set_vec2fResponse.h"

namespace webots_ros
{

struct field_set_vec2f
{

typedef field_set_vec2fRequest Request;
typedef field_set_vec2fResponse Response;
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
struct MD5Sum< ::webots_ros::field_set_vec2f > {
  static const char* value()
  {
    return "e0edef5860d282e4d35ef264e28feecc";
  }

  static const char* value(const ::webots_ros::field_set_vec2f&) { return value(); }
};

template<>
struct DataType< ::webots_ros::field_set_vec2f > {
  static const char* value()
  {
    return "webots_ros/field_set_vec2f";
  }

  static const char* value(const ::webots_ros::field_set_vec2f&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::field_set_vec2fRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::field_set_vec2f >::value();
  }
  static const char* value(const ::webots_ros::field_set_vec2fRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::field_set_vec2fRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::field_set_vec2f >::value();
  }
  static const char* value(const ::webots_ros::field_set_vec2fRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::field_set_vec2fResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::field_set_vec2f >::value();
  }
  static const char* value(const ::webots_ros::field_set_vec2fResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::field_set_vec2fResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::field_set_vec2f >::value();
  }
  static const char* value(const ::webots_ros::field_set_vec2fResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_FIELD_SET_VEC2F_H
