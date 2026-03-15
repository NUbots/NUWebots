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

#ifndef WEBOTS_ROS_MESSAGE_AUTOMOBILE_GET_DIMENSIONS_H
#define WEBOTS_ROS_MESSAGE_AUTOMOBILE_GET_DIMENSIONS_H

#include "ros/service_traits.h"

#include "automobile_get_dimensionsRequest.h"
#include "automobile_get_dimensionsResponse.h"

namespace webots_ros
{

struct automobile_get_dimensions
{

typedef automobile_get_dimensionsRequest Request;
typedef automobile_get_dimensionsResponse Response;
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
struct MD5Sum< ::webots_ros::automobile_get_dimensions > {
  static const char* value()
  {
    return "3cc904008b725304e06b1ad175139ee0";
  }

  static const char* value(const ::webots_ros::automobile_get_dimensions&) { return value(); }
};

template<>
struct DataType< ::webots_ros::automobile_get_dimensions > {
  static const char* value()
  {
    return "webots_ros/automobile_get_dimensions";
  }

  static const char* value(const ::webots_ros::automobile_get_dimensions&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::automobile_get_dimensionsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::automobile_get_dimensions >::value();
  }
  static const char* value(const ::webots_ros::automobile_get_dimensionsRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::automobile_get_dimensionsRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::automobile_get_dimensions >::value();
  }
  static const char* value(const ::webots_ros::automobile_get_dimensionsRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::automobile_get_dimensionsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::automobile_get_dimensions >::value();
  }
  static const char* value(const ::webots_ros::automobile_get_dimensionsResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::automobile_get_dimensionsResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::automobile_get_dimensions >::value();
  }
  static const char* value(const ::webots_ros::automobile_get_dimensionsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_AUTOMOBILE_GET_DIMENSIONS_H
