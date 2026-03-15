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

#ifndef WEBOTS_ROS_MESSAGE_NODE_GET_VELOCITY_H
#define WEBOTS_ROS_MESSAGE_NODE_GET_VELOCITY_H

#include "ros/service_traits.h"

#include "node_get_velocityRequest.h"
#include "node_get_velocityResponse.h"

namespace webots_ros
{

struct node_get_velocity
{

typedef node_get_velocityRequest Request;
typedef node_get_velocityResponse Response;
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
struct MD5Sum< ::webots_ros::node_get_velocity > {
  static const char* value()
  {
    return "f50dcf3848a1b2dce54e5fbe9ff12eac";
  }

  static const char* value(const ::webots_ros::node_get_velocity&) { return value(); }
};

template<>
struct DataType< ::webots_ros::node_get_velocity > {
  static const char* value()
  {
    return "webots_ros/node_get_velocity";
  }

  static const char* value(const ::webots_ros::node_get_velocity&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::node_get_velocityRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::node_get_velocity >::value();
  }
  static const char* value(const ::webots_ros::node_get_velocityRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::node_get_velocityRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::node_get_velocity >::value();
  }
  static const char* value(const ::webots_ros::node_get_velocityRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::node_get_velocityResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::node_get_velocity >::value();
  }
  static const char* value(const ::webots_ros::node_get_velocityResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::node_get_velocityResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::node_get_velocity >::value();
  }
  static const char* value(const ::webots_ros::node_get_velocityResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_NODE_GET_VELOCITY_H
