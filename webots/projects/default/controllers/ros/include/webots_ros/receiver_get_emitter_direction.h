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

#ifndef WEBOTS_ROS_MESSAGE_RECEIVER_GET_EMITTER_DIRECTION_H
#define WEBOTS_ROS_MESSAGE_RECEIVER_GET_EMITTER_DIRECTION_H

#include "ros/service_traits.h"

#include "receiver_get_emitter_directionRequest.h"
#include "receiver_get_emitter_directionResponse.h"

namespace webots_ros
{

struct receiver_get_emitter_direction
{

typedef receiver_get_emitter_directionRequest Request;
typedef receiver_get_emitter_directionResponse Response;
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
struct MD5Sum< ::webots_ros::receiver_get_emitter_direction > {
  static const char* value()
  {
    return "cad5a90bedce22b568c949b40e9cc6e0";
  }

  static const char* value(const ::webots_ros::receiver_get_emitter_direction&) { return value(); }
};

template<>
struct DataType< ::webots_ros::receiver_get_emitter_direction > {
  static const char* value()
  {
    return "webots_ros/receiver_get_emitter_direction";
  }

  static const char* value(const ::webots_ros::receiver_get_emitter_direction&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::receiver_get_emitter_directionRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::receiver_get_emitter_direction >::value();
  }
  static const char* value(const ::webots_ros::receiver_get_emitter_directionRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::receiver_get_emitter_directionRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::receiver_get_emitter_direction >::value();
  }
  static const char* value(const ::webots_ros::receiver_get_emitter_directionRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::receiver_get_emitter_directionResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::receiver_get_emitter_direction >::value();
  }
  static const char* value(const ::webots_ros::receiver_get_emitter_directionResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::receiver_get_emitter_directionResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::receiver_get_emitter_direction >::value();
  }
  static const char* value(const ::webots_ros::receiver_get_emitter_directionResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_RECEIVER_GET_EMITTER_DIRECTION_H
