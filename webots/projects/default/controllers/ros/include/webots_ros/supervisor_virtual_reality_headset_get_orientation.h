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

#ifndef WEBOTS_ROS_MESSAGE_SUPERVISOR_VIRTUAL_REALITY_HEADSET_GET_ORIENTATION_H
#define WEBOTS_ROS_MESSAGE_SUPERVISOR_VIRTUAL_REALITY_HEADSET_GET_ORIENTATION_H

#include "ros/service_traits.h"

#include "supervisor_virtual_reality_headset_get_orientationRequest.h"
#include "supervisor_virtual_reality_headset_get_orientationResponse.h"

namespace webots_ros
{

struct supervisor_virtual_reality_headset_get_orientation
{

typedef supervisor_virtual_reality_headset_get_orientationRequest Request;
typedef supervisor_virtual_reality_headset_get_orientationResponse Response;
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
struct MD5Sum< ::webots_ros::supervisor_virtual_reality_headset_get_orientation > {
  static const char* value()
  {
    return "1da5db090570556f67921a94715018da";
  }

  static const char* value(const ::webots_ros::supervisor_virtual_reality_headset_get_orientation&) { return value(); }
};

template<>
struct DataType< ::webots_ros::supervisor_virtual_reality_headset_get_orientation > {
  static const char* value()
  {
    return "webots_ros/supervisor_virtual_reality_headset_get_orientation";
  }

  static const char* value(const ::webots_ros::supervisor_virtual_reality_headset_get_orientation&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::supervisor_virtual_reality_headset_get_orientationRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::supervisor_virtual_reality_headset_get_orientation >::value();
  }
  static const char* value(const ::webots_ros::supervisor_virtual_reality_headset_get_orientationRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::supervisor_virtual_reality_headset_get_orientationRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::supervisor_virtual_reality_headset_get_orientation >::value();
  }
  static const char* value(const ::webots_ros::supervisor_virtual_reality_headset_get_orientationRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::supervisor_virtual_reality_headset_get_orientationResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::supervisor_virtual_reality_headset_get_orientation >::value();
  }
  static const char* value(const ::webots_ros::supervisor_virtual_reality_headset_get_orientationResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::supervisor_virtual_reality_headset_get_orientationResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::supervisor_virtual_reality_headset_get_orientation >::value();
  }
  static const char* value(const ::webots_ros::supervisor_virtual_reality_headset_get_orientationResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_SUPERVISOR_VIRTUAL_REALITY_HEADSET_GET_ORIENTATION_H
