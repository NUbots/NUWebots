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

#ifndef WEBOTS_ROS_MESSAGE_ROBOT_WAIT_FOR_USER_INPUT_EVENT_H
#define WEBOTS_ROS_MESSAGE_ROBOT_WAIT_FOR_USER_INPUT_EVENT_H

#include "ros/service_traits.h"

#include "robot_wait_for_user_input_eventRequest.h"
#include "robot_wait_for_user_input_eventResponse.h"

namespace webots_ros
{

struct robot_wait_for_user_input_event
{

typedef robot_wait_for_user_input_eventRequest Request;
typedef robot_wait_for_user_input_eventResponse Response;
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
struct MD5Sum< ::webots_ros::robot_wait_for_user_input_event > {
  static const char* value()
  {
    return "117ed5fb1f5c37c2a398a290a90fc55d";
  }

  static const char* value(const ::webots_ros::robot_wait_for_user_input_event&) { return value(); }
};

template<>
struct DataType< ::webots_ros::robot_wait_for_user_input_event > {
  static const char* value()
  {
    return "webots_ros/robot_wait_for_user_input_event";
  }

  static const char* value(const ::webots_ros::robot_wait_for_user_input_event&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::robot_wait_for_user_input_eventRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::robot_wait_for_user_input_event >::value();
  }
  static const char* value(const ::webots_ros::robot_wait_for_user_input_eventRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::robot_wait_for_user_input_eventRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::robot_wait_for_user_input_event >::value();
  }
  static const char* value(const ::webots_ros::robot_wait_for_user_input_eventRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::robot_wait_for_user_input_eventResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::robot_wait_for_user_input_event >::value();
  }
  static const char* value(const ::webots_ros::robot_wait_for_user_input_eventResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::robot_wait_for_user_input_eventResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::robot_wait_for_user_input_event >::value();
  }
  static const char* value(const ::webots_ros::robot_wait_for_user_input_eventResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_ROBOT_WAIT_FOR_USER_INPUT_EVENT_H
