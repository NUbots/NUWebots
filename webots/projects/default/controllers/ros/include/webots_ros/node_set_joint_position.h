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

#ifndef WEBOTS_ROS_MESSAGE_NODE_SET_JOINT_POSITION_H
#define WEBOTS_ROS_MESSAGE_NODE_SET_JOINT_POSITION_H

#include "ros/service_traits.h"

#include "node_set_joint_positionRequest.h"
#include "node_set_joint_positionResponse.h"

namespace webots_ros
{

struct node_set_joint_position
{

typedef node_set_joint_positionRequest Request;
typedef node_set_joint_positionResponse Response;
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
struct MD5Sum< ::webots_ros::node_set_joint_position > {
  static const char* value()
  {
    return "5c6d60bc617f1348200743f9a8544085";
  }

  static const char* value(const ::webots_ros::node_set_joint_position&) { return value(); }
};

template<>
struct DataType< ::webots_ros::node_set_joint_position > {
  static const char* value()
  {
    return "webots_ros/node_set_joint_position";
  }

  static const char* value(const ::webots_ros::node_set_joint_position&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::node_set_joint_positionRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::node_set_joint_position >::value();
  }
  static const char* value(const ::webots_ros::node_set_joint_positionRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::node_set_joint_positionRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::node_set_joint_position >::value();
  }
  static const char* value(const ::webots_ros::node_set_joint_positionRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::node_set_joint_positionResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::node_set_joint_position >::value();
  }
  static const char* value(const ::webots_ros::node_set_joint_positionResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::node_set_joint_positionResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::node_set_joint_position >::value();
  }
  static const char* value(const ::webots_ros::node_set_joint_positionResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_NODE_SET_JOINT_POSITION_H
