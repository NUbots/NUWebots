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

#ifndef WEBOTS_ROS_MESSAGE_NODE_GET_POSE_H
#define WEBOTS_ROS_MESSAGE_NODE_GET_POSE_H

#include "ros/service_traits.h"

#include "node_get_poseRequest.h"
#include "node_get_poseResponse.h"

namespace webots_ros
{

struct node_get_pose
{

typedef node_get_poseRequest Request;
typedef node_get_poseResponse Response;
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
struct MD5Sum< ::webots_ros::node_get_pose > {
  static const char* value()
  {
    return "c971ec247720786da5fd4aa095defd52";
  }

  static const char* value(const ::webots_ros::node_get_pose&) { return value(); }
};

template<>
struct DataType< ::webots_ros::node_get_pose > {
  static const char* value()
  {
    return "webots_ros/node_get_pose";
  }

  static const char* value(const ::webots_ros::node_get_pose&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::node_get_poseRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::node_get_pose >::value();
  }
  static const char* value(const ::webots_ros::node_get_poseRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::node_get_poseRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::node_get_pose >::value();
  }
  static const char* value(const ::webots_ros::node_get_poseRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::node_get_poseResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::node_get_pose >::value();
  }
  static const char* value(const ::webots_ros::node_get_poseResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::node_get_poseResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::node_get_pose >::value();
  }
  static const char* value(const ::webots_ros::node_get_poseResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_NODE_GET_POSE_H
