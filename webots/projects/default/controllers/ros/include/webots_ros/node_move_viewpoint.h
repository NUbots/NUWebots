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

#ifndef WEBOTS_ROS_MESSAGE_NODE_MOVE_VIEWPOINT_H
#define WEBOTS_ROS_MESSAGE_NODE_MOVE_VIEWPOINT_H

#include "ros/service_traits.h"

#include "node_move_viewpointRequest.h"
#include "node_move_viewpointResponse.h"

namespace webots_ros
{

struct node_move_viewpoint
{

typedef node_move_viewpointRequest Request;
typedef node_move_viewpointResponse Response;
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
struct MD5Sum< ::webots_ros::node_move_viewpoint > {
  static const char* value()
  {
    return "594d3b785623c78d3382c6628faa0f8c";
  }

  static const char* value(const ::webots_ros::node_move_viewpoint&) { return value(); }
};

template<>
struct DataType< ::webots_ros::node_move_viewpoint > {
  static const char* value()
  {
    return "webots_ros/node_move_viewpoint";
  }

  static const char* value(const ::webots_ros::node_move_viewpoint&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::node_move_viewpointRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::node_move_viewpoint >::value();
  }
  static const char* value(const ::webots_ros::node_move_viewpointRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::node_move_viewpointRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::node_move_viewpoint >::value();
  }
  static const char* value(const ::webots_ros::node_move_viewpointRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::node_move_viewpointResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::node_move_viewpoint >::value();
  }
  static const char* value(const ::webots_ros::node_move_viewpointResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::node_move_viewpointResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::node_move_viewpoint >::value();
  }
  static const char* value(const ::webots_ros::node_move_viewpointResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_NODE_MOVE_VIEWPOINT_H
