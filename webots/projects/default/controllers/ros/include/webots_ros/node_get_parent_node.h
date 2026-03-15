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

#ifndef WEBOTS_ROS_MESSAGE_NODE_GET_PARENT_NODE_H
#define WEBOTS_ROS_MESSAGE_NODE_GET_PARENT_NODE_H

#include "ros/service_traits.h"

#include "node_get_parent_nodeRequest.h"
#include "node_get_parent_nodeResponse.h"

namespace webots_ros
{

struct node_get_parent_node
{

typedef node_get_parent_nodeRequest Request;
typedef node_get_parent_nodeResponse Response;
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
struct MD5Sum< ::webots_ros::node_get_parent_node > {
  static const char* value()
  {
    return "818a5363f8ab50910f19b3cf3acc208b";
  }

  static const char* value(const ::webots_ros::node_get_parent_node&) { return value(); }
};

template<>
struct DataType< ::webots_ros::node_get_parent_node > {
  static const char* value()
  {
    return "webots_ros/node_get_parent_node";
  }

  static const char* value(const ::webots_ros::node_get_parent_node&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::node_get_parent_nodeRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::node_get_parent_node >::value();
  }
  static const char* value(const ::webots_ros::node_get_parent_nodeRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::node_get_parent_nodeRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::node_get_parent_node >::value();
  }
  static const char* value(const ::webots_ros::node_get_parent_nodeRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::node_get_parent_nodeResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::node_get_parent_node >::value();
  }
  static const char* value(const ::webots_ros::node_get_parent_nodeResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::node_get_parent_nodeResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::node_get_parent_node >::value();
  }
  static const char* value(const ::webots_ros::node_get_parent_nodeResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_NODE_GET_PARENT_NODE_H
