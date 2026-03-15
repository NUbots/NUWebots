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

#ifndef WEBOTS_ROS_MESSAGE_NODE_ADD_FORCE_WITH_OFFSET_H
#define WEBOTS_ROS_MESSAGE_NODE_ADD_FORCE_WITH_OFFSET_H

#include "ros/service_traits.h"

#include "node_add_force_with_offsetRequest.h"
#include "node_add_force_with_offsetResponse.h"

namespace webots_ros
{

struct node_add_force_with_offset
{

typedef node_add_force_with_offsetRequest Request;
typedef node_add_force_with_offsetResponse Response;
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
struct MD5Sum< ::webots_ros::node_add_force_with_offset > {
  static const char* value()
  {
    return "7bbdb8bfcc982af458fd685526128ebd";
  }

  static const char* value(const ::webots_ros::node_add_force_with_offset&) { return value(); }
};

template<>
struct DataType< ::webots_ros::node_add_force_with_offset > {
  static const char* value()
  {
    return "webots_ros/node_add_force_with_offset";
  }

  static const char* value(const ::webots_ros::node_add_force_with_offset&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::node_add_force_with_offsetRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::node_add_force_with_offset >::value();
  }
  static const char* value(const ::webots_ros::node_add_force_with_offsetRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::node_add_force_with_offsetRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::node_add_force_with_offset >::value();
  }
  static const char* value(const ::webots_ros::node_add_force_with_offsetRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::node_add_force_with_offsetResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::node_add_force_with_offset >::value();
  }
  static const char* value(const ::webots_ros::node_add_force_with_offsetResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::node_add_force_with_offsetResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::node_add_force_with_offset >::value();
  }
  static const char* value(const ::webots_ros::node_add_force_with_offsetResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_NODE_ADD_FORCE_WITH_OFFSET_H
