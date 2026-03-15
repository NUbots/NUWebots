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

#ifndef WEBOTS_ROS_MESSAGE_NODE_GET_ID_H
#define WEBOTS_ROS_MESSAGE_NODE_GET_ID_H

#include "ros/service_traits.h"

#include "node_get_idRequest.h"
#include "node_get_idResponse.h"

namespace webots_ros
{

struct node_get_id
{

typedef node_get_idRequest Request;
typedef node_get_idResponse Response;
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
struct MD5Sum< ::webots_ros::node_get_id > {
  static const char* value()
  {
    return "16a06b427b76c7c64b73962f4f092416";
  }

  static const char* value(const ::webots_ros::node_get_id&) { return value(); }
};

template<>
struct DataType< ::webots_ros::node_get_id > {
  static const char* value()
  {
    return "webots_ros/node_get_id";
  }

  static const char* value(const ::webots_ros::node_get_id&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::node_get_idRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::node_get_id >::value();
  }
  static const char* value(const ::webots_ros::node_get_idRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::node_get_idRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::node_get_id >::value();
  }
  static const char* value(const ::webots_ros::node_get_idRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::node_get_idResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::node_get_id >::value();
  }
  static const char* value(const ::webots_ros::node_get_idResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::node_get_idResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::node_get_id >::value();
  }
  static const char* value(const ::webots_ros::node_get_idResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_NODE_GET_ID_H
