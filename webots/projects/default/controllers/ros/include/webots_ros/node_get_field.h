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

#ifndef WEBOTS_ROS_MESSAGE_NODE_GET_FIELD_H
#define WEBOTS_ROS_MESSAGE_NODE_GET_FIELD_H

#include "ros/service_traits.h"

#include "node_get_fieldRequest.h"
#include "node_get_fieldResponse.h"

namespace webots_ros
{

struct node_get_field
{

typedef node_get_fieldRequest Request;
typedef node_get_fieldResponse Response;
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
struct MD5Sum< ::webots_ros::node_get_field > {
  static const char* value()
  {
    return "80105ddcb86dd98488ad3cf099686b86";
  }

  static const char* value(const ::webots_ros::node_get_field&) { return value(); }
};

template<>
struct DataType< ::webots_ros::node_get_field > {
  static const char* value()
  {
    return "webots_ros/node_get_field";
  }

  static const char* value(const ::webots_ros::node_get_field&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::node_get_fieldRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::node_get_field >::value();
  }
  static const char* value(const ::webots_ros::node_get_fieldRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::node_get_fieldRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::node_get_field >::value();
  }
  static const char* value(const ::webots_ros::node_get_fieldRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::node_get_fieldResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::node_get_field >::value();
  }
  static const char* value(const ::webots_ros::node_get_fieldResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::node_get_fieldResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::node_get_field >::value();
  }
  static const char* value(const ::webots_ros::node_get_fieldResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_NODE_GET_FIELD_H
