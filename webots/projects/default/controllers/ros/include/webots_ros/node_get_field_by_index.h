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

#ifndef WEBOTS_ROS_MESSAGE_NODE_GET_FIELD_BY_INDEX_H
#define WEBOTS_ROS_MESSAGE_NODE_GET_FIELD_BY_INDEX_H

#include "ros/service_traits.h"

#include "node_get_field_by_indexRequest.h"
#include "node_get_field_by_indexResponse.h"

namespace webots_ros
{

struct node_get_field_by_index
{

typedef node_get_field_by_indexRequest Request;
typedef node_get_field_by_indexResponse Response;
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
struct MD5Sum< ::webots_ros::node_get_field_by_index > {
  static const char* value()
  {
    return "24bbfc7fa321c886f58114d630e6f1fb";
  }

  static const char* value(const ::webots_ros::node_get_field_by_index&) { return value(); }
};

template<>
struct DataType< ::webots_ros::node_get_field_by_index > {
  static const char* value()
  {
    return "webots_ros/node_get_field_by_index";
  }

  static const char* value(const ::webots_ros::node_get_field_by_index&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::node_get_field_by_indexRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::node_get_field_by_index >::value();
  }
  static const char* value(const ::webots_ros::node_get_field_by_indexRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::node_get_field_by_indexRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::node_get_field_by_index >::value();
  }
  static const char* value(const ::webots_ros::node_get_field_by_indexRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::node_get_field_by_indexResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::node_get_field_by_index >::value();
  }
  static const char* value(const ::webots_ros::node_get_field_by_indexResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::node_get_field_by_indexResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::node_get_field_by_index >::value();
  }
  static const char* value(const ::webots_ros::node_get_field_by_indexResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_NODE_GET_FIELD_BY_INDEX_H
