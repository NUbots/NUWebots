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

#ifndef WEBOTS_ROS_MESSAGE_NODE_GET_NUMBER_OF_FIELDS_H
#define WEBOTS_ROS_MESSAGE_NODE_GET_NUMBER_OF_FIELDS_H

#include "ros/service_traits.h"

#include "node_get_number_of_fieldsRequest.h"
#include "node_get_number_of_fieldsResponse.h"

namespace webots_ros
{

struct node_get_number_of_fields
{

typedef node_get_number_of_fieldsRequest Request;
typedef node_get_number_of_fieldsResponse Response;
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
struct MD5Sum< ::webots_ros::node_get_number_of_fields > {
  static const char* value()
  {
    return "921bd6384f46ac8733931a50b9739877";
  }

  static const char* value(const ::webots_ros::node_get_number_of_fields&) { return value(); }
};

template<>
struct DataType< ::webots_ros::node_get_number_of_fields > {
  static const char* value()
  {
    return "webots_ros/node_get_number_of_fields";
  }

  static const char* value(const ::webots_ros::node_get_number_of_fields&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::node_get_number_of_fieldsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::node_get_number_of_fields >::value();
  }
  static const char* value(const ::webots_ros::node_get_number_of_fieldsRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::node_get_number_of_fieldsRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::node_get_number_of_fields >::value();
  }
  static const char* value(const ::webots_ros::node_get_number_of_fieldsRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::node_get_number_of_fieldsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::node_get_number_of_fields >::value();
  }
  static const char* value(const ::webots_ros::node_get_number_of_fieldsResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::node_get_number_of_fieldsResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::node_get_number_of_fields >::value();
  }
  static const char* value(const ::webots_ros::node_get_number_of_fieldsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_NODE_GET_NUMBER_OF_FIELDS_H
