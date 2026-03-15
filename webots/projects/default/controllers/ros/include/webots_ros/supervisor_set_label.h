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

#ifndef WEBOTS_ROS_MESSAGE_SUPERVISOR_SET_LABEL_H
#define WEBOTS_ROS_MESSAGE_SUPERVISOR_SET_LABEL_H

#include "ros/service_traits.h"

#include "supervisor_set_labelRequest.h"
#include "supervisor_set_labelResponse.h"

namespace webots_ros
{

struct supervisor_set_label
{

typedef supervisor_set_labelRequest Request;
typedef supervisor_set_labelResponse Response;
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
struct MD5Sum< ::webots_ros::supervisor_set_label > {
  static const char* value()
  {
    return "5ee78a04723ef11d3847c8d8c557058c";
  }

  static const char* value(const ::webots_ros::supervisor_set_label&) { return value(); }
};

template<>
struct DataType< ::webots_ros::supervisor_set_label > {
  static const char* value()
  {
    return "webots_ros/supervisor_set_label";
  }

  static const char* value(const ::webots_ros::supervisor_set_label&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::supervisor_set_labelRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::supervisor_set_label >::value();
  }
  static const char* value(const ::webots_ros::supervisor_set_labelRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::supervisor_set_labelRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::supervisor_set_label >::value();
  }
  static const char* value(const ::webots_ros::supervisor_set_labelRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::supervisor_set_labelResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::supervisor_set_label >::value();
  }
  static const char* value(const ::webots_ros::supervisor_set_labelResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::supervisor_set_labelResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::supervisor_set_label >::value();
  }
  static const char* value(const ::webots_ros::supervisor_set_labelResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_SUPERVISOR_SET_LABEL_H
