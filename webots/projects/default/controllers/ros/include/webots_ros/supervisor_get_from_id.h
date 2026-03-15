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

#ifndef WEBOTS_ROS_MESSAGE_SUPERVISOR_GET_FROM_ID_H
#define WEBOTS_ROS_MESSAGE_SUPERVISOR_GET_FROM_ID_H

#include "ros/service_traits.h"

#include "supervisor_get_from_idRequest.h"
#include "supervisor_get_from_idResponse.h"

namespace webots_ros
{

struct supervisor_get_from_id
{

typedef supervisor_get_from_idRequest Request;
typedef supervisor_get_from_idResponse Response;
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
struct MD5Sum< ::webots_ros::supervisor_get_from_id > {
  static const char* value()
  {
    return "398e4930ac99b6ae9bc51fa66b0d9846";
  }

  static const char* value(const ::webots_ros::supervisor_get_from_id&) { return value(); }
};

template<>
struct DataType< ::webots_ros::supervisor_get_from_id > {
  static const char* value()
  {
    return "webots_ros/supervisor_get_from_id";
  }

  static const char* value(const ::webots_ros::supervisor_get_from_id&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::supervisor_get_from_idRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::supervisor_get_from_id >::value();
  }
  static const char* value(const ::webots_ros::supervisor_get_from_idRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::supervisor_get_from_idRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::supervisor_get_from_id >::value();
  }
  static const char* value(const ::webots_ros::supervisor_get_from_idRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::supervisor_get_from_idResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::supervisor_get_from_id >::value();
  }
  static const char* value(const ::webots_ros::supervisor_get_from_idResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::supervisor_get_from_idResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::supervisor_get_from_id >::value();
  }
  static const char* value(const ::webots_ros::supervisor_get_from_idResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_SUPERVISOR_GET_FROM_ID_H
