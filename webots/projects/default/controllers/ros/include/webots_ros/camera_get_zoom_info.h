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

#ifndef WEBOTS_ROS_MESSAGE_CAMERA_GET_ZOOM_INFO_H
#define WEBOTS_ROS_MESSAGE_CAMERA_GET_ZOOM_INFO_H

#include "ros/service_traits.h"

#include "camera_get_zoom_infoRequest.h"
#include "camera_get_zoom_infoResponse.h"

namespace webots_ros
{

struct camera_get_zoom_info
{

typedef camera_get_zoom_infoRequest Request;
typedef camera_get_zoom_infoResponse Response;
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
struct MD5Sum< ::webots_ros::camera_get_zoom_info > {
  static const char* value()
  {
    return "5cc977379b136acabbe49ef90364861a";
  }

  static const char* value(const ::webots_ros::camera_get_zoom_info&) { return value(); }
};

template<>
struct DataType< ::webots_ros::camera_get_zoom_info > {
  static const char* value()
  {
    return "webots_ros/camera_get_zoom_info";
  }

  static const char* value(const ::webots_ros::camera_get_zoom_info&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::camera_get_zoom_infoRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::camera_get_zoom_info >::value();
  }
  static const char* value(const ::webots_ros::camera_get_zoom_infoRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::camera_get_zoom_infoRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::camera_get_zoom_info >::value();
  }
  static const char* value(const ::webots_ros::camera_get_zoom_infoRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::camera_get_zoom_infoResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::camera_get_zoom_info >::value();
  }
  static const char* value(const ::webots_ros::camera_get_zoom_infoResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::camera_get_zoom_infoResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::camera_get_zoom_info >::value();
  }
  static const char* value(const ::webots_ros::camera_get_zoom_infoResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_CAMERA_GET_ZOOM_INFO_H
