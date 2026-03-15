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

#ifndef WEBOTS_ROS_MESSAGE_DISPLAY_IMAGE_LOAD_H
#define WEBOTS_ROS_MESSAGE_DISPLAY_IMAGE_LOAD_H

#include "ros/service_traits.h"

#include "display_image_loadRequest.h"
#include "display_image_loadResponse.h"

namespace webots_ros
{

struct display_image_load
{

typedef display_image_loadRequest Request;
typedef display_image_loadResponse Response;
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
struct MD5Sum< ::webots_ros::display_image_load > {
  static const char* value()
  {
    return "5c71ee48be952f1ade34fd3da67710f3";
  }

  static const char* value(const ::webots_ros::display_image_load&) { return value(); }
};

template<>
struct DataType< ::webots_ros::display_image_load > {
  static const char* value()
  {
    return "webots_ros/display_image_load";
  }

  static const char* value(const ::webots_ros::display_image_load&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::display_image_loadRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::display_image_load >::value();
  }
  static const char* value(const ::webots_ros::display_image_loadRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::display_image_loadRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::display_image_load >::value();
  }
  static const char* value(const ::webots_ros::display_image_loadRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::display_image_loadResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::display_image_load >::value();
  }
  static const char* value(const ::webots_ros::display_image_loadResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::display_image_loadResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::display_image_load >::value();
  }
  static const char* value(const ::webots_ros::display_image_loadResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_DISPLAY_IMAGE_LOAD_H
