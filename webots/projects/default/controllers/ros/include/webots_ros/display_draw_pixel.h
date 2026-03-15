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

#ifndef WEBOTS_ROS_MESSAGE_DISPLAY_DRAW_PIXEL_H
#define WEBOTS_ROS_MESSAGE_DISPLAY_DRAW_PIXEL_H

#include "ros/service_traits.h"

#include "display_draw_pixelRequest.h"
#include "display_draw_pixelResponse.h"

namespace webots_ros
{

struct display_draw_pixel
{

typedef display_draw_pixelRequest Request;
typedef display_draw_pixelResponse Response;
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
struct MD5Sum< ::webots_ros::display_draw_pixel > {
  static const char* value()
  {
    return "e8e87fb7b87ab83a24bea771b7fc11df";
  }

  static const char* value(const ::webots_ros::display_draw_pixel&) { return value(); }
};

template<>
struct DataType< ::webots_ros::display_draw_pixel > {
  static const char* value()
  {
    return "webots_ros/display_draw_pixel";
  }

  static const char* value(const ::webots_ros::display_draw_pixel&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::display_draw_pixelRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::display_draw_pixel >::value();
  }
  static const char* value(const ::webots_ros::display_draw_pixelRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::display_draw_pixelRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::display_draw_pixel >::value();
  }
  static const char* value(const ::webots_ros::display_draw_pixelRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::display_draw_pixelResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::display_draw_pixel >::value();
  }
  static const char* value(const ::webots_ros::display_draw_pixelResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::display_draw_pixelResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::display_draw_pixel >::value();
  }
  static const char* value(const ::webots_ros::display_draw_pixelResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_DISPLAY_DRAW_PIXEL_H
