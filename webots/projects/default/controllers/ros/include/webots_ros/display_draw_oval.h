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

#ifndef WEBOTS_ROS_MESSAGE_DISPLAY_DRAW_OVAL_H
#define WEBOTS_ROS_MESSAGE_DISPLAY_DRAW_OVAL_H

#include "ros/service_traits.h"

#include "display_draw_ovalRequest.h"
#include "display_draw_ovalResponse.h"

namespace webots_ros
{

struct display_draw_oval
{

typedef display_draw_ovalRequest Request;
typedef display_draw_ovalResponse Response;
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
struct MD5Sum< ::webots_ros::display_draw_oval > {
  static const char* value()
  {
    return "257804d9f2e4639bae589e190802d29f";
  }

  static const char* value(const ::webots_ros::display_draw_oval&) { return value(); }
};

template<>
struct DataType< ::webots_ros::display_draw_oval > {
  static const char* value()
  {
    return "webots_ros/display_draw_oval";
  }

  static const char* value(const ::webots_ros::display_draw_oval&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::display_draw_ovalRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::display_draw_oval >::value();
  }
  static const char* value(const ::webots_ros::display_draw_ovalRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::display_draw_ovalRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::display_draw_oval >::value();
  }
  static const char* value(const ::webots_ros::display_draw_ovalRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::display_draw_ovalResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::display_draw_oval >::value();
  }
  static const char* value(const ::webots_ros::display_draw_ovalResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::display_draw_ovalResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::display_draw_oval >::value();
  }
  static const char* value(const ::webots_ros::display_draw_ovalResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_DISPLAY_DRAW_OVAL_H
