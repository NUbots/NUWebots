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

#ifndef WEBOTS_ROS_MESSAGE_DISPLAY_DRAW_TEXT_H
#define WEBOTS_ROS_MESSAGE_DISPLAY_DRAW_TEXT_H

#include "ros/service_traits.h"

#include "display_draw_textRequest.h"
#include "display_draw_textResponse.h"

namespace webots_ros
{

struct display_draw_text
{

typedef display_draw_textRequest Request;
typedef display_draw_textResponse Response;
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
struct MD5Sum< ::webots_ros::display_draw_text > {
  static const char* value()
  {
    return "b82c6acdec67a202bbbbd0b3aba5aa0c";
  }

  static const char* value(const ::webots_ros::display_draw_text&) { return value(); }
};

template<>
struct DataType< ::webots_ros::display_draw_text > {
  static const char* value()
  {
    return "webots_ros/display_draw_text";
  }

  static const char* value(const ::webots_ros::display_draw_text&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::display_draw_textRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::display_draw_text >::value();
  }
  static const char* value(const ::webots_ros::display_draw_textRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::display_draw_textRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::display_draw_text >::value();
  }
  static const char* value(const ::webots_ros::display_draw_textRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::display_draw_textResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::display_draw_text >::value();
  }
  static const char* value(const ::webots_ros::display_draw_textResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::display_draw_textResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::display_draw_text >::value();
  }
  static const char* value(const ::webots_ros::display_draw_textResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_DISPLAY_DRAW_TEXT_H
