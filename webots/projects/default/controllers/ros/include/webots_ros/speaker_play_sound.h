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

#ifndef WEBOTS_ROS_MESSAGE_SPEAKER_PLAY_SOUND_H
#define WEBOTS_ROS_MESSAGE_SPEAKER_PLAY_SOUND_H

#include "ros/service_traits.h"

#include "speaker_play_soundRequest.h"
#include "speaker_play_soundResponse.h"

namespace webots_ros
{

struct speaker_play_sound
{

typedef speaker_play_soundRequest Request;
typedef speaker_play_soundResponse Response;
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
struct MD5Sum< ::webots_ros::speaker_play_sound > {
  static const char* value()
  {
    return "9c17e6742fccca17f3542e68a9800dd3";
  }

  static const char* value(const ::webots_ros::speaker_play_sound&) { return value(); }
};

template<>
struct DataType< ::webots_ros::speaker_play_sound > {
  static const char* value()
  {
    return "webots_ros/speaker_play_sound";
  }

  static const char* value(const ::webots_ros::speaker_play_sound&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::speaker_play_soundRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::speaker_play_sound >::value();
  }
  static const char* value(const ::webots_ros::speaker_play_soundRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::speaker_play_soundRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::speaker_play_sound >::value();
  }
  static const char* value(const ::webots_ros::speaker_play_soundRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::speaker_play_soundResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::speaker_play_sound >::value();
  }
  static const char* value(const ::webots_ros::speaker_play_soundResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::speaker_play_soundResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::speaker_play_sound >::value();
  }
  static const char* value(const ::webots_ros::speaker_play_soundResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_SPEAKER_PLAY_SOUND_H
