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

#ifndef WEBOTS_ROS_MESSAGE_SPEAKER_IS_SOUND_PLAYING_H
#define WEBOTS_ROS_MESSAGE_SPEAKER_IS_SOUND_PLAYING_H

#include "ros/service_traits.h"

#include "speaker_is_sound_playingRequest.h"
#include "speaker_is_sound_playingResponse.h"

namespace webots_ros
{

struct speaker_is_sound_playing
{

typedef speaker_is_sound_playingRequest Request;
typedef speaker_is_sound_playingResponse Response;
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
struct MD5Sum< ::webots_ros::speaker_is_sound_playing > {
  static const char* value()
  {
    return "5e90e3a791904b68b561b7067a8d366b";
  }

  static const char* value(const ::webots_ros::speaker_is_sound_playing&) { return value(); }
};

template<>
struct DataType< ::webots_ros::speaker_is_sound_playing > {
  static const char* value()
  {
    return "webots_ros/speaker_is_sound_playing";
  }

  static const char* value(const ::webots_ros::speaker_is_sound_playing&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::speaker_is_sound_playingRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::speaker_is_sound_playing >::value();
  }
  static const char* value(const ::webots_ros::speaker_is_sound_playingRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::speaker_is_sound_playingRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::speaker_is_sound_playing >::value();
  }
  static const char* value(const ::webots_ros::speaker_is_sound_playingRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::speaker_is_sound_playingResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::speaker_is_sound_playing >::value();
  }
  static const char* value(const ::webots_ros::speaker_is_sound_playingResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::speaker_is_sound_playingResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::speaker_is_sound_playing >::value();
  }
  static const char* value(const ::webots_ros::speaker_is_sound_playingResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_SPEAKER_IS_SOUND_PLAYING_H
