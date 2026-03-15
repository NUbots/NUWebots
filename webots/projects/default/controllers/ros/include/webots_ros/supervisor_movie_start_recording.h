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

#ifndef WEBOTS_ROS_MESSAGE_SUPERVISOR_MOVIE_START_RECORDING_H
#define WEBOTS_ROS_MESSAGE_SUPERVISOR_MOVIE_START_RECORDING_H

#include "ros/service_traits.h"

#include "supervisor_movie_start_recordingRequest.h"
#include "supervisor_movie_start_recordingResponse.h"

namespace webots_ros
{

struct supervisor_movie_start_recording
{

typedef supervisor_movie_start_recordingRequest Request;
typedef supervisor_movie_start_recordingResponse Response;
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
struct MD5Sum< ::webots_ros::supervisor_movie_start_recording > {
  static const char* value()
  {
    return "96ca298eece1e7a6fe756c404839bdcc";
  }

  static const char* value(const ::webots_ros::supervisor_movie_start_recording&) { return value(); }
};

template<>
struct DataType< ::webots_ros::supervisor_movie_start_recording > {
  static const char* value()
  {
    return "webots_ros/supervisor_movie_start_recording";
  }

  static const char* value(const ::webots_ros::supervisor_movie_start_recording&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::supervisor_movie_start_recordingRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::supervisor_movie_start_recording >::value();
  }
  static const char* value(const ::webots_ros::supervisor_movie_start_recordingRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::supervisor_movie_start_recordingRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::supervisor_movie_start_recording >::value();
  }
  static const char* value(const ::webots_ros::supervisor_movie_start_recordingRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::supervisor_movie_start_recordingResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::supervisor_movie_start_recording >::value();
  }
  static const char* value(const ::webots_ros::supervisor_movie_start_recordingResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::supervisor_movie_start_recordingResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::supervisor_movie_start_recording >::value();
  }
  static const char* value(const ::webots_ros::supervisor_movie_start_recordingResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_SUPERVISOR_MOVIE_START_RECORDING_H
