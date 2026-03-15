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

#ifndef WEBOTS_ROS_MESSAGE_SUPERVISOR_MOVIE_STOP_RECORDING_H
#define WEBOTS_ROS_MESSAGE_SUPERVISOR_MOVIE_STOP_RECORDING_H

#include "ros/service_traits.h"

#include "supervisor_movie_stop_recordingRequest.h"
#include "supervisor_movie_stop_recordingResponse.h"

namespace webots_ros
{

struct supervisor_movie_stop_recording
{

typedef supervisor_movie_stop_recordingRequest Request;
typedef supervisor_movie_stop_recordingResponse Response;
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
struct MD5Sum< ::webots_ros::supervisor_movie_stop_recording > {
  static const char* value()
  {
    return "2a42f8e83a0d1e81ff806bb0cbf4e594";
  }

  static const char* value(const ::webots_ros::supervisor_movie_stop_recording&) { return value(); }
};

template<>
struct DataType< ::webots_ros::supervisor_movie_stop_recording > {
  static const char* value()
  {
    return "webots_ros/supervisor_movie_stop_recording";
  }

  static const char* value(const ::webots_ros::supervisor_movie_stop_recording&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::supervisor_movie_stop_recordingRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::supervisor_movie_stop_recording >::value();
  }
  static const char* value(const ::webots_ros::supervisor_movie_stop_recordingRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::supervisor_movie_stop_recordingRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::supervisor_movie_stop_recording >::value();
  }
  static const char* value(const ::webots_ros::supervisor_movie_stop_recordingRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::supervisor_movie_stop_recordingResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::supervisor_movie_stop_recording >::value();
  }
  static const char* value(const ::webots_ros::supervisor_movie_stop_recordingResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::supervisor_movie_stop_recordingResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::supervisor_movie_stop_recording >::value();
  }
  static const char* value(const ::webots_ros::supervisor_movie_stop_recordingResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_SUPERVISOR_MOVIE_STOP_RECORDING_H
