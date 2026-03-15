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

#ifndef WEBOTS_ROS_MESSAGE_LIDAR_GET_LAYER_RANGE_IMAGE_H
#define WEBOTS_ROS_MESSAGE_LIDAR_GET_LAYER_RANGE_IMAGE_H

#include "ros/service_traits.h"

#include "lidar_get_layer_range_imageRequest.h"
#include "lidar_get_layer_range_imageResponse.h"

namespace webots_ros
{

struct lidar_get_layer_range_image
{

typedef lidar_get_layer_range_imageRequest Request;
typedef lidar_get_layer_range_imageResponse Response;
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
struct MD5Sum< ::webots_ros::lidar_get_layer_range_image > {
  static const char* value()
  {
    return "4d25c95147eb8b7728942d09e84dc175";
  }

  static const char* value(const ::webots_ros::lidar_get_layer_range_image&) { return value(); }
};

template<>
struct DataType< ::webots_ros::lidar_get_layer_range_image > {
  static const char* value()
  {
    return "webots_ros/lidar_get_layer_range_image";
  }

  static const char* value(const ::webots_ros::lidar_get_layer_range_image&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::lidar_get_layer_range_imageRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::lidar_get_layer_range_image >::value();
  }
  static const char* value(const ::webots_ros::lidar_get_layer_range_imageRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::lidar_get_layer_range_imageRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::lidar_get_layer_range_image >::value();
  }
  static const char* value(const ::webots_ros::lidar_get_layer_range_imageRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::lidar_get_layer_range_imageResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::lidar_get_layer_range_image >::value();
  }
  static const char* value(const ::webots_ros::lidar_get_layer_range_imageResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::lidar_get_layer_range_imageResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::lidar_get_layer_range_image >::value();
  }
  static const char* value(const ::webots_ros::lidar_get_layer_range_imageResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_LIDAR_GET_LAYER_RANGE_IMAGE_H
