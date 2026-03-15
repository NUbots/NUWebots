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

#ifndef WEBOTS_ROS_MESSAGE_NODE_ENABLE_CONTACT_POINTS_TRACKING_H
#define WEBOTS_ROS_MESSAGE_NODE_ENABLE_CONTACT_POINTS_TRACKING_H

#include "ros/service_traits.h"

#include "node_enable_contact_points_trackingRequest.h"
#include "node_enable_contact_points_trackingResponse.h"

namespace webots_ros
{

struct node_enable_contact_points_tracking
{

typedef node_enable_contact_points_trackingRequest Request;
typedef node_enable_contact_points_trackingResponse Response;
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
struct MD5Sum< ::webots_ros::node_enable_contact_points_tracking > {
  static const char* value()
  {
    return "dccb3b91ba4f7b41232e5752a12d0fe4";
  }

  static const char* value(const ::webots_ros::node_enable_contact_points_tracking&) { return value(); }
};

template<>
struct DataType< ::webots_ros::node_enable_contact_points_tracking > {
  static const char* value()
  {
    return "webots_ros/node_enable_contact_points_tracking";
  }

  static const char* value(const ::webots_ros::node_enable_contact_points_tracking&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::node_enable_contact_points_trackingRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::node_enable_contact_points_tracking >::value();
  }
  static const char* value(const ::webots_ros::node_enable_contact_points_trackingRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::node_enable_contact_points_trackingRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::node_enable_contact_points_tracking >::value();
  }
  static const char* value(const ::webots_ros::node_enable_contact_points_trackingRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::node_enable_contact_points_trackingResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::node_enable_contact_points_tracking >::value();
  }
  static const char* value(const ::webots_ros::node_enable_contact_points_trackingResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::node_enable_contact_points_trackingResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::node_enable_contact_points_tracking >::value();
  }
  static const char* value(const ::webots_ros::node_enable_contact_points_trackingResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_NODE_ENABLE_CONTACT_POINTS_TRACKING_H
