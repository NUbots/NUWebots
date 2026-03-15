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

#ifndef WEBOTS_ROS_MESSAGE_ROBOT_WAIT_FOR_USER_INPUT_EVENTREQUEST_H
#define WEBOTS_ROS_MESSAGE_ROBOT_WAIT_FOR_USER_INPUT_EVENTREQUEST_H

#include <string>
#include <vector>
#include <map>

#include "ros/types.h"
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"



namespace webots_ros
{
template <class ContainerAllocator>
struct robot_wait_for_user_input_eventRequest_
{
  typedef robot_wait_for_user_input_eventRequest_<ContainerAllocator> Type;

  robot_wait_for_user_input_eventRequest_()
    : eventType(0)
    , timeout(0)  {
    }
  robot_wait_for_user_input_eventRequest_(const ContainerAllocator& _alloc)
    : eventType(0)
    , timeout(0)  {
    }

   typedef int32_t  _eventType_type;
  _eventType_type eventType;

   typedef int32_t  _timeout_type;
  _timeout_type timeout;



  typedef boost::shared_ptr< ::webots_ros::robot_wait_for_user_input_eventRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::webots_ros::robot_wait_for_user_input_eventRequest_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

};

typedef ::webots_ros::robot_wait_for_user_input_eventRequest_<std::allocator<void> > robot_wait_for_user_input_eventRequest;

typedef boost::shared_ptr< ::webots_ros::robot_wait_for_user_input_eventRequest > robot_wait_for_user_input_eventRequestPtr;
typedef boost::shared_ptr< ::webots_ros::robot_wait_for_user_input_eventRequest const> robot_wait_for_user_input_eventRequestConstPtr;

// constants requiring out of line definition

template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::webots_ros::robot_wait_for_user_input_eventRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::webots_ros::robot_wait_for_user_input_eventRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace webots_ros

namespace ros
{
namespace message_traits
{

// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/groovy/share/std_msgs/msg'], 'webots_ros

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::robot_wait_for_user_input_eventRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::robot_wait_for_user_input_eventRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::robot_wait_for_user_input_eventRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::robot_wait_for_user_input_eventRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::robot_wait_for_user_input_eventRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::robot_wait_for_user_input_eventRequest_<ContainerAllocator> const>
  : FalseType
  { };

template<class ContainerAllocator>
struct MD5Sum< ::webots_ros::robot_wait_for_user_input_eventRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "117ed5fb1f5c37c2a398a290a90fc55d";
  }

  static const char* value(const ::webots_ros::robot_wait_for_user_input_eventRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf9df5232b65af94fULL;
  static const uint64_t static_value2 = 0x73f79fe6d84301bbULL;
};

template<class ContainerAllocator>
struct DataType< ::webots_ros::robot_wait_for_user_input_eventRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "webots_ros/robot_wait_for_user_input_eventRequest";
  }

  static const char* value(const ::webots_ros::robot_wait_for_user_input_eventRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::webots_ros::robot_wait_for_user_input_eventRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 eventType\n\
int32 timeout\n\\n\
\n\
";
  }

  static const char* value(const ::webots_ros::robot_wait_for_user_input_eventRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::webots_ros::robot_wait_for_user_input_eventRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
            stream.next(m.eventType);
      stream.next(m.timeout);

    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  };

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::webots_ros::robot_wait_for_user_input_eventRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::webots_ros::robot_wait_for_user_input_eventRequest_<ContainerAllocator>& v)
  {
        s << indent << "eventType: ";
    Printer<int32_t>::stream(s, indent + "  ", v.eventType);
    s << indent << "timeout: ";
    Printer<int32_t>::stream(s, indent + "  ", v.timeout);

  }
};

} // namespace message_operations
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_ROBOT_WAIT_FOR_USER_INPUT_EVENTREQUEST_H
