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

#ifndef WEBOTS_ROS_MESSAGE_NODE_GET_CENTER_OF_MASSRESPONSE_H
#define WEBOTS_ROS_MESSAGE_NODE_GET_CENTER_OF_MASSRESPONSE_H

#include <string>
#include <vector>
#include <map>

#include "ros/types.h"
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"

#include <geometry_msgs/Point.h>


namespace webots_ros
{
template <class ContainerAllocator>
struct node_get_center_of_massResponse_
{
  typedef node_get_center_of_massResponse_<ContainerAllocator> Type;

  node_get_center_of_massResponse_()
    : centerOfMass()  {
    }
  node_get_center_of_massResponse_(const ContainerAllocator& _alloc)
    : centerOfMass(_alloc)  {
    }

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _centerOfMass_type;
  _centerOfMass_type centerOfMass;



  typedef boost::shared_ptr< ::webots_ros::node_get_center_of_massResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::webots_ros::node_get_center_of_massResponse_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

};

typedef ::webots_ros::node_get_center_of_massResponse_<std::allocator<void> > node_get_center_of_massResponse;

typedef boost::shared_ptr< ::webots_ros::node_get_center_of_massResponse > node_get_center_of_massResponsePtr;
typedef boost::shared_ptr< ::webots_ros::node_get_center_of_massResponse const> node_get_center_of_massResponseConstPtr;

// constants requiring out of line definition

template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::webots_ros::node_get_center_of_massResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::webots_ros::node_get_center_of_massResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace webots_ros

namespace ros
{
namespace message_traits
{

// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/groovy/share/std_msgs/msg'], 'webots_ros': ['/home/simon/my_Webots_Projects/controllers/ros_controller/catkin_ws/src/webots_ros/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::node_get_center_of_massResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::node_get_center_of_massResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::node_get_center_of_massResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::node_get_center_of_massResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::node_get_center_of_massResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::node_get_center_of_massResponse_<ContainerAllocator> const>
  : FalseType
  { };

template<class ContainerAllocator>
struct MD5Sum< ::webots_ros::node_get_center_of_massResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "08baa9a118e7086ad4f02f73fae3dd80";
  }

  static const char* value(const ::webots_ros::node_get_center_of_massResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0147e4f36cba5cdaULL;
  static const uint64_t static_value2 = 0x7fa39c089e493413ULL;
};

template<class ContainerAllocator>
struct DataType< ::webots_ros::node_get_center_of_massResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "webots_ros/node_get_center_of_massResponse";
  }

  static const char* value(const ::webots_ros::node_get_center_of_massResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::webots_ros::node_get_center_of_massResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Point centerOfMass\n\\n\
\n\
";
  }

  static const char* value(const ::webots_ros::node_get_center_of_massResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::webots_ros::node_get_center_of_massResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
            stream.next(m.centerOfMass);

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
struct Printer< ::webots_ros::node_get_center_of_massResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::webots_ros::node_get_center_of_massResponse_<ContainerAllocator>& v)
  {
        s << indent << "centerOfMass: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.centerOfMass);

  }
};

} // namespace message_operations
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_NODE_GET_CENTER_OF_MASSRESPONSE_H
