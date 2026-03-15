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

#ifndef WEBOTS_ROS_MESSAGE_DISPLAY_DRAW_POLYGONREQUEST_H
#define WEBOTS_ROS_MESSAGE_DISPLAY_DRAW_POLYGONREQUEST_H

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
struct display_draw_polygonRequest_
{
  typedef display_draw_polygonRequest_<ContainerAllocator> Type;

  display_draw_polygonRequest_()
    : x(0)
    , y(0)
    , size(0)  {
    }
  display_draw_polygonRequest_(const ContainerAllocator& _alloc)
    : x(0)
    , y(0)
    , size(0)  {
    }

   typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _x_type;
  _x_type x;

   typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _y_type;
  _y_type y;

   typedef int32_t  _size_type;
  _size_type size;



  typedef boost::shared_ptr< ::webots_ros::display_draw_polygonRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::webots_ros::display_draw_polygonRequest_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

};

typedef ::webots_ros::display_draw_polygonRequest_<std::allocator<void> > display_draw_polygonRequest;

typedef boost::shared_ptr< ::webots_ros::display_draw_polygonRequest > display_draw_polygonRequestPtr;
typedef boost::shared_ptr< ::webots_ros::display_draw_polygonRequest const> display_draw_polygonRequestConstPtr;

// constants requiring out of line definition

template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::webots_ros::display_draw_polygonRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::webots_ros::display_draw_polygonRequest_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::webots_ros::display_draw_polygonRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::display_draw_polygonRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::display_draw_polygonRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::display_draw_polygonRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::display_draw_polygonRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::display_draw_polygonRequest_<ContainerAllocator> const>
  : FalseType
  { };

template<class ContainerAllocator>
struct MD5Sum< ::webots_ros::display_draw_polygonRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dfa1f0ed4de4cc45db83bc60add40eb6";
  }

  static const char* value(const ::webots_ros::display_draw_polygonRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf9df5232b65af94fULL;
  static const uint64_t static_value2 = 0x73f79fe6d84301bbULL;
};

template<class ContainerAllocator>
struct DataType< ::webots_ros::display_draw_polygonRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "webots_ros/display_draw_polygonRequest";
  }

  static const char* value(const ::webots_ros::display_draw_polygonRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::webots_ros::display_draw_polygonRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32[] x\n\
int32[] y\n\
int32 size\n\\n\
\n\
";
  }

  static const char* value(const ::webots_ros::display_draw_polygonRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::webots_ros::display_draw_polygonRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
            stream.next(m.x);
      stream.next(m.y);
      stream.next(m.size);

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
struct Printer< ::webots_ros::display_draw_polygonRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::webots_ros::display_draw_polygonRequest_<ContainerAllocator>& v)
  {
        s << indent << "x[]: ";
    for (size_t i = 0; i < v.x.size(); ++i)
    {
    	s << indent << "  x[" << i << "]: ";
    	Printer<int32_t>::stream(s, indent + "  ", v.x[i]);
    }
    s << indent << "y[]: ";
    for (size_t i = 0; i < v.y.size(); ++i)
    {
    	s << indent << "  y[" << i << "]: ";
    	Printer<int32_t>::stream(s, indent + "  ", v.y[i]);
    }
    s << indent << "size: ";
    Printer<int32_t>::stream(s, indent + "  ", v.size);

  }
};

} // namespace message_operations
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_DISPLAY_DRAW_POLYGONREQUEST_H
