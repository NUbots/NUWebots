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

#ifndef WEBOTS_ROS_MESSAGE_DISPLAY_IMAGE_NEWREQUEST_H
#define WEBOTS_ROS_MESSAGE_DISPLAY_IMAGE_NEWREQUEST_H

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
struct display_image_newRequest_
{
  typedef display_image_newRequest_<ContainerAllocator> Type;

  display_image_newRequest_()
    : width(0)
    , height(0)
    , data()
    , format(0)  {
    }
  display_image_newRequest_(const ContainerAllocator& _alloc)
    : width(0)
    , height(0)
    , data(_alloc)
    , format(0)  {
    }

   typedef int32_t  _width_type;
  _width_type width;

   typedef int32_t  _height_type;
  _height_type height;

   typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _data_type;
  _data_type data;

   typedef int32_t  _format_type;
  _format_type format;



  typedef boost::shared_ptr< ::webots_ros::display_image_newRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::webots_ros::display_image_newRequest_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

};

typedef ::webots_ros::display_image_newRequest_<std::allocator<void> > display_image_newRequest;

typedef boost::shared_ptr< ::webots_ros::display_image_newRequest > display_image_newRequestPtr;
typedef boost::shared_ptr< ::webots_ros::display_image_newRequest const> display_image_newRequestConstPtr;

// constants requiring out of line definition

template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::webots_ros::display_image_newRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::webots_ros::display_image_newRequest_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::webots_ros::display_image_newRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::display_image_newRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::display_image_newRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::display_image_newRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::display_image_newRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::display_image_newRequest_<ContainerAllocator> const>
  : FalseType
  { };

template<class ContainerAllocator>
struct MD5Sum< ::webots_ros::display_image_newRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "80a036f354960d09033ab0f8d6dffcf7";
  }

  static const char* value(const ::webots_ros::display_image_newRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf9df5232b65af94fULL;
  static const uint64_t static_value2 = 0x73f79fe6d84301bbULL;
};

template<class ContainerAllocator>
struct DataType< ::webots_ros::display_image_newRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "webots_ros/display_image_newRequest";
  }

  static const char* value(const ::webots_ros::display_image_newRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::webots_ros::display_image_newRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 width\n\
int32 height\n\
char[] data\n\
int32 format\n\\n\
\n\
";
  }

  static const char* value(const ::webots_ros::display_image_newRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::webots_ros::display_image_newRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
            stream.next(m.width);
      stream.next(m.height);
      stream.next(m.data);
      stream.next(m.format);

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
struct Printer< ::webots_ros::display_image_newRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::webots_ros::display_image_newRequest_<ContainerAllocator>& v)
  {
        s << indent << "width: ";
    Printer<int32_t>::stream(s, indent + "  ", v.width);
    s << indent << "height: ";
    Printer<int32_t>::stream(s, indent + "  ", v.height);
    s << indent << "data[]: ";
    for (size_t i = 0; i < v.data.size(); ++i)
    {
    	s << indent << "  data[" << i << "]: ";
    	Printer<uint8_t>::stream(s, indent + "  ", v.data[i]);
    }
    s << indent << "format: ";
    Printer<int32_t>::stream(s, indent + "  ", v.format);

  }
};

} // namespace message_operations
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_DISPLAY_IMAGE_NEWREQUEST_H
