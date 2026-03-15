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

#ifndef WEBOTS_ROS_MESSAGE_MOUSE_GET_STATERESPONSE_H
#define WEBOTS_ROS_MESSAGE_MOUSE_GET_STATERESPONSE_H

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
struct mouse_get_stateResponse_
{
  typedef mouse_get_stateResponse_<ContainerAllocator> Type;

  mouse_get_stateResponse_()
    : left(0)
    , middle(0)
    , right(0)
    , u(0.0)
    , v(0.0)
    , x(0.0)
    , y(0.0)
    , z(0.0)  {
    }
  mouse_get_stateResponse_(const ContainerAllocator& _alloc)
    : left(0)
    , middle(0)
    , right(0)
    , u(0.0)
    , v(0.0)
    , x(0.0)
    , y(0.0)
    , z(0.0)  {
    }

   typedef uint8_t  _left_type;
  _left_type left;

   typedef uint8_t  _middle_type;
  _middle_type middle;

   typedef uint8_t  _right_type;
  _right_type right;

   typedef double  _u_type;
  _u_type u;

   typedef double  _v_type;
  _v_type v;

   typedef double  _x_type;
  _x_type x;

   typedef double  _y_type;
  _y_type y;

   typedef double  _z_type;
  _z_type z;



  typedef boost::shared_ptr< ::webots_ros::mouse_get_stateResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::webots_ros::mouse_get_stateResponse_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

};

typedef ::webots_ros::mouse_get_stateResponse_<std::allocator<void> > mouse_get_stateResponse;

typedef boost::shared_ptr< ::webots_ros::mouse_get_stateResponse > mouse_get_stateResponsePtr;
typedef boost::shared_ptr< ::webots_ros::mouse_get_stateResponse const> mouse_get_stateResponseConstPtr;

// constants requiring out of line definition

template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::webots_ros::mouse_get_stateResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::webots_ros::mouse_get_stateResponse_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::webots_ros::mouse_get_stateResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::mouse_get_stateResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::mouse_get_stateResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::mouse_get_stateResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::mouse_get_stateResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::mouse_get_stateResponse_<ContainerAllocator> const>
  : FalseType
  { };

template<class ContainerAllocator>
struct MD5Sum< ::webots_ros::mouse_get_stateResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f4314fc92ab9c1f74ef71e42c0634a99";
  }

  static const char* value(const ::webots_ros::mouse_get_stateResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0147e4f36cba5cdaULL;
  static const uint64_t static_value2 = 0x7fa39c089e493413ULL;
};

template<class ContainerAllocator>
struct DataType< ::webots_ros::mouse_get_stateResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "webots_ros/mouse_get_stateResponse";
  }

  static const char* value(const ::webots_ros::mouse_get_stateResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::webots_ros::mouse_get_stateResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 left\n\
uint8 middle\n\
uint8 right\n\
float64 u\n\
float64 v\n\
float64 x\n\
float64 y\n\
float64 z\n\\n\
\n\
";
  }

  static const char* value(const ::webots_ros::mouse_get_stateResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::webots_ros::mouse_get_stateResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
            stream.next(m.left);
      stream.next(m.middle);
      stream.next(m.right);
      stream.next(m.u);
      stream.next(m.v);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);

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
struct Printer< ::webots_ros::mouse_get_stateResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::webots_ros::mouse_get_stateResponse_<ContainerAllocator>& v)
  {
        s << indent << "left: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.left);
    s << indent << "middle: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.middle);
    s << indent << "right: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.right);
    s << indent << "u: ";
    Printer<double>::stream(s, indent + "  ", v.u);
    s << indent << "v: ";
    Printer<double>::stream(s, indent + "  ", v.v);
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<double>::stream(s, indent + "  ", v.z);

  }
};

} // namespace message_operations
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_MOUSE_GET_STATERESPONSE_H
