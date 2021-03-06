/* Auto-generated by genmsg_cpp for file /home/olivier/ros_workspace/nao_behavior_tree/srv/BallPosForHand.srv */
#ifndef NAO_BEHAVIOR_TREE_SERVICE_BALLPOSFORHAND_H
#define NAO_BEHAVIOR_TREE_SERVICE_BALLPOSFORHAND_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "ros/service_traits.h"




namespace nao_behavior_tree
{
template <class ContainerAllocator>
struct BallPosForHandRequest_ {
  typedef BallPosForHandRequest_<ContainerAllocator> Type;

  BallPosForHandRequest_()
  : min_h(0)
  , max_h(0)
  {
  }

  BallPosForHandRequest_(const ContainerAllocator& _alloc)
  : min_h(0)
  , max_h(0)
  {
  }

  typedef int32_t _min_h_type;
  int32_t min_h;

  typedef int32_t _max_h_type;
  int32_t max_h;


  typedef boost::shared_ptr< ::nao_behavior_tree::BallPosForHandRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nao_behavior_tree::BallPosForHandRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct BallPosForHandRequest
typedef  ::nao_behavior_tree::BallPosForHandRequest_<std::allocator<void> > BallPosForHandRequest;

typedef boost::shared_ptr< ::nao_behavior_tree::BallPosForHandRequest> BallPosForHandRequestPtr;
typedef boost::shared_ptr< ::nao_behavior_tree::BallPosForHandRequest const> BallPosForHandRequestConstPtr;


template <class ContainerAllocator>
struct BallPosForHandResponse_ {
  typedef BallPosForHandResponse_<ContainerAllocator> Type;

  BallPosForHandResponse_()
  : pos_x(0.0)
  , pos_y(0.0)
  , pos_z(0.0)
  {
  }

  BallPosForHandResponse_(const ContainerAllocator& _alloc)
  : pos_x(0.0)
  , pos_y(0.0)
  , pos_z(0.0)
  {
  }

  typedef float _pos_x_type;
  float pos_x;

  typedef float _pos_y_type;
  float pos_y;

  typedef float _pos_z_type;
  float pos_z;


  typedef boost::shared_ptr< ::nao_behavior_tree::BallPosForHandResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nao_behavior_tree::BallPosForHandResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct BallPosForHandResponse
typedef  ::nao_behavior_tree::BallPosForHandResponse_<std::allocator<void> > BallPosForHandResponse;

typedef boost::shared_ptr< ::nao_behavior_tree::BallPosForHandResponse> BallPosForHandResponsePtr;
typedef boost::shared_ptr< ::nao_behavior_tree::BallPosForHandResponse const> BallPosForHandResponseConstPtr;

struct BallPosForHand
{

typedef BallPosForHandRequest Request;
typedef BallPosForHandResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct BallPosForHand
} // namespace nao_behavior_tree

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::nao_behavior_tree::BallPosForHandRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::nao_behavior_tree::BallPosForHandRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::nao_behavior_tree::BallPosForHandRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "6fbdb3f4f625c750fdfa49da0b25855c";
  }

  static const char* value(const  ::nao_behavior_tree::BallPosForHandRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x6fbdb3f4f625c750ULL;
  static const uint64_t static_value2 = 0xfdfa49da0b25855cULL;
};

template<class ContainerAllocator>
struct DataType< ::nao_behavior_tree::BallPosForHandRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_behavior_tree/BallPosForHandRequest";
  }

  static const char* value(const  ::nao_behavior_tree::BallPosForHandRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::nao_behavior_tree::BallPosForHandRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 min_h\n\
int32 max_h\n\
\n\
";
  }

  static const char* value(const  ::nao_behavior_tree::BallPosForHandRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::nao_behavior_tree::BallPosForHandRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::nao_behavior_tree::BallPosForHandResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::nao_behavior_tree::BallPosForHandResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::nao_behavior_tree::BallPosForHandResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "03225cff21da80c8dcb47075eac84a6b";
  }

  static const char* value(const  ::nao_behavior_tree::BallPosForHandResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x03225cff21da80c8ULL;
  static const uint64_t static_value2 = 0xdcb47075eac84a6bULL;
};

template<class ContainerAllocator>
struct DataType< ::nao_behavior_tree::BallPosForHandResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_behavior_tree/BallPosForHandResponse";
  }

  static const char* value(const  ::nao_behavior_tree::BallPosForHandResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::nao_behavior_tree::BallPosForHandResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float32 pos_x\n\
float32 pos_y\n\
float32 pos_z\n\
\n\
\n\
";
  }

  static const char* value(const  ::nao_behavior_tree::BallPosForHandResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::nao_behavior_tree::BallPosForHandResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::nao_behavior_tree::BallPosForHandRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.min_h);
    stream.next(m.max_h);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct BallPosForHandRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::nao_behavior_tree::BallPosForHandResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.pos_x);
    stream.next(m.pos_y);
    stream.next(m.pos_z);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct BallPosForHandResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<nao_behavior_tree::BallPosForHand> {
  static const char* value() 
  {
    return "074da7694199f61d03c662ae92d91008";
  }

  static const char* value(const nao_behavior_tree::BallPosForHand&) { return value(); } 
};

template<>
struct DataType<nao_behavior_tree::BallPosForHand> {
  static const char* value() 
  {
    return "nao_behavior_tree/BallPosForHand";
  }

  static const char* value(const nao_behavior_tree::BallPosForHand&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<nao_behavior_tree::BallPosForHandRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "074da7694199f61d03c662ae92d91008";
  }

  static const char* value(const nao_behavior_tree::BallPosForHandRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<nao_behavior_tree::BallPosForHandRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_behavior_tree/BallPosForHand";
  }

  static const char* value(const nao_behavior_tree::BallPosForHandRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<nao_behavior_tree::BallPosForHandResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "074da7694199f61d03c662ae92d91008";
  }

  static const char* value(const nao_behavior_tree::BallPosForHandResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<nao_behavior_tree::BallPosForHandResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_behavior_tree/BallPosForHand";
  }

  static const char* value(const nao_behavior_tree::BallPosForHandResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // NAO_BEHAVIOR_TREE_SERVICE_BALLPOSFORHAND_H

