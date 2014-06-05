/* Auto-generated by genmsg_cpp for file /home/olivier/ros_workspace/nao_behavior_tree/srv/BallPosChangeBT.srv */
#ifndef NAO_BEHAVIOR_TREE_SERVICE_BALLPOSCHANGEBT_H
#define NAO_BEHAVIOR_TREE_SERVICE_BALLPOSCHANGEBT_H
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
struct BallPosChangeBTRequest_ {
  typedef BallPosChangeBTRequest_<ContainerAllocator> Type;

  BallPosChangeBTRequest_()
  : object(0)
  , NAO(0)
  , hand(0)
  {
  }

  BallPosChangeBTRequest_(const ContainerAllocator& _alloc)
  : object(0)
  , NAO(0)
  , hand(0)
  {
  }

  typedef int32_t _object_type;
  int32_t object;

  typedef int32_t _NAO_type;
  int32_t NAO;

  typedef int32_t _hand_type;
  int32_t hand;


  typedef boost::shared_ptr< ::nao_behavior_tree::BallPosChangeBTRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nao_behavior_tree::BallPosChangeBTRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct BallPosChangeBTRequest
typedef  ::nao_behavior_tree::BallPosChangeBTRequest_<std::allocator<void> > BallPosChangeBTRequest;

typedef boost::shared_ptr< ::nao_behavior_tree::BallPosChangeBTRequest> BallPosChangeBTRequestPtr;
typedef boost::shared_ptr< ::nao_behavior_tree::BallPosChangeBTRequest const> BallPosChangeBTRequestConstPtr;


template <class ContainerAllocator>
struct BallPosChangeBTResponse_ {
  typedef BallPosChangeBTResponse_<ContainerAllocator> Type;

  BallPosChangeBTResponse_()
  {
  }

  BallPosChangeBTResponse_(const ContainerAllocator& _alloc)
  {
  }


  typedef boost::shared_ptr< ::nao_behavior_tree::BallPosChangeBTResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nao_behavior_tree::BallPosChangeBTResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct BallPosChangeBTResponse
typedef  ::nao_behavior_tree::BallPosChangeBTResponse_<std::allocator<void> > BallPosChangeBTResponse;

typedef boost::shared_ptr< ::nao_behavior_tree::BallPosChangeBTResponse> BallPosChangeBTResponsePtr;
typedef boost::shared_ptr< ::nao_behavior_tree::BallPosChangeBTResponse const> BallPosChangeBTResponseConstPtr;

struct BallPosChangeBT
{

typedef BallPosChangeBTRequest Request;
typedef BallPosChangeBTResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct BallPosChangeBT
} // namespace nao_behavior_tree

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::nao_behavior_tree::BallPosChangeBTRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::nao_behavior_tree::BallPosChangeBTRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::nao_behavior_tree::BallPosChangeBTRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "7af9c257fdc42efb9ebc8fae25456191";
  }

  static const char* value(const  ::nao_behavior_tree::BallPosChangeBTRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x7af9c257fdc42efbULL;
  static const uint64_t static_value2 = 0x9ebc8fae25456191ULL;
};

template<class ContainerAllocator>
struct DataType< ::nao_behavior_tree::BallPosChangeBTRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_behavior_tree/BallPosChangeBTRequest";
  }

  static const char* value(const  ::nao_behavior_tree::BallPosChangeBTRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::nao_behavior_tree::BallPosChangeBTRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 object\n\
int32 NAO\n\
int32 hand\n\
\n\
";
  }

  static const char* value(const  ::nao_behavior_tree::BallPosChangeBTRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::nao_behavior_tree::BallPosChangeBTRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::nao_behavior_tree::BallPosChangeBTResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::nao_behavior_tree::BallPosChangeBTResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::nao_behavior_tree::BallPosChangeBTResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::nao_behavior_tree::BallPosChangeBTResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::nao_behavior_tree::BallPosChangeBTResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_behavior_tree/BallPosChangeBTResponse";
  }

  static const char* value(const  ::nao_behavior_tree::BallPosChangeBTResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::nao_behavior_tree::BallPosChangeBTResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
";
  }

  static const char* value(const  ::nao_behavior_tree::BallPosChangeBTResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::nao_behavior_tree::BallPosChangeBTResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::nao_behavior_tree::BallPosChangeBTRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.object);
    stream.next(m.NAO);
    stream.next(m.hand);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct BallPosChangeBTRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::nao_behavior_tree::BallPosChangeBTResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct BallPosChangeBTResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<nao_behavior_tree::BallPosChangeBT> {
  static const char* value() 
  {
    return "7af9c257fdc42efb9ebc8fae25456191";
  }

  static const char* value(const nao_behavior_tree::BallPosChangeBT&) { return value(); } 
};

template<>
struct DataType<nao_behavior_tree::BallPosChangeBT> {
  static const char* value() 
  {
    return "nao_behavior_tree/BallPosChangeBT";
  }

  static const char* value(const nao_behavior_tree::BallPosChangeBT&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<nao_behavior_tree::BallPosChangeBTRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "7af9c257fdc42efb9ebc8fae25456191";
  }

  static const char* value(const nao_behavior_tree::BallPosChangeBTRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<nao_behavior_tree::BallPosChangeBTRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_behavior_tree/BallPosChangeBT";
  }

  static const char* value(const nao_behavior_tree::BallPosChangeBTRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<nao_behavior_tree::BallPosChangeBTResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "7af9c257fdc42efb9ebc8fae25456191";
  }

  static const char* value(const nao_behavior_tree::BallPosChangeBTResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<nao_behavior_tree::BallPosChangeBTResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_behavior_tree/BallPosChangeBT";
  }

  static const char* value(const nao_behavior_tree::BallPosChangeBTResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // NAO_BEHAVIOR_TREE_SERVICE_BALLPOSCHANGEBT_H
