/* Auto-generated by genmsg_cpp for file /home/olivier/ros_workspace/nao_behavior_tree/srv/BallPosGetBT.srv */
#ifndef NAO_BEHAVIOR_TREE_SERVICE_BALLPOSGETBT_H
#define NAO_BEHAVIOR_TREE_SERVICE_BALLPOSGETBT_H
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
struct BallPosGetBTRequest_ {
  typedef BallPosGetBTRequest_<ContainerAllocator> Type;

  BallPosGetBTRequest_()
  : NAO(0)
  {
  }

  BallPosGetBTRequest_(const ContainerAllocator& _alloc)
  : NAO(0)
  {
  }

  typedef int32_t _NAO_type;
  int32_t NAO;


  typedef boost::shared_ptr< ::nao_behavior_tree::BallPosGetBTRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nao_behavior_tree::BallPosGetBTRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct BallPosGetBTRequest
typedef  ::nao_behavior_tree::BallPosGetBTRequest_<std::allocator<void> > BallPosGetBTRequest;

typedef boost::shared_ptr< ::nao_behavior_tree::BallPosGetBTRequest> BallPosGetBTRequestPtr;
typedef boost::shared_ptr< ::nao_behavior_tree::BallPosGetBTRequest const> BallPosGetBTRequestConstPtr;


template <class ContainerAllocator>
struct BallPosGetBTResponse_ {
  typedef BallPosGetBTResponse_<ContainerAllocator> Type;

  BallPosGetBTResponse_()
  : hand(0)
  {
  }

  BallPosGetBTResponse_(const ContainerAllocator& _alloc)
  : hand(0)
  {
  }

  typedef int32_t _hand_type;
  int32_t hand;


  typedef boost::shared_ptr< ::nao_behavior_tree::BallPosGetBTResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nao_behavior_tree::BallPosGetBTResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct BallPosGetBTResponse
typedef  ::nao_behavior_tree::BallPosGetBTResponse_<std::allocator<void> > BallPosGetBTResponse;

typedef boost::shared_ptr< ::nao_behavior_tree::BallPosGetBTResponse> BallPosGetBTResponsePtr;
typedef boost::shared_ptr< ::nao_behavior_tree::BallPosGetBTResponse const> BallPosGetBTResponseConstPtr;

struct BallPosGetBT
{

typedef BallPosGetBTRequest Request;
typedef BallPosGetBTResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct BallPosGetBT
} // namespace nao_behavior_tree

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::nao_behavior_tree::BallPosGetBTRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::nao_behavior_tree::BallPosGetBTRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::nao_behavior_tree::BallPosGetBTRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "90eb40daba5b4c8c59365cf231e3cb25";
  }

  static const char* value(const  ::nao_behavior_tree::BallPosGetBTRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x90eb40daba5b4c8cULL;
  static const uint64_t static_value2 = 0x59365cf231e3cb25ULL;
};

template<class ContainerAllocator>
struct DataType< ::nao_behavior_tree::BallPosGetBTRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_behavior_tree/BallPosGetBTRequest";
  }

  static const char* value(const  ::nao_behavior_tree::BallPosGetBTRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::nao_behavior_tree::BallPosGetBTRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 NAO\n\
\n\
";
  }

  static const char* value(const  ::nao_behavior_tree::BallPosGetBTRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::nao_behavior_tree::BallPosGetBTRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::nao_behavior_tree::BallPosGetBTResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::nao_behavior_tree::BallPosGetBTResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::nao_behavior_tree::BallPosGetBTResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "1098ac8bdac1e2905587eac5f4f1bdd1";
  }

  static const char* value(const  ::nao_behavior_tree::BallPosGetBTResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x1098ac8bdac1e290ULL;
  static const uint64_t static_value2 = 0x5587eac5f4f1bdd1ULL;
};

template<class ContainerAllocator>
struct DataType< ::nao_behavior_tree::BallPosGetBTResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_behavior_tree/BallPosGetBTResponse";
  }

  static const char* value(const  ::nao_behavior_tree::BallPosGetBTResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::nao_behavior_tree::BallPosGetBTResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 hand\n\
\n\
\n\
";
  }

  static const char* value(const  ::nao_behavior_tree::BallPosGetBTResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::nao_behavior_tree::BallPosGetBTResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::nao_behavior_tree::BallPosGetBTRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.NAO);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct BallPosGetBTRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::nao_behavior_tree::BallPosGetBTResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.hand);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct BallPosGetBTResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<nao_behavior_tree::BallPosGetBT> {
  static const char* value() 
  {
    return "8ba731454bb6486a79e62cf47afe8146";
  }

  static const char* value(const nao_behavior_tree::BallPosGetBT&) { return value(); } 
};

template<>
struct DataType<nao_behavior_tree::BallPosGetBT> {
  static const char* value() 
  {
    return "nao_behavior_tree/BallPosGetBT";
  }

  static const char* value(const nao_behavior_tree::BallPosGetBT&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<nao_behavior_tree::BallPosGetBTRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8ba731454bb6486a79e62cf47afe8146";
  }

  static const char* value(const nao_behavior_tree::BallPosGetBTRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<nao_behavior_tree::BallPosGetBTRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_behavior_tree/BallPosGetBT";
  }

  static const char* value(const nao_behavior_tree::BallPosGetBTRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<nao_behavior_tree::BallPosGetBTResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8ba731454bb6486a79e62cf47afe8146";
  }

  static const char* value(const nao_behavior_tree::BallPosGetBTResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<nao_behavior_tree::BallPosGetBTResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_behavior_tree/BallPosGetBT";
  }

  static const char* value(const nao_behavior_tree::BallPosGetBTResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // NAO_BEHAVIOR_TREE_SERVICE_BALLPOSGETBT_H

