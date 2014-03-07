; Auto-generated. Do not edit!


(cl:in-package nao_behavior_tree-msg)


;//! \htmlinclude Bearing.msg.html

(cl:defclass <Bearing> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (robotDetected
    :reader robotDetected
    :initarg :robotDetected
    :type cl:boolean
    :initform cl:nil)
   (absolute
    :reader absolute
    :initarg :absolute
    :type cl:float
    :initform 0.0)
   (relative
    :reader relative
    :initarg :relative
    :type cl:float
    :initform 0.0))
)

(cl:defclass Bearing (<Bearing>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Bearing>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Bearing)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nao_behavior_tree-msg:<Bearing> is deprecated: use nao_behavior_tree-msg:Bearing instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Bearing>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_behavior_tree-msg:header-val is deprecated.  Use nao_behavior_tree-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'robotDetected-val :lambda-list '(m))
(cl:defmethod robotDetected-val ((m <Bearing>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_behavior_tree-msg:robotDetected-val is deprecated.  Use nao_behavior_tree-msg:robotDetected instead.")
  (robotDetected m))

(cl:ensure-generic-function 'absolute-val :lambda-list '(m))
(cl:defmethod absolute-val ((m <Bearing>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_behavior_tree-msg:absolute-val is deprecated.  Use nao_behavior_tree-msg:absolute instead.")
  (absolute m))

(cl:ensure-generic-function 'relative-val :lambda-list '(m))
(cl:defmethod relative-val ((m <Bearing>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_behavior_tree-msg:relative-val is deprecated.  Use nao_behavior_tree-msg:relative instead.")
  (relative m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Bearing>) ostream)
  "Serializes a message object of type '<Bearing>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'robotDetected) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'absolute))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'relative))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Bearing>) istream)
  "Deserializes a message object of type '<Bearing>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'robotDetected) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'absolute) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'relative) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Bearing>)))
  "Returns string type for a message object of type '<Bearing>"
  "nao_behavior_tree/Bearing")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Bearing)))
  "Returns string type for a message object of type 'Bearing"
  "nao_behavior_tree/Bearing")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Bearing>)))
  "Returns md5sum for a message object of type '<Bearing>"
  "f889ef5163854f388ed046e2be75353c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Bearing)))
  "Returns md5sum for a message object of type 'Bearing"
  "f889ef5163854f388ed046e2be75353c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Bearing>)))
  "Returns full string definition for message of type '<Bearing>"
  (cl:format cl:nil "# Bearing values~%~%Header header~%~%bool robotDetected~%~%float32 absolute~%float32 relative~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Bearing)))
  "Returns full string definition for message of type 'Bearing"
  (cl:format cl:nil "# Bearing values~%~%Header header~%~%bool robotDetected~%~%float32 absolute~%float32 relative~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Bearing>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Bearing>))
  "Converts a ROS message object to a list"
  (cl:list 'Bearing
    (cl:cons ':header (header msg))
    (cl:cons ':robotDetected (robotDetected msg))
    (cl:cons ':absolute (absolute msg))
    (cl:cons ':relative (relative msg))
))
