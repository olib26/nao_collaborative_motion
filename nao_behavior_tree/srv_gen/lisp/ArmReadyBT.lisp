; Auto-generated. Do not edit!


(cl:in-package nao_behavior_tree-srv)


;//! \htmlinclude ArmReadyBT-request.msg.html

(cl:defclass <ArmReadyBT-request> (roslisp-msg-protocol:ros-message)
  ((NAO
    :reader NAO
    :initarg :NAO
    :type cl:integer
    :initform 0)
   (arm
    :reader arm
    :initarg :arm
    :type cl:integer
    :initform 0))
)

(cl:defclass ArmReadyBT-request (<ArmReadyBT-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmReadyBT-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmReadyBT-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nao_behavior_tree-srv:<ArmReadyBT-request> is deprecated: use nao_behavior_tree-srv:ArmReadyBT-request instead.")))

(cl:ensure-generic-function 'NAO-val :lambda-list '(m))
(cl:defmethod NAO-val ((m <ArmReadyBT-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_behavior_tree-srv:NAO-val is deprecated.  Use nao_behavior_tree-srv:NAO instead.")
  (NAO m))

(cl:ensure-generic-function 'arm-val :lambda-list '(m))
(cl:defmethod arm-val ((m <ArmReadyBT-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_behavior_tree-srv:arm-val is deprecated.  Use nao_behavior_tree-srv:arm instead.")
  (arm m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmReadyBT-request>) ostream)
  "Serializes a message object of type '<ArmReadyBT-request>"
  (cl:let* ((signed (cl:slot-value msg 'NAO)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'arm)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmReadyBT-request>) istream)
  "Deserializes a message object of type '<ArmReadyBT-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'NAO) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'arm) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmReadyBT-request>)))
  "Returns string type for a service object of type '<ArmReadyBT-request>"
  "nao_behavior_tree/ArmReadyBTRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmReadyBT-request)))
  "Returns string type for a service object of type 'ArmReadyBT-request"
  "nao_behavior_tree/ArmReadyBTRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmReadyBT-request>)))
  "Returns md5sum for a message object of type '<ArmReadyBT-request>"
  "d090919db246d41a7da74bd56eadc6ec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmReadyBT-request)))
  "Returns md5sum for a message object of type 'ArmReadyBT-request"
  "d090919db246d41a7da74bd56eadc6ec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmReadyBT-request>)))
  "Returns full string definition for message of type '<ArmReadyBT-request>"
  (cl:format cl:nil "int32 NAO~%int32 arm~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmReadyBT-request)))
  "Returns full string definition for message of type 'ArmReadyBT-request"
  (cl:format cl:nil "int32 NAO~%int32 arm~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmReadyBT-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmReadyBT-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmReadyBT-request
    (cl:cons ':NAO (NAO msg))
    (cl:cons ':arm (arm msg))
))
;//! \htmlinclude ArmReadyBT-response.msg.html

(cl:defclass <ArmReadyBT-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ArmReadyBT-response (<ArmReadyBT-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmReadyBT-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmReadyBT-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nao_behavior_tree-srv:<ArmReadyBT-response> is deprecated: use nao_behavior_tree-srv:ArmReadyBT-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmReadyBT-response>) ostream)
  "Serializes a message object of type '<ArmReadyBT-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmReadyBT-response>) istream)
  "Deserializes a message object of type '<ArmReadyBT-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmReadyBT-response>)))
  "Returns string type for a service object of type '<ArmReadyBT-response>"
  "nao_behavior_tree/ArmReadyBTResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmReadyBT-response)))
  "Returns string type for a service object of type 'ArmReadyBT-response"
  "nao_behavior_tree/ArmReadyBTResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmReadyBT-response>)))
  "Returns md5sum for a message object of type '<ArmReadyBT-response>"
  "d090919db246d41a7da74bd56eadc6ec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmReadyBT-response)))
  "Returns md5sum for a message object of type 'ArmReadyBT-response"
  "d090919db246d41a7da74bd56eadc6ec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmReadyBT-response>)))
  "Returns full string definition for message of type '<ArmReadyBT-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmReadyBT-response)))
  "Returns full string definition for message of type 'ArmReadyBT-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmReadyBT-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmReadyBT-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmReadyBT-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ArmReadyBT)))
  'ArmReadyBT-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ArmReadyBT)))
  'ArmReadyBT-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmReadyBT)))
  "Returns string type for a service object of type '<ArmReadyBT>"
  "nao_behavior_tree/ArmReadyBT")