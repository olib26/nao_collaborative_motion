; Auto-generated. Do not edit!


(cl:in-package nao_behavior_tree-srv)


;//! \htmlinclude GetArmReadyBT-request.msg.html

(cl:defclass <GetArmReadyBT-request> (roslisp-msg-protocol:ros-message)
  ((NAO
    :reader NAO
    :initarg :NAO
    :type cl:integer
    :initform 0))
)

(cl:defclass GetArmReadyBT-request (<GetArmReadyBT-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetArmReadyBT-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetArmReadyBT-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nao_behavior_tree-srv:<GetArmReadyBT-request> is deprecated: use nao_behavior_tree-srv:GetArmReadyBT-request instead.")))

(cl:ensure-generic-function 'NAO-val :lambda-list '(m))
(cl:defmethod NAO-val ((m <GetArmReadyBT-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_behavior_tree-srv:NAO-val is deprecated.  Use nao_behavior_tree-srv:NAO instead.")
  (NAO m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetArmReadyBT-request>) ostream)
  "Serializes a message object of type '<GetArmReadyBT-request>"
  (cl:let* ((signed (cl:slot-value msg 'NAO)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetArmReadyBT-request>) istream)
  "Deserializes a message object of type '<GetArmReadyBT-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'NAO) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetArmReadyBT-request>)))
  "Returns string type for a service object of type '<GetArmReadyBT-request>"
  "nao_behavior_tree/GetArmReadyBTRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetArmReadyBT-request)))
  "Returns string type for a service object of type 'GetArmReadyBT-request"
  "nao_behavior_tree/GetArmReadyBTRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetArmReadyBT-request>)))
  "Returns md5sum for a message object of type '<GetArmReadyBT-request>"
  "0e5c583016bfad7605d4c06a7dca15b3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetArmReadyBT-request)))
  "Returns md5sum for a message object of type 'GetArmReadyBT-request"
  "0e5c583016bfad7605d4c06a7dca15b3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetArmReadyBT-request>)))
  "Returns full string definition for message of type '<GetArmReadyBT-request>"
  (cl:format cl:nil "int32 NAO~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetArmReadyBT-request)))
  "Returns full string definition for message of type 'GetArmReadyBT-request"
  (cl:format cl:nil "int32 NAO~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetArmReadyBT-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetArmReadyBT-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetArmReadyBT-request
    (cl:cons ':NAO (NAO msg))
))
;//! \htmlinclude GetArmReadyBT-response.msg.html

(cl:defclass <GetArmReadyBT-response> (roslisp-msg-protocol:ros-message)
  ((arm
    :reader arm
    :initarg :arm
    :type cl:integer
    :initform 0))
)

(cl:defclass GetArmReadyBT-response (<GetArmReadyBT-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetArmReadyBT-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetArmReadyBT-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nao_behavior_tree-srv:<GetArmReadyBT-response> is deprecated: use nao_behavior_tree-srv:GetArmReadyBT-response instead.")))

(cl:ensure-generic-function 'arm-val :lambda-list '(m))
(cl:defmethod arm-val ((m <GetArmReadyBT-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_behavior_tree-srv:arm-val is deprecated.  Use nao_behavior_tree-srv:arm instead.")
  (arm m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetArmReadyBT-response>) ostream)
  "Serializes a message object of type '<GetArmReadyBT-response>"
  (cl:let* ((signed (cl:slot-value msg 'arm)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetArmReadyBT-response>) istream)
  "Deserializes a message object of type '<GetArmReadyBT-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'arm) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetArmReadyBT-response>)))
  "Returns string type for a service object of type '<GetArmReadyBT-response>"
  "nao_behavior_tree/GetArmReadyBTResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetArmReadyBT-response)))
  "Returns string type for a service object of type 'GetArmReadyBT-response"
  "nao_behavior_tree/GetArmReadyBTResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetArmReadyBT-response>)))
  "Returns md5sum for a message object of type '<GetArmReadyBT-response>"
  "0e5c583016bfad7605d4c06a7dca15b3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetArmReadyBT-response)))
  "Returns md5sum for a message object of type 'GetArmReadyBT-response"
  "0e5c583016bfad7605d4c06a7dca15b3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetArmReadyBT-response>)))
  "Returns full string definition for message of type '<GetArmReadyBT-response>"
  (cl:format cl:nil "int32 arm~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetArmReadyBT-response)))
  "Returns full string definition for message of type 'GetArmReadyBT-response"
  (cl:format cl:nil "int32 arm~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetArmReadyBT-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetArmReadyBT-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetArmReadyBT-response
    (cl:cons ':arm (arm msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetArmReadyBT)))
  'GetArmReadyBT-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetArmReadyBT)))
  'GetArmReadyBT-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetArmReadyBT)))
  "Returns string type for a service object of type '<GetArmReadyBT>"
  "nao_behavior_tree/GetArmReadyBT")