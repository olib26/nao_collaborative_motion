; Auto-generated. Do not edit!


(cl:in-package nao_behavior_tree-srv)


;//! \htmlinclude BallPosGetBT-request.msg.html

(cl:defclass <BallPosGetBT-request> (roslisp-msg-protocol:ros-message)
  ((NAO
    :reader NAO
    :initarg :NAO
    :type cl:integer
    :initform 0))
)

(cl:defclass BallPosGetBT-request (<BallPosGetBT-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BallPosGetBT-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BallPosGetBT-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nao_behavior_tree-srv:<BallPosGetBT-request> is deprecated: use nao_behavior_tree-srv:BallPosGetBT-request instead.")))

(cl:ensure-generic-function 'NAO-val :lambda-list '(m))
(cl:defmethod NAO-val ((m <BallPosGetBT-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_behavior_tree-srv:NAO-val is deprecated.  Use nao_behavior_tree-srv:NAO instead.")
  (NAO m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BallPosGetBT-request>) ostream)
  "Serializes a message object of type '<BallPosGetBT-request>"
  (cl:let* ((signed (cl:slot-value msg 'NAO)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BallPosGetBT-request>) istream)
  "Deserializes a message object of type '<BallPosGetBT-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'NAO) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BallPosGetBT-request>)))
  "Returns string type for a service object of type '<BallPosGetBT-request>"
  "nao_behavior_tree/BallPosGetBTRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BallPosGetBT-request)))
  "Returns string type for a service object of type 'BallPosGetBT-request"
  "nao_behavior_tree/BallPosGetBTRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BallPosGetBT-request>)))
  "Returns md5sum for a message object of type '<BallPosGetBT-request>"
  "8ba731454bb6486a79e62cf47afe8146")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BallPosGetBT-request)))
  "Returns md5sum for a message object of type 'BallPosGetBT-request"
  "8ba731454bb6486a79e62cf47afe8146")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BallPosGetBT-request>)))
  "Returns full string definition for message of type '<BallPosGetBT-request>"
  (cl:format cl:nil "int32 NAO~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BallPosGetBT-request)))
  "Returns full string definition for message of type 'BallPosGetBT-request"
  (cl:format cl:nil "int32 NAO~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BallPosGetBT-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BallPosGetBT-request>))
  "Converts a ROS message object to a list"
  (cl:list 'BallPosGetBT-request
    (cl:cons ':NAO (NAO msg))
))
;//! \htmlinclude BallPosGetBT-response.msg.html

(cl:defclass <BallPosGetBT-response> (roslisp-msg-protocol:ros-message)
  ((hand
    :reader hand
    :initarg :hand
    :type cl:integer
    :initform 0))
)

(cl:defclass BallPosGetBT-response (<BallPosGetBT-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BallPosGetBT-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BallPosGetBT-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nao_behavior_tree-srv:<BallPosGetBT-response> is deprecated: use nao_behavior_tree-srv:BallPosGetBT-response instead.")))

(cl:ensure-generic-function 'hand-val :lambda-list '(m))
(cl:defmethod hand-val ((m <BallPosGetBT-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_behavior_tree-srv:hand-val is deprecated.  Use nao_behavior_tree-srv:hand instead.")
  (hand m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BallPosGetBT-response>) ostream)
  "Serializes a message object of type '<BallPosGetBT-response>"
  (cl:let* ((signed (cl:slot-value msg 'hand)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BallPosGetBT-response>) istream)
  "Deserializes a message object of type '<BallPosGetBT-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'hand) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BallPosGetBT-response>)))
  "Returns string type for a service object of type '<BallPosGetBT-response>"
  "nao_behavior_tree/BallPosGetBTResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BallPosGetBT-response)))
  "Returns string type for a service object of type 'BallPosGetBT-response"
  "nao_behavior_tree/BallPosGetBTResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BallPosGetBT-response>)))
  "Returns md5sum for a message object of type '<BallPosGetBT-response>"
  "8ba731454bb6486a79e62cf47afe8146")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BallPosGetBT-response)))
  "Returns md5sum for a message object of type 'BallPosGetBT-response"
  "8ba731454bb6486a79e62cf47afe8146")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BallPosGetBT-response>)))
  "Returns full string definition for message of type '<BallPosGetBT-response>"
  (cl:format cl:nil "int32 hand~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BallPosGetBT-response)))
  "Returns full string definition for message of type 'BallPosGetBT-response"
  (cl:format cl:nil "int32 hand~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BallPosGetBT-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BallPosGetBT-response>))
  "Converts a ROS message object to a list"
  (cl:list 'BallPosGetBT-response
    (cl:cons ':hand (hand msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'BallPosGetBT)))
  'BallPosGetBT-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'BallPosGetBT)))
  'BallPosGetBT-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BallPosGetBT)))
  "Returns string type for a service object of type '<BallPosGetBT>"
  "nao_behavior_tree/BallPosGetBT")