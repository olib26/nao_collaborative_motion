; Auto-generated. Do not edit!


(cl:in-package nao_behavior_tree-srv)


;//! \htmlinclude BallPosForHead-request.msg.html

(cl:defclass <BallPosForHead-request> (roslisp-msg-protocol:ros-message)
  ((min_h
    :reader min_h
    :initarg :min_h
    :type cl:integer
    :initform 0)
   (max_h
    :reader max_h
    :initarg :max_h
    :type cl:integer
    :initform 0))
)

(cl:defclass BallPosForHead-request (<BallPosForHead-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BallPosForHead-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BallPosForHead-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nao_behavior_tree-srv:<BallPosForHead-request> is deprecated: use nao_behavior_tree-srv:BallPosForHead-request instead.")))

(cl:ensure-generic-function 'min_h-val :lambda-list '(m))
(cl:defmethod min_h-val ((m <BallPosForHead-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_behavior_tree-srv:min_h-val is deprecated.  Use nao_behavior_tree-srv:min_h instead.")
  (min_h m))

(cl:ensure-generic-function 'max_h-val :lambda-list '(m))
(cl:defmethod max_h-val ((m <BallPosForHead-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_behavior_tree-srv:max_h-val is deprecated.  Use nao_behavior_tree-srv:max_h instead.")
  (max_h m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BallPosForHead-request>) ostream)
  "Serializes a message object of type '<BallPosForHead-request>"
  (cl:let* ((signed (cl:slot-value msg 'min_h)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'max_h)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BallPosForHead-request>) istream)
  "Deserializes a message object of type '<BallPosForHead-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'min_h) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'max_h) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BallPosForHead-request>)))
  "Returns string type for a service object of type '<BallPosForHead-request>"
  "nao_behavior_tree/BallPosForHeadRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BallPosForHead-request)))
  "Returns string type for a service object of type 'BallPosForHead-request"
  "nao_behavior_tree/BallPosForHeadRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BallPosForHead-request>)))
  "Returns md5sum for a message object of type '<BallPosForHead-request>"
  "dc8602d6c6bd69c5f7e48c7d10bea4cc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BallPosForHead-request)))
  "Returns md5sum for a message object of type 'BallPosForHead-request"
  "dc8602d6c6bd69c5f7e48c7d10bea4cc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BallPosForHead-request>)))
  "Returns full string definition for message of type '<BallPosForHead-request>"
  (cl:format cl:nil "int32 min_h~%int32 max_h~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BallPosForHead-request)))
  "Returns full string definition for message of type 'BallPosForHead-request"
  (cl:format cl:nil "int32 min_h~%int32 max_h~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BallPosForHead-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BallPosForHead-request>))
  "Converts a ROS message object to a list"
  (cl:list 'BallPosForHead-request
    (cl:cons ':min_h (min_h msg))
    (cl:cons ':max_h (max_h msg))
))
;//! \htmlinclude BallPosForHead-response.msg.html

(cl:defclass <BallPosForHead-response> (roslisp-msg-protocol:ros-message)
  ((pos
    :reader pos
    :initarg :pos
    :type cl:integer
    :initform 0))
)

(cl:defclass BallPosForHead-response (<BallPosForHead-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BallPosForHead-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BallPosForHead-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nao_behavior_tree-srv:<BallPosForHead-response> is deprecated: use nao_behavior_tree-srv:BallPosForHead-response instead.")))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <BallPosForHead-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_behavior_tree-srv:pos-val is deprecated.  Use nao_behavior_tree-srv:pos instead.")
  (pos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BallPosForHead-response>) ostream)
  "Serializes a message object of type '<BallPosForHead-response>"
  (cl:let* ((signed (cl:slot-value msg 'pos)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BallPosForHead-response>) istream)
  "Deserializes a message object of type '<BallPosForHead-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pos) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BallPosForHead-response>)))
  "Returns string type for a service object of type '<BallPosForHead-response>"
  "nao_behavior_tree/BallPosForHeadResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BallPosForHead-response)))
  "Returns string type for a service object of type 'BallPosForHead-response"
  "nao_behavior_tree/BallPosForHeadResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BallPosForHead-response>)))
  "Returns md5sum for a message object of type '<BallPosForHead-response>"
  "dc8602d6c6bd69c5f7e48c7d10bea4cc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BallPosForHead-response)))
  "Returns md5sum for a message object of type 'BallPosForHead-response"
  "dc8602d6c6bd69c5f7e48c7d10bea4cc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BallPosForHead-response>)))
  "Returns full string definition for message of type '<BallPosForHead-response>"
  (cl:format cl:nil "int64 pos~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BallPosForHead-response)))
  "Returns full string definition for message of type 'BallPosForHead-response"
  (cl:format cl:nil "int64 pos~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BallPosForHead-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BallPosForHead-response>))
  "Converts a ROS message object to a list"
  (cl:list 'BallPosForHead-response
    (cl:cons ':pos (pos msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'BallPosForHead)))
  'BallPosForHead-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'BallPosForHead)))
  'BallPosForHead-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BallPosForHead)))
  "Returns string type for a service object of type '<BallPosForHead>"
  "nao_behavior_tree/BallPosForHead")