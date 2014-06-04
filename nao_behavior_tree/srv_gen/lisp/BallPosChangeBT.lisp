; Auto-generated. Do not edit!


(cl:in-package nao_behavior_tree-srv)


;//! \htmlinclude BallPosChangeBT-request.msg.html

(cl:defclass <BallPosChangeBT-request> (roslisp-msg-protocol:ros-message)
  ((object
    :reader object
    :initarg :object
    :type cl:integer
    :initform 0)
   (NAO
    :reader NAO
    :initarg :NAO
    :type cl:integer
    :initform 0)
   (hand
    :reader hand
    :initarg :hand
    :type cl:integer
    :initform 0))
)

(cl:defclass BallPosChangeBT-request (<BallPosChangeBT-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BallPosChangeBT-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BallPosChangeBT-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nao_behavior_tree-srv:<BallPosChangeBT-request> is deprecated: use nao_behavior_tree-srv:BallPosChangeBT-request instead.")))

(cl:ensure-generic-function 'object-val :lambda-list '(m))
(cl:defmethod object-val ((m <BallPosChangeBT-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_behavior_tree-srv:object-val is deprecated.  Use nao_behavior_tree-srv:object instead.")
  (object m))

(cl:ensure-generic-function 'NAO-val :lambda-list '(m))
(cl:defmethod NAO-val ((m <BallPosChangeBT-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_behavior_tree-srv:NAO-val is deprecated.  Use nao_behavior_tree-srv:NAO instead.")
  (NAO m))

(cl:ensure-generic-function 'hand-val :lambda-list '(m))
(cl:defmethod hand-val ((m <BallPosChangeBT-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_behavior_tree-srv:hand-val is deprecated.  Use nao_behavior_tree-srv:hand instead.")
  (hand m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BallPosChangeBT-request>) ostream)
  "Serializes a message object of type '<BallPosChangeBT-request>"
  (cl:let* ((signed (cl:slot-value msg 'object)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'NAO)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'hand)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BallPosChangeBT-request>) istream)
  "Deserializes a message object of type '<BallPosChangeBT-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'object) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
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
      (cl:setf (cl:slot-value msg 'hand) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BallPosChangeBT-request>)))
  "Returns string type for a service object of type '<BallPosChangeBT-request>"
  "nao_behavior_tree/BallPosChangeBTRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BallPosChangeBT-request)))
  "Returns string type for a service object of type 'BallPosChangeBT-request"
  "nao_behavior_tree/BallPosChangeBTRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BallPosChangeBT-request>)))
  "Returns md5sum for a message object of type '<BallPosChangeBT-request>"
  "7af9c257fdc42efb9ebc8fae25456191")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BallPosChangeBT-request)))
  "Returns md5sum for a message object of type 'BallPosChangeBT-request"
  "7af9c257fdc42efb9ebc8fae25456191")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BallPosChangeBT-request>)))
  "Returns full string definition for message of type '<BallPosChangeBT-request>"
  (cl:format cl:nil "int32 object~%int32 NAO~%int32 hand~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BallPosChangeBT-request)))
  "Returns full string definition for message of type 'BallPosChangeBT-request"
  (cl:format cl:nil "int32 object~%int32 NAO~%int32 hand~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BallPosChangeBT-request>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BallPosChangeBT-request>))
  "Converts a ROS message object to a list"
  (cl:list 'BallPosChangeBT-request
    (cl:cons ':object (object msg))
    (cl:cons ':NAO (NAO msg))
    (cl:cons ':hand (hand msg))
))
;//! \htmlinclude BallPosChangeBT-response.msg.html

(cl:defclass <BallPosChangeBT-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass BallPosChangeBT-response (<BallPosChangeBT-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BallPosChangeBT-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BallPosChangeBT-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nao_behavior_tree-srv:<BallPosChangeBT-response> is deprecated: use nao_behavior_tree-srv:BallPosChangeBT-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BallPosChangeBT-response>) ostream)
  "Serializes a message object of type '<BallPosChangeBT-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BallPosChangeBT-response>) istream)
  "Deserializes a message object of type '<BallPosChangeBT-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BallPosChangeBT-response>)))
  "Returns string type for a service object of type '<BallPosChangeBT-response>"
  "nao_behavior_tree/BallPosChangeBTResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BallPosChangeBT-response)))
  "Returns string type for a service object of type 'BallPosChangeBT-response"
  "nao_behavior_tree/BallPosChangeBTResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BallPosChangeBT-response>)))
  "Returns md5sum for a message object of type '<BallPosChangeBT-response>"
  "7af9c257fdc42efb9ebc8fae25456191")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BallPosChangeBT-response)))
  "Returns md5sum for a message object of type 'BallPosChangeBT-response"
  "7af9c257fdc42efb9ebc8fae25456191")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BallPosChangeBT-response>)))
  "Returns full string definition for message of type '<BallPosChangeBT-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BallPosChangeBT-response)))
  "Returns full string definition for message of type 'BallPosChangeBT-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BallPosChangeBT-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BallPosChangeBT-response>))
  "Converts a ROS message object to a list"
  (cl:list 'BallPosChangeBT-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'BallPosChangeBT)))
  'BallPosChangeBT-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'BallPosChangeBT)))
  'BallPosChangeBT-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BallPosChangeBT)))
  "Returns string type for a service object of type '<BallPosChangeBT>"
  "nao_behavior_tree/BallPosChangeBT")