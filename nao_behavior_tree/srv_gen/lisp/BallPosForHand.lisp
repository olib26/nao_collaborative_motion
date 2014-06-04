; Auto-generated. Do not edit!


(cl:in-package nao_behavior_tree-srv)


;//! \htmlinclude BallPosForHand-request.msg.html

(cl:defclass <BallPosForHand-request> (roslisp-msg-protocol:ros-message)
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

(cl:defclass BallPosForHand-request (<BallPosForHand-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BallPosForHand-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BallPosForHand-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nao_behavior_tree-srv:<BallPosForHand-request> is deprecated: use nao_behavior_tree-srv:BallPosForHand-request instead.")))

(cl:ensure-generic-function 'min_h-val :lambda-list '(m))
(cl:defmethod min_h-val ((m <BallPosForHand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_behavior_tree-srv:min_h-val is deprecated.  Use nao_behavior_tree-srv:min_h instead.")
  (min_h m))

(cl:ensure-generic-function 'max_h-val :lambda-list '(m))
(cl:defmethod max_h-val ((m <BallPosForHand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_behavior_tree-srv:max_h-val is deprecated.  Use nao_behavior_tree-srv:max_h instead.")
  (max_h m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BallPosForHand-request>) ostream)
  "Serializes a message object of type '<BallPosForHand-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BallPosForHand-request>) istream)
  "Deserializes a message object of type '<BallPosForHand-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BallPosForHand-request>)))
  "Returns string type for a service object of type '<BallPosForHand-request>"
  "nao_behavior_tree/BallPosForHandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BallPosForHand-request)))
  "Returns string type for a service object of type 'BallPosForHand-request"
  "nao_behavior_tree/BallPosForHandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BallPosForHand-request>)))
  "Returns md5sum for a message object of type '<BallPosForHand-request>"
  "074da7694199f61d03c662ae92d91008")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BallPosForHand-request)))
  "Returns md5sum for a message object of type 'BallPosForHand-request"
  "074da7694199f61d03c662ae92d91008")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BallPosForHand-request>)))
  "Returns full string definition for message of type '<BallPosForHand-request>"
  (cl:format cl:nil "int32 min_h~%int32 max_h~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BallPosForHand-request)))
  "Returns full string definition for message of type 'BallPosForHand-request"
  (cl:format cl:nil "int32 min_h~%int32 max_h~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BallPosForHand-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BallPosForHand-request>))
  "Converts a ROS message object to a list"
  (cl:list 'BallPosForHand-request
    (cl:cons ':min_h (min_h msg))
    (cl:cons ':max_h (max_h msg))
))
;//! \htmlinclude BallPosForHand-response.msg.html

(cl:defclass <BallPosForHand-response> (roslisp-msg-protocol:ros-message)
  ((pos_x
    :reader pos_x
    :initarg :pos_x
    :type cl:float
    :initform 0.0)
   (pos_y
    :reader pos_y
    :initarg :pos_y
    :type cl:float
    :initform 0.0)
   (pos_z
    :reader pos_z
    :initarg :pos_z
    :type cl:float
    :initform 0.0))
)

(cl:defclass BallPosForHand-response (<BallPosForHand-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BallPosForHand-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BallPosForHand-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nao_behavior_tree-srv:<BallPosForHand-response> is deprecated: use nao_behavior_tree-srv:BallPosForHand-response instead.")))

(cl:ensure-generic-function 'pos_x-val :lambda-list '(m))
(cl:defmethod pos_x-val ((m <BallPosForHand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_behavior_tree-srv:pos_x-val is deprecated.  Use nao_behavior_tree-srv:pos_x instead.")
  (pos_x m))

(cl:ensure-generic-function 'pos_y-val :lambda-list '(m))
(cl:defmethod pos_y-val ((m <BallPosForHand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_behavior_tree-srv:pos_y-val is deprecated.  Use nao_behavior_tree-srv:pos_y instead.")
  (pos_y m))

(cl:ensure-generic-function 'pos_z-val :lambda-list '(m))
(cl:defmethod pos_z-val ((m <BallPosForHand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_behavior_tree-srv:pos_z-val is deprecated.  Use nao_behavior_tree-srv:pos_z instead.")
  (pos_z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BallPosForHand-response>) ostream)
  "Serializes a message object of type '<BallPosForHand-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pos_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pos_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pos_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BallPosForHand-response>) istream)
  "Deserializes a message object of type '<BallPosForHand-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pos_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pos_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pos_z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BallPosForHand-response>)))
  "Returns string type for a service object of type '<BallPosForHand-response>"
  "nao_behavior_tree/BallPosForHandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BallPosForHand-response)))
  "Returns string type for a service object of type 'BallPosForHand-response"
  "nao_behavior_tree/BallPosForHandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BallPosForHand-response>)))
  "Returns md5sum for a message object of type '<BallPosForHand-response>"
  "074da7694199f61d03c662ae92d91008")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BallPosForHand-response)))
  "Returns md5sum for a message object of type 'BallPosForHand-response"
  "074da7694199f61d03c662ae92d91008")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BallPosForHand-response>)))
  "Returns full string definition for message of type '<BallPosForHand-response>"
  (cl:format cl:nil "float32 pos_x~%float32 pos_y~%float32 pos_z~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BallPosForHand-response)))
  "Returns full string definition for message of type 'BallPosForHand-response"
  (cl:format cl:nil "float32 pos_x~%float32 pos_y~%float32 pos_z~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BallPosForHand-response>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BallPosForHand-response>))
  "Converts a ROS message object to a list"
  (cl:list 'BallPosForHand-response
    (cl:cons ':pos_x (pos_x msg))
    (cl:cons ':pos_y (pos_y msg))
    (cl:cons ':pos_z (pos_z msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'BallPosForHand)))
  'BallPosForHand-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'BallPosForHand)))
  'BallPosForHand-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BallPosForHand)))
  "Returns string type for a service object of type '<BallPosForHand>"
  "nao_behavior_tree/BallPosForHand")