; Auto-generated. Do not edit!


(cl:in-package wheel_odom-msg)


;//! \htmlinclude CarSpeed.msg.html

(cl:defclass <CarSpeed> (roslisp-msg-protocol:ros-message)
  ((left_speed
    :reader left_speed
    :initarg :left_speed
    :type cl:float
    :initform 0.0)
   (right_speed
    :reader right_speed
    :initarg :right_speed
    :type cl:float
    :initform 0.0)
   (vx
    :reader vx
    :initarg :vx
    :type cl:float
    :initform 0.0)
   (vy
    :reader vy
    :initarg :vy
    :type cl:float
    :initform 0.0)
   (vth
    :reader vth
    :initarg :vth
    :type cl:float
    :initform 0.0))
)

(cl:defclass CarSpeed (<CarSpeed>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CarSpeed>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CarSpeed)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name wheel_odom-msg:<CarSpeed> is deprecated: use wheel_odom-msg:CarSpeed instead.")))

(cl:ensure-generic-function 'left_speed-val :lambda-list '(m))
(cl:defmethod left_speed-val ((m <CarSpeed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wheel_odom-msg:left_speed-val is deprecated.  Use wheel_odom-msg:left_speed instead.")
  (left_speed m))

(cl:ensure-generic-function 'right_speed-val :lambda-list '(m))
(cl:defmethod right_speed-val ((m <CarSpeed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wheel_odom-msg:right_speed-val is deprecated.  Use wheel_odom-msg:right_speed instead.")
  (right_speed m))

(cl:ensure-generic-function 'vx-val :lambda-list '(m))
(cl:defmethod vx-val ((m <CarSpeed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wheel_odom-msg:vx-val is deprecated.  Use wheel_odom-msg:vx instead.")
  (vx m))

(cl:ensure-generic-function 'vy-val :lambda-list '(m))
(cl:defmethod vy-val ((m <CarSpeed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wheel_odom-msg:vy-val is deprecated.  Use wheel_odom-msg:vy instead.")
  (vy m))

(cl:ensure-generic-function 'vth-val :lambda-list '(m))
(cl:defmethod vth-val ((m <CarSpeed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wheel_odom-msg:vth-val is deprecated.  Use wheel_odom-msg:vth instead.")
  (vth m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CarSpeed>) ostream)
  "Serializes a message object of type '<CarSpeed>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CarSpeed>) istream)
  "Deserializes a message object of type '<CarSpeed>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_speed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_speed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vy) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vth) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CarSpeed>)))
  "Returns string type for a message object of type '<CarSpeed>"
  "wheel_odom/CarSpeed")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CarSpeed)))
  "Returns string type for a message object of type 'CarSpeed"
  "wheel_odom/CarSpeed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CarSpeed>)))
  "Returns md5sum for a message object of type '<CarSpeed>"
  "262aadc0a5d48657eda9fd254eaeedbe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CarSpeed)))
  "Returns md5sum for a message object of type 'CarSpeed"
  "262aadc0a5d48657eda9fd254eaeedbe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CarSpeed>)))
  "Returns full string definition for message of type '<CarSpeed>"
  (cl:format cl:nil "float32 left_speed~%float32 right_speed~%float32 vx~%float32 vy~%float32 vth~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CarSpeed)))
  "Returns full string definition for message of type 'CarSpeed"
  (cl:format cl:nil "float32 left_speed~%float32 right_speed~%float32 vx~%float32 vy~%float32 vth~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CarSpeed>))
  (cl:+ 0
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CarSpeed>))
  "Converts a ROS message object to a list"
  (cl:list 'CarSpeed
    (cl:cons ':left_speed (left_speed msg))
    (cl:cons ':right_speed (right_speed msg))
    (cl:cons ':vx (vx msg))
    (cl:cons ':vy (vy msg))
    (cl:cons ':vth (vth msg))
))
