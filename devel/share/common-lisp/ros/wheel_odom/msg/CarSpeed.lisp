; Auto-generated. Do not edit!


(cl:in-package wheel_odom-msg)


;//! \htmlinclude CarSpeed.msg.html

(cl:defclass <CarSpeed> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (car_delta_x
    :reader car_delta_x
    :initarg :car_delta_x
    :type cl:float
    :initform 0.0)
   (car_delta_y
    :reader car_delta_y
    :initarg :car_delta_y
    :type cl:float
    :initform 0.0)
   (car_delta_th
    :reader car_delta_th
    :initarg :car_delta_th
    :type cl:float
    :initform 0.0)
   (left_speed
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

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CarSpeed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wheel_odom-msg:header-val is deprecated.  Use wheel_odom-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'car_delta_x-val :lambda-list '(m))
(cl:defmethod car_delta_x-val ((m <CarSpeed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wheel_odom-msg:car_delta_x-val is deprecated.  Use wheel_odom-msg:car_delta_x instead.")
  (car_delta_x m))

(cl:ensure-generic-function 'car_delta_y-val :lambda-list '(m))
(cl:defmethod car_delta_y-val ((m <CarSpeed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wheel_odom-msg:car_delta_y-val is deprecated.  Use wheel_odom-msg:car_delta_y instead.")
  (car_delta_y m))

(cl:ensure-generic-function 'car_delta_th-val :lambda-list '(m))
(cl:defmethod car_delta_th-val ((m <CarSpeed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wheel_odom-msg:car_delta_th-val is deprecated.  Use wheel_odom-msg:car_delta_th instead.")
  (car_delta_th m))

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
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'car_delta_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'car_delta_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'car_delta_th))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
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
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'car_delta_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'car_delta_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'car_delta_th) (roslisp-utils:decode-single-float-bits bits)))
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
  "1f649a2ed7af503fb863d87829267e47")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CarSpeed)))
  "Returns md5sum for a message object of type 'CarSpeed"
  "1f649a2ed7af503fb863d87829267e47")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CarSpeed>)))
  "Returns full string definition for message of type '<CarSpeed>"
  (cl:format cl:nil "Header header~%~%float32 car_delta_x~%float32 car_delta_y~%float32 car_delta_th~%float32 left_speed~%float32 right_speed~%float32 vx~%float32 vy~%float32 vth~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CarSpeed)))
  "Returns full string definition for message of type 'CarSpeed"
  (cl:format cl:nil "Header header~%~%float32 car_delta_x~%float32 car_delta_y~%float32 car_delta_th~%float32 left_speed~%float32 right_speed~%float32 vx~%float32 vy~%float32 vth~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CarSpeed>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CarSpeed>))
  "Converts a ROS message object to a list"
  (cl:list 'CarSpeed
    (cl:cons ':header (header msg))
    (cl:cons ':car_delta_x (car_delta_x msg))
    (cl:cons ':car_delta_y (car_delta_y msg))
    (cl:cons ':car_delta_th (car_delta_th msg))
    (cl:cons ':left_speed (left_speed msg))
    (cl:cons ':right_speed (right_speed msg))
    (cl:cons ':vx (vx msg))
    (cl:cons ':vy (vy msg))
    (cl:cons ':vth (vth msg))
))
