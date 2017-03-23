; Auto-generated. Do not edit!


(cl:in-package imu-msg)


;//! \htmlinclude CarPose.msg.html

(cl:defclass <CarPose> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (roll
    :reader roll
    :initarg :roll
    :type cl:float
    :initform 0.0)
   (roll_relative
    :reader roll_relative
    :initarg :roll_relative
    :type cl:float
    :initform 0.0)
   (pitch
    :reader pitch
    :initarg :pitch
    :type cl:float
    :initform 0.0)
   (pitch_relative
    :reader pitch_relative
    :initarg :pitch_relative
    :type cl:float
    :initform 0.0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (yaw_relative
    :reader yaw_relative
    :initarg :yaw_relative
    :type cl:float
    :initform 0.0))
)

(cl:defclass CarPose (<CarPose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CarPose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CarPose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name imu-msg:<CarPose> is deprecated: use imu-msg:CarPose instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CarPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu-msg:header-val is deprecated.  Use imu-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'roll-val :lambda-list '(m))
(cl:defmethod roll-val ((m <CarPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu-msg:roll-val is deprecated.  Use imu-msg:roll instead.")
  (roll m))

(cl:ensure-generic-function 'roll_relative-val :lambda-list '(m))
(cl:defmethod roll_relative-val ((m <CarPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu-msg:roll_relative-val is deprecated.  Use imu-msg:roll_relative instead.")
  (roll_relative m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <CarPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu-msg:pitch-val is deprecated.  Use imu-msg:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'pitch_relative-val :lambda-list '(m))
(cl:defmethod pitch_relative-val ((m <CarPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu-msg:pitch_relative-val is deprecated.  Use imu-msg:pitch_relative instead.")
  (pitch_relative m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <CarPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu-msg:yaw-val is deprecated.  Use imu-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'yaw_relative-val :lambda-list '(m))
(cl:defmethod yaw_relative-val ((m <CarPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu-msg:yaw_relative-val is deprecated.  Use imu-msg:yaw_relative instead.")
  (yaw_relative m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CarPose>) ostream)
  "Serializes a message object of type '<CarPose>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll_relative))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch_relative))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw_relative))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CarPose>) istream)
  "Deserializes a message object of type '<CarPose>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll_relative) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch_relative) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw_relative) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CarPose>)))
  "Returns string type for a message object of type '<CarPose>"
  "imu/CarPose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CarPose)))
  "Returns string type for a message object of type 'CarPose"
  "imu/CarPose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CarPose>)))
  "Returns md5sum for a message object of type '<CarPose>"
  "765b9e32d0441b8f51f41e5ac9438bb0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CarPose)))
  "Returns md5sum for a message object of type 'CarPose"
  "765b9e32d0441b8f51f41e5ac9438bb0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CarPose>)))
  "Returns full string definition for message of type '<CarPose>"
  (cl:format cl:nil "Header header~%~%float32 roll~%float32 roll_relative~%float32 pitch~%float32 pitch_relative~%float32 yaw~%float32 yaw_relative~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CarPose)))
  "Returns full string definition for message of type 'CarPose"
  (cl:format cl:nil "Header header~%~%float32 roll~%float32 roll_relative~%float32 pitch~%float32 pitch_relative~%float32 yaw~%float32 yaw_relative~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CarPose>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CarPose>))
  "Converts a ROS message object to a list"
  (cl:list 'CarPose
    (cl:cons ':header (header msg))
    (cl:cons ':roll (roll msg))
    (cl:cons ':roll_relative (roll_relative msg))
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':pitch_relative (pitch_relative msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':yaw_relative (yaw_relative msg))
))
