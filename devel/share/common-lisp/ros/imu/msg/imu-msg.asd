
(cl:in-package :asdf)

(defsystem "imu-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CarPose" :depends-on ("_package_CarPose"))
    (:file "_package_CarPose" :depends-on ("_package"))
  ))