
(cl:in-package :asdf)

(defsystem "wheel_odom-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "CarSpeed" :depends-on ("_package_CarSpeed"))
    (:file "_package_CarSpeed" :depends-on ("_package"))
  ))