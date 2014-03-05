
(cl:in-package :asdf)

(defsystem "naos_localization-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Odometry" :depends-on ("_package_Odometry"))
    (:file "_package_Odometry" :depends-on ("_package"))
  ))