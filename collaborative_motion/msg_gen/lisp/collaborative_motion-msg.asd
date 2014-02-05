
(cl:in-package :asdf)

(defsystem "collaborative_motion-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PosImage" :depends-on ("_package_PosImage"))
    (:file "_package_PosImage" :depends-on ("_package"))
  ))