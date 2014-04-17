
(cl:in-package :asdf)

(defsystem "first_approaches-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PosImage" :depends-on ("_package_PosImage"))
    (:file "_package_PosImage" :depends-on ("_package"))
  ))