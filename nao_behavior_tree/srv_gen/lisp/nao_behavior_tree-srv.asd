
(cl:in-package :asdf)

(defsystem "nao_behavior_tree-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "BallPosForHead" :depends-on ("_package_BallPosForHead"))
    (:file "_package_BallPosForHead" :depends-on ("_package"))
    (:file "GetArmReadyBT" :depends-on ("_package_GetArmReadyBT"))
    (:file "_package_GetArmReadyBT" :depends-on ("_package"))
    (:file "ArmReadyBT" :depends-on ("_package_ArmReadyBT"))
    (:file "_package_ArmReadyBT" :depends-on ("_package"))
    (:file "BallPosForHand" :depends-on ("_package_BallPosForHand"))
    (:file "_package_BallPosForHand" :depends-on ("_package"))
    (:file "BallPosGetBT" :depends-on ("_package_BallPosGetBT"))
    (:file "_package_BallPosGetBT" :depends-on ("_package"))
    (:file "BallPosChangeBT" :depends-on ("_package_BallPosChangeBT"))
    (:file "_package_BallPosChangeBT" :depends-on ("_package"))
  ))