
(cl:in-package :asdf)

(defsystem "lidar_localization-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "saveOdometry" :depends-on ("_package_saveOdometry"))
    (:file "_package_saveOdometry" :depends-on ("_package"))
    (:file "saveScanContext" :depends-on ("_package_saveScanContext"))
    (:file "_package_saveScanContext" :depends-on ("_package"))
  ))