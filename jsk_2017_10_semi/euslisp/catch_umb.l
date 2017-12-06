#!/usr/bin/env roseus

(load "package://fetcheus/fetch-interface.l")

(ros::roseus "catch_umb")
(fetch-init)

(ros::load-ros-manifest "jsk_2017_10_semi")

(defvar detected_umb_pos_x 0)
(defvar detected_umb_pos_y 0)
(defvar detected_umb_pos_z 0)

;;define /umb_pos subscriber
(defun umb-pos-sub (msg)
    (setq detected_umb_pos_x (send msg :x))
    (setq detected_umb_pos_y (send msg :y))
    (setq detected_umb_pos_z (send msg :z))
    (ros::ros-info (format nil "~A ~A ~A~%" (send msg :x) (send msg :y) (send msg :z)))
    (terpri)
)

(ros::subscribe "/detected_umb_pos"
    jsk_2017_10_semi::umb_pos
    #'umb-pos-sub 1)

(load "make_models.l")
(setq umb (instance bodyset-link :init (make-cascoords) :bodies (list (make-umb))))
(send umb :locate #f(1000 0 300))

;(send *ri* :angle-vector #f(0 0 0 0 0 0 0 0 0 0))
(objects (list *fetch* umb))

;only move after merged "enable to run go-grasp on kinematic simulation mode "
(send *ri* :stop-grasp)

(send *fetch* :rarm :inverse-kinematics (make-coords :pos #f(900 0 900)))
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
(send *ri* :wait-interpolation)

(send *fetch* :rarm :inverse-kinematics (make-coords :pos #f(1000 0 900)))
(send *ri* :angle-vector (send *fetch* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(unix::sleep 2)

(send *fetch* :rarm :end-coords :assoc umb)
(send *ri* :start-grasp)
(send *ri* :wait-interpolation)

(do-until-key ())

(send *fetch* :rarm :inverse-kinematics (make-coords :pos #f(1000 0 1200)))
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
(send *ri* :wait-interpolation)

(unix::sleep 1)

(send *fetch* :rarm :inverse-kinematics (make-coords :pos #f( 900 0 1200)))
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
(send *ri* :wait-interpolation)

(send *fetch* :rarm :inverse-kinematics (make-coords :pos #f( 900 0 700)))
(send *ri* :angle-vector (send *fetch* :angle-vector) 10000)
(send *ri* :wait-interpolation)

(send *ri* :stop-grasp)
(send *fetch* :rarm :end-coords :dissoc umb)

(unix:sleep 1)

(send *fetch* :rarm :inverse-kinematics (make-coords :pos #f(900 0 900)))
(send *ri* :angle-vector (send *fetch* :angle-vector) 3000)
(send *ri* :wait-interpolation)
