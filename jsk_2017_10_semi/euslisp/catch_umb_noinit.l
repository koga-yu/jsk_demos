#!/usr/bin/env roseus
(load "package://jsk_maps/src/eng2-scene.l")
(setq *scene* (make-eng2-scene))

(defvar detected_umb_pos_x 0)
(defvar detected_umb_pos_y 0)
(defvar detected_umb_pos_z 0)

(setq *tfb* (instance ros::transform-broadcaster :init))

;;define detected_umb_pos subscriber
(defun umb-pos-sub (msg)
    (setq ret (make-coords :pos (float-vector detected_umb_pos_x detected_umb_pos_y detected_umb_pos_z)))
    (send *tfb* :send-transform ret "base_link" "umb_handle")
    (setq detected_umb_pos_x (send msg :x))
    (setq detected_umb_pos_y (send msg :y))
    (setq detected_umb_pos_z (send msg :z))
    (ros::ros-info (format nil "~A ~A ~A~%" (send msg :x) (send msg :y) (send msg :z)))
    (terpri)
)

(print "add subscribe")
(ros::subscribe "detected_umb_pos"
    jsk_2017_10_semi::umb_pos
    #'umb-pos-sub)

(print "do until key")
(ros::rate 10)
(do-until-key
    (ros::spin-once)
    (ros::sleep)
    )
(ros::ros-info (format nil "umb detected at ~A ~A ~A~%" detected_umb_pos_x detected_umb_pos_y detected_umb_pos_z))

(load "make_models.l")
(setq umb (instance bodyset-link :init (make-cascoords) :bodies (list (make-umb))))
(send umb :locate #f(1000 0 300))

;(send *ri* :angle-vector #f(0 0 0 0 0 0 0 0 0 0))
(objects (list *fetch* umb))

;only move after merged "enable to run go-grasp on kinematic simulation mode "
(send *ri* :stop-grasp)

(setq inv-success
    (send *fetch* :rarm :inverse-kinematics
        (make-coords :pos (float-vector (- detected_umb_pos_x 200) detected_umb_pos_y detected_umb_pos_z))
    )
)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
(send *ri* :wait-interpolation)

(do-until-key ())

(send *fetch* :rarm :inverse-kinematics
    (make-coords :pos (float-vector (- detected_umb_pos_x 60) detected_umb_pos_y detected_umb_pos_z))
)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
(send *ri* :wait-interpolation)

(do-until-key ())

(send *fetch* :rarm :end-coords :assoc umb)
(send *ri* :start-grasp)
(send *fetch* :rarm :inverse-kinematics
    (make-coords :pos (float-vector (- detected_umb_pos_x 60) detected_umb_pos_y detected_umb_pos_z))
)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
(send *ri* :wait-interpolation)

(do-until-key ())

(setq inv-success
    (send *fetch* :rarm :inverse-kinematics
        (make-coords :pos (float-vector (- detected_umb_pos_x 80) detected_umb_pos_y (+ detected_umb_pos_z 50)))
    )
)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
(send *ri* :wait-interpolation)

(do-until-key ())

(setq inv-success
    (send *fetch* :rarm :inverse-kinematics
        (make-coords :pos (float-vector 300 -200 1300))
    )
)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
(send *ri* :wait-interpolation)

(do-until-key ())
(send *ri* :move-to (send *scene* :spot "/eng2/7f/room73B2-door-front"))

(do-until-key ())

(setq inv-success
    (send *fetch* :rarm :inverse-kinematics
        (make-coords :pos (float-vector 700 0 1400))
    )
)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
(send *ri* :wait-interpolation)

(do-until-key ())

(setq inv-success
    (send *fetch* :rarm :inverse-kinematics
        (make-coords :pos (float-vector 700 0 900))
    )
)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
(send *ri* :wait-interpolation)

(do-until-key ())

(send *ri* :stop-grasp)

;(send *fetch* :rarm :inverse-kinematics (make-coords :pos #f(900 0 1200)))
;(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
;(send *ri* :wait-interpolation)

;(do-until-key ())
;(unix::sleep 1)

;(send *fetch* :rarm :inverse-kinematics (make-coords :pos #f( 900 0 700)))
;(send *ri* :angle-vector (send *fetch* :angle-vector) 10000)
;(send *ri* :wait-interpolation)

;(send *ri* :stop-grasp)
;(send *fetch* :rarm :end-coords :dissoc umb)

(unix:sleep 1)

;(send *fetch* :rarm :inverse-kinematics (make-coords :pos #f(900 0 900)))
;(send *ri* :angle-vector (send *fetch* :angle-vector) 3000)
;(send *ri* :wait-interpolation)
