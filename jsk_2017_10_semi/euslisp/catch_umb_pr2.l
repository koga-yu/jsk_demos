#!/bin/usr/env roseus

(load "package://pr2eus/pr2-interface.l")
(pr2-init)

(load "make_models.l")
(setq umb (instance bodyset-link :init (make-cascoords) :bodies (list (make-umb))))
(send umb :locate #f(1000 0 300))

;(send *ri* :angle-vector #f(0 0 0 0 0 0 0 0 0 0))
(objects (list *pr2* umb))

;only move after merged "enable to run go-grasp on kinematic simulation mode "
(send *ri* :stop-grasp :rarm)

(send *pr2* :rarm :inverse-kinematics (make-coords :pos #f(600 0 900)))
(send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
(send *ri* :wait-interpolation)

(send *pr2* :rarm :inverse-kinematics (make-coords :pos #f(700 0 900)))
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(unix::sleep 2)

(send *pr2* :rarm :end-coords :assoc umb)
(send *ri* :start-grasp :rarm)
(send *ri* :wait-interpolation)

(do-until-key ())

(send *pr2* :rarm :inverse-kinematics (make-coords :pos #f(700 0 1200)))
(send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
(send *ri* :wait-interpolation)

(unix::sleep 1)

(send *pr2* :rarm :inverse-kinematics (make-coords :pos #f(600 0 1200)))
(send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
(send *ri* :wait-interpolation)

(send *pr2* :rarm :inverse-kinematics (make-coords :pos #f(600 0 700)))
(send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
(send *ri* :wait-interpolation)

(send *ri* :stop-grasp :rarm)
(send *pr2* :rarm :end-coords :dissoc umb)

(unix:sleep 1)

(send *pr2* :rarm :inverse-kinematics (make-coords :pos #f(600 0 900)))
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)
