#!/bin/usr/env roseus

(load "package://fetcheus/fetch-interface.l")
(fetch-init)

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
