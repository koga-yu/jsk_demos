#!/bin/usr/env roseus

(load "package://fetcheus/fetch-interface.l")
(fetch-init)

(load "make_models.l")
(setq umb (instance bodyset-link :init (make-cascoords) :bodies (list (make-umb))))
(send umb :locate #f(800 0 300))

(send *ri* :angle-vector #f(0 0 0 0 0 0 0 0 0 0))
(objects (list *fetch* umb))
(send *fetch* :rarm :inverse-kinematics (make-coords :pos #f(700 0 900)))))
(send *ri* :angle-vector (send *fetch* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *fetch* :rarm :inverse-kinematics (make-coords :pos #f(800 0 900)))))
(send *ri* :angle-vector (send *fetch* :angle-vector) 3000)
(send *ri* :wait-interpolation)

;(send *ri* :start-grasp)
