#!/bin/usr/env roseus

(load "package://fetcheus/fetch-interface.l")
(fetch-init)

(load "make_models.l")
(setq umb (instance bodyset-link :init (make-cascoords) :bodies (list (make-umb))))
(send umb :locate #f(1000 0 300))

(send *ri* :angle-vector #f(0 0 0 0 0 0 0 0 0 0))
(objects (list *fetch* umb))
(send *fetch* :rarm :inverse-kinematics (make-coords :pos #f(900 0 900)))
(send *ri* :angle-vector (send *fetch* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *fetch* :rarm :inverse-kinematics (make-coords :pos #f(1000 0 900)))
(send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
(send *ri* :wait-interpolation)

(send *fetch* :rarm :end-coords :assoc umb)
;(send *ri* :start-grasp)

(send *fetch* :rarm :inverse-kinematics (make-coords :pos #f(1000 0 1200)))
(send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
(send *ri* :wait-interpolation)

;(unix::usleep 1000)

(dotimes (i 10)
    (send *fetch* :rarm :inverse-kinematics (make-coords :pos (float-vector 900 0 (- 1200 (* i 50)))))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
)
