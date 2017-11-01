#!/usr/bin/env roseus

;make models of umbrella and stand

(defun make-umb nil
    (setq body-u (make-cylinder 40 500))
    (send body-u :rotate 0.5 :z)
    (send body-u :locate #f(0 0 0))
    (setq handle (make-cylinder 50 5))
    (send handle :locate #f(0 0 0))
    (setq handle_o (make-cylinder 30 10))
    (send handle_o :locate #f(0 0 0))
    (setq handle (body- handle handle_o))
    (send handle :rotate 0.2 :z)
    (send handle :rotate pi/2 :x)
    (send handle :locate #f(41 501 0))
    (setq body-u (body+ body-u handle))
    body-u
)

;(setq umb-abody (instance bodyset-link :init (make-cascoords) :bodies (list (make-umbrella-abody))))
;(setq umb-handle (instance bodyset-link :init (make-cascoords) :bodies (list (make-umbrella-handle))))
(setq umb (instance bodyset-link :init (make-cascoords) :bodies (list (make-umb))))

(objects (list umb))

