#!/usr/bin/env roseus

;make models of umbrella and stand

(defun make-umb nil
    (setq body-u (make-solid-of-revolution (list #f(2 0 500) #f(2 0 400) #f(2 0 300) #f(2 0 200) #f(2 0 0) #f(50 0 500))))
    (send body-u :rotate 0.3 :z)
    (send body-u :locate #f(0 0 0))

    (setq shaft (make-cylinder 10 70))
    (send shaft :locate #f(0 0 501))
    (send shaft :rotate 0.2 :z)

    (setq handle (make-cylinder 50 5))
    (send handle :locate #f(0 0 0))
    (setq handle_o (make-cylinder 30 10))
    (send handle_o :locate #f(0 0 0))
    (setq handle (body- handle handle_o))
    (send handle :rotate 0.2 :z)
    (send handle :rotate pi/2 :x)
    (send handle :locate #f(43 503 0))
    (setq body-u (body+ body-u shaft))
    body-u
)

;(setq umb-abody (instance bodyset-link :init (make-cascoords) :bodies (list (make-umbrella-abody))))
;(setq umb-handle (instance bodyset-link :init (make-cascoords) :bodies (list (make-handle))))
(setq umb (instance bodyset-link :init (make-cascoords) :bodies (list (make-umb))))

(objects (list umb))
