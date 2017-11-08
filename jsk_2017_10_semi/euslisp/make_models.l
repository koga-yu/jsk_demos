#!/usr/bin/env roseus

;make models of umbrella and stand

(defun make-umb nil
    (setq body-u (make-solid-of-revolution (list #f(2 0 500) #f(2 0 400) #f(2 0 300) #f(2 0 200) #f(2 0 0) #f(50 0 500))))
    (send body-u :rotate 0.3 :z)
    (send body-u :locate #f(0 0 0))

    (setq shaft (make-cylinder 10 70))
    (send shaft :locate #f(0 0 501))
    (send shaft :rotate 0.2 :z)

    (setq handle (make-prism (list #f(0 0 0) #f(15 0 0) #f(15 21 0) #f(30 36 0) #f(51 15 0) #f(60 30 0) #f(30 60 0) #f(0 30 0)) 15))
    (send handle :locate #f(0 0 572))
    (send handle :rotate pi/2 :x)
    (send handle :translate #f(-7.5 0 -7.5))

    (setq body-u (body+ body-u shaft))
    (setq body-u (body+ body-u handle))
    body-u
)
