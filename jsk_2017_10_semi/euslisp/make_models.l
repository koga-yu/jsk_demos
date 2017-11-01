#!/usr/bin/env roseus

;make models of umbrella and stand

(defun make-umbrella nil
    (setq body (make-cylinder 50 500))
    (send body :locate #f(300 0 0))
    (setq handle (make-cylinder 50 5))
    (send handle :locate #f(0 0 0))
    (setq handle_o (make-cylinder 40 5))
    (send handle_o :locate #f(0 0 0))
    ;(setq handle (body- handle handle_o))
    handle
)

(setq umb (instance bodyset-link :init (make-cascoords) :bodies (list (make-umbrella))))
(objects (list umb))
