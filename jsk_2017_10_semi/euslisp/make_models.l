#!/usr/bin/env roseus

;make models of umbrella and stand

(defun make-umbrella nil
    (setq body (make-cylinder 50 500))
    (send body :locate #f(300 0 0))
    body
)
