#!/usr/bin/env roseus

(defun make-ambrella nil
    (setq body (make-cylinder 50 500))
    (send body :locate #f(300 )
