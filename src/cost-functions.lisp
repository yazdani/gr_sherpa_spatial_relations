
(in-package :sherpa)

(defun make-spatial-relation-cost-function (location axis pred)
  (roslisp:ros-info (sherpa-spatial-relations) "calculate the costmap")
  (let ((world->location-transformation (cl-transforms:transform-inv location)))
    (lambda (x y)
      (let* ((point (cl-transforms:transform-point world->location-transformation
                                                   (cl-transforms:make-3d-vector x y 0)))
             (coord (ecase axis
                      (:x (cl-transforms:x point))
                      (:y (cl-transforms:y point))))
             (mode (sqrt (+  (* (cl-transforms:x point) (cl-transforms:x point))
                            (* (cl-transforms:y point) (cl-transforms:y point))))))
        (if (funcall pred coord 0.0d0)
	    (abs (/ coord mode))
            0.0d0)))))

(defun make-constant-height-function (height)
  (lambda (x y)
    (declare (ignore x y))
    (list height)))