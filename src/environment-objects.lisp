(defparameter *mesh-files* '((tree1 "package://sherpa_spatial_relations/models/tree-5.stl" nil)
			     (tree2 "package://sherpa_spatial_relations/models/tree-2.stl" nil)
			     (tree3 "package://sherpa_spatial_relations/models/tree-3.stl" nil)
			     (tree4 "package://sherpa_spatial_relations/models/tree-4.stl" nil)
			     (quad1 "package://sherpa_spatial_relations/models/quadrotor.stl" nil)
))

(defclass environment-object (object)
  ((types :reader environment-object-types :initarg :types)))

(defgeneric environment-object-dimensions (object)
  (:method ((object environment-object))
    (bounding-box-dimensions (aabb object)))
  (:method ((object-type symbol))
    (or (cutlery-dimensions object-type)
        (let ((mesh-specification (assoc object-type *mesh-files*)))
          (assert
           mesh-specification ()
           "Couldn't fine a mesh for object type ~a." object-type)
          (destructuring-bind (type uri &optional flip-winding-order)
              mesh-specification
            (declare (ignore type))
            (let ((model-filename (physics-utils:parse-uri uri)))
              (with-file-cache
                  model model-filename (physics-utils:load-3d-model
                                        model-filename
                                        :flip-winding-order flip-winding-order)
                (values
                 (physics-utils:calculate-aabb
                  (physics-utils:3d-model-vertices model))))))))))

(defmethod add-object ((world bt-world) (type (eql 'mesh)) name pose
                       &key mass mesh (color '(0.5 0.5 0.5 1.0)) types (scale 1.0)
                         disable-face-culling)
  (let ((mesh-model (physics-utils:scale-3d-model
                     (etypecase mesh
                       (symbol (let ((uri (physics-utils:parse-uri (cadr (assoc mesh *mesh-files*)))))
  (with-file-cache model uri                                  
                                     (physics-utils:load-3d-model
                                      uri :flip-winding-order (caddr (assoc mesh *mesh-files*)))
                                   model)))
                       (string (let ((uri  (physics-utils:parse-uri mesh)))
                                 (with-file-cache model uri (physics-utils:load-3d-model uri)
                                   model)))
                       (physics-utils:3d-model mesh))
                     scale)))
    (make-household-object world name (or types (list mesh))
                           (list
                            (make-instance 'rigid-body
                              :name name :mass mass :pose (ensure-pose pose)
                              :collision-shape (make-instance 'convex-hull-mesh-shape
                                                 :points (physics-utils:3d-model-vertices mesh-model)
                                                 :faces (physics-utils:3d-model-faces mesh-model)
                                                 :color color
                                                 :disable-face-culling disable-face-culling))))))
