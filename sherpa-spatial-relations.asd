(defsystem sherpa-spatial-relations
  :depends-on (designators
               cram-roslisp-common
               cram-reasoning
               roslisp
               bullet-reasoning
               cram-pr2-knowledge
               cram-environment-representation
               projection-process-modules
               cram-plan-knowledge
               bullet-reasoning-designators
               pr2-manipulation-knowledge
               object-location-designators
	       actionlib)
 :components 
 ((:module "src"
	   :components
	   ((:file "package")
	    (:file "prolog" :depends-on ("package"))
	    (:file "knowledge" :depends-on("package"))
            (:file "build-test-world" :depends-on("package"))
            (:file "designator-integration" :depends-on("package"))))))