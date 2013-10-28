(defsystem sherpa-spatial-relations
 :depends-on (cram-roslisp-common
	      designators
	      cram-reasoning
	      roslisp
	      semantic-map-costmap
	      cram-language
	      cram-plan-knowledge
	      occupancy-grid-costmap     
	      object-location-designators)
 :components 
 ((:module "src"
	   :components
	   ((:file "package")
	    (:file "prolog" :depends-on ("package"))
	    (:file "knowledge" :depends-on("package"))
            (:file "designator-integration" :depends-on("package"))))))