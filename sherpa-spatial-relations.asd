(defsystem sherpa-spatial-relations
 :depends-on (roslisp
	      cram-language
	      cram-roslisp-common
              actionlib
	      pr2-manipulation-process-module)
 :components 
 ((:module "src"
	   :components
	   ((:file "package")
	    (:file "prolog" :depends-on ("package"))
	    (:file "knowledge" :depends-on("package"))
            (:file "build-test-world" :depends-on("package"))
            (:file "designator-integration" :depends-on("package"))))))