;;; Copyright (c) 2014, Fereshta Yazdani <yazdani@cs.uni-bremen.de>
;;; All rights reserved.
;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to 
;;;       endorse or promote products derived from this software without 
;;;       specific prior written permission.
;;; 
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.


(in-package :sherpa)

(defun start-scenario ()
  (start-myros)
  (right-arm-trajectory)
  (end-myros))

(defun start-myros ()
  (cram-roslisp-common:startup-ros :anonymous nil))

(defun end-myros ()
  (cram-roslisp-common:shutdown-ros))

(defun execute-right-arm-trajectory (trajec)
  (roslisp:ros-info (sherpa-spatial-relations) "Execute-right")
  (let* ((act-cli (actionlib:make-action-client
                   "/r_arm_controller/joint_trajectory_action"
                   "pr2_controllers_msgs/JointTrajectoryAction"))
         (act-goal (actionlib:make-action-goal 
                       act-cli
                     :trajectory (pr2-manip-pm::remove-trajectory-joints
                                  #("r_shoulder_pan_joint" "r_shoulder_lift_joint" "r_upper_arm_roll_joint" "r_elbow_flex_joint" "r_forearm_roll_joint" "r_wrist_flex_joint" "r_wrist_roll_joint") 
                                  trajec :invert t))))
    (actionlib:wait-for-server act-cli)
    (actionlib:call-goal act-cli act-goal))
  (roslisp:ros-info (sherpa-spatial-relations) "Finished"))

(defun default-position-to-trajectory ()
  (roslisp:ros-info (sherpa-spatial-relations) "Default Position to Trajectory")
  (roslisp:make-msg
   "trajectory_msgs/JointTrajectory"
   (stamp header)
   (roslisp:ros-time)
   joint_names #("r_shoulder_pan_joint" "r_shoulder_lift_joint" "r_upper_arm_roll_joint" "r_elbow_flex_joint" "r_forearm_roll_joint" "r_wrist_flex_joint" "r_wrist_roll_joint")
   points (vector
           (roslisp:make-message
            "trajectory_msgs/JointTrajectoryPoint"
            positions #(0.0 0.0 0.0 0.0 0.0 0.0 0.0);(vector position)
            velocities #(0)
            accelerations #(0)
            time_from_start 2.0))))

(defun position-to-trajectory ()
  (roslisp:ros-info (sherpa-spatial-relations) "Position to Trajectory")
  (roslisp:make-msg
   "trajectory_msgs/JointTrajectory"
   (stamp header)
   (roslisp:ros-time)
   joint_names #("r_shoulder_pan_joint" "r_shoulder_lift_joint" "r_upper_arm_roll_joint" "r_elbow_flex_joint" "r_forearm_roll_joint" "r_wrist_flex_joint" "r_wrist_roll_joint")
   points (vector
           (roslisp:make-message
            "trajectory_msgs/JointTrajectoryPoint"
            positions #(0.0 -0.5 0.0 0.0 0.0 0.0 1.5);(vector position)
            velocities #(0)
            accelerations #(0)
            time_from_start 2.0))))

(defun right-arm-trajectory () 
  (roslisp:ros-info (sherpa-spatial-relations) "Moving right arm")
  (execute-right-arm-trajectory (default-position-to-trajectory))
  (start-myros)
  (execute-right-arm-trajectory (position-to-trajectory)))

