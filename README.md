dmp_gestures
============

Gesture learning and execution via DMPs

Based on the previous work on https://github.com/awesomebytes/dmp_reem_razer a more generalized version of gesture learning and reproduction using dynamic motion primitives.

Uses https://github.com/awesomebytes/dmp based on http://wiki.ros.org/dmp .

Uses https://github.com/awesomebytes/razer_hydra for using with Razer Hydra.

Uses REEM as robot example, but should be general enough to use with any robot using ROS.
Follow this tutorial to setup REEM http://wiki.ros.org/Robots/REEM/Tutorials/Launching%20a%20REEM%20Gazebo%20simulation 

TODO: 
Uses ---- for leap motion.

Uses ---- for Kinect.

=====

This package is divided conceptually in:

* dmp_training: 
  * Learn from end effectors (Razer Hydra, Kinect... 3D/6D poses).
  * Learn from joints (joint states of a robot).
  * Management interface for gestures (save learnt gestures, visualize them)

* dmp_generation:
  * Generate from end effectors pose (generate from 3D/6D pose to another 3D/6D pose motions).
  * Generate from joint state (generate from one joint state (current, probably) to another joint state).
* dmp_execution:
  * Trajectory executor (via MoveIt!).

 TODO:
  * Clean up previous code into this package
  * Downsample points in recorded trajectories
  * Research multiple group goal sending in MoveIt!
  * Learn from joint states DMPs
  * Execute from joint state
  * Management interface + Rviz state viewer
  * Demos: Handshake from Razer Hydra (maybe Leap motion also) arm+torso(+hand), Wave from previously known robot movement torso+arm, moving a box from Kinect captured data with both arms

  Low priority...
  * Recompute motion if final goal is changing
  * Recompute motion if there is an obstacle

Must be done but lazy:
* Update CMakeLists.txt and package.xml with dependences.