---
permalink: /
title: "About me"
excerpt: "About me"
author_profile: true
redirect_from: 
  - /about/
  - /about.html
---


Hello there! I am a Robotics Software engineer at Terran Robotics, Bloomington, Indiana. I graduated from University of Pennsylvania majoring in Robotics.
I have had the opportunity to be a part of multiple projects affiliated to [GRASP](https://www.grasp.upenn.edu/) robotics lab at UPenn. 
My projects have exposed me to various topics related to motion-planning, perception, computer vision and controls.
Being a Robotics engineer, I am a hands-on individual who likes working on practical projects, and at the same time, I also enjoy the challenge of coding and programming. 
This combination of skills allows me to approach projects from both a theoretical and practical perspective, making me a well-rounded individual in the field of Robotics and software development.
In my leisure time, I like to go on hikes and play soccer. I am also a big motorsports racing fan.


Research Experience
======

mLAB- Autonomous Go-Kart Group | GRASP-UPenn
------

Latest Update:- UPenn are the WINNERS of the Autonomous go-kart challenge hosted at Purdue University!!!

<img src="images/win2.png?raw=true" width="300" height="300"> <img src="images/win_self.png?raw=true" width="300" height="300">

Throughout my active participation in this competition for two semesters, I acquired valuable experience in the implementation of both reactive and pre-mapping based controls. This practical engagement enabled me to apply the theoretical concepts I learned during my studies at UPenn, effectively reshaping and putting them into practice. Furthermore, my proficiency in developing software pipelines and leveraging the capabilities of the Robot Operating System (ROS) proved to be advantageous in this context.

The competition itself was divided into two distinct parts. In the first phase, our objective was to navigate the track without relying on a pre-mapped path, instead adopting a reactive-based approach to control the autonomous go-kart. Our goal was to react to the observed track in real-time, ensuring safe operation throughout five laps to secure victory. To achieve this, we decided to leverage the power of image-based grass detection using a single monocular camera. By transforming the camera's image into a pre-calibrated bird's eye view and adjusting depth measurements on a per-pixel basis, we were able to obtain an extremely reliable lane detection capability. These detections were then converted into a Laser Scan format, which served as input for our gap-follow algorithm. Our approach demonstrated solid performance, especially under favorable lighting conditions. You can watch the back-end visualization of our algorithm and the actual run below.


<iframe width="300" height="150" src="https://www.youtube.com/embed/YTyGHn2WP5s" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe><iframe width="300" height="150" src="https://www.youtube.com/embed/xCRqczp_Acs" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

In the second part of the competition, our team was tasked with implementing pre-mapping based controls.To secure a victory in this coategory, the requirement was to complete a total of five laps within the shortest possible time. My specific responsibility involved the development of a lightweight Python library capable of fusing data from multiple sensors, such as GPS, IMU, and Wheel Odometry, using a partial state update. You may be wondering why we opted to create a separate library instead of utilizing existing open-source options. It's a valid question, and initially, we started the project by leveraging the well-known Robot Localization library in ROS. However, upon careful consideration, we determined that using this library would be excessive for our specific requirements. Taking inspiration from its approach, I devised a partial update scheme for the Extended Kalman Filter (EKF) algorithm, which proved to be highly effective for our needs.

To integrate this localization approach with the pure-pursuit algorithm, we established pre-mapping pipelines. Initially, we manually drove the go-kart, collecting waypoints along the track using our sensor fusion system. These collected waypoints served as the foundation for creating a path that the pure-pursuit algorithm would follow. We employed a simple linear interpolation scheme to generate a smooth and accurate trajectory for the pure-pursuit algorithm to track. I invite you to watch our winning pure-pursuit run in the competition below.

<iframe width="300" height="150" src="https://www.youtube.com/embed/CzRc5b1CP3M" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

The following is the final result of EKF fusion using GPS and IMU. The pipeline consists of first filtering out GPS jumps and then using an EKF to fuse GPS positions with IMU measurements. The results below show the filtered results on the upper left hand side, the fused covariance on the lower left hand side. The raw data of the GPS and its covariance are shown on the top and bottom right respectively. 

<img src="images/fusion_latest.png?raw=true" width="600" height="600">

<!-- This data was collected on a moving go-kart controlled via-joystick which can be seen in the short gif below.

<img src="images/shortav.gif?raw=true" width="300" height="500"> -->

As part of exploring alternate approaches for future competitions, I am currently working on fusing cone detections from both LIDAR and camera sensors. This ongoing project involves various stages, including calibration and the development of separate cone detection pipelines. We have also implemented an overlay of camera detections using YOLO-v7 to filter the point cloud data. Our ultimate objective is to utilize this robust and fused pipeline to perform Simultaneous Localization and Mapping (SLAM) using cones as landmarks. I invite you to have a look at the current results below.

[<img src="images/f1.gif?raw=true" width="300" height="300">](https://youtu.be/AknzieI0od0)
[<img src="images/f2.gif?raw=true" width="300" height="300">](https://youtu.be/5mYCOwyy4mo)

Professional Experience
======

Terran Robotics (Bloomington, IN) - **Senior Robotics Software Engineer**
------



Skymul (Atlanta) - **Perception Intern**
------

My role at Skymul had been to integrate hardware components and create an end-to-end software pipeline for performing waypoint navigation on a mobile platform. During the period of my internship, I've had the opportunity to interact with hardware and learn how to use software drivers to integrate them into a complete system. I worked mainly on integrating three different hardware platforms with the robot. The aim was to test the accuracy of these sensor platforms and decide the best for the robot’s task at hand. 

Projects
======

Object detection and instance segmentation
------
Implemented YOLO, SOLO, and Faster-RCNN pipelines for object detection and instance segmentation tasks from scratch. Performed post-processing and analysed performence using mAP metric. [GitHub](https://github.com/divyanshurs/object_detection_and_segmentation)

<img src="images/ins1.png?raw=true" width="300" height="300"> <img src="images/ins2.png?raw=true" width="300" height="300">

<!-- Depth estimation using Optical Flow
------
In this project, I used a sequence of image to calculate optical flow and used different confidence threshold to remove bad flow values. Moreover, I calculated the epipole considering pure translational motion and used it to calcualte the depth estimates accordingly. [GitHub](https://github.com/divyanshurs/depth_map_optical_flow.git)

<img src="images/thres10.png?raw=true" width="300" height="300"> <img src="images/thres30.png?raw=true" width="300" height="300"><img src="images/dep10.png?raw=true" width="300" height="300"> <img src="images/dep30.png?raw=true" width="300" height="300">  -->

<!-- Trajectory optimization for cube manipulation
------
The objective of this project, completed as part of the MEAM 517 course, was to design control systems for the vertices of a polytope (a cube in this case) to enable precise movement to a desired position and orientation. The main challenges encountered were managing surface friction and the number of contacts the cube had at any given moment.

We implemented two methods namely the offline iLQR and online MPC approach to achieve this task. The results of which are given below. 

<img src="images/square.gif?raw=true" width="300" height="300"> <img src="images/traj_opt_min.gif?raw=true" width="300" height="300"> -->

Path planning approaches for a planar quadrotor
------
This project was a semester long implementation of several methods implemented for planar quadrotor control as a part of the MEAM 517 (Control and Optimization with learning in Robotics). These methods include MPC, iLQR, LQR to follow a nominal trajectory, and minimum snap trajctory planning in differtially flat space of the quadrotor. The results for the same are as below. [GitHub](https://github.com/divyanshurs/path_planning_planar_quadrotor)

<img src="images/MPC.gif?raw=true" width="300" height="300"><img src="images/ilqr.png?raw=true" width="300" height="300"> <img src="images/traj_track.gif?raw=true" width="300" height="300"><img src="images/min_snap1.png?raw=true" width="300" height="300">

Two-View and Multi-View Stereo for 3D reconstruction 
------
The aim of this project was to use two view and multiple view images to form a 3D reconstruction of the object of interest. For multi-view stereo the plane sweep algorithm was implemented. [GitHub](https://github.com/divyanshurs/two-view_and_multi-view_stereo)

<img src="images/t1.png?raw=true" width="300" height="300"><img src="images/result.gif?raw=true" width="300" height="300">

SLAM using Particle Filter for humanoid Robot
------
The aim of this project was to perform particle filter based SLAM using the IMU and the LIDAR data from a THOR-OP Humanoid Robot. The IMU data avaialble was filtered and used with lidar data to perform SLAM. The lidar data is transformed into the map co-ordinates by applying suitable transformations. Based on the paricle filter approach the best particle with maximum correlation is chosen and the log odds of the map is updated. This scan-matching technique is used to update the obstacles in real-time on a gridmap as well as localize the robot in the world. [GitHub](https://github.com/divyanshurs/particle_filter_SLAM_humanoid_robot)

<img src="images/s1.png?raw=true" width="300" height="300"><img src="images/final0.png?raw=true" width="300" height="300"><img src="images/final1.png?raw=true" width="300" height="300"><img src="images/final3.png?raw=true" width="300" height="300">

Autonomous Pick and Place Challenge
------
The aim of this project was to use the library features developed for the Franka Panda Arm during the semester to develop a strategy for picking up static and dynamic blocks and placing them on the reward table. The aim of the strategy was to maximize the score and minimize the time. The Github repository for the entire project can be found [here](https://github.com/divyanshurs/Autonomous-Pick-and-Place-Challenge).

<img src="images/ppr.gif?raw=true" width="300" height="300"> <img src="images/pps.gif?raw=true" width="300" height="300">
