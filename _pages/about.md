---
permalink: /
title: "Portfolio"
excerpt: "Rohit Bhikule's Professional Portfolio - Software Engineer specializing in Robotics"
author_profile: true
---
About Me
======
<div class="experience-block">
Hello there! I am a Robotics Software engineer at Terran Robotics. I graduated from University of Pennsylvania majoring in Robotics.
I have had the opportunity to be a part of multiple projects affiliated to <a href="https://www.grasp.upenn.edu/">GRASP</a> robotics lab at UPenn. 
My projects have exposed me to various topics related to motion-planning, perception, computer vision and controls.
In my leisure time, I like to go on hikes and play soccer. I am also a big motorsports racing fan.
</div>

Experience
======
<div class="experience-block">

  <div class="hyperlink">
    <span class="highlight_role">Senior Robotics Software Engineer</span> | 🔗 <a href="https://terranrobotics.ai/">Terran Robotics</a> <span class="location">Bloomington, IN</span>
  </div>

  <div class="experience-content">
    <ul>
      <li>Developed a custom <span class="highlight_important">GTSAM-based localization solution</span> to estimate cable-driven parallel robots pose using AprilTags and encoder data, and implemented a parallel processing pipeline with distributed computing framework python ray in ROS to achieve <span class="highlight_important">10 Hz pose output with 5 mm accuracy</span>.</li>
      <li>Designed and implemented adaptive algorithm to autonomously perform pick-and-place operations on earthern-clay blobs and hammer them into walls using pointcloud data, utilized a neural network to predict the depthmap based on the hammer hits and optimize hits accordingly, resulted in <span class="highlight_important">30% faster speed, 2x more accuracy</span>.</li>
      <li>Scripted python deployment scripts for maintaining and managing services across kubernetes cluster for robot operations.</li>
      <li>Scripted pipeline to convert stl CAD files of house walls to 2-dimensional goal depth maps and plan operations.</li>
    </ul>
  </div>

  <div class="hyperlink">
    <span class="highlight_role">Perception Intern</span> | 🔗 <a href="https://skymul.com/">Skymul</a> <span class="location">Atlanta, GA</span>
  </div>

  <div class="experience-content">
    <ul>
      <li>Built a novel algorithm to detect rebar intersections and pose from noisy pointclouds real-time (<span class="highlight_important">6 FPS</span>) in a densely multilayered rebar network on quadraped robot.</li>
    </ul>
  </div>


  <div class="hyperlink">
    <span class="highlight_role">Research Assistant</span> | 🔗 <a href="https://xlab.upenn.edu/">xLab</a> <span class="location">Philadelphia, PA</span>
  </div>

  <div class="experience-content">
    <ul>
      <li>Achieved <span class="highlight_important">1st place</span> in the <span class="highlight_important">12th F1Tenth Autonomous Grand Prix</span> at the <span class="highlight_important">CPS-IoT 2023 conference</span>.</li>
      <li>Implemented a finite state machine to switch between overtaking, adaptive cruise control and pure pursuit modes on an F1Tenth car.</li>
    </ul>
  </div>
</div>

<style>
  body {
    background-color: #f5f5f5;
  }

  .experience-block {
      margin: 20px 0;
      padding: 15px;
      border-radius: 12px;
      background: #ffffff;
      box-shadow: 0 -2px 4px rgba(0,0,0,0.02),
                  -2px 0 4px rgba(0,0,0,0.1), 
                  2px 0 4px rgba(0,0,0,0.1),
                  0 2px 4px rgba(0,0,0,0.2);
  }

  .experience-content {
      margin-left: 20px;
      padding: 10px 0;
  }

  .highlight {
      background: #e3f2fd;
      color: #1565c0;
      padding: 2px 6px;
      border-radius: 4px;
      font-weight: bold;
  }

  .highlight_important {
      color: #000000;
      font-weight: bold;
  }

  .highlight_role {
      color: #1565c0;
      font-weight: bold;
      font-size: 1.2em;
  }

  .hyperlink {
      position: relative;
  }

  .hyperlink a {
      color: #1565c0;
      text-decoration: none;
  }

  .location {
      color: #666;
      font-style: italic;
      font-size: 0.9em;
  }

  h2 {
      border-bottom: 2px solid #2962ff;
      padding-bottom: 5px;
      margin-bottom: 15px;
  }
</style>


Projects
======

Object detection and instance segmentation
------
Implemented YOLO, SOLO, and Faster-RCNN pipelines for object detection and instance segmentation tasks from scratch. Performed post-processing and analysed performence using mAP metric. [GitHub](https://github.com/divyanshurs/object_detection_and_segmentation)

<img src="images/ins1.png?raw=true" width="300" height="300"> <img src="images/ins2.png?raw=true" width="300" height="300">

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
