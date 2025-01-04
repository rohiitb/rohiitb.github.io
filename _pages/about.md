---
permalink: /
title: "Portfolio"
excerpt: "Rohit Bhikule's Professional Portfolio - Software Engineer specializing in Robotics"
author_profile: true
---

<div class="experience-block">
<strong>About Me</strong>
<div class="experience-block">
Hello there! I am a Robotics Software engineer at Terran Robotics. I graduated from University of Pennsylvania majoring in Robotics.
I have had the opportunity to be a part of multiple projects affiliated to <a href="https://www.grasp.upenn.edu/">GRASP</a> robotics lab at UPenn. 
My projects have exposed me to various topics related to motion-planning, perception, computer vision and controls.
In my leisure time, I like to go on hikes and play soccer. I am also a big motorsports racing fan.
</div>
</div>

Experience
======
<div class="experience-block">

  <div class="hyperlink">
    <span class="highlight_role">Senior Robotics Software Engineer</span> | 🔗 <a href="https://terranrobotics.ai/">Terran Robotics</a> <span class="location">Bloomington, IN</span> <br> <span class="timeframe">June 2023 - Present</span>
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
    <span class="highlight_role">Perception Intern</span> | 🔗 <a href="https://skymul.com/">Skymul</a> <span class="location">Atlanta, GA</span> <br> <span class="timeframe">May 2022 - Aug 2022</span>
  </div>

  <div class="experience-content">
    <ul>
      <li>Built a novel algorithm to <span class="highlight_important">detect rebar intersections</span> and pose from noisy pointclouds real-time (6 FPS) in a densely multilayered rebar network on quadraped robot.</li>
      <li>Developed end-to-end pipeline to get the pointcloud of construction sites using photogrammetry.</li>
    </ul>
  </div>


  <div class="hyperlink">
    <span class="highlight_role">Research Assistant</span> | 🔗 <a href="https://xlab.upenn.edu/">xLab</a> <span class="location">Philadelphia, PA</span> <br> <span class="timeframe">Jan 2023 - May 2023</span>
  </div>

  <div class="experience-content">
    <ul>
      <li>Achieved <span class="highlight_important">1st place</span> in the <span class="highlight_important">12th F1Tenth Autonomous Grand Prix</span> at the <span class="highlight_important">CPS-IoT 2023 conference</span>.</li>
      <li>Implemented a finite state machine to switch between overtaking, adaptive cruise control and pure pursuit modes on an F1Tenth car.</li>
    </ul>
      <div style="text-align: center;">
        <div style="margin-bottom: 20px;">
          <img src="images/team.jpeg" style="height: 250px; width: 200px; margin: 10px 30px;"> 
          <img src="images/car.jpeg" style="height: 250px; width: 300px; margin: 10px 30px;">
        </div>
        <div>
          <iframe width="600" height="400" src="https://www.youtube.com/embed/mOYDDL2_ZcI" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
        </div>
      </div>
  </div>
</div>

<style>
  body {
    background-color: #f5f5f5;
  }

  .experience-block {
      margin: 10px 0;
      padding: 10px;
      border-radius: 12px;
      background: #ffffff;
      box-shadow: 0 -2px 4px rgba(0,0,0,0.02),
                  -2px 0 4px rgba(0,0,0,0.1), 
                  2px 0 4px rgba(0,0,0,0.1),
                  0 2px 4px rgba(0,0,0,0.2);
  }

  .experience-content {
      margin-left: 10px;
      padding: 5px 0;
      margin-bottom: 10px;
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
      font-size: 1.1em;
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

  .timeframe {
      color: #666;
      font-style: italic;
      font-size: 0.8em;
      margin-left: 10px;
  }
</style>


Projects
======
<div class="experience-block">

  <div class="hyperlink">
    <span class="highlight_role">Object Detection and Instance Segmentation</span>
  </div>

  <div class="experience-content">
    <ul>
      <li>Implemented <span class="highlight_important">Mask-RCNN</span> object detection and instance segmentation pipeline from scratch on COCO dataset. 🔗<a href="https://github.com/rohiitb/MaskRCNN">GitHub</a></li>
      <li>Scripted <span class="highlight_important">YOLO-v1</span> object detection system from ground up. Achieved mAP of 0.43. 🔗<a href="https://github.com/rohiitb/YOLO">GitHub</a></li>
      <li>Implemented <span class="highlight_important">SOLO</span> instance segmentation model. 🔗<a href="https://github.com/rohiitb/SOLO_Instance_segmentation">GitHub</a></li>
    </ul>
    <div style="text-align: center;">
        <a href="https://github.com/rohiitb/YOLO">
          <img src="images/yolo.JPG" style="height: 250px; width: 250px; margin: 10px 30px;"> 
        </a>
        <a href="https://github.com/rohiitb/MaskRCNN">
          <img src="images/ins2.png" style="height: 250px; width: 250px; margin: 10px 30px;">
        </a>
    </div>
  </div>

  <br>

  <div class="hyperlink">
    <span class="highlight_role">3D Reconstruction</span>
  </div>

  <div class="experience-content">
    <ul>
      <li>Deployed a <span class="highlight_important">NeLF</span> model on an M1 chip using LensStudio and ONNX after applying knowledge distillation and model pruning, reducing the model size to 9MB (down from 125MB in MobileNeRF) and improving framerate by 10 FPS. 🔗<a href="https://github.com/rohiitb/mobileNeLF">GitHub</a></li>
      <li>Implemented <span class="highlight_important">tinyNeRF</span> pipeline in Pytorch to generate novel views of scene and pointcloud. 🔗<a href="https://github.com/rohiitb/NeRF">GitHub</a></li>
      <li>Implemented 3D reconstruction using <span class="highlight_important">two-view and multi-view stereo</span> with plane sweep algorithm. 🔗<a href="https://github.com/rohiitb/two-view_and_multi-view_stereo">GitHub</a></li>
    </ul>
    <div style="text-align: center;">
        <a href="https://github.com/rohiitb/mobileNeLF">
          <img src="images/nelf.gif" style="height: 250px; width: 250px; margin: 10px 30px;">
        </a>
        <a href="https://github.com/rohiitb/NeRF">
          <img src="images/nerf_render.gif" style="height: 250px; width: 250px; margin: 10px 30px;">
        </a>
        <a href="https://github.com/rohiitb/two-view_and_multi-view_stereo">
          <img src="images/recon_gif.gif" style="height: 250px; width: 250px; margin: 10px 30px;">
        </a>
    </div>
  </div>

  <br>


  <div class="hyperlink">
    <span class="highlight_role">Quadrotor Navigation and Control</span>
  </div>

  <div class="experience-content">
    <ul>
      <li>Implemented geometric non-linear controller for collision-free quadrotor navigation along optimal trajectories generated using A* path planning and trajectory smoothing.</li>
      <li>Implemented stereo MSCKF visual-inertial odometry for robust high-speed robot pose estimation. 🔗<a href="https://github.com/rohiitb/msckf_vio_python">GitHub</a></li>
      <li>Implemented Unscented Kalman filter for 3D orientation tracking using IMU sensor data, with model parameters learned from Vicon motion capture ground truth. 🔗<a href="https://github.com/rohiitb/Orientation_tracking_using_UKF">GitHub</a></li>
    </ul>
    <div style="text-align: center;">
        <img src="images/crazyflie.gif" style="height: 250px; width: 250px; margin: 10px 30px;"> 
        <img src="images/msckf_gif.gif" style="height: 250px; width: 250px; margin: 10px 30px;">
    </div>
  </div>

  <br>

  <div class="hyperlink">
    <span class="highlight_role">SLAM using Particle Filter for Humanoid Robot</span>
  </div>

  <div class="experience-content">
    <ul>
      <li>Implemented particle filter SLAM for a THOR-OP humanoid robot using IMU and LIDAR data. The system performs real-time obstacle mapping and robot localization through scan matching and map updates. 🔗<a href="https://github.com/rohiitb/MonteCarlo_localization_SLAM">GitHub</a></li>
    </ul>
    <div style="text-align: center;">
        <img src="images/map1.PNG" style="height: 250px; width: 250px; margin: 10px 30px;">
    </div>
  </div>

  <br>

  <div class="hyperlink">
    <span class="highlight_role">Autonomous Pick and Place Challenge</span>
  </div>

  <div class="experience-content">
    <ul>
      <li>Developed a library for controlling Franka Panda Arm to efficiently pick static and dynamic blocks using apriltags and place them on table. Project code available on 🔗<a href="https://github.com/rohiitb/meam520_labs">GitHub</a></li>
      <li></li>
    </ul>
    <div style="text-align: center;">
        <img src="images/ppr.gif" style="height: 250px; width: 250px; margin: 10px 30px;"> 
        <img src="images/pps.gif" style="height: 250px; width: 250px; margin: 10px 30px;">
    </div>
  </div>
  
  <br>

</div>
