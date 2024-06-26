<div align="center">
  <img src="https://github.com/parham/parham.github.io/blob/main/assets/img/favicon.png"/ width="200">
</div>

<!-- PROJECT LOGO -->
<br />
<p align="center">
  <a href="https://www.ulaval.ca/en/" target="_blank">
    <img src="https://ssc.ca/sites/default/files/logo-ulaval-reseaux-sociaux.jpg" alt="Logo" width="280" height="100">
  </a>

  <h3 align="center">LeManchot-DC</h3>

  <p align="center">
	Drone-enabled Multi-modal system for inspection of industrial components
    <br/>
    <br/>
  </p>
</p>

## Citation

In case of any use or reference, please cite:

```
@article{nooralishahi2022drone,
  title={Drone-Enabled Multimodal Platform for Inspection of Industrial Components},
  author={Nooralishahi, Parham and L{\'o}pez, Fernando and Maldague, Xavier PV},
  journal={IEEE Access},
  volume={10},
  pages={41429--41443},
  year={2022},
  publisher={IEEE}
}
```

# LeManchot-DC
In the rise of recent advancements in unmanned aerial vehicles, many studies have focused on using multi-modal platforms for remote inspection of industrial and construction sites. The acquisition of multiple data modalities assists the inspectors in acquiring comprehensive information about the surrounding environment and the targeted components. Despite the benefits of multi-modal platforms, the calibration and fusion of the obtained data modalities present many challenges that need to be addressed. One of the main problems in such systems is the dissimilarity of feature appearances in different spectrums. One of the main approaches is to employ a calibration board with geometrically known features to estimate intrinsic and extrinsic parameters and accurately align the images in thermal and visible spectral bands. This study presents a comprehensive platform for drone-based multi-modal inspection of industrial and construction components, including three main components: 

-  a sensor setup that can be used as a standalone system or a payload for a drone; 
-  a multi-modal embedded system and 
-  a novel calibration board for multi-modal data fusion. 

The multi-modal embedded system provides the required features to record, transmit, and visualize the thermal, visible, and depth data synchronously. Additionally, the system presents a multi-modal fusion technique to form RGBD&T data containing thermal and texture information of the obtained 3D view. Moreover, this study introduces a novel self-heating calibration board that uses Thermoelectric Peltier modules to provide an identifiable and sharp pattern in thermal and visible images. The calibration board is designed with an aim also to be used as Ground Control Point (GCP) in drone surveys.

<p align="center">
  <img src="resources/setup_top.jpg" width="400" title="Setup">
  <img src="resources/setup_cad_model_top (1).png" width="400" title="SETUP's CAD Design">
  <img src="resources/setup_on_drone_2 (1).jpg" width="400" title="Setup installed on DJI M600 Pro drone">
</p>

## Videos

<p align="center">
	<a href="https://youtu.be/Hobju6ha29c"><img src="https://img.youtube.com/vi/Hobju6ha29c/sddefault.jpg" width="300"></img></a>
	<a href="https://youtu.be/J80_gY7oOSI"><img src="https://img.youtube.com/vi/J80_gY7oOSI/sddefault.jpg" width="300"></img></a>
	<a href="https://youtu.be/XMF93IpMkQE"><img src="https://img.youtube.com/vi/XMF93IpMkQE/sddefault.jpg" width="300"></img></a>
</p>

## Multi-Modal Platform for Inspection of Industrial Components
This study introduces a multi-modal acquisition and processing platform for inspecting industrial components using Unmanned Aerial Vehicles. The setup is designed to attach the sensors to the aerial platform and stabilize them against the drone's vibrations while operating. The platform includes thermal, visible, and stereo depth cameras and communicates directly with the flight controller to obtain the drone's inertial and GPS data. Also, embedded software is developed for calibration, acquisition, transmission, and fusion of multi-modal data. The data fusion method presented in this study generates RGBD&T data frames that contain the thermal, visible, and depth information of the observed scene and the drone's telemetry data. Later, the RGBD&T data is projected to the system coordinate to form a point cloud of the observed scene.

<p align="center">
  <img src="resources/system_abstract_design.png" width="700" title="System Design">
  <img src="resources/software_abstract_design.png" width="700" title="Software Abstract Design">
</p>

## Demo
<p align="center">
  <img src="resources/drone_software.jpg" width="500" title="Screenshot of the drone mode">
</p>

<a href="https://youtu.be/J80_gY7oOSI" target="_blank">
Video #1 - Standalone mode
</a>
<br/>
<a href="https://youtu.be/Hobju6ha29c" target="_blank">
Video #2 - Drone mode
</a>

## Requirement
* Docker
* ROS Melodic
* OpenCV
* rospy
* pillow
* imageio
* console-menu
* kivy
* kivymd
* imutils
* flask
* psutil

## How To Run
For running the system, two scripts are prepared to automate the execution:

```
git clone https://github.com/parham/lemanchot-dc
cd lemanchot-dc
chmod u+x lemanchot_run.sh
chmod u+x run_docker.sh
./lemanchot_run.sh -a
./run_docker.sh
```

## Team
**Parham Nooralishahi** is a specialist in embedded and intelligent vision systems and currently is a Ph.D. student at Universite Laval working on developing drone-enabled techniques for the inspection of large and complex industrial components using multi-modal data processing. He is a researcher with a demonstrated history of working in the telecommunication industry and industrial inspection and in-depth expertise in robotics & drones, embedded systems, advanced computer vision and machine learning techniques. He has a Master’s degree in Computer Science (Artificial Intelligence). During his bachelor's degree, he was involved in designing and developing the controlling and monitoring systems for fixed-wing drone for search and rescue purposes. Also, during his Master's degree, he worked extensively on machine learning and computer vision techniques for robotic and soft computing applications.

**Xavier P.V. Maldague** received the B.Sc., M.Sc., and Ph.D. degrees in electrical engineering from Universite Laval, Quebec City, Canada, in 1982, 1984, and 1989, respectively. He has been a Full Professor with the Department of Electrical and Computing Engineering, Universite Laval, Quebec City, Canada, since 1989, where he was the Head of the Department from 2003 to 2008 and 2018. He has trained over 50 graduate students (M.Sc. and Ph.D.) and has more than 300 publications. His research interests include infrared thermography, nondestructive evaluation (NDE) techniques, and vision/digital systems for industrial inspection. He is an Honorary Fellow of the Indian Society of Nondestructive Testing. He is also a Fellow of the Canadian Engineering Institute, the American Society of Nondestructive Testing, and the Alexander von Humbolt Foundation, Germany. He holds the Tier 1 Canada Research Chair in Infrared Vision. He has been the Chair of the Quantitative Infrared Thermography (QIRT) Council since 2004.

## Contact
Parham Nooralishahi - parham.nooralishahi@gmail.com | [@phm](https://www.linkedin.com/in/parham-nooralishahi/) <br/>

## Acknowledgements
This research is supported by the Canada Research Chair in Multi-polar Infrared Vision (MiViM), the Natural Sciences, and Engineering Research Council (NSERC) of Canada through a discovery grant and by the "oN Duty!" NSERC Collaborative Research and Training Experience (CREATE) Program. Special thanks to TORNGATS company for their support in testing and manufacturing of the parts.

