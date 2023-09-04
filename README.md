<!-- PROJECT SHIELDS -->

<!--

*** I'm using markdown "reference style" links for readability.

*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).

*** See the bottom of this document for the declaration of the reference variables

*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.

*** https://www.markdownguide.org/basic-syntax/#reference-style-links

-->

[![Contributors][contributors-shield]][contributors-url]  [![Forks][forks-shield]][forks-url] [![Stargazers][stars-shield]][stars-url] [![Issues][issues-shield]][issues-url] [![LinkedIn][linkedin-shield]][linkedin-url]

  
<!-- PROJECT LOGO -->

<br  />

<div  align="center">

<a  href="https://github.com/krrish-jindal/AMR_FSM/blob/main/assets/AMR-No_bg_2.png?raw=true">

<img  src="https://github.com/krrish-jindal/AMR_FSM/blob/main/assets/AMR-No_bg_2.png?raw=false"  alt="Logo"  width="430"  height="220">
</a>

  

<h2  align="center">AMR_FSM</h2>

  

<p  align="center">

This is the repo for the <a  href="https://github.com/krrish-jindal/AMR_FSM">AMR</a> Project, The project's objective is to develop an autonomous mobile robot (AMR) capable of seamless navigation in a given environment, while also allowing manual control through teleoperation.


<br  />

<a  href="https://github.com/krrish-jindal/AMR_FSM/blob/main/assets"><strong>Demo video »</strong></a>

<img  src="https://github.com/krrish-jindal/AMR_FSM/blob/main/assets/AMR_obs_n_pcl.gif?raw=true"  alt="Logo"  width="640"  height="320">


<br  />

<br  />


</p>

</div>
 

<p  align="right">(<a  href="#readme-top">back to top</a>)</p>

## Exploded View

   This is visual representation of how the components of an assembly or design are positioned and arranged relative to each other in URDF.


<img  src="https://github.com/krrish-jindal/AMR_FSM/blob/main/assets/explode_2.gif?raw=true"  alt="Logo"  width="640"  height="400">


<br  />


## Built With

  

* [![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)](https://www.sphinx-docs.org)

* [![OpenCV](https://img.shields.io/badge/opencv-%23white.svg?style=for-the-badge&logo=opencv&logoColor=white)](https://opencv.org/)

* [![Raspberry Pi](https://img.shields.io/badge/-RaspberryPi-C51A4A?style=for-the-badge&logo=Raspberry-Pi)](https://www.raspberrypi.org/)

* [![Espressif](https://img.shields.io/badge/espressif-E7352C?style=for-the-badge&logo=espressif&logoColor=white)](https://www.espressif.com/)

* [![Arduino](https://img.shields.io/badge/Arduino-00979D?style=for-the-badge&logo=Arduino&logoColor=white)](https://www.arduino.cc/)

* [![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)](https://ubuntu.com/)

* [![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/)

  

<p  align="right">(<a  href="#readme-top">back to top</a>)</p>

  


<!-- GETTING STARTED -->
## Getting Started

This is an example of how you may give instructions on setting up your project locally.
To get a local copy up and running follow these simple example steps.

### Prerequisites

This is an example of how to list things you need to use the software and how to install them.
* Ros
  - Refer to our [Ros installation guide](https://atom-robotics-lab.github.io/wiki/markdown/ros/installation.html)
  - Installing Navigation specific dependencies: map-server, move_base and amcl
    ```sh
    sudo apt install ros-noetic-navigation
    ```
* Serial Setup
	* For serial setup to run motor driver using USB refer to this:
	```sh
	https://github.com/wjwwood/serial.git
	```

### Installation

1. Clone the repo inside your `Ros Workspace`
   ```sh
   git clone git@github.com:krrish-jindal/AMR_FSM.git
   ```
2. Build the package
   ```sh
   cd ~/catkin_ws
   catkin_make
   ```
3. Launch the packages file by
   ```sh
   roslaunch <package_name> <launch_file>
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- USAGE EXAMPLES -->
## Usage


- The `cad_assembly_amr_final_1` dir contains all the bot model description files and main launch file.
- The `navigation` dir contains all the config files for enabling navigation and planning.

In order to run the simulation you are required to do the following:</br>

* For running the `All Necessary Nodes `
    ```sh
    roslaunch cad_assembly_amr_final_1 display.launch
    ```

 - For launch the `navigation file.`
    ```sh
    roslaunch navigation navigation.launch
    ```
    **_NOTE:_** Make sure that you have installed the navigation dependencies before running the navigation launch file.<br />

  - Setting the goal
    In the Rviz add a 2d on the map.<br /> <br />

  Voila! The bot will start moving towards the goal now.<br /><br />


<p align="right">(<a href="#readme-top">back to top</a>)</p>




# DFD

  

<img  src = "https://github.com/krrish-jindal/AMR_FSM/blob/main/assets/FSM_AMR_DFD_Th-2.png?raw=true"  alt="Logo"  width="700"  height="500">
  

# Circuit Diagram

  

<img  src = "https://github.com/krrish-jindal/AMR_FSM/blob/main/assets/circuit_dig.jpg?raw=true"  alt="Logo"  width="700"  height="600"  >
 

<p  align="right">(<a  href="#readme-top">back to top</a>)</p>

  
  
  

<!-- ACKNOWLEDGMENTS-->

## Acknowledgement

  

* [ATOM wiki](https://krrish-jindal/AMR_FSM-lab.github.io/wiki)

* [ROS Official Documentation](http://wiki.ros.org/Documentation)

* [Opencv Official Documentation](https://docs.opencv.org/4.x/)

* [Rviz Documentation](http://wiki.ros.org/rviz)

* [Gazebo Tutorials](https://classic.gazebosim.org/tutorials)

* [Ubuntu Installation guide](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)




  

<p  align="right">(<a  href="#readme-top">back to top</a>)</p>

  
  
  

<!-- MARKDOWN LINKS & IMAGES -->

<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->

[contributors-shield]: https://img.shields.io/github/contributors/krrish-jindal/AMR_FSM.svg?style=for-the-badge

[contributors-url]: https://github.com/krrish-jindal/AMR_FSM/graphs/contributors

[forks-shield]: https://img.shields.io/github/forks/krrish-jindal/AMR_FSM.svg?style=for-the-badge

[forks-url]: https://github.com/krrish-jindal/AMR_FSM/network/members

[stars-shield]: https://img.shields.io/github/stars/krrish-jindal/AMR_FSM.svg?style=for-the-badge

[stars-url]: https://github.com/krrish-jindal/AMR_FSM/stargazers

[issues-shield]: https://img.shields.io/github/issues/krrish-jindal/AMR_FSM.svg?style=for-the-badge

[issues-url]: https://github.com/krrish-jindal/AMR_FSM/issues

[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555

[linkedin-url]: https://www.linkedin.com/in/krrish-jindal-815716212