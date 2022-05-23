# APF-Based Collision Avoidance Application

## What can I expect from this repository?
The APF-Based Collision Avoidance Application consist on the main repository of Diego Rodríguez's Ph.D. (diego.rodriguez@ikerlan.es - link to the document or site when finished). This application has been developed for the `ros_melodic` distribution and it the RT control is base on the `orocos_package`.

The current repo is divided into three sub-repos listed below:
* **ca_application (link)**: this repo contains the developed `ros_packages` for the main application functionalities. Among this funcitonalities, you can found the `scene_segementation`, `ca_apf_gui`, and other `ros_packages`.
* **ca_control (link)**: this repo contains the developed RT controllers required to run the robot in both scenarios: the real robot and simulations.
* **ca_robot_setups (link)**: this repo contains the main configurations and launch files to run the robot in simulations and in the laboratory. It is an auxiliar package that requires the `ur10e_hw_integration` (link) to run.

## Dependencies
To run the **APF-Based Collision Avoidance Application** it is required the following `ros_packages` with their corresponding development branches. The list below highlights the required external dependencies and versions requierd:
* **Dependency 1** (version + link).
* **Dependency 2** (version + link).

## Installation
The following installation process is required to succesfully run the **APF-Based Collision Avoidance Application**. Open a new terminal (`ctrl`+`alt`+`t`) and the copy and run the following commands:
```
cd ros_wss
mkdir -p ws_ca_apf/src && cd ws_ca_apf #or whatever name you want for your work space
catkin init
cd src
git clone -b <branch> https://git.url
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin build
```

Once it is succesfully compiled, source the solution:
```
source ~/ros_wss/ws_ca_apf/devel/setup.bash
```

In case this sourcing is required to last permanently on your computer, don't forget to echo it into the compiling environment variables:
```
echo "source ~/ros_wss/ws_ca_apf/devel/setup.bash" >> ~/.bashrc
```

## Official releases
There is still no official original release for the actual packages distribuition that contains the APF-based controller combined with a suitable singularity-free control strategy.
* **Release TAG Name**: 
  * Description: 
  * Developers list:
  * Mantainers list:
  * Package list:
    * *Package 1*: 
    * *Package 2*:
    * *...*:
    * *Package n*:


## Mantainers
The repo is currently mantained by:
* **Diego Rodríguez (KAU)**: diego.rodriguez@ikerlan.es
* **Ander González (KAU)**: ander.gonzalez@ikerlan.es
* **Josu Sinovas (KAU)**: jsinovas@ikerlan.es
