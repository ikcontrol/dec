# ca_apf_simulation
This `ros_package` contains the simulation nodes specifically designed to interact with `Gazebo` and othe simulation tools for the collision avoidance application.

## Nodes list
This `ros_package` contains the following ros nodes:
* **obstacle_spawner_node**: a `ros C++` node that contains a short program to read a `*.urdf.xacro` file containing the obstacle information. Once the file is read, the obstacle is published on a `Gazebo` opened environment to interact with its positioning.

## Launchers list
The following `*.launch` files might come in handy when developing simulated application:
* **spawn_obstacle.launch**: this launch files spawn a green ball that simulates a static person in the surrounding of the robot scenario.
