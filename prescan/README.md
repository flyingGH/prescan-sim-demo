# Prescan & ROS Library Integration

## Introduction
This README provides the necessary instructions for running scripts that facilitate the creation of ROS messages processed by the Python code in the Autonomous Motion Control (AMC) lab at Eindhoven University of Technology. This document assumes a successful extraction and configuration of the `Custom_Prescan-ROS_Library` in Simulink.

## Pre-requisites
- Ensure that the `Custom_Prescan-ROS_Library` has been successfully extracted and configured within your Simulink environment.
- Familiarity with Prescan and Simulink is assumed.

## Instructions for Running Scripts
### UpdateModel.m Script
Before executing the `UpdateModel.m` script, any modifications made to the Prescan experiment (addition or deletion of dynamic elements such as pedestrians, vehicles, or traffic lights) require you to **click the BUILD button in Prescan**. Failure to do so will result in an error prompt, reminding you to build the experiment in Prescan first.

### AirSensorMaxDetectableObjects.m Script
This script determines the maximum number of detectable objects for the air sensor. It parses through the experiment's objects and agents, updating the detectable object count accordingly.

### Virtual_Physical_Switch.m Script
This script establishes the necessary blocks for receiving ROS position messages from the VICON tracking system and transforms these into a format compatible with Prescan. The script incorporates a switch mechanism to toggle between the virtual and physical car inputs, ensuring the simulated car follows the correct trajectory.

### Agent_Sensor.m Script
This script generates the required blocks to create ROS messages for detected objects, such as pedestrians or vehicles. It outputs the detected object's ID, velocity, range, and angular information in the ROS message.

### TrafficLight_ROSmsg.m Script
This script creates blocks essential for generating ROS messages related to traffic light statuses (red, green, or orange) and their positions. These messages are then utilized within the Python code.

## Additional Notes
- After running the scripts, **save the Simulink file** and **restart MATLAB**. This is necessary as the Prescan initialization variables are cleared from MATLAB's workspace during script execution.
- To restart MATLAB, use the 'Prescan Process Manager' and launch MATLAB using the dedicated button. This ensures that all Prescan initialization variables are restored in MATLAB's workspace.
- During script execution, MATLAB may display warning messages. These can be safely ignored as they result from clearing Prescan's initialization variables from the workspace. Such warnings reinforce the need to launch MATLAB through the Prescan Process Manager to reinitialize these variables.

## Additional Information
You are free to add as many virtual pedestrians and cars as needed within the simulation. However, there is a limitation of four physical cars in the lab.

## Support
If you encounter any issues or have further questions, please contact the lab supervisor or refer to the Prescan documentation for additional guidance.

---
*This README is part of the AMC lab's documentation on integrating Prescan experiments with ROS messages for advanced vehicle simulation and tracking.*
