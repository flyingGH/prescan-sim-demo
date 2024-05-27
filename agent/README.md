# F1TENTH Agent

Welcome to the agent submodule of the PreScan F1Tenth project.

## Project Structure

The agent part is organized into two main python submodules: *agent* and *bridge*. The first contains pure agency logic (Python), while the latter manages the ROS communication. This coding structure effectively decouples the agent logic from ROS, leading to efficiency and reusability. Agents are plug and play, meaning that each agent can be swapped with another one without many coding modifications, by using a different agent name during launch. Each module is organized with a base abstract class and implementable children classes to allow for further expansion.

![Information pipeline of this project](/docs/images/system-components-real.png)

During runtime, the main nodes running are:
- `agent_manager:` the main node that is responsible for executing the agent logic within the ROS environment. It contains the agent as well as the bridge interface.
- `safety_filter:` the priority node that is responsible for overriding the normal action pipeline with an emergency action (lowering the speed) whenever there are upcoming pedestrians and traffic lights.
- `vrpn_client_node:` a node which captures ROS messages streamed by the VICON tracker software.
- `vicon_to_odom:` a node that converts VICON messages into odometry data, required by the agent to estimate the next action.
- `cmd_vel_to_vesc: ` a node that converts velocity and steering commands into a format suitable for the Vedder's Electronic Speed Controller (VESC).
- `visualizer:` a node that visualizes positional data, traffic light and pedestrian states, as well as the path and the waypoints received from Prescan. The data is streamed in the form of Marker objects through ROS messages.