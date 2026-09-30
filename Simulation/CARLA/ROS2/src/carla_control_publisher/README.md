# **CarlaControlPublisher Node**

## **Overview**
The `CarlaControlPublisher` node is a ROS 2 interface for merging steering and longitudinal (throttle & brake) control commands to the **CARLA Simulator**.  
It subscribes to steering and throttle command topics and publishes corresponding control messages to the CARLA ego vehicle.  


## **Published Topics**

| Topic | Message Type | Description |
|--------|---------------|-------------|
| `/carla/hero/ackermann_control_cmd` | `ackermann_msgs/msg/AckermannDriveStamped` | Publishes the steering angle, the target speed and the acceleration to the CARLA ego vehicle. |



## **Subscribed Topics**

| Topic | Message Type | Description |
|--------|---------------|-------------|
| `/vehicle/steering_cmd` | `std_msgs/msg/Float64` | Receives desired tire steering angle in radians |
| `/vehicle/throttle_cmd` | `std_msgs/msg/Float64` | Receives the desired longitudinal acceleration in m/s^2. Negative is to reduce speed by braking |
| `/vehicle/speed` | `std_msgs/msg/Float64` | Receives the current ego speed in m/s. The node adds the acceleration to it to get the target speed. |


## **Parameters**

The node declares no parameters.


## **Example Usage**

### **Run the Node**
```bash
ros2 run carla_control_publisher carla_control_publisher_node 