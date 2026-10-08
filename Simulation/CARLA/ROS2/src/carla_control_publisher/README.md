# **CarlaControlPublisher Node**

## **Overview**
The `CarlaControlPublisher` node is a ROS 2 interface for merging steering and longitudinal (throttle & brake) control commands to the **CARLA Simulator**.  
It subscribes to steering and throttle command topics and publishes corresponding control messages to the CARLA ego vehicle.  


## **Published Topics**

| Topic | Message Type | Description |
|--------|---------------|-------------|
| `/carla/hero/vehicle_control_cmd` | `carla_msgs/msg/CarlaEgoVehicleControl` | Publishes combined throttle, steering, and braking commands to control the ego vehicle in CARLA. |



## **Subscribed Topics**

| Topic | Message Type | Description |
|--------|---------------|-------------|
| `/vehicle/steering_cmd` | `std_msgs/msg/Float32` | Receives desired tire steering angle in radians |
| `/vehicle/throttle_cmd` | `std_msgs/msg/Float32` | Receives normalized throttle command input (-1.0 to 1.0). Negative is to reduce speed by braking|


## **Parameters**

| Name | Type | Default | Description |
|------|------|----------|-------------|
| `publish_rate` | float | 10.0 Hz | Frequency at which control messages are published to CARLA. |


## **Example Usage**

### **Run the Node**
```bash
ros2 run carla_control_publisher pub_carla_control 