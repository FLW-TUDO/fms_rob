# FMS ROB
An interface API between the fleet management system and the ROBOTNIK rb1_base robots.  

## Architecture
```mermaid
graph LR;
  broker>MQTT Broker]-->router;
  router(Command Router)-->pick(Pick Client);
  pick-->move(Move Base Server);
  router-->place(Place Client);
  router-->dock_cl(Dock Undock Client);
  place-->move;
  drive(Drive Client)-->move;
  dock_se(Dock Undock Server)-->dock_cl;
  pick-->dock_po(Dock Pose Server);
  place-->park(Park Pose Server);
  [//]: # pick-->dynam(dynamic reconf server);
  [//]: # dynam-->dock_se;
  subgraph Clients
  pick
  place
  drive
  dock_cl
  end
  subgraph ROS Service Servers
  dock_po
  park
  end
  subgraph ROS Action Servers
  dock_se
  move
  end

```

## Usage
To interface with the API, the user sends messages via MQTT in JSON format.

MQTT Topics used:
```
/robotnik/mqtt_ros_command  
/robotnik/mqtt_ros_info
```

MQTT Settings:
```
Broker: gopher.phynetlab.com  
Port: 8888
```

Message structure:
```
```

Sample Message:
```
{
"robot_id": "rb1_base_b",
"command_id": "",
"pose": {
"translation": {
"x": 0,
"y": 0,
"z": 0
},
"rotation": {
"x": 0,
"y": 0,
"z": 0,
"w": 1
}},
"action": "pick",
"cart_id": "KLT_6_neu",
"station_id": "AS_5_neu",
"bound_mode": "inbound",
"cancellation_stamp": 0,
"follow_id": ""
}
```

To interface **directly** with the action clients, the following topics can be used:
```
/+ROBOT_ID+/rob_action
/+ROBOT_ID+/rob_action_status
```
where robot id is replaced by *rb1_base_a*, *rb1_base_b*, etc  

To request actions, the *RobActionSelect* custom message is to be used. The message structure is as follows:
```
Header header  
geometry_msgs/Pose goal  
string command_id  
string cart_id  
string station_id  
string bound_mode  
string action  
time cancellation_stamp  
```
To receive status info, the *RobActionStatus* custom message is to be used. The message structure is as follows:
```
Header header  
string command_id  
string cart_id  
string station_id  
string bound_mode  
string action  
uint8 status  
```


Possible actions:

* **pick**: Navigates the robot to a calculated location infront of the requested cart
* **dock**: Performs the actual docking operation with the cart, which consists of:
  
* **undock**: Performs the undocking operation which consists of:
* **place**: Places the cart near one of the stations in one of 3 locations (*bound mode*): *inbound*, *outbound*, or *queue*
* **drive**: Navigates the robot to a pose spectified by the user
* **cancel**:
* **cancelAll**:
* **cancelAtAndBefore**:

## Bash Commands
```
killA, killB,..., killAll
```

## Roadmap
* Adding interlocks for failsafe operation
* Adding *home* & *return* actions
* Testing preemtive requests

## Future Work
* Implement a SMACH state machine architecture to replace / contain current architecture
* Use *move_base_flex (mbf)* action server for robots' navigation

