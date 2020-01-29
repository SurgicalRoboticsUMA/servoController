# POLOLU SERVO CONTROLLER

ROS node to control the [Micro Maestro 6-Channel USB Servo Controller](https://www.pololu.com/product/1350).

## Maestro Servo Controller Linux Software
Download the software from the [Pololu webpage](https://www.pololu.com/docs/0J40/3.b)

```bash
cd Downloads/
tar -xzvf maestro-linux-150116.tar.gz
```

Follow the README.txt instructions. Then, to run the program, go to the folder *maesro-linux*:
 
```bash
sudo ./MaestroControlCenter
```

If you get a “Permission denied” error when trying to connect to the programmer, you should make sure to copy the 99-pololu.rules file into /etc/udev/rules.d and then unplug the programmer and plug it back in again. The install script normally takes care of installing that file for you. [More info](https://www.pololu.com/docs/0J67/all).

## To build

```bash
git clone (enlace de descarga!!)
catkin build
```

## Device functions

* *controller.setTarget(position,servo)*: sends a position to the motor plugged in channel *servo* (from 0 to 5).
* *controller.getServoPosition(position,servo)*: gets the current position of the motor. 

## To launch

```bash
roslaunch servo servo.launch
```

### servo_node

This node handles two motors to control the opening and closing of a robotic grasper using the Phantom Omni. The maximum values of the motors (set previously with the Maestro Control Center software) are set in a yaml file. 

To launch this node uncomment the following in */servo/launch/servo.launch*/:
```bash
<rosparam file="$(find servo)/yaml/servo_params.yaml" command="load" ns="srcHals"/>
<node name="servo" pkg="servo" type="servo_node" output="screen"/>
```
And the following in */servo/CMakeLists.txt*:
```bash
add_executable(servo_node src/servo_node.cpp src/servoController.cpp)
```

### servo_node_fourMotors

This node handles four motors, and it was developed to control an EndoWrist instrument.

To launch this node uncomment the following in */servo/launch/servo.launch*/:
```bash
<rosparam file="$(find servo)/yaml/servo_params.yaml" command="load" ns="srcHals"/>
<node name="servo" pkg="servo" type="servo_node" output="screen"/>
```
And the following in */servo/CMakeLists.txt*:
```bash
add_executable(servo_node src/servo_node.cpp src/servoController.cpp)
```

