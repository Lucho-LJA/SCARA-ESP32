# SCARA-ESP32
The project is about the code to Control one SCARA with ESP32 and ROS. It is the code of Board, destined to [ESP32 with 38 pins](https://uelectronics.com/producto/esp32-38-pines-esp-wroom-32/) and following the [ESP32 documentation](https://randomnerdtutorials.com/esp32-pinout-reference-gpios/).

## GENERAL DESCRIPTION
The code is used to cominicate with ROS using rosserial_python with ROS distro: Noetic. The project use plataformio framwork and VSCode. All libraries are open source and they have theirs respective licences. This is the first version and only there is one configuration with [Monster Motor Shield](https://content.instructables.com/pdfs/E6E/FQXR/ITUYX4YO/Monster-Motor-Shield-VNH2SP30.pdf) and electromacnetic actuator, however I would like expand to other configurations.

The Scara Robot have 3 degrees of freedom, 2 rotary and 1 linear. So the project use 3 motors with control through potenciometers. The actuator is a electroiman on/of and it have implemented a simple PID control. Also, is used a Monster Motor Shield to two motors, where we read the electric current in order to monitor the motors. 

## DEPENDECIES
- Platformio (IDE into VSCode recommended)
- VSCode (Used it to upload the programe)
- ESP32 38 pines
- Monster Motor Shield for 2 motors
- ROS noetic (Ubuntu 20.04)
- python3
- rosserial_python package (to communication with ROS enviroment)

## INSTRUCTIONS
- Clone this repository with: `git clone https://github.com/Lucho-LJA/SCARA-ESP32.git`
- Open Folder with VScode
- Open file [CONFIG.H](/include/CONFIG.h)
- Modify the next variables with your information:
    - `ROBOT_NAME :` Name of robot and the first word of your topic
    - `ROUTER_SSID :` Name of your NETWORK WIFI.
    - `ROUTER_PASWORD :` PASWORD of your WIFI in plain text
    - `IP_ESP32 :` Static IP of your ESP32 
    - `IP_GATEWAY :` IP of your ROUTER 
    - `IP_SUBNET :` SUBNET of your network 
    - `ROS_SERVER :` IP of you ROS_SERVER (Computer where run rosserial_python package)
    - `ROS_SERVER_PORT :` PORT of your SERVER. If you work with 2 or more robots, use a diferent Port to each one.
- Chose the robot and board type (Currently only exist one option)
- Define the variables with your owner information about pins, control, setup, etc untill "CONFIGURATION FOLLOWING YOUR SELECTION" section. A simple PID is implemented.
- Build project and verify any issue
- Connect your ESP32 using USB PORT
- Upload Code and verify if exist connection with your WIFI network using the monitor Serial
- Now your ESP32 is watting the conection with the server, so run the server following the instructions bellow.

## Instructions to install and run rosserial_python
- update your repositories with: `sudo apt update`
- Install package with: `sudo apt-get install ros-noetic-rosserial`
- run command: `roscore`
- open new terminal and run command: `rosrun rosserial_python serial_node.py _port:=tcp _tcp_port:=<11422>`.
    - `<11422>` must be channge with your `ROS_SERVER_PORT`. eg. `_tcp_port:=1147`.
    - if you want to control with serial o defect values view [Ros Wiki](http://wiki.ros.org/Documentation)
- If you found errors when run the command. Is posible that the package version is other diferent to the normal, so do next:
    - with the terminal or grafic editor open the file: `/opt/ros/noetic/lib/rosserial_python/serial_node.py`.
    - Watch the lines 58 and 59 and verify if this code is there:
    ```python

        tcp_portnum = int(rospy.get_param('/rosserial_embeddedlinux/tcp_port', '11411'))
        fork_server = rospy.get_param('/rosserial_embeddedlinux/fork_server', False)

    ```
    - if you confirmed the above, then the last command isn't usable. there are many solutions, but I did the most easily for me and change the code. However, if you want to resolve it of other way, that is also good. So, I changed the lines 58 and 59 with:
    ```python

    if (rospy.has_param('~tcp_port')):
        tcp_portnum = int(rospy.get_param('~tcp_port', '11411'))
    else:
        tcp_portnum = int(rospy.get_param('/rosserial_embeddedlinux/tcp_port','11411'))
    if (rospy.has_param('~fork_server')):
        fork_server = rospy.get_param('~fork_server')
    else:
        fork_server = rospy.get_param('/rosserial_embeddedlinux/fork_server', False)

    ```
    If you are curious and review the documentation about rosserial python, you should have noticed that the code is the same that other version. If you go to oficial documentation on [github](https://github.com/ros-drivers/rosserial/blob/noetic-devel/rosserial_python/nodes/serial_node.py), you must see better.
- Now, the "serial_node" node is running and if the ESP32 is running too, then the server will recognize it automatically.
- In order to check the communication, run the command `rostopic list` in other terminal and you will can see the topics, that they was created for the ESP32. 


