
<!-- PROJECT LOGO -->
<br />


  <h3 align="center">ROS2 Driver</h3>

  <p align="center">
    PolyExplore, Inc.
    <br />
    <a href="https://www.polyexplore.com/"><strong>Visit Our Website</strong></a>
  </p>
  <p align="center">
  <a href="https://github.com/othneildrew/Best-README-Template">
    <img src="images/logo.png" alt="Logo" width="160" height="160">
  </a>
</p>



<!-- TABLE OF CONTENTS -->
<details open="open">
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#installation">Installation</a>
      <!-- <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul> -->
    </li>
    <li><a href="#ethernet-output">Ethernet Output</a></li>
    <li><a href="#local-map-origin">Local Map Origin</a></li>
    <li><a href="#imu-data">IMU Data</a></li>
    <li><a href="#geoid-height">Geoid Height</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>

<p align="center">
    <img src="images/product.png" alt="Logo" width="400" height="300">
</p>

<!-- ABOUT THE PROJECT -->
## Installation
1. Install ROS2 following this [link](https://index.ros.org/doc/ros2/Installation/#installationguide)  
Please make sure development tools are all installed, check [this](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/#id5)
2. Clone the repo
   ```sh
   git clone https://github.com/polyexplore/ROS2_Driver.git
   ```
2. Copy our polyx_node folder to your source directory, for example ~/colcon_ws/src/
   ```sh
   mv ROS2_driver/polyx_node ~/colcon_ws/src/
   ```
3. Build
   ```sh
   cd ~/colcon_ws/
   colcon build --symlink-install
   ```
4. Once compiles successfully, please source your workspace again
   ```sh
   source install/setup.bash
   ```
We have successfully built our driver on Dashing and Foxy-FitzRoy



<!-- GETTING STARTED -->

## Ethernet Output
1. Connect our device to the eithernet port, figure out the device's static ip, for example 10.1.10.194
2. Open the param/polyx_talker_params.yaml, you will see the following parameterss
    ```yaml
    /polyx_ns:
      polyx_node_talker:
        ros__parameters:
          use_sim_time: False
          eth_enable: True
          eth_server: 192.168.130.97 #"10.1.10.194"
          eth_port: "8888"
          polyx_output: 0xFF
    ```
  * eth_server to the static ip you found out in the last step
  * eth_enable turns on/off ethernet port
  * eth_port is fixed to "8888"
  * polyx_output is the bitmask for output messages, by default everything is turned on
3. launch the talker
    ```sh
    ros2 launch polyx_node polyx_talker_launch.py 
    ``` 
4. check the topics published
    ```sh
    ros2 topic list
    ```
    you should be able to see some topics, for example like below
    ```sh
    /polyx_ns/polyx_Kalman
    /polyx_ns/polyx_compactNav
    /polyx_ns/polyx_correctedIMU
    ```
## Local Map Origin
By default the ROS2 driver uses the first navigation solution as the origin. To set a specific position as the origin of your local map, please use the following function defined in polyx_convert.cpp:
  ```cpp
  void SetCustomOrigin(
    double               latitude,   // radian
    double               longitude,  // radian
    double               altitude,   // meters
    struct origin_type&  org);
   ```
In polyx_node_talker.cpp, look for the follwoing part and replace SetOrigin() with SetCustomOrigin().
  ```cpp
  if (!is_origin_set) {
    SetOrigin(msg, myorigin);
    is_origin_set = true;
  }
  ```
After any modification, please compile again using the following commands
```sh
cd ~/colcon_ws && colcon build --symlink-install
```

## IMU Data

The ROS driver can output both the scaled raw IMU data and the corrected IMU data if the user configured the system to output these messages. Note that the corrected IMU data are available only after the initialization of the inertial navigator. This message contains IMU data corrected for the sensor biases estimated by the fusion algorithm.

## Geoid Height
The Geoid message contains the height of the Geoid above the ellipsoid. Thus the height above Geoid, treated normally as the height of mean sea level (MSL), can be computed as follows:
<p align="center">
Height above MSL = Height above ellipsoid - Geoid height.
</p>

## Contact
Support - [@Support](https://www.polyexplore.com/) - support@polyexplore.com

**Thank you!**
