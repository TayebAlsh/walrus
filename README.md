# walrus

The following sensor, actuator & communication libraries are included:
- BlueRobotics MS5837 Depth Sensor
- YOST Labs IMU 
- Servo library driver 
- AsyncUDP_Teensy41
- QNEthernet

In addition to these libraries, the software package contains a class that enables 
functionality of the remotely operated vehicle by sending and receiving vehicle states and 
vehicle inputs, respectively. The class interfaces the different sensors and actuators while
managing reliable bidirectional UDP based communication with the towing vehicle.

The source code has been developed for, and tested, on a Teensy 4.1 micro-controller. 
The objectives of the source code is to support bi-directional communication for an 
actuated tethered vehicle with onboard sensing and dynamic capabilities.

![Picture1](https://user-images.githubusercontent.com/83678386/228845231-ecb7e2a6-dcdd-4726-a611-a5c74db20bbe.png)
To install the required `eigen` library, run the following command in your terminal:




```sh
git clone https://github.com/bolderflight/eigen.git lib/eigen