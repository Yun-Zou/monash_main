# monash_main
Main module for autonomous drone project. This module has the drone monitoring and command files, combined launch files, general scripts, and any custom message types that need to be defined for multiple packages.

## Combined Launch Files
There are a series of combined launch files in `monash_main/launch/`. These are launch files which have many of the required launch lines needed for specific configurations such as using T265 camera, no camera, or simulation. It won't run the flight launch files, that will be run seperately.

## Drone Monitoring and Commanding
This module has the Drone Monitoring and Commanding files which lets you view the topics and image stream of the drone while it is running through an internet connection. It runs on ROSLibJS for ROS communication and web_video_server for image streaming. It uses a web frontend which should be converted to python ROSLibPy and loaded on a robot for collaborative purposes.

### How to Run
1. Ensure you are on a common Wi-Fi connection with your drone and device (eduroam doesn't work, as it is restricted)
2. Run `roslaunch rosbridge_server rosbridge_websocket.launch` on both devices (or one of the combined launch files)
3. Check the IP of the drone in the Wi-Fi connection by typing `ifconfig` in the terminal
4. Open the `monash_main/web/laptop_monitor.html` file in an internet browser
5. Type in the IP of the drone in the top input and click the button. If connection is successful, then the textbox should highlight green

- Position, Status and Target information are on the left panel. 
- Command requesting is on the right panel. These requests are only requests and are subject to conditions and ArduPilot rules and won't execute if not possible.
- You can request image streams by clicking on the buttons on the bottom panel, this may slow down the drone and you may encounter latency/dropped frames.