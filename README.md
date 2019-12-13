# ESP ROS Example

* Port [uROSnode](https://github.com/openrobots-dev/uROSnode) for ESP32 (IDF)

## How to run the demo

* Make sure that you installed the latest esp-idf version (currently available v4.1).
* Clone this project.
* Change the Remote IP and the Local IP in components/urosconf.h file.
* Setting Wifi Connection.
* Copy ``keyboard`` folder to your catkin workspace (i.e ``catkin_ws/src``)
* Build all sources in catkin ws.
* Run ROS master Node (``roscore``).
* Open other Tab then run keyboard Node (``rosrun keyboard control``).
* Press w,s,a,d,q,e keys to send direction commands or press space key to send stop command to ESP Node.
* Open other Tab to listen messages from ESP node which published by topic ``esp_sensor`` (i.e ``rostopic echo esp_sensor``).