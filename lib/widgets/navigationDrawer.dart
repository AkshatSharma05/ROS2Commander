import 'package:flutter/material.dart';

class sideBar extends StatelessWidget {
  const sideBar({super.key});

  @override
  Widget build(BuildContext context) {
    return Drawer(
      child: ListView(
        padding: EdgeInsets.zero,
        children: [
          DrawerHeader(
            decoration: BoxDecoration(
              color: Colors.blueAccent,
            ),
            child: Text(
              'ROS2Commander',
              style: TextStyle(
                fontFamily: 'Roboto',
                color: Colors.white,
                fontSize: 32,
              ),
            ),
          ),
          Padding(
            padding: const EdgeInsets.all(8.0),
            child: Text(
              'Instructions: \n1. Enter the linear and angular velocities. \n2. Enter the topic name and IP address. \n3. Press the buttons to control the robot. \n4. Make sure the mobile and the host are connected to the same network and rosbridge is running on the host. Command: \n (ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0)',
            ),
          ),
          SizedBox(height: 40),
          Padding(
            padding: const EdgeInsets.all(8.0),
            child: Text(
              "This project is licensed under the MIT License. \n\n© 2025 Akshat Sharma",
            ),
          ),
        ],
      ),
    );
  }
}