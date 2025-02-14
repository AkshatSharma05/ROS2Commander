import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:web_socket_channel/io.dart';
import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:flutter/services.dart';
import 'package:flutter_joystick/flutter_joystick.dart';

void main() => runApp(MaterialApp(
      home: Home(),
    ));

class Home extends StatefulWidget {
  const Home({super.key});
  _HomeState createState() => _HomeState();
}

class _HomeState extends State<Home> {
  bool useJoystick = false;
  @override
  Widget build(BuildContext context) {
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.portraitUp,
      DeviceOrientation.portraitDown,
    ]);

    double linearVel = 1.0;
    double angularVel = 5.0;

    late ROSBridgeClient rosClient;
    String selectedTopic = "/cmd_vel";
    String ipAddress = "192.168.1.1";

    rosClient = ROSBridgeClient(ipAddress); // Change to your ROS2 device IP

    /// Joystick UI
    Widget _buildJoystick() {
      return Center(
        child: Joystick(
          mode: JoystickMode.all,
          listener: (details) {
            double linearX = -details.y * linearVel;
            double angularZ = -details.x * angularVel;
            rosClient.publish(selectedTopic, linearX, angularZ);
          },
        ),
      );
    }

    Widget _buildButtonGrid() {
          final List<IconData> icons = [
      Icons.north_west,
      Icons.keyboard_arrow_up,
      Icons.north_east,
      Icons.keyboard_arrow_left,
      Icons.stop,
      Icons.keyboard_arrow_right,
      Icons.south_west,
      Icons.keyboard_arrow_down,
      Icons.south_east,
    ];

    final List<Map<String, double>> velocities = [
      {"linearX": linearVel, "angularZ": -angularVel}, // Forward left
      {"linearX": linearVel, "angularZ": 0.0}, // Forward
      {"linearX": linearVel, "angularZ": angularVel}, // Forward right
      {"linearX": 0.0, "angularZ": -angularVel}, // Rotate left
      {"linearX": 0.0, "angularZ": 0.0}, // Stop (center button)
      {"linearX": 0.0, "angularZ": angularVel}, // Rotate right
      {"linearX": -linearVel, "angularZ": -angularVel}, // Backward left
      {"linearX": -linearVel, "angularZ": 0.0}, // Backward
      {"linearX": -linearVel, "angularZ": angularVel}, // Backward right
    ];
    
      return Align(
        alignment: Alignment.center,
        child: GridView.count(
          crossAxisCount: 3,
          shrinkWrap: true,
          // like a for loop
          children: List.generate(9, (index) {
            return Padding(
              padding: const EdgeInsets.all(8.0),
              child: ElevatedButton(
                onPressed: () => rosClient.publish(
                    selectedTopic,
                    velocities[index]["linearX"]!,
                    velocities[index]["angularZ"]!),
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.lightBlue,
                  foregroundColor: Colors.white,
                ),
                child: Icon(
                  color: Colors.black,
                  icons[index],
                  size: 60.0, // Increase the icon size
                ),
              ),
            );
          }),
        ),
      );
    }

    return Scaffold(
      backgroundColor: const Color.fromARGB(255, 41, 41, 41),
      appBar: AppBar(
        backgroundColor: Colors.blueAccent,
        foregroundColor: Colors.white,
        title: Text('ROS2Commander'),
        centerTitle: true,
        actions: [
          Switch(
            value: useJoystick,
            onChanged: (bool value) {
              setState(() {
                useJoystick = value;
              });
            },
          ),
        ],
      ),
      body: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          children: [
            Expanded(
              child: useJoystick ? _buildJoystick() : _buildButtonGrid(),
            ),
            Row(
              children: [
                Expanded(
                  child: TextField(
                    decoration: InputDecoration(
                      border: OutlineInputBorder(),
                      enabledBorder: OutlineInputBorder(
                        borderSide: BorderSide(
                            color: Colors.blue,
                            width: 2), // Set the border color when enabled
                      ),
                      focusedBorder: OutlineInputBorder(
                        borderSide: BorderSide(
                            color: Colors.green,
                            width: 2), // Set the border color when focused
                      ),
                      fillColor: Colors.white,
                      filled: true,
                      labelText: 'Linear Velocity',
                    ),
                    onSubmitted: (String value) {
                      linearVel = double.parse(value);
                    },
                  ),
                ),
                SizedBox(width: 10),
                Expanded(
                  child: TextField(
                    decoration: InputDecoration(
                      border: OutlineInputBorder(),
                      enabledBorder: OutlineInputBorder(
                        borderSide: BorderSide(
                            color: Colors.blue,
                            width: 2), // Set the border color when enabled
                      ),
                      focusedBorder: OutlineInputBorder(
                        borderSide: BorderSide(
                            color: Colors.green,
                            width: 2), // Set the border color when focused
                      ),
                      fillColor: Colors.white,
                      filled: true,
                      labelText: 'Angular Velocity',
                    ),
                    onSubmitted: (String value) {
                      angularVel = double.parse(value);
                    },
                  ),
                ),
              ],
            ),
            SizedBox(height: 10),
            Column(
              children: [
                TextField(
                  decoration: InputDecoration(
                    border: OutlineInputBorder(),
                    enabledBorder: OutlineInputBorder(
                      borderSide: BorderSide(
                          color: Colors.blue,
                          width: 2), // Set the border color when enabled
                    ),
                    focusedBorder: OutlineInputBorder(
                      borderSide: BorderSide(
                          color: Colors.green,
                          width: 2), // Set the border color when focused
                    ),
                    fillColor: Colors.white,
                    filled: true,
                    labelText: 'Enter Topic',
                  ),
                  onSubmitted: (String value) {
                    selectedTopic = value;
                  },
                ),
                SizedBox(height: 10),
                TextField(
                  decoration: InputDecoration(
                    border: OutlineInputBorder(),
                    enabledBorder: OutlineInputBorder(
                      borderSide: BorderSide(
                          color: Colors.blue,
                          width: 2), // Set the border color when enabled
                    ),
                    focusedBorder: OutlineInputBorder(
                      borderSide: BorderSide(
                          color: Colors.green,
                          width: 2), // Set the border color when focused
                    ),
                    fillColor: Colors.white,
                    filled: true,
                    labelText: 'Enter IP Address',
                  ),
                  onSubmitted: (String value) {
                    rosClient.close();
                    ipAddress = value;
                    rosClient = ROSBridgeClient(ipAddress);
                  },
                ),
              ],
            ),
          ],
        ),
      ),
      drawer: const NavigationDrawer(),
    );
  }
}

/// ROS 2 WebSocket Client for rosbridge_server
class ROSBridgeClient {
  late WebSocketChannel _channel;

  ROSBridgeClient(String ipAddress, {int port = 9090}) {
    _channel = IOWebSocketChannel.connect('ws://$ipAddress:$port');
  }

  void publish(String topic, double linearX, double angularZ) {
    var message = jsonEncode({
      "op": "publish",
      "topic": topic,
      "msg": {
        "linear": {"x": linearX, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": angularZ}
      },
      "type": "geometry_msgs/Twist"
    });
    _channel.sink.add(message);
  }

  void close() {
    _channel.sink.close();
  }
}

class NavigationDrawer extends StatelessWidget {
  const NavigationDrawer({super.key});

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
