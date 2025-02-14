import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:web_socket_channel/io.dart';
import 'package:web_socket_channel/web_socket_channel.dart';

void main() => runApp(MaterialApp(
  home: Home(),
));

class Home extends StatelessWidget {
  const Home({super.key});

  @override
  
  Widget build(BuildContext context) {
  double linearVel = 1.0;
  double angularVel = 5.0;

  late ROSBridgeClient rosClient;
  String selectedTopic = "/cmd_vel";
  String ipAddress = "192.168.1.1";

  rosClient = ROSBridgeClient(ipAddress); // Change to your ROS2 device IP

    final List<IconData> icons = [
      Icons.arrow_upward_rounded,
      Icons.keyboard_arrow_up,
      Icons.arrow_upward_rounded,
      Icons.keyboard_arrow_left,
      Icons.stop,
      Icons.keyboard_arrow_right,
      Icons.arrow_downward_rounded,
      Icons.keyboard_arrow_down,
      Icons.arrow_downward_rounded,
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

    return Scaffold(
      backgroundColor: const Color.fromARGB(255, 41, 41, 41),
      appBar: AppBar(
        backgroundColor: Colors.blueAccent,
        foregroundColor: Colors.white,
        title: Text('ROS2Commander'),
        centerTitle: true,
      ),
      body: 
      Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          children: [
            Expanded(
              child: Align(
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
                            velocities[index]["angularZ"]!
                          ),
                        style: ElevatedButton.styleFrom(
                          backgroundColor: Colors.lightBlue,
                          foregroundColor: Colors.white,
                        ),
                        child: Icon(
                          color: Colors.black,
                          icons[index],
                          size: 40.0, // Increase the icon size
                        ),
                      ),
                    );
                  }),
                ),
              ),
            ),
            Row(
              children: [
                Expanded(
                  child: TextField(
                    decoration: InputDecoration(
                      border: OutlineInputBorder(),
                      enabledBorder: OutlineInputBorder(
                        borderSide: BorderSide(color: Colors.blue, width: 2), // Set the border color when enabled
                      ),
                      focusedBorder: OutlineInputBorder(
                        borderSide: BorderSide(color: Colors.green, width: 2), // Set the border color when focused
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
                        borderSide: BorderSide(color: Colors.blue, width: 2), // Set the border color when enabled
                      ),
                      focusedBorder: OutlineInputBorder(
                        borderSide: BorderSide(color: Colors.green, width: 2), // Set the border color when focused
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
                      borderSide: BorderSide(color: Colors.blue, width: 2), // Set the border color when enabled
                      ),
                      focusedBorder: OutlineInputBorder(
                      borderSide: BorderSide(color: Colors.green, width: 2), // Set the border color when focused
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
                      borderSide: BorderSide(color: Colors.blue, width: 2), // Set the border color when enabled
                      ),
                      focusedBorder: OutlineInputBorder(
                      borderSide: BorderSide(color: Colors.green, width: 2), // Set the border color when focused
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