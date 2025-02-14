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

  late ROSBridgeClient rosClient;
  String selectedTopic = "/cmd_vel";

  rosClient = ROSBridgeClient("192.168.1.87"); // Change to your ROS2 device IP

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
      {"linearX": 1.0, "angularZ": -1.0}, // Forward left
      {"linearX": 1.0, "angularZ": 0.0}, // Forward
      {"linearX": 1.0, "angularZ": 1.0}, // Forward right
      {"linearX": 0.0, "angularZ": -1.0}, // Rotate left
      {"linearX": 0.0, "angularZ": 0.0}, // Stop (center button)
      {"linearX": 0.0, "angularZ": 1.0}, // Rotate right
      {"linearX": -1.0, "angularZ": -1.0}, // Backward left
      {"linearX": -1.0, "angularZ": 0.0}, // Backward
      {"linearX": -1.0, "angularZ": 1.0}, // Backward right
    ];

    return Scaffold(
      appBar: AppBar(
        title: Text('ROS2 Commander'),
        centerTitle: true,
      ),
      body: Padding(
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
                          icons[index],
                          size: 40.0, // Increase the icon size
                        ),
                      ),
                    );
                  }),
                ),
              ),
            ),
            TextField(
              decoration: InputDecoration(
                border: OutlineInputBorder(),
                labelText: 'Enter Topic',
              ),
              onSubmitted: (String value) {
                selectedTopic = value;
                print('Selected topic: $selectedTopic');
              },
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