import 'package:flutter/material.dart';
import '../services/rosbridgeclient.dart';

class ButtonsWidget extends StatelessWidget {
  final double linearVel;
  final double angularVel;
  final ROSBridgeClient rosClient;
  final String selectedTopic;

  const ButtonsWidget({
    super.key,
    required this.linearVel,
    required this.angularVel,
    required this.rosClient,
    required this.selectedTopic,
  });

  @override
  Widget build(BuildContext context) {
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
                size: 60.0,
              ),
            ),
          );
        }),
      ),
    );
  }
}
