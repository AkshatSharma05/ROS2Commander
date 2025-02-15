import 'package:flutter_joystick/flutter_joystick.dart';
import '../services/rosbridgeclient.dart';
import 'package:flutter/material.dart';

class JoystickWidget extends StatelessWidget {
  final double linearVel;
  final double angularVel;
  final ROSBridgeClient rosClient;
  final String selectedTopic;

  const JoystickWidget({
    super.key,
    required this.linearVel,
    required this.angularVel,
    required this.rosClient,
    required this.selectedTopic,
  });

  @override
  Widget build(BuildContext context) {
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
}