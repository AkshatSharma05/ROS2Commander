import 'package:flutter_joystick/flutter_joystick.dart';
import '../services/rosbridgeclient.dart';
import 'package:flutter/material.dart';

class JoystickWidget extends StatefulWidget {
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
  _JoystickWidgetState createState() => _JoystickWidgetState();
}

class _JoystickWidgetState extends State<JoystickWidget> {
  late double linearVel;
  late double angularVel;
  late ROSBridgeClient rosClient;
  late String selectedTopic;

  @override
  void initState() {
    super.initState();
    linearVel = widget.linearVel;
    angularVel = widget.angularVel;
    rosClient = widget.rosClient;
    selectedTopic = widget.selectedTopic;
  }

  @override
  void didUpdateWidget(covariant JoystickWidget oldWidget) {
    super.didUpdateWidget(oldWidget);
    
    if (oldWidget.linearVel != widget.linearVel ||
        oldWidget.angularVel != widget.angularVel ||
        oldWidget.rosClient != widget.rosClient ||
        oldWidget.selectedTopic != widget.selectedTopic) {
      
      setState(() {
        linearVel = widget.linearVel;
        angularVel = widget.angularVel;
        rosClient = widget.rosClient;
        selectedTopic = widget.selectedTopic;
      });
    }
  }

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
