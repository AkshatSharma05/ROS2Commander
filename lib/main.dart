import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'services/rosbridgeclient.dart';
import 'widgets/joystickWidget.dart';
import 'widgets/buttonsWidget.dart';
import 'widgets/navigationDrawer.dart';


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
    String selectedTopic = "/cmd_vel"; //default topic
    String ipAddress = "192.168.1.1"; //default IP

    rosClient = ROSBridgeClient(ipAddress); // Change to your ROS2 device IP

    Widget _buildJoystick() {
      return JoystickWidget(
        linearVel: linearVel,
        angularVel: angularVel,
        rosClient: rosClient,
        selectedTopic: selectedTopic,
      );
    }

    Widget _buildButtonGrid() {
      return ButtonsWidget(
        linearVel: linearVel,
        angularVel: angularVel,
        rosClient: rosClient,
        selectedTopic: selectedTopic,
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
      drawer: const sideBar(),
    );
  }
}


