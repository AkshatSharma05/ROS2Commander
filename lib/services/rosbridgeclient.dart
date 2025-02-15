import 'dart:convert';
import 'package:web_socket_channel/io.dart';
import 'package:web_socket_channel/web_socket_channel.dart';

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