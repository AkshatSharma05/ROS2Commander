import 'package:flutter/material.dart';

void main() => runApp(MaterialApp(
  home: Home(),
));

class Home extends StatelessWidget {
  const Home({super.key});

  @override
  Widget build(BuildContext context) {
    return
  Scaffold(
    appBar: AppBar(
      title: Text('ROS2 Commander'),
      centerTitle: true,
    ),
    body: Center(
      child: Text(
        'Hello World',
        style: TextStyle(
          fontSize: 20.0,
          fontWeight: FontWeight.bold,
          letterSpacing: 2.0,
          color: Colors.grey[600],
          fontFamily: 'IndieFlower',
        ),
      ),
    ),
    floatingActionButton: FloatingActionButton(
      onPressed: () {},
      child: Icon(
        Icons.add
        ),
    )
    );
  }
}