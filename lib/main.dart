import 'package:flutter/material.dart';

void main() => runApp(MaterialApp(
  home: Home(),
));

class Home extends StatelessWidget {
  const Home({super.key});

  @override
  Widget build(BuildContext context) {
    final List<IconData> icons = [
      Icons.arrow_upward_rounded,
      Icons.keyboard_arrow_up,
      Icons.arrow_upward_rounded,
      Icons.keyboard_arrow_left,
      Icons.stop, // This will not be used as the center is empty
      Icons.keyboard_arrow_right,
      Icons.arrow_downward_rounded,
      Icons.keyboard_arrow_down,
      Icons.arrow_downward_rounded,
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
                  children: List.generate(9, (index) {
                    return Padding(
                      padding: const EdgeInsets.all(8.0),
                      child: ElevatedButton(
                        onPressed: () {},
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
            ),
          ],
        ),
      ),
    );
  }
}