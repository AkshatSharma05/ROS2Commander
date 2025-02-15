# ROS2Commander

![play_store_512](https://github.com/user-attachments/assets/35967244-6cb4-4005-a90c-0fdf1c8f7bb8)

R2C (ROS2Commander) is a mobile app made in flutter for the purpose of controlling bots utilizing ROS2.

# Prerequisites

Ensure you have the following installed:

- Flutter (https://flutter.dev/docs/get-started/install)

- Dart (included with Flutter)

- A running ROS2 setup

- rosbridge_server running on your ROS2 system

# Installations

- Clone the repository:

    ```git clone https://github.com/AkshatSharma05/ROS2Commander.git ```

    ```cd ROS2Commander``` 

- Install Flutter dependencies:

    ```flutter pub get```

- Ensure rosbridge_server is running on your ROS2 machine:

    ```ros2 launch rosbridge_server rosbridge_websocket.launch.xml address:=0.0.0.0```

# Running the App

Connect your mobile device to the same network as your ROS2 system.

- Run the app on your device:

    ```flutter run```

- Building the APK

    To generate a release APK:

    ```flutter build apk --release```

# Usage

- Enter the ROS2 topic in the provided text field.

- Use the directional buttons to send velocity commands (Twist messages).

- Ensure the ROS2 machine is running rosbridge_server to receive the WebSocket messages.

# Example
*Click to play the video*

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/XGR663oIk3I/0.jpg)](https://www.youtube.com/watch?v=XGR663oIk3I)

\
\
*This project is licensed under the terms of the MIT license.*
