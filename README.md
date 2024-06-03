# Fully Autonomous Sumo Robot

## Overview
This project features a fully autonomous sumo robot designed for sumo wrestling competitions. The robot uses various sensors and motors to detect the opponent, navigate the ring, and attempt to push the opponent out of the ring while avoiding being pushed out itself.

## Features
- **Infrared Sensors:** Detects the presence of the opponent and the edges of the sumo ring.
- **Ultrasonic Sensors:** Measures distances to avoid obstacles and opponents.
- **IR Remote Control:** Allows remote starting of the robot.
- **Motor Control:** Controls the movement of the robot including forward, backward, and rotation movements.

## Components
- **IR Sensors:** For detecting the edge of the ring and the opponent.
- **Ultrasonic Sensors:** For distance measurement.
- **Motors and Motor Driver:** For driving the robot.
- **IR Receiver:** For remote control.
- **Microcontroller:** Arduino board to control the robot.

## Installation and Setup
1. **Clone the Repository:**
   ```sh
   git clone https://github.com/MohamedAwadin/Fully_Autonomous_Sumo_Robot.git
   cd Fully_Autonomous_Sumo_Robot
   ```
2. **Open in Arduino IDE:**
   Open the `main.ino` file in Arduino IDE.

3. **Upload to Arduino:**
   Connect your Arduino to the computer and upload the code.

## Files Description
- **main.ino:** Main file containing the setup and loop functions.
- **MotorControl.h / MotorControl.cpp:** Functions to control the motors.
- **SensorHandling.h / SensorHandling.cpp:** Functions to handle sensor readings.
- **DirectionHandling.h / DirectionHandling.cpp:** Functions to manage the robot's direction based on sensor inputs.
- **InterruptHandling.h / InterruptHandling.cpp:** Interrupt handling functions for the IR sensors.

## Usage
1. **Power the Robot:**
   Ensure the robot is powered on.
2. **Start with Remote:**
   Use the IR remote to start the robot. The default start command is `0x10EFD827` (Replace with your actual command).

## Contributing
Contributions are welcome! Please fork this repository and submit a pull request for any enhancements or bug fixes.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
