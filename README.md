# Autonomous Quadcopter

Autonomous quadcopter flight software for a GPS-denied PX4/MAVROS vehicle, with a laptop ground-control station (GCS). The system uses visual-inertial odometry (VIO), optical flow, and lidar rather than GPS for position holding, stabilization, altitude control, and drift reduction.

## System overview

The vehicle is controlled by a Pixhawk flight controller and a Jetson Nano 4 GB companion computer. The laptop GCS communicates with the Jetson over a Wi-Fi network. The Jetson communicates with the Pixhawk through a UART connection and runs the ROS 1 flight software, sensor monitoring, emergency response, and command listener.

The system is intended for GPS-denied operation:

```text
Laptop GCS  <-- Wi-Fi / TCP -->  Jetson Nano  <-- UART / MAVLink -->  Pixhawk
  |                                |                                  |
  |                                +-- USB camera                      +-- Optical flow over I2C
  |                                +-- Lidar over USB-to-TTL           +-- ESC and motor outputs
  |                                +-- ROS 1 / VIO processing
  |
  +-- Live video and VIO drift display
```

## Hardware

The prototype uses the following hardware:

| Component | Function |
| --- | --- |
| Pixhawk with Radiolink 2.4 GHz radio link | Flight-control, stabilization, RC/manual safety link, and vehicle I/O |
| Jetson Nano 4 GB | Companion computer running ROS 1, MAVROS, VIO, sensor processing, and the GCS command listener |
| Electronic speed controllers (ESCs) | Convert Pixhawk motor commands into motor drive signals |
| Brushless motors and propellers | Produce the four-rotor thrust and attitude control response |
| Power distribution board (PDB) | Distributes battery power to the ESCs and vehicle power system |
| USB camera | Provides the camera stream used by VIO and the live video feed |
| Lidar | Supplies altitude/range measurements for height control and landing detection |
| Optical flow sensor | Provides motion information for position holding and stabilization without GPS |
| Mission Planner ground station | Provides Pixhawk telemetry, configuration, parameter monitoring, calibration, and manual mission supervision |

## Hardware connections

| Connection | Physical interface | Purpose |
| --- | --- | --- |
| Laptop GCS to Jetson Nano | Wi-Fi network, TCP port `5005` | Start, kill, disarm, ping, state messages, and flight logs |
| Jetson Nano to Pixhawk | UART, using MAVLink through MAVROS | Vehicle state, sensor data, flight modes, arming, and setpoints |
| Pixhawk to Mission Planner | Telemetry radio link using MAVLink telemetry | Sends Pixhawk telemetry to Mission Planner for configuration, monitoring, calibration, and operator supervision |
| Lidar to Jetson Nano | USB-to-TTL converter | Sends lidar altitude/range data to the companion computer |
| USB camera to Jetson Nano | USB | Captures frames for VIO and video processing |
| Optical flow sensor to Pixhawk | I2C | Supplies optical-flow motion data to the flight controller |
| Pixhawk to ESCs | Motor output signals and vehicle power wiring | Controls motor speed through the ESCs |
| Battery/PDB to ESCs and electronics | Power wiring | Distributes regulated and motor power; follow the hardware power budget when assembling |

The Pixhawk remains responsible for the fast low-level attitude and motor-control loop. The Jetson provides higher-level autonomy, perception, telemetry supervision, and the network interface to the GCS.

Mission Planner connects to the Pixhawk through the telemetry link. It is used for Pixhawk setup, parameter configuration, sensor calibration, telemetry monitoring, and manual supervision. This telemetry connection is separate from the Wi-Fi connection used by the custom laptop GCS to communicate with the Jetson, while both interfaces can observe and supervise the vehicle through their respective MAVLink paths.

## Communication protocols and interfaces

### Laptop to Jetson

The GCS and Jetson command listener communicate over a Wi-Fi network using a persistent TCP connection on port `5005`. Commands are newline-delimited text messages:

- `START` starts the flight manager after connection validation.
- `KILL` stops flight processes and activates the software failsafe.
- `DISARM` requests the highest-priority hardware disarm path.
- `PING` keeps the connection watchdog alive.

The Jetson returns acknowledgements, state updates, log messages, and connection-failure messages. The listener also monitors the connection and triggers a failsafe if the GCS disconnects or stops sending keepalive traffic during an active mission.

### Jetson to Pixhawk

The Jetson uses a UART serial link to communicate with the Pixhawk through MAVLink and MAVROS. ROS nodes publish and subscribe to MAVROS topics and call MAVROS services for vehicle state, sensor telemetry, arming, mode changes, and local-position setpoints. The emergency system can command `AUTO.LAND` when a critical fault is detected.

### Pixhawk to Mission Planner telemetry

The Pixhawk is connected to Mission Planner through a telemetry radio link. MAVLink telemetry carries vehicle status, attitude, position estimates, battery information, flight mode, parameters, and diagnostic data to Mission Planner. Mission Planner can also send configuration, calibration, and operator commands back to the Pixhawk through the same telemetry connection. The telemetry link provides a separate ground-station path for setup and supervision during GPS-denied testing.

### Onboard sensor interfaces

The optical flow sensor is connected directly to the Pixhawk over I2C so the flight controller can use motion information for stabilization and position holding. The lidar is connected to the Jetson using a USB-to-TTL converter and contributes altitude and landing information to the companion-computer software. The USB camera is connected directly to the Jetson and is processed by the VIO module.

## Folder structure

## Project layout

```text
 CMakeLists.txt
 package.xml
 scripts/
   Main.py
   Autonomous_Flight.py
   Emergency_Failsafe.py
   GUI_Listener.py
   Sensor_check.py
gcs/
  DroneGUI.py
  VIOdrift_correction.py
requirements-gcs.txt
```

The files under `scripts` run on the ROS/Jetson side. They remain in one directory because the ROS entry point imports the other scripts directly. The files under `gcs` run on the laptop and share the visual-inertial-odometry module.

### ROS / Jetson scripts

- `scripts/Main.py` is the mission entry point. It performs preflight sensor calibration, starts the emergency response system, and launches autonomous flight.
- `scripts/Sensor_check.py` subscribes to MAVROS sensor topics and builds the shared sensor-health status used before and during flight.
- `scripts/Emergency_Failsafe.py` monitors telemetry, IMU, battery, altitude, attitude, optical flow, range, EKF status, and the flight-loop watchdog. It commands emergency landing when required.
- `scripts/Autonomous_Flight.py` implements the flight state machine, local-position setpoints, takeoff, hover, return, drift correction, and landing behavior.
- `scripts/GUI_Listener.py` is the Jetson TCP server. It accepts GCS commands, starts the flight manager, forwards state/log messages, and handles kill/disarm watchdog behavior.

### Laptop GCS files

- `gcs/DroneGUI.py` provides the operator interface, live video view, VIO drift display, mission controls, TCP status, and flight log.
- `gcs/VIOdrift_correction.py` provides the camera-based visual odometry engine. It tracks image features, estimates accumulated image motion, reports drift, and exposes the standalone VIO test mode.

The repository root contains `package.xml` and `CMakeLists.txt` for catkin, `requirements-gcs.txt` for laptop Python dependencies, and this README for deployment and system documentation.

## Software algorithms

### Preflight sensor calibration

`Sensor_check.py` evaluates MAVROS telemetry and sensor streams before the mission is allowed to start. It checks connectivity, IMU behavior, magnetometer, barometer, optical-flow quality, lidar/rangefinder data, battery state, local position, and EKF status. A shared `SensorHealthStatus` object makes the results available to the flight and emergency-response modules.

### Autonomous flight state machine

`Autonomous_Flight.py` uses a state machine for initialization, readiness checks, offboard-mode request, arming, takeoff, hover, return, landing, and completion. Local-position setpoints are published at a fixed rate. The target altitude, hover duration, drift limits, and landing thresholds are defined as mission constants.

### VIO and drift reduction

The VIO module converts camera frames to grayscale, detects image features, and tracks them between frames using pyramidal Lucas-Kanade optical flow. The accumulated feature displacement estimates camera motion relative to the locked origin. The result is reported in pixels and converted approximately to metres using camera focal length and altitude. The flight controller uses position error and PID correction to reduce horizontal drift while hovering.

VIO is complementary to the optical-flow sensor: the camera provides visual drift estimation and video, while the optical-flow sensor provides onboard motion information through the Pixhawk I2C interface for stabilization and position holding.

### Altitude and landing control

Lidar range data is used to validate low-altitude measurements, maintain altitude behavior, detect stable altitude, and confirm touchdown. The system combines lidar/rangefinder information with MAVROS local-position and barometric/EKF data rather than relying on GPS.

### Emergency response and watchdogs

`Emergency_Failsafe.py` runs an independent monitoring timer. It detects telemetry loss, invalid or stale sensor data, EKF failures, low battery, altitude ceiling violations, excessive tilt or velocity, lost offboard mode, and a stalled flight-control loop. A triggered event commands `AUTO.LAND`, records the emergency code, notifies the GCS, and prevents duplicate emergency actions.

The GCS listener adds a network watchdog. A lost TCP connection, missing keepalive messages, or operator `KILL`/`DISARM` command can stop the flight manager and activate the failsafe path.

## ROS / Jetson setup

Build this package inside a catkin workspace:

```bash
cd ~/catkin_ws/src
git clone <repository-url> autonomous_quadcopter
cd ..
catkin_make
source devel/setup.bash
```

Run the command listener after starting `roscore` and MAVROS:

```bash
rosrun autonomous_quadcopter GUI_Listener.py
```

The listener launches the flight manager configured in `GUI_Listener.py` when it receives `START`.

## Ground-control station setup

Install the laptop dependencies:

```bash
python -m pip install -r requirements-gcs.txt
python gcs/VIOdrift_correction.py --camera 0
python gcs/DroneGUI.py
```

Update `JETSON_IP` in `gcs/DroneGUI.py` for the target Jetson before connecting. The GCS uses TCP port `5005`.

## Hardware note

The ROS scripts require a working ROS 1/MAVROS/PX4 environment and connected flight sensors. The syntax checks in this repository do not replace an actual hardware or simulation flight test.
