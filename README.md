# Autonomous Quadcopter

Autonomous quadcopter flight software for a PX4/MAVROS vehicle, with a laptop ground-control station (GCS).

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
