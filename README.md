# TurtleBot2i Navigation and Mapping System Documentation

This documentation explains how the TurtleBot2i robot system works, from the sensors that measure movement to the algorithms that help it navigate autonomously. It covers how odometry, IMU, and the Extended Kalman Filter work together for accurate positioning.

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Wheel Odometry](#wheel-odometry)
3. [Inertial Measurement Unit (IMU)](#inertial-measurement-unit-imu)
4. [Extended Kalman Filter (EKF)](#extended-kalman-filter-ekf)
5. [Mobile Base and Kobuki Hardware](#mobile-base-and-kobuki-hardware)
6. [Kobuki Base Configuration](#kobuki-base-configuration)
7. [Step-by-Step: Using EKF Correctly](#step-by-step-using-ekf-correctly)
8. [Implementation Guide: Setting Up EKF with Kobuki](#implementation-guide-setting-up-ekf-with-kobuki)
9. [Mapping the Environment](#mapping-the-environment)
10. [Navigating with a Known Map](#navigating-with-a-known-map)
11. [Coordinate Systems (TF Frames)](#coordinate-systems-tf-frames)
12. [Troubleshooting](#troubleshooting)

---

## System Overview

The TurtleBot2i robot uses multiple sensors working together to know where it is and how to navigate:

**The Basic Idea:**
- The robot has wheels with sensors that measure movement (wheel encoders)
- The robot has a sensor that measures rotation and acceleration (IMU)
- These sensors are combined together to get a better estimate of position than using just one sensor
- This combined data lets the robot build maps and navigate autonomously

**Why combine sensors?**
- Wheel measurements alone become inaccurate over time due to wheel slipping and small errors accumulating
- The rotation sensor alone also drifts over time
- When combined intelligently, they cancel out each other's weaknesses
- This is called **sensor fusion**

**The Pipeline:**
Sensors → Filter (EKF) → Accurate Position → Mapping or Navigation

The robot first measures with its sensors, then uses a mathematical filter to clean up the data and get a better estimate of its actual position, and finally uses that information for mapping or autonomous navigation.

---

## Wheel Odometry

**What is it?**
Odometry is the robot estimating its position by measuring how far its wheels have moved. The Kobuki mobile base has sensors on each wheel (encoders) that count wheel rotations.

**The Problem:**
- When wheels slip on carpet or slippery surfaces, the encoder doesn't know this happened
- Small measurement errors add up over time
- After moving around for a while, the robot's believed position might be several meters away from its actual position
- This is called "drift"

**Example:**
- Robot commands: "Move 10 meters forward"
- Actual movement: 10 meters (but maybe 10.2 due to wheel slip)
- After many movements, errors compound: 10 total movements with slight errors = 0.5 meters of drift

**What odometry provides:**
The robot knows its X position, Y position, forward/backward speed, and rotation speed—all based on wheel measurements.

**Key Point:** Odometry is useful as a starting estimate, but it's not reliable for long-term positioning without correction from other sensors.

---

## Inertial Measurement Unit (IMU)

**What is an IMU?**
An IMU is a sensor that measures motion. It contains:
- **Accelerometers**: Measure changes in movement (acceleration)
- **Gyroscopes**: Measure rotation speed

For a robot on a flat floor, the IMU is mainly useful for measuring which direction the robot is facing (yaw rotation).

**IMU in This System:**
The TurtleBot2i can use two IMU sources:
1. **Built-in Kobuki IMU**: The Kobuki mobile base has an IMU integrated into it
2. **External IMU (work_arduino_sensors)**: An optional IMU connected via the `imu_finger_node.py` which runs from the `work_arduino_sensors` package. This provides additional IMU data that can be used for better orientation estimation.

**The Problem with IMU Alone:**
Gyroscopes have a problem called "bias drift"—they slowly accumulate errors over time. If the robot sits still for an hour, the gyro might start reporting that it's rotating even though it's not. This is why IMU data alone can't be trusted for long-term orientation tracking.

**Why IMU Helps:**
While wheel odometry drifts over distance (because of slipping), IMU rotation measurements drift over time. By combining both:
- Over short distances, wheel odometry is accurate
- Over short times, IMU rotation is accurate
- Together, they balance each other's weaknesses

---

## Extended Kalman Filter (EKF)

**What is EKF?**
The Extended Kalman Filter is a mathematical algorithm that combines measurements from multiple sensors to get a better estimate of the robot's position and orientation. Think of it like having three friends telling you directions, and you trust the consensus more than any single friend.

**How it Works (Simplified):**
1. **Make a prediction**: Use the last known position and the wheel odometry to guess where the robot should be now
2. **Get new measurements**: Receive data from the IMU about rotation
3. **Compare**: Does the IMU measurement match the prediction? If yes, confidence increases. If no, something's probably wrong.
4. **Adjust**: Update the robot's estimated position based on how well predictions matched measurements
5. **Repeat**: Do this 30 times per second

**The Key Insight:**
The filter doesn't just average the sensors. It's smart about which sensor to trust more in different situations:
- For position changes, it trusts the wheels more (they don't drift over distance)
- For rotation, it trusts the IMU more (it doesn't drift over time)
- It automatically combines them in a way that minimizes total error

**Configuration:**
The behavior of the filter is controlled by parameters in the configuration file. These parameters tell the filter:
- How often to run (frequency)
- Which sensors to use (wheels and/or IMU)
- How much to trust each sensor
- What frame names to use for communication

**Output:**
Instead of trusting raw odometry, the robot uses the filtered position estimate, which is more accurate and stable over time.

---

## Mobile Base and Kobuki Hardware

**What is Kobuki?**
Kobuki is the name of the mobile base—the physical robot platform with wheels, motors, and sensors. It's designed to be simple and reliable for indoor navigation.

**What Kobuki Provides:**
- **Two independent wheels** with built-in motor controllers
- **Wheel encoders** that measure wheel rotation for odometry
- **Built-in IMU** for measuring rotation and acceleration
- **Bumper sensors** to detect collisions
- **Cliff sensors** to prevent the robot from falling off edges
- **Motor control interface** via serial/USB connection

**How Kobuki Communicates:**
The Kobuki base connects to the main computer via serial connection and publishes sensor data:
- Wheel odometry (from encoders)
- IMU measurements (rotation and acceleration)
- Sensor states (bumpers, cliffs, battery)
- Joint state information

**The Kobuki Node:**
A ROS package called `kobuki_node` acts as the driver that:
- Communicates with the Kobuki hardware
- Publishes sensor measurements as ROS topics
- Receives motor velocity commands
- Manages the nodelet architecture (a performance optimization)

**Key Feature: Velocity Multiplexer**
Since multiple programs might want to control the robot's speed, a multiplexer ensures only one command at a time is used. This prevents conflicts when both autonomous navigation and joystick teleop try to command the robot simultaneously.

---

## Kobuki Base Configuration

The Kobuki hardware has many tunable parameters that control how it behaves. These are stored in a configuration file.

**Where to Find It:**
```
$(rospack find kobuki_node)/param/base.yaml
```

**How to Edit:**
```
nano $(rospack find kobuki_node)/param/base.yaml
```

**What's in There:**
The configuration file contains settings such as:
- **Serial port settings**: How to communicate with the hardware
- **PID control parameters**: How quickly the motors respond to commands
- **Acceleration limits**: How fast the robot can speed up or slow down
- **Safety parameters**: Maximum speed, timeout behaviors
- **Sensor configurations**: Which sensors are enabled
- **IMU calibration**: Offsets and scales for accurate measurements
- **Battery thresholds**: When to warn about low battery

**Important Parameters:**
- **Device Port**: Usually `/dev/arduino` and `/dev/ttyACM1` to imu and hokuyo, respectively;
- **Acceleration Profiles**: Affects how smoothly the robot starts and stops;
- **Safety Timeouts**: How long before the robot stops if commands stop arriving;
- **IMU Parameters**: Calibration values for accurate orientation measurement.

**Why This Matters:**
These parameters directly affect how the Kobuki hardware behaves:
- Wrong port setting = robot won't connect at all
- Aggressive acceleration = jerky motion and encoder errors
- Safety timeout too short = robot stops unexpectedly
- IMU calibration off = position estimates become unreliable

**When to Modify:**
- If the robot doesn't respond, check the port setting
- If motion is jerky or unstable, adjust acceleration parameters
- If you notice the robot isn't moving smoothly, look at PID gains
- If adding a heavy payload, consider adjusting acceleration limits

**Best Practice:**
Start with the default configuration provided by the Kobuki developers. Only modify specific parameters if you understand their effects. Make changes one at a time and test thoroughly between changes.

---

## Step-by-Step: Using EKF Correctly

### Quick Start: Launching Mapping with EKF

To start mapping the environment with EKF-fused odometry:

```bash
roslaunch turtlebot2i_bringup turtlebot2i_mapping.launch rviz:=true
```

This single command launches:
- Kobuki mobile base driver (publishes `/odom_raw`)
- IMU sensor driver (publishes `/imu/data_raw`)
- EKF filter (fuses both and publishes `/odometry/filtered`)
- GMapping SLAM (uses `/odometry/filtered` for accurate mapping)
- RViz visualization

**What happens:**
1. The robot connects to the Kobuki base
2. Both raw odometry and IMU data start publishing
3. EKF combines them into filtered odometry
4. GMapping builds a map using the filtered odometry

### Quick Start: Launching Navigation with EKF

To navigate using a pre-built map with EKF-fused odometry:

```bash
roslaunch turtlebot2i_bringup turtlebot2i_navigate.launch rviz:=true
```

This launches:
- Kobuki mobile base driver
- IMU sensor driver
- EKF filter (publishes `/odometry/filtered`)
- Map server (loads your saved map)
- AMCL localization (uses filtered odometry)
- Move Base path planner
- RViz visualization

**What happens:**
1. The robot connects and publishes sensors
2. EKF fuses odometry and IMU
3. AMCL uses the filtered odometry to estimate position in the map
4. Move Base plans and executes paths

### Verifying EKF is Working

**Check that `/odometry/filtered` is publishing:**

```bash
rostopic echo /odometry/filtered -n 5
```

Should show position and orientation data updating at 30 Hz.

**Compare raw vs filtered odometry:**

```bash
# Terminal 1: Raw odometry
rostopic hz /odom_raw

# Terminal 2: Filtered odometry
rostopic hz /odometry/filtered
```

Both should show ~30 Hz. The filtered version should have less noise/jitter.

**Monitor EKF node:**

```bash
rosnode list | grep ekf
```

Should show `/ekf_filter_node` is running.

### Verifying Sensors Are Publishing

Before launching, check that sensors are available:

```bash
# Check raw odometry (from Kobuki encoders)
rostopic echo /odom_raw -n 3

# Check IMU (from MPU-9250)
rostopic echo /imu/data_raw -n 3

# Check laser scanner
rostopic echo /scan -n 3
```

All should show data without errors.

### Confirming EKF is Used by Mapping and Navigation

**For GMapping:**
In the launch file, verify the remap is present:
```xml
<remap from="odom" to="/odometry/filtered"/>
```

**For AMCL:**
In the launch file, verify:
```xml
<remap from="odom" to="/odometry/filtered"/>
```

**Verify in RViz:**
1. Open RViz
2. Add display: Topic → `/tf`
3. Check the transform tree: `map → odom → base_footprint`
4. The `odom → base_footprint` transform should come from `/ekf_filter_node`

### Troubleshooting: EKF Not Publishing

**Symptom:** `/odometry/filtered` topic doesn't exist

**Check:**
1. Is the EKF node running?
   ```bash
   rosnode list | grep ekf_filter_node
   ```

2. Are sensors publishing?
   ```bash
   rostopic list | grep "odom_raw\|imu"
   ```

3. Check for EKF errors:
   ```bash
   roslaunch turtlebot2i_bringup turtlebot2i_mapping.launch --screen
   ```
   Look for error messages about missing topics or configuration

**Fix:**
- Ensure `turtlebot2i_ekf.yaml` exists and is loaded in the launch file
- Verify sensor topics are named correctly (`/odom_raw`, `/imu/data_raw`)
- Check that Kobuki `publish_tf: false` is set (EKF should publish TF)

### Troubleshooting: Mapping Ignores EKF

**Symptom:** GMapping doesn't seem to use filtered odometry

**Check:**
1. Verify GMapping is remapped:
   ```bash
   rosnode info /gmapping
   ```
   Should show `odom` remapped to `/odometry/filtered`

2. Check which odometry GMapping is subscribing to:
   ```bash
   rostopic info /odometry/filtered
   ```
   Subscribers should include `/gmapping`

**Fix:**
- Add this line to the GMapping launch:
  ```xml
  <remap from="odom" to="/odometry/filtered"/>
  ```
- Restart GMapping after adding the remap

### Troubleshooting: Navigation Unstable

**Symptom:** AMCL can't localize or robot drifts

**Check:**
1. Is AMCL using filtered odometry?
   ```bash
   rosnode info /amcl
   ```
   Should show `odom` remapped to `/odometry/filtered`

2. Check EKF covariance (uncertainty):
   ```bash
   rostopic echo /odometry/filtered/pose/covariance -n 1
   ```
   First 3 diagonal values should be small and not growing unbounded

**Fix:**
- Add this line to AMCL launch:
  ```xml
  <remap from="odom" to="/odometry/filtered"/>
  ```
- Set robot's initial position in RViz using "2D Pose Estimate" button
- Ensure IMU is properly connected and publishing data

---

## Implementation Guide: Setting Up EKF with Kobuki

This section provides a detailed, step-by-step guide to configure the Extended Kalman Filter to use Kobuki encoder odometry and MPU-9250 (or other) IMU data for accurate fused odometry.

### Architecture

The data flow is:

```
Kobuki Encoders → /odom_raw ─┐
                             ├→ EKF Filter → /odometry/filtered → GMapping / AMCL
MPU-9250 IMU → /imu/data_raw ─┘
```

The EKF combines wheel encoder data with IMU measurements to produce a cleaner `/odometry/filtered` output, which is then used by mapping and navigation algorithms.

### Configuration Step 1: Disable Kobuki TF and IMU Heading

Edit the Kobuki configuration file:

```
nano $(rospack find kobuki_node)/param/base.yaml
```

Set these parameters:

```yaml
publish_tf: false
use_imu_heading: false
```

**Why?**
- `publish_tf: false` prevents Kobuki from broadcasting the `odom → base_footprint` transform, allowing the EKF to do this instead
- `use_imu_heading: false` tells Kobuki to use encoder-only odometry without mixing in its internal gyro, which would conflict with the external MPU-9250 IMU

### Configuration Step 2: Remap Kobuki Odometry Topic

In your robot launch file (typically `minimal.launch`), remap the Kobuki odometry output:

```xml
<node pkg="nodelet" type="nodelet" name="mobile_base" args="load kobuki_node/KobukiNodelet ...">
  <remap from="mobile_base/odom" to="odom_raw"/>
  <!-- other parameters -->
</node>
```

**Result:**
- Raw encoder odometry is available at `/odom_raw`
- Filtered EKF output will be published at `/odometry/filtered`

### Configuration Step 3: Create EKF Configuration File

Ensure the EKF configuration file exists at:

```
param/turtlebot2i_ekf.yaml
```

Use this configuration:

```yaml
ekf_filter_node:
  # Operational parameters
  frequency: 30                    # Run filter at 30 Hz
  sensor_timeout: 0.1              # Stop if sensors fail for 100 ms
  two_d_mode: true                 # 2D robot on flat surface

  # Frame definitions
  map_frame: map
  odom_frame: odom
  base_link_frame: base_footprint
  world_frame: odom                # Use odometry frame as world reference
  publish_tf: true                 # Publish odom → base_footprint transform

  # Wheel encoder odometry input
  odom0: /odom_raw
  odom0_config: [true,  true,  false,    # X, Y position (enabled)
                 false, false, false,     # Roll, Pitch, Yaw (disabled)
                 true,  false, false,     # X velocity (enabled)
                 false, false, false,     # Angular velocity (disabled)
                 false, false, false]     # Acceleration (disabled)
  odom0_queue_size: 10
  odom0_differential: false
  odom0_relative: false

  # IMU input (e.g., MPU-9250)
  imu0: /imu/data_raw
  imu0_config: [false, false, false,     # Position (disabled)
                false, false, true,      # Yaw orientation (enabled)
                false, false, false,     # Linear velocity (disabled)
                false, false, true,      # Yaw angular velocity (enabled)
                false, false, false]     # Acceleration (disabled)
  imu0_queue_size: 10
  imu0_differential: false
  imu0_relative: true                   # Critical: relative measurements
  imu0_remove_gravitational_acceleration: true

  use_control: false                    # Don't use velocity commands for prediction
```

**Key Configuration Choices:**

| Setting | Value | Reason |
|---------|-------|--------|
| `imu0_relative: true` | Treat IMU as rate measurement, not absolute heading | Avoids conflicts with AMCL and 2D Pose Estimate corrections |
| `odom0_config` X,Y enabled | Use wheel encoder position | Encoders are accurate for short distances |
| `imu0_config` yaw enabled | Use IMU rotation rate | Gyros are accurate for short times |
| `two_d_mode: true` | Ignore roll and pitch | Robot operates on flat floors |
| `frequency: 30` | Match sensor publish rate | Good balance of responsiveness and computation |

### Configuration Step 4: Load EKF Parameters in Launch File

In your mapping or navigation launch file, load the EKF parameters **before** starting the node:

```xml
<!-- Load EKF configuration parameters -->
<rosparam file="$(find turtlebot2i_bringup)/param/turtlebot2i_ekf.yaml" command="load"/>

<!-- Start the EKF filter node -->
<node pkg="robot_localization" type="ekf_localization_node" name="ekf_filter_node" output="screen"/>
```

**Important:** Parameters must be loaded before the node starts, otherwise the node runs with defaults and ignores your configuration.

### Configuration Step 5: Remap GMapping and AMCL to Use Filtered Odometry

In your mapping launch file:

```xml
<!-- GMapping should use filtered odometry -->
<include file="$(find turtlebot2i_bringup)/launch/gmapping.launch">
  <arg name="odom_topic" value="/odometry/filtered"/>
</include>
```

In your navigation launch file:

```xml
<!-- AMCL should use filtered odometry -->
<node pkg="amcl" type="amcl" name="amcl">
  <remap from="odom" to="/odometry/filtered"/>
  <!-- other parameters -->
</node>
```

This ensures localization algorithms work with the fused odometry, not raw wheel data.

### Configuration Step 6: Verify the Transform Tree

Check that the EKF is properly broadcasting the `odom → base_footprint` transform:

```bash
rosrun tf tf_monitor
```

You should see:

```
Node: /ekf_filter_node
Average rate: 30 Hz
```

**Warning:** If you see both `/kobuki_node` and `/ekf_filter_node` publishing `odom → base_footprint`, you have competing broadcasters. Ensure `publish_tf: false` is set in the Kobuki configuration.

### Configuration Step 7: Verify Odometry Topics

List available odometry topics:

```bash
rostopic list | grep odom
```

Expected output:

```
/odom_raw
/odometry/filtered
```

### Configuration Step 8: Validate the Setup

Keep the robot completely still and check that the filtered odometry remains stable:

```bash
rostopic echo /odometry/filtered
```

Expected behavior:
- Position values should remain nearly constant
- Small millimeter-level noise is normal
- Continuous drift indicates a configuration error

### Quick Verification Checklist

- [ ] Kobuki `publish_tf: false` is set
- [ ] Kobuki `use_imu_heading: false` is set
- [ ] Kobuki odometry remapped to `/odom_raw`
- [ ] EKF configuration loaded before node starts
- [ ] GMapping remapped to `/odometry/filtered`
- [ ] AMCL remapped to `/odometry/filtered`
- [ ] TF tree shows only one `/ekf_filter_node` publishing `odom → base_footprint`
- [ ] `/odom_raw` and `/odometry/filtered` topics both exist
- [ ] Stationary robot shows stable position in `/odometry/filtered`

### Common Issues and Fixes

| Issue | Cause | Fix |
|-------|-------|-----|
| Robot teleports after 2D Pose Estimate | `imu0_relative: false` | Set `imu0_relative: true` in EKF config |
| Two TF broadcasters for `odom → base_footprint` | Kobuki still publishing TF | Set `publish_tf: false` in Kobuki config |
| Odometry drifts while robot is stationary | Kobuki internal gyro still active | Set `use_imu_heading: false` in Kobuki config |
| GMapping ignores filtered odometry | Missing remap | Add `<remap from="odom" to="/odometry/filtered"/>` |
| Robot moves diagonally | IMU yaw conflicting with encoder heading | Verify `imu0_relative: true` |
| AMCL localization unstable | Reading raw odometry | Remap AMCL to use `/odometry/filtered` |
| EKF publishes zero values | Incorrect topic names or YAML syntax | Check launch order and parameter names |

---

## Mapping the Environment

**What is Mapping?**
Mapping is the process of the robot exploring an environment and building a digital map of it. The map shows which areas are empty space and which areas have obstacles.

**How It Works:**
1. The robot moves around and continuously scans with its laser
2. For each laser scan, the system knows where the laser is located (from EKF-fused odometry)
3. The system records where obstacles are relative to the laser location
4. All scans are combined together to create an overall map
5. When the robot revisits a familiar area, the system recognizes it and corrects any accumulated odometry drift

**Why Use EKF for Mapping?**
- Better odometry = better map alignment
- The robot knows more accurately where each scan came from
- Fewer errors in the final map

**Mapping Process:**
- The robot should move slowly so the laser has time to match features between scans
- The robot should explore the entire environment
- The robot should revisit areas to create loop closures (when the system recognizes a familiar spot)

**What Gets Saved:**
The map is saved as:
- An image file showing the occupancy grid (black = obstacles, white = free space, gray = unknown)
- A text file with map metadata (resolution, origin, threshold values)

---

## Navigating with a Known Map

**What is Navigation?**
Navigation is when the robot uses a pre-built map to figure out where it is and move to a desired location. Unlike mapping (which builds the map), navigation assumes the map already exists.

**The Navigation Process:**
1. The robot starts at an approximate known location
2. The robot compares its laser scans to the known map
3. The system estimates exactly where the robot is in the map
4. A path planner figures out how to reach the goal location
5. A motion controller sends velocity commands to move along that path

**Why Use EKF for Navigation?**
- Better odometry helps the localization algorithm converge faster
- The robot knows more accurately where it is
- Path planning is more reliable

**Key Difference from Mapping:**
- Mapping: The robot doesn't know the map beforehand and builds it
- Navigation: The robot knows the map and uses it to localize

**Navigation Steps:**
1. Load a previously saved map
2. Tell the robot approximately where it starts
3. The system automatically refines the exact position by comparing scans to map
4. Send goal locations and the robot navigates there automatically

**What Happens During Navigation:**
- The EKF fuses wheel and IMU data to predict where the robot should be
- The laser scanner is compared against the known map
- Any difference between prediction and map comparison is used to correct position
- The path planner continuously adjusts the path if obstacles are in the way
- The velocity controller sends smooth commands to reach waypoints

---

## Coordinate Systems (TF Frames)

**What are Coordinate Frames?**
The robot operates within multiple coordinate systems. For example:
- The **map frame** is the global coordinate system of the environment
- The **robot frame** is the coordinate system at the robot's center
- The **laser frame** is the coordinate system at the laser scanner

These frames are connected by transforms, which describe the position and orientation relationship between them.

**The Coordinate Frame Hierarchy:**
- `map`: The global, fixed coordinate system (doesn't move)
- `odom`: The coordinate system that moves with the robot based on odometry
- `base_footprint`: The center point of the robot on the floor
- `base_link`: The robot's body frame
- `laser_link`: Where the laser scanner is mounted
- `imu_link`: Where the IMU sensor is located

**Key Relationships:**
- The `map -> odom` transform is computed by mapping algorithms (like GMapping during mapping, or AMCL during navigation). This corrects for accumulated odometry drift.
- The `odom -> base_footprint` transform is computed by the EKF. This is based on wheel encoders and IMU.
- The `base_footprint -> laser_link` and other sensor links are fixed transforms defined in the URDF (the robot description).

**Why This Matters:**
When the robot needs to know where something is (like when it wants to move to a target), it must convert coordinates from one frame to another using these transforms. If any transform is wrong, the robot will think things are in the wrong location.

## Troubleshooting

**General Debugging Approach:**
1. Check if the robot is powered on and connected
2. Verify sensors are publishing data (check sensor topics)
3. Look at the console output for error messages
4. Monitor the EKF output to see if it's producing filtered data
5. Use visualization tools to see what the robot perceives

**Common Issues:**

**The robot doesn't move or respond to commands:**
- The Kobuki base may not be connected to the computer
- The serial port might be wrong in the configuration
- Check that you've launched the mobile base software

**The filtered odometry isn't being published:**
- The raw sensors might not be publishing data
- The EKF configuration file might have errors
- Sensor data might be arriving too slowly (timeout issue)

**Robot position jumps around in the map visualization:**
- This is normal when the mapping system detects it's revisited a previously mapped area and corrects the map
- If jumps are extremely large and frequent, check that the laser scanner has good quality data

**The robot can't navigate to a goal:**
- The map might not match the actual environment
- The robot's initial position in the map might be wrong
- The path planning algorithm might not be finding a path

**Robot motion is jerky or unstable:**
- The Kobuki base might have incorrect acceleration settings
- The ground surface might be slippery (wheel slipping)
- Check that the IMU is properly calibrated

**The laser scans don't align with the map:**
- The laser might not be mounted in the location the system thinks it is
- The robot coordinate system (URDF) might be incorrect
- The transforms between frames might be wrong

**Best Debugging Practice:**
Start simple—first verify the base hardware works, then add the EKF, then add mapping or navigation. This way you can isolate which component has the problem.

