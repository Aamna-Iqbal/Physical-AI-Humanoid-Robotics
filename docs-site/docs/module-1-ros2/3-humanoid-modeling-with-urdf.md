---
sidebar_position: 3
---

# Humanoid Modeling with URDF

This chapter introduces the Unified Robot Description Format (URDF), an XML format used in ROS to describe all elements of a robot model. We will cover the basic components of a URDF file, including links, joints, and sensors.

## What is URDF?

URDF is a standard format for representing a robot model in ROS. It describes the physical structure of a robot, including its links, joints, and their relationships. This information is used by various ROS packages for tasks like simulation, visualization, and motion planning.

## Core Components

### Links

**Links** are the rigid parts of the robot. They have physical properties like mass, inertia, and a collision geometry. Each link has a visual representation, which is what you see in a simulator or visualizer, and a collision geometry, which is used for collision detection.

### Joints

**Joints** connect the links together. They define how the links can move relative to each other. There are several types of joints:

-   **revolute**: A rotational joint with a single axis of rotation.
-   **continuous**: A revolute joint with no angle limits.
-   **prismatic**: A translational joint with a single axis of translation.
-   **fixed**: A joint that does not allow any motion.
-   **floating**: A joint that allows motion in all 6 degrees of freedom.
-   **planar**: A joint that allows motion in a 2D plane.

### Sensors

Sensors, like cameras and laser scanners, can also be included in a URDF file. They are attached to a link and have their own properties, such as the sensor's type, update rate, and field of view.

## Example URDF File

Here is an example of a basic URDF file for a simple two-link robot arm.

```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Arm Link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <box size="0.4 0.1 0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.4 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Joint -->
  <joint name="arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.3"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="3"/>
  </joint>

</robot>
```