# Baxter Common

Common Baxter robot description, end-effector description, messages, and services for the ROS Noetic checkout.

## Packages

| Package | Purpose |
|---------|---------|
| `baxter_common` | Metapackage |
| `baxter_description` | Baxter URDF, xacro, meshes, and Gazebo description hooks |
| `baxter_core_msgs` | Baxter custom messages and services |
| `baxter_maintenance_msgs` | Maintenance message definitions |
| `rethink_ee_description` | Electric and pneumatic end-effector descriptions |

## Useful Checks

After building and sourcing the workspace:

```bash
rospack find baxter_description
rosrun xacro xacro $(rospack find baxter_description)/urdf/baxter.urdf.xacro >/tmp/baxter.urdf
```

The CI smoke test expands selected Baxter and end-effector xacro files in Docker.

## Known Warnings

Noetic xacro can print `Child elements of a <xacro:include> tag are ignored` for the top-level Baxter description. The verified Docker and CI launch-parse checks still pass with that warning.
