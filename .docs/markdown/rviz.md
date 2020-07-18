# Live Visualization with RViz {#rviz}

Robowflex offers a helper class, robowflex::IO::RVIZHelper, which makes it easy to display information in RViz from scripts that use Robowflex.
This document gives a brief tutorial of how you might use RVIZHelper in your scripts.
You can also look at [ur5_visualization.cpp](ur5__visualization_8cpp_source.html) which demonstrates RVIZHelper.

## Using RVIZHelper in a Script

You first need to create your robot and setup ROS:
```cpp
// Startup ROS
ROS ros(argc, argv);

// Create the default UR5 robot.
auto ur5 = std::make_shared<UR5Robot>();
ur5->initialize();
```

With your robot, we can create the helper:
```cpp
IO::RVIZHelper rviz(ur5);
```

This republishes the robot's information under an easily typable name on the parameter server.
By default, the name is `/robowflex/robot_description` and `/robowflex/robot_description_semantic`.
However, you can change this if you are visualizing multiple robots:
```cpp
IO::RVIZHelper rviz(ur5, "ur5");
```

This will publish `/ur5/robot_description` and `/ur5/robot_description_semantic`.

You should probably insert a pause in your program at this point (i.e., `std::cin.get()`, `sleep()`, or something), as you need to setup RViz to look at these topics.

## Setting up RViz

Robowflex namespaces things to avoid conflicting with other ROS happenings, but this means RViz needs a little configuration.
By default, the helper publishes:
- Planning scenes under `/<name>/scenes` with the type `moveit_msgs::PlanningScene`, displayable with the RViz `PlanningScene` widget.
- Trajectories under `/<name>/trajectory` with the type `moveit_msgs::DisplayTrajectory`, displayable with the RViz `Trajectory` widget.
- Markers under the default `/visualization_marker_array` with the type `visualization_msgs::MarkerArray`, displayable with the RViz `MarkerArray` widget.

`PlanningScene` and `Trajectory` need the robot description topic set to whatever you are publishing the robot under (see above).
They also need the topic adjusted to the namespace as well.

Here is an example with the default configuration:
<img style="width:50%;max-width:720px;" src="https://s3.amazonaws.com/zk-bucket/robowflex/rviz.png" />

Now we are ready to display some stuff!

## What you can Display

- Trajectories with `robowflex::IO::RVIZHelper::updateTrajectory()` and `robowflex::IO::RVIZHelper::updateTrajectories()`
- Planning scenes with `robowflex::IO::RVIZHelper::updateScene()` and `robowflex::IO::RVIZHelper::removeScene()`
- [RViz Markers](http://wiki.ros.org/rviz/DisplayTypes/Marker) (a way to show primitive shapes and other information like text or arrows in a scene), which are managed by the helper class so you can easily add and remove named markers:
  - `robowflex::IO::RVIZHelper::addGeometryMarker()` to add any `robowflex::Geometry` as a marker.
  - `robowflex::IO::RVIZHelper::addArrowMarker()` to add arrows.
  - `robowflex::IO::RVIZHelper::addTextMarker()` to add front-facing text.
  - `robowflex::IO::RVIZHelper::addGoalMarker()` to add the goal regions of a motion planning request as markers to the scene.
    Currently only solid primitives (non-mesh) regions are displayed.
    This also visualizes (if available) orientation constraints as arrows.
    One larger arrow for the provided orientation, and 6 smaller arrows corresponding to the boundaries of the tolerances provided along each axis.
    The goal is also labeled by text according to the input name.
    
Note that all markers added through the `add*Marker()` functions will only be shown after `robowflex::IO::RVIZHelper::updateMarkers()` is called.
This function also can be called in a loop to "modify" the markers if they are updating position.
Markers removed with `robowflex::IO::RVIZHelper::removeMarker()` will only be removed from RViz after `updateMarkers()` is called again.
You can have multiple markers under one name, however all markers under that name will be removed by a `removeMarker()` call.
