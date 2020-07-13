# Scripts {#scripts}

A list of provided example scripts that demonstrate some of the functionality of Robowflex.

## robowflex_library

There are many example scripts of how to do motion planning, benchmarking, input / output, and others using the core library.

### UR5 Tests
- [ur5_test.cpp](ur5__test_8cpp_source.html)
A basic test with the UR5 that shows a simple pose-based motion plan with the default motion planner and a motion planner without simplification.

- [ur5_benchmark.cpp](ur5__benchmark_8cpp_source.html)
Default benchmarking with the UR5.

- [ur5_pool.cpp](ur5__pool_8cpp_source.html)
Using robowflex::PoolPlanner for asynchronous motion planning.

- [ur5_visualization.cpp](ur5__visualization_8cpp_source.html)
Demonstration of robowflex::IO::RVIZHelper to display planning in RViz with robowflex.

- [ur5_cylinder.cpp](ur5__cylinder_8cpp_source.html)
A demonstration of robowflex::MotionRequestBuilder::addCylinderSideGrasp(), also visualized with robowflex::IO::RVIZHelper.

- [ur5_io.cpp](ur5__io_8cpp_source.html)
Basic IO testing using the UR5. Shows robowflex::IO::Bag and other IO functions.

### Fetch Tests
- [fetch_test.cpp](fetch__test_8cpp_source.html)
Demonstration of motion planning using the Fetch along with its unique functions.

- [fetch_benchmark.cpp](fetch__benchmark_8cpp_source.html)
Default benchmarking with the Fetch.

- [fetch_visualization.cpp](fetch__visualization_8cpp_source.html)
Demonstration of robowflex::IO::RVIZHelper to display planning in RViz with robowflex.

### Robonaut 2 Tests
- [r2_test.cpp](r2__test_8cpp_source.html)
Demonstration of motion planning from a YAML file and using a robowflex::MotionRequestBuilder for Robonaut 2.
Additionally, shows how to dump a single robot state to an animation file.
By default the motion request builder example is commented out.

- [r2_hdf5.cpp](r2__hdf5_8cpp_source.html)
Using robowflex::R2Robot::loadSMTData to load experiment data for Robonaut 2.

### WAM7 Tests
- [wam7_benchmark.cpp](wam7__benchmark_8cpp_source.html)
Basic benchmarking with the WAM7 arm.

- [wam7_test.cpp](wam7__test_8cpp_source.html)
Basic motion planning with the WAM7 arm.

### Others
- [hdf5_io.cpp](hdf5__io_8cpp_source.html)
Demonstrating robowflex::IO::HDF5File loading of files.

- [plugin_io.cpp](plugin__io_8cpp_source.html)
Demonstrating robowflex::IO::PluginManager loading a plugin.

## robowflex_ompl

- [ur5_ompl_interface.cpp](ur5__ompl__interface_8cpp_source.html)
A basic example of motion planning for the UR5 using the more bare-metal robowflex::OMPL::OMPLInterfacePlanner.

## robowflex_tesseract

- [tesseract_benchmark.cpp](tesseract__benchmark_8cpp_source.html)
An example of benchmarking using the tesseract-based motion planner robowflex::hypercube::OMPLChainPlanner.

- [ur5_tesseract.cpp](ur5__tesseract_8cpp_source.html)
An example of motion planning using the tesseract-based motion planner robowflex::hypercube::OMPLChainPlanner.

## robowflex_movegroup

- [tapedeck.cpp](tapedeck_8cpp_source.html)
A utility script that saves all motion plan requests that go to a `move_group` instance to YAML files. 
Demonstrates the robowflex::movegroup::MoveGroupHelper class.

- [mixtape.cpp](tapedeck_8cpp_source.html)
A utility script that reads motion plan requests from `tapedeck`.
