<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<tagfile>
  <compound kind="file">
    <name>design.md</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_doc/doc/markdown/</path>
    <filename>design_8md</filename>
  </compound>
  <compound kind="file">
    <name>home.md</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_doc/doc/markdown/</path>
    <filename>home_8md</filename>
  </compound>
  <compound kind="file">
    <name>scripts.md</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_doc/doc/markdown/</path>
    <filename>scripts_8md</filename>
  </compound>
  <compound kind="file">
    <name>benchmarking.h</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/include/robowflex_library/</path>
    <filename>benchmarking_8h</filename>
    <includes id="class__forward_8h" name="class_forward.h" local="no" imported="no">robowflex_library/class_forward.h</includes>
    <includes id="bag_8h" name="bag.h" local="no" imported="no">robowflex_library/io/bag.h</includes>
    <class kind="class">robowflex::Benchmarker</class>
    <class kind="class">robowflex::Benchmarker::Options</class>
    <class kind="class">robowflex::Benchmarker::Results</class>
    <class kind="class">robowflex::Benchmarker::Results::Run</class>
    <class kind="class">robowflex::Benchmarker::Results::Run::toString</class>
    <class kind="class">robowflex::BenchmarkOutputter</class>
    <class kind="class">robowflex::JSONBenchmarkOutputter</class>
    <class kind="class">robowflex::TrajectoryBenchmarkOutputter</class>
    <class kind="class">robowflex::OMPLBenchmarkOutputter</class>
    <namespace>robowflex</namespace>
  </compound>
  <compound kind="file">
    <name>class_forward.h</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/include/robowflex_library/</path>
    <filename>class__forward_8h</filename>
    <member kind="define">
      <type>#define</type>
      <name>ROBOWFLEX_CLASS_FORWARD</name>
      <anchorfile>class__forward_8h.html</anchorfile>
      <anchor>a2cb5ebc2437dd62e6f96ef92a4ec2b4d</anchor>
      <arglist>(C)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>fetch.h</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/include/robowflex_library/detail/</path>
    <filename>fetch_8h</filename>
    <includes id="robot_8h" name="robot.h" local="no" imported="no">robowflex_library/robot.h</includes>
    <includes id="planning_8h" name="planning.h" local="no" imported="no">robowflex_library/planning.h</includes>
    <class kind="class">robowflex::FetchRobot</class>
    <class kind="class">robowflex::OMPL::FetchOMPLPipelinePlanner</class>
    <namespace>robowflex</namespace>
    <namespace>robowflex::OMPL</namespace>
  </compound>
  <compound kind="file">
    <name>r2.h</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/include/robowflex_library/detail/</path>
    <filename>r2_8h</filename>
    <includes id="robot_8h" name="robot.h" local="no" imported="no">robowflex_library/robot.h</includes>
    <includes id="planning_8h" name="planning.h" local="no" imported="no">robowflex_library/planning.h</includes>
    <class kind="class">robowflex::R2Robot</class>
    <class kind="class">robowflex::OMPL::R2OMPLPipelinePlanner</class>
    <namespace>robowflex</namespace>
    <namespace>robowflex::OMPL</namespace>
  </compound>
  <compound kind="file">
    <name>ur5.h</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/include/robowflex_library/detail/</path>
    <filename>ur5_8h</filename>
    <includes id="robot_8h" name="robot.h" local="no" imported="no">robowflex_library/robot.h</includes>
    <includes id="planning_8h" name="planning.h" local="no" imported="no">robowflex_library/planning.h</includes>
    <class kind="class">robowflex::UR5Robot</class>
    <class kind="class">robowflex::OMPL::UR5OMPLPipelinePlanner</class>
    <namespace>robowflex</namespace>
    <namespace>robowflex::OMPL</namespace>
  </compound>
  <compound kind="file">
    <name>geometry.h</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/include/robowflex_library/</path>
    <filename>geometry_8h</filename>
    <includes id="class__forward_8h" name="class_forward.h" local="no" imported="no">robowflex_library/class_forward.h</includes>
    <class kind="class">robowflex::Geometry</class>
    <class kind="class">robowflex::Geometry::ShapeType</class>
    <namespace>robowflex</namespace>
  </compound>
  <compound kind="file">
    <name>io.h</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/include/robowflex_library/</path>
    <filename>io_8h</filename>
    <namespace>robowflex</namespace>
    <namespace>robowflex::IO</namespace>
    <member kind="function">
      <type>const std::string</type>
      <name>resolvePackage</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a8eeeed7331bc650b06edd7303bffdaee</anchor>
      <arglist>(const std::string &amp;path)</arglist>
    </member>
    <member kind="function">
      <type>const std::string</type>
      <name>resolvePath</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a826655a9fb6eb0e25f130038bd6d7b3e</anchor>
      <arglist>(const std::string &amp;path)</arglist>
    </member>
    <member kind="function">
      <type>const std::string</type>
      <name>loadXMLToString</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a5dd541346890d73d822ef60dd2f613a1</anchor>
      <arglist>(const std::string &amp;path)</arglist>
    </member>
    <member kind="function">
      <type>const std::string</type>
      <name>loadXacroToString</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a25f75b885a3fa3f1394d5635d1a39c0e</anchor>
      <arglist>(const std::string &amp;path)</arglist>
    </member>
    <member kind="function">
      <type>const std::string</type>
      <name>loadFileToString</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a88a45ee4c89766f0c209b379b1beb82a</anchor>
      <arglist>(const std::string &amp;path)</arglist>
    </member>
    <member kind="function">
      <type>const std::string</type>
      <name>runCommand</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>aabd3ba6574462927b5aac495a6a1e032</anchor>
      <arglist>(const std::string &amp;cmd)</arglist>
    </member>
    <member kind="function">
      <type>const std::pair&lt; bool, YAML::Node &gt;</type>
      <name>loadFileToYAML</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a5dc8b1061b726b65ba86762af283f776</anchor>
      <arglist>(const std::string &amp;path)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>createFile</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>ad82de1798068dd05831c5eb168fe285e</anchor>
      <arglist>(std::ofstream &amp;out, const std::string &amp;file)</arglist>
    </member>
    <member kind="function">
      <type>const std::pair&lt; bool, std::vector&lt; std::string &gt; &gt;</type>
      <name>listDirectory</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a7bce336f927ad6de91f66790fdf9efab</anchor>
      <arglist>(const std::string &amp;directory)</arglist>
    </member>
    <member kind="function">
      <type>const std::string</type>
      <name>getHostname</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>aef6d4bc3a2b8ab2c6b2385d936f3a2b5</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>boost::posix_time::ptime</type>
      <name>getDate</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a407d1992227eef3b0031d5c8e74aa278</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>tokenize</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>aad1c0d0a27f1920ec75babb52490faac</anchor>
      <arglist>(const std::string &amp;string, const std::string &amp;separators)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>YAMLToFile</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a14b1fd382262138158a5adbad8691ea0</anchor>
      <arglist>(const YAML::Node &amp;node, const std::string &amp;file)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>messageToYAMLFile</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>ab412be03d8a95eb75be650509931ab80</anchor>
      <arglist>(T &amp;msg, const std::string &amp;file)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>YAMLFileToMessage</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>afc0e96b30b4528708e238b3635addda7</anchor>
      <arglist>(T &amp;msg, const std::string &amp;file)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>bag.h</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/include/robowflex_library/io/</path>
    <filename>bag_8h</filename>
    <includes id="macros_8h" name="macros.h" local="no" imported="no">robowflex_library/macros.h</includes>
    <class kind="class">robowflex::IO::Bag</class>
    <namespace>robowflex</namespace>
    <namespace>robowflex::IO</namespace>
  </compound>
  <compound kind="file">
    <name>handler.h</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/include/robowflex_library/io/</path>
    <filename>handler_8h</filename>
    <class kind="class">robowflex::IO::Handler</class>
    <namespace>robowflex</namespace>
    <namespace>robowflex::IO</namespace>
  </compound>
  <compound kind="file">
    <name>hdf5.h</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/include/robowflex_library/io/</path>
    <filename>hdf5_8h</filename>
    <includes id="class__forward_8h" name="class_forward.h" local="no" imported="no">robowflex_library/class_forward.h</includes>
    <class kind="class">robowflex::IO::HDF5Data</class>
    <class kind="class">robowflex::IO::HDF5File</class>
    <namespace>robowflex</namespace>
    <namespace>robowflex::IO</namespace>
  </compound>
  <compound kind="file">
    <name>visualization.h</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/include/robowflex_library/io/</path>
    <filename>visualization_8h</filename>
    <includes id="class__forward_8h" name="class_forward.h" local="no" imported="no">robowflex_library/class_forward.h</includes>
    <class kind="class">robowflex::IO::RVIZHelper</class>
    <namespace>robowflex</namespace>
    <namespace>robowflex::IO</namespace>
  </compound>
  <compound kind="file">
    <name>yaml.h</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/include/robowflex_library/io/</path>
    <filename>io_2yaml_8h</filename>
    <namespace>robowflex</namespace>
    <namespace>robowflex::IO</namespace>
    <member kind="function">
      <type>bool</type>
      <name>isNode</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a23927fb72a17ab8ce0d0654d06b5532e</anchor>
      <arglist>(const YAML::Node &amp;node)</arglist>
    </member>
    <member kind="function">
      <type>YAML::Node</type>
      <name>toNode</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>aed4974f62a649f4a6107ac3d6e919b33</anchor>
      <arglist>(const geometry_msgs::Pose &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>YAML::Node</type>
      <name>toNode</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a819245f223ff0eb589ad28aa153f2485</anchor>
      <arglist>(const moveit_msgs::PlanningScene &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>YAML::Node</type>
      <name>toNode</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a799d8a2989dee49fff0dd91d17a883be</anchor>
      <arglist>(const moveit_msgs::MotionPlanRequest &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>YAML::Node</type>
      <name>toNode</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a414861fda8e5df5e954e0f65a97b2ed7</anchor>
      <arglist>(const moveit_msgs::RobotTrajectory &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>fromYAMLFile</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a50e952949006a40e6b7b5d589958e01d</anchor>
      <arglist>(moveit_msgs::PlanningScene &amp;msg, const std::string &amp;file)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>fromYAMLFile</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a0500ed90fd25e45a7a0476fee615bcc3</anchor>
      <arglist>(moveit_msgs::MotionPlanRequest &amp;msg, const std::string &amp;file)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>yaml.h</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/include/robowflex_library/</path>
    <filename>yaml_8h</filename>
    <includes id="macros_8h" name="macros.h" local="no" imported="no">robowflex_library/macros.h</includes>
  </compound>
  <compound kind="file">
    <name>macros.h</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/include/robowflex_library/</path>
    <filename>macros_8h</filename>
    <member kind="define">
      <type>#define</type>
      <name>ROBOWFLEX_AT_LEAST_INDIGO</name>
      <anchorfile>macros_8h.html</anchorfile>
      <anchor>aecb14573ba0e2696283d4a8c9c8dac93</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ROBOWFLEX_AT_LEAST_LUNAR</name>
      <anchorfile>macros_8h.html</anchorfile>
      <anchor>a68fdc5803aa8bfeb77b4a5df3173d5d3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ROBOWFLEX_AT_LEAST_KINETIC</name>
      <anchorfile>macros_8h.html</anchorfile>
      <anchor>aa6c58faad24f1328810b04d4e061c64d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ROBOWFLEX_AT_LEAST_MELODIC</name>
      <anchorfile>macros_8h.html</anchorfile>
      <anchor>a1a9ac25ecfa55309b79ab3532331a6b5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>IS_BOOST_164</name>
      <anchorfile>macros_8h.html</anchorfile>
      <anchor>ae0471ce73063b1fef4c45b2929d1b261</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ROBOWFLEX_YAML_FLOW</name>
      <anchorfile>macros_8h.html</anchorfile>
      <anchor>a8d80e8c7c7c78cc84ab0f7af6cf2634c</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ROBOWFLEX_PRAGMA_HELPER0</name>
      <anchorfile>macros_8h.html</anchorfile>
      <anchor>a55c06931fdef32dd79bdc8f933e21f25</anchor>
      <arglist>(x)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ROBOWFLEX_PRAGMA_HELPER1</name>
      <anchorfile>macros_8h.html</anchorfile>
      <anchor>a88db03a6bcf9fef77c4ba3b38dcd5a4a</anchor>
      <arglist>(x, y)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ROBOWFLEX_PUSH_DISABLE_GCC_WARNING</name>
      <anchorfile>macros_8h.html</anchorfile>
      <anchor>a0f0536510815f9b8c1eb554ae2f07ee4</anchor>
      <arglist>(warning)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ROBOWFLEX_POP_GCC</name>
      <anchorfile>macros_8h.html</anchorfile>
      <anchor>a335b9b2561e97ed978f5d3c263463b17</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ROBOWFLEX_PUSH_DISABLE_CLANG_WARNING</name>
      <anchorfile>macros_8h.html</anchorfile>
      <anchor>a14207782048abde18f6d27676929dd89</anchor>
      <arglist>(warning)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ROBOWFLEX_POP_CLANG</name>
      <anchorfile>macros_8h.html</anchorfile>
      <anchor>aaab1cfda5263c3e5180b2980ab3d406b</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>planning.h</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/include/robowflex_library/</path>
    <filename>planning_8h</filename>
    <includes id="class__forward_8h" name="class_forward.h" local="no" imported="no">robowflex_library/class_forward.h</includes>
    <includes id="pool_8h" name="pool.h" local="no" imported="no">robowflex_library/pool.h</includes>
    <includes id="handler_8h" name="handler.h" local="no" imported="no">robowflex_library/io/handler.h</includes>
    <class kind="class">robowflex::Planner</class>
    <class kind="class">robowflex::PoolPlanner</class>
    <class kind="class">robowflex::PipelinePlanner</class>
    <class kind="class">robowflex::MotionRequestBuilder</class>
    <class kind="class">robowflex::OMPL::Settings</class>
    <class kind="class">robowflex::OMPL::OMPLPipelinePlanner</class>
    <namespace>robowflex</namespace>
    <namespace>robowflex::OMPL</namespace>
    <member kind="function">
      <type>std::map&lt; std::string, double &gt;</type>
      <name>getFinalJointPositions</name>
      <anchorfile>namespacerobowflex.html</anchorfile>
      <anchor>acf413cf9ebc78076a09d96a90fade076</anchor>
      <arglist>(const planning_interface::MotionPlanResponse &amp;response)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>loadOMPLConfig</name>
      <anchorfile>namespacerobowflex_1_1OMPL.html</anchorfile>
      <anchor>a31492e70aba96a5bf04e48eb9840f6be</anchor>
      <arglist>(IO::Handler &amp;handler, const std::string &amp;config_file, std::vector&lt; std::string &gt; &amp;configs)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>pool.h</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/include/robowflex_library/</path>
    <filename>pool_8h</filename>
    <class kind="class">robowflex::Pool</class>
    <class kind="class">robowflex::Pool::Joblet</class>
    <class kind="class">robowflex::Pool::Job</class>
    <namespace>robowflex</namespace>
  </compound>
  <compound kind="file">
    <name>robot.h</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/include/robowflex_library/</path>
    <filename>robot_8h</filename>
    <includes id="class__forward_8h" name="class_forward.h" local="no" imported="no">robowflex_library/class_forward.h</includes>
    <includes id="handler_8h" name="handler.h" local="no" imported="no">robowflex_library/io/handler.h</includes>
    <class kind="class">robowflex::Robot</class>
    <class kind="class">robowflex::ParamRobot</class>
    <namespace>robowflex</namespace>
  </compound>
  <compound kind="file">
    <name>robowflex.h</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/include/robowflex_library/</path>
    <filename>robowflex_8h</filename>
    <includes id="class__forward_8h" name="class_forward.h" local="no" imported="no">robowflex_library/class_forward.h</includes>
    <includes id="macros_8h" name="macros.h" local="no" imported="no">robowflex_library/macros.h</includes>
    <includes id="util_8h" name="util.h" local="no" imported="no">robowflex_library/util.h</includes>
    <includes id="yaml_8h" name="yaml.h" local="no" imported="no">robowflex_library/yaml.h</includes>
    <includes id="io_8h" name="io.h" local="no" imported="no">robowflex_library/io.h</includes>
    <includes id="bag_8h" name="bag.h" local="no" imported="no">robowflex_library/io/bag.h</includes>
    <includes id="handler_8h" name="handler.h" local="no" imported="no">robowflex_library/io/handler.h</includes>
    <includes id="visualization_8h" name="visualization.h" local="no" imported="no">robowflex_library/io/visualization.h</includes>
    <includes id="io_2yaml_8h" name="yaml.h" local="no" imported="no">robowflex_library/io/yaml.h</includes>
    <includes id="pool_8h" name="pool.h" local="no" imported="no">robowflex_library/pool.h</includes>
    <includes id="geometry_8h" name="geometry.h" local="no" imported="no">robowflex_library/geometry.h</includes>
    <includes id="tf_8h" name="tf.h" local="no" imported="no">robowflex_library/tf.h</includes>
    <includes id="scene_8h" name="scene.h" local="no" imported="no">robowflex_library/scene.h</includes>
    <includes id="robot_8h" name="robot.h" local="no" imported="no">robowflex_library/robot.h</includes>
    <includes id="planning_8h" name="planning.h" local="no" imported="no">robowflex_library/planning.h</includes>
    <includes id="benchmarking_8h" name="benchmarking.h" local="no" imported="no">robowflex_library/benchmarking.h</includes>
    <namespace>robowflex</namespace>
  </compound>
  <compound kind="file">
    <name>scene.h</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/include/robowflex_library/</path>
    <filename>scene_8h</filename>
    <includes id="class__forward_8h" name="class_forward.h" local="no" imported="no">robowflex_library/class_forward.h</includes>
    <class kind="class">robowflex::Scene</class>
    <namespace>robowflex</namespace>
  </compound>
  <compound kind="file">
    <name>tf.h</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/include/robowflex_library/</path>
    <filename>tf_8h</filename>
    <includes id="class__forward_8h" name="class_forward.h" local="no" imported="no">robowflex_library/class_forward.h</includes>
    <namespace>robowflex</namespace>
    <namespace>robowflex::TF</namespace>
    <member kind="function">
      <type>Eigen::Vector3d</type>
      <name>vectorMsgToEigen</name>
      <anchorfile>namespacerobowflex_1_1TF.html</anchorfile>
      <anchor>a1ed7fbaac296f875a3dfc136ff80bc99</anchor>
      <arglist>(const geometry_msgs::Vector3 &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>geometry_msgs::Vector3</type>
      <name>vectorEigenToMsg</name>
      <anchorfile>namespacerobowflex_1_1TF.html</anchorfile>
      <anchor>a1708cf70a26bb21cbdeea7df8e7dec9b</anchor>
      <arglist>(const Eigen::Vector3d &amp;vector)</arglist>
    </member>
    <member kind="function">
      <type>Eigen::Affine3d</type>
      <name>poseMsgToEigen</name>
      <anchorfile>namespacerobowflex_1_1TF.html</anchorfile>
      <anchor>af53d77db21f3736f386c805cbbdd37ba</anchor>
      <arglist>(const geometry_msgs::Pose &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>geometry_msgs::Pose</type>
      <name>poseEigenToMsg</name>
      <anchorfile>namespacerobowflex_1_1TF.html</anchorfile>
      <anchor>a306bde67c676d1879f5579c96f4f0a24</anchor>
      <arglist>(const Eigen::Affine3d &amp;pose)</arglist>
    </member>
    <member kind="function">
      <type>Eigen::Quaterniond</type>
      <name>quaternionMsgToEigen</name>
      <anchorfile>namespacerobowflex_1_1TF.html</anchorfile>
      <anchor>aa36db77a74d939b2d1c6eac8579f6921</anchor>
      <arglist>(const geometry_msgs::Quaternion &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>geometry_msgs::Quaternion</type>
      <name>quaternionEigenToMsg</name>
      <anchorfile>namespacerobowflex_1_1TF.html</anchorfile>
      <anchor>ae84c5fbe81f21d64a9caa67bad89563d</anchor>
      <arglist>(const Eigen::Quaterniond &amp;quaternion)</arglist>
    </member>
    <member kind="function">
      <type>moveit_msgs::BoundingVolume</type>
      <name>getBoundingVolume</name>
      <anchorfile>namespacerobowflex_1_1TF.html</anchorfile>
      <anchor>a69a9ae40884d2c53e418bd56b80bfd1a</anchor>
      <arglist>(const Eigen::Affine3d &amp;pose, const GeometryConstPtr &amp;geometry)</arglist>
    </member>
    <member kind="function">
      <type>moveit_msgs::PositionConstraint</type>
      <name>getPositionConstraint</name>
      <anchorfile>namespacerobowflex_1_1TF.html</anchorfile>
      <anchor>a3fb881f1dd59704d6f6237825161774f</anchor>
      <arglist>(const std::string &amp;ee_name, const std::string &amp;base_name, const Eigen::Affine3d &amp;pose, const GeometryConstPtr &amp;geometry)</arglist>
    </member>
    <member kind="function">
      <type>moveit_msgs::OrientationConstraint</type>
      <name>getOrientationConstraint</name>
      <anchorfile>namespacerobowflex_1_1TF.html</anchorfile>
      <anchor>a1519d7a4836023d69ae0cd587a527906</anchor>
      <arglist>(const std::string &amp;ee_name, const std::string &amp;base_name, const Eigen::Quaterniond &amp;orientation, const Eigen::Vector3d &amp;tolerances)</arglist>
    </member>
    <member kind="function">
      <type>Eigen::Quaterniond</type>
      <name>sampleOrientation</name>
      <anchorfile>namespacerobowflex_1_1TF.html</anchorfile>
      <anchor>a6e8c3dcd0a542c8923fe4eea954d6948</anchor>
      <arglist>(const Eigen::Quaterniond &amp;orientation, const Eigen::Vector3d &amp;tolerances)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>util.h</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/include/robowflex_library/</path>
    <filename>util_8h</filename>
    <class kind="class">robowflex::Exception</class>
    <class kind="class">robowflex::ROS</class>
    <namespace>robowflex</namespace>
    <member kind="function">
      <type>void</type>
      <name>explode</name>
      <anchorfile>namespacerobowflex.html</anchorfile>
      <anchor>aa3c6c6a0a52b8ce690f26c87aad1103e</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>fetch_test.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/scripts/</path>
    <filename>fetch__test_8cpp</filename>
    <includes id="util_8h" name="util.h" local="no" imported="no">robowflex_library/util.h</includes>
    <includes id="robot_8h" name="robot.h" local="no" imported="no">robowflex_library/robot.h</includes>
    <includes id="scene_8h" name="scene.h" local="no" imported="no">robowflex_library/scene.h</includes>
    <includes id="planning_8h" name="planning.h" local="no" imported="no">robowflex_library/planning.h</includes>
    <includes id="fetch_8h" name="fetch.h" local="no" imported="no">robowflex_library/detail/fetch.h</includes>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>fetch__test_8cpp.html</anchorfile>
      <anchor>a3c04138a5bfe5d72780bb7e82a18e627</anchor>
      <arglist>(int argc, char **argv)</arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static const std::string</type>
      <name>GROUP</name>
      <anchorfile>fetch__test_8cpp.html</anchorfile>
      <anchor>a892b6dd5a49224f3e7646b914c4462fb</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>hdf5_io.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/scripts/</path>
    <filename>hdf5__io_8cpp</filename>
    <includes id="hdf5_8h" name="hdf5.h" local="no" imported="no">robowflex_library/io/hdf5.h</includes>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>hdf5__io_8cpp.html</anchorfile>
      <anchor>a3c04138a5bfe5d72780bb7e82a18e627</anchor>
      <arglist>(int argc, char **argv)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>r2_hdf5.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/scripts/</path>
    <filename>r2__hdf5_8cpp</filename>
    <includes id="util_8h" name="util.h" local="no" imported="no">robowflex_library/util.h</includes>
    <includes id="robot_8h" name="robot.h" local="no" imported="no">robowflex_library/robot.h</includes>
    <includes id="scene_8h" name="scene.h" local="no" imported="no">robowflex_library/scene.h</includes>
    <includes id="planning_8h" name="planning.h" local="no" imported="no">robowflex_library/planning.h</includes>
    <includes id="r2_8h" name="r2.h" local="no" imported="no">robowflex_library/detail/r2.h</includes>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>r2__hdf5_8cpp.html</anchorfile>
      <anchor>a3c04138a5bfe5d72780bb7e82a18e627</anchor>
      <arglist>(int argc, char **argv)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>r2_test.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/scripts/</path>
    <filename>r2__test_8cpp</filename>
    <includes id="util_8h" name="util.h" local="no" imported="no">robowflex_library/util.h</includes>
    <includes id="geometry_8h" name="geometry.h" local="no" imported="no">robowflex_library/geometry.h</includes>
    <includes id="robot_8h" name="robot.h" local="no" imported="no">robowflex_library/robot.h</includes>
    <includes id="scene_8h" name="scene.h" local="no" imported="no">robowflex_library/scene.h</includes>
    <includes id="planning_8h" name="planning.h" local="no" imported="no">robowflex_library/planning.h</includes>
    <includes id="visualization_8h" name="visualization.h" local="no" imported="no">robowflex_library/io/visualization.h</includes>
    <includes id="r2_8h" name="r2.h" local="no" imported="no">robowflex_library/detail/r2.h</includes>
    <member kind="function">
      <type>int</type>
      <name>planFromFile</name>
      <anchorfile>r2__test_8cpp.html</anchorfile>
      <anchor>a271ccdc2dd7b209d961901c3e532cb5a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>planAndBuild</name>
      <anchorfile>r2__test_8cpp.html</anchorfile>
      <anchor>ad74d62e8da0040b4dede1ddd2bcb6e33</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>r2__test_8cpp.html</anchorfile>
      <anchor>a3c04138a5bfe5d72780bb7e82a18e627</anchor>
      <arglist>(int argc, char **argv)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>ur5_benchmark.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/scripts/</path>
    <filename>ur5__benchmark_8cpp</filename>
    <includes id="util_8h" name="util.h" local="no" imported="no">robowflex_library/util.h</includes>
    <includes id="geometry_8h" name="geometry.h" local="no" imported="no">robowflex_library/geometry.h</includes>
    <includes id="robot_8h" name="robot.h" local="no" imported="no">robowflex_library/robot.h</includes>
    <includes id="scene_8h" name="scene.h" local="no" imported="no">robowflex_library/scene.h</includes>
    <includes id="planning_8h" name="planning.h" local="no" imported="no">robowflex_library/planning.h</includes>
    <includes id="benchmarking_8h" name="benchmarking.h" local="no" imported="no">robowflex_library/benchmarking.h</includes>
    <includes id="ur5_8h" name="ur5.h" local="no" imported="no">robowflex_library/detail/ur5.h</includes>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>ur5__benchmark_8cpp.html</anchorfile>
      <anchor>a3c04138a5bfe5d72780bb7e82a18e627</anchor>
      <arglist>(int argc, char **argv)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>ur5_io.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/scripts/</path>
    <filename>ur5__io_8cpp</filename>
    <includes id="util_8h" name="util.h" local="no" imported="no">robowflex_library/util.h</includes>
    <includes id="robot_8h" name="robot.h" local="no" imported="no">robowflex_library/robot.h</includes>
    <includes id="scene_8h" name="scene.h" local="no" imported="no">robowflex_library/scene.h</includes>
    <includes id="io_8h" name="io.h" local="no" imported="no">robowflex_library/io.h</includes>
    <includes id="bag_8h" name="bag.h" local="no" imported="no">robowflex_library/io/bag.h</includes>
    <includes id="ur5_8h" name="ur5.h" local="no" imported="no">robowflex_library/detail/ur5.h</includes>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>ur5__io_8cpp.html</anchorfile>
      <anchor>a3c04138a5bfe5d72780bb7e82a18e627</anchor>
      <arglist>(int argc, char **argv)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>ur5_pool.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/scripts/</path>
    <filename>ur5__pool_8cpp</filename>
    <includes id="util_8h" name="util.h" local="no" imported="no">robowflex_library/util.h</includes>
    <includes id="geometry_8h" name="geometry.h" local="no" imported="no">robowflex_library/geometry.h</includes>
    <includes id="robot_8h" name="robot.h" local="no" imported="no">robowflex_library/robot.h</includes>
    <includes id="scene_8h" name="scene.h" local="no" imported="no">robowflex_library/scene.h</includes>
    <includes id="planning_8h" name="planning.h" local="no" imported="no">robowflex_library/planning.h</includes>
    <includes id="ur5_8h" name="ur5.h" local="no" imported="no">robowflex_library/detail/ur5.h</includes>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>ur5__pool_8cpp.html</anchorfile>
      <anchor>a3c04138a5bfe5d72780bb7e82a18e627</anchor>
      <arglist>(int argc, char **argv)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>ur5_test.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/scripts/</path>
    <filename>ur5__test_8cpp</filename>
    <includes id="util_8h" name="util.h" local="no" imported="no">robowflex_library/util.h</includes>
    <includes id="geometry_8h" name="geometry.h" local="no" imported="no">robowflex_library/geometry.h</includes>
    <includes id="robot_8h" name="robot.h" local="no" imported="no">robowflex_library/robot.h</includes>
    <includes id="scene_8h" name="scene.h" local="no" imported="no">robowflex_library/scene.h</includes>
    <includes id="planning_8h" name="planning.h" local="no" imported="no">robowflex_library/planning.h</includes>
    <includes id="ur5_8h" name="ur5.h" local="no" imported="no">robowflex_library/detail/ur5.h</includes>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>ur5__test_8cpp.html</anchorfile>
      <anchor>a3c04138a5bfe5d72780bb7e82a18e627</anchor>
      <arglist>(int argc, char **argv)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>ur5_visualization.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/scripts/</path>
    <filename>ur5__visualization_8cpp</filename>
    <includes id="util_8h" name="util.h" local="no" imported="no">robowflex_library/util.h</includes>
    <includes id="geometry_8h" name="geometry.h" local="no" imported="no">robowflex_library/geometry.h</includes>
    <includes id="robot_8h" name="robot.h" local="no" imported="no">robowflex_library/robot.h</includes>
    <includes id="scene_8h" name="scene.h" local="no" imported="no">robowflex_library/scene.h</includes>
    <includes id="planning_8h" name="planning.h" local="no" imported="no">robowflex_library/planning.h</includes>
    <includes id="visualization_8h" name="visualization.h" local="no" imported="no">robowflex_library/io/visualization.h</includes>
    <includes id="ur5_8h" name="ur5.h" local="no" imported="no">robowflex_library/detail/ur5.h</includes>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>ur5__visualization_8cpp.html</anchorfile>
      <anchor>a3c04138a5bfe5d72780bb7e82a18e627</anchor>
      <arglist>(int argc, char **argv)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>wam7_benchmark.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/scripts/</path>
    <filename>wam7__benchmark_8cpp</filename>
    <includes id="util_8h" name="util.h" local="no" imported="no">robowflex_library/util.h</includes>
    <includes id="geometry_8h" name="geometry.h" local="no" imported="no">robowflex_library/geometry.h</includes>
    <includes id="robot_8h" name="robot.h" local="no" imported="no">robowflex_library/robot.h</includes>
    <includes id="scene_8h" name="scene.h" local="no" imported="no">robowflex_library/scene.h</includes>
    <includes id="planning_8h" name="planning.h" local="no" imported="no">robowflex_library/planning.h</includes>
    <includes id="benchmarking_8h" name="benchmarking.h" local="no" imported="no">robowflex_library/benchmarking.h</includes>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>wam7__benchmark_8cpp.html</anchorfile>
      <anchor>a3c04138a5bfe5d72780bb7e82a18e627</anchor>
      <arglist>(int argc, char **argv)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>wam7_test.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/scripts/</path>
    <filename>wam7__test_8cpp</filename>
    <includes id="util_8h" name="util.h" local="no" imported="no">robowflex_library/util.h</includes>
    <includes id="geometry_8h" name="geometry.h" local="no" imported="no">robowflex_library/geometry.h</includes>
    <includes id="robot_8h" name="robot.h" local="no" imported="no">robowflex_library/robot.h</includes>
    <includes id="scene_8h" name="scene.h" local="no" imported="no">robowflex_library/scene.h</includes>
    <includes id="planning_8h" name="planning.h" local="no" imported="no">robowflex_library/planning.h</includes>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>wam7__test_8cpp.html</anchorfile>
      <anchor>a3c04138a5bfe5d72780bb7e82a18e627</anchor>
      <arglist>(int argc, char **argv)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>benchmarking.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/src/</path>
    <filename>benchmarking_8cpp</filename>
    <includes id="io_8h" name="io.h" local="no" imported="no">robowflex_library/io.h</includes>
    <includes id="io_2yaml_8h" name="yaml.h" local="no" imported="no">robowflex_library/io/yaml.h</includes>
    <includes id="scene_8h" name="scene.h" local="no" imported="no">robowflex_library/scene.h</includes>
    <includes id="planning_8h" name="planning.h" local="no" imported="no">robowflex_library/planning.h</includes>
    <includes id="benchmarking_8h" name="benchmarking.h" local="no" imported="no">robowflex_library/benchmarking.h</includes>
  </compound>
  <compound kind="file">
    <name>builder.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/src/</path>
    <filename>builder_8cpp</filename>
    <includes id="io_8h" name="io.h" local="no" imported="no">robowflex_library/io.h</includes>
    <includes id="io_2yaml_8h" name="yaml.h" local="no" imported="no">robowflex_library/io/yaml.h</includes>
    <includes id="tf_8h" name="tf.h" local="no" imported="no">robowflex_library/tf.h</includes>
    <includes id="robot_8h" name="robot.h" local="no" imported="no">robowflex_library/robot.h</includes>
    <includes id="planning_8h" name="planning.h" local="no" imported="no">robowflex_library/planning.h</includes>
  </compound>
  <compound kind="file">
    <name>fetch.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/src/detail/</path>
    <filename>fetch_8cpp</filename>
    <includes id="io_8h" name="io.h" local="no" imported="no">robowflex_library/io.h</includes>
    <includes id="fetch_8h" name="fetch.h" local="no" imported="no">robowflex_library/detail/fetch.h</includes>
  </compound>
  <compound kind="file">
    <name>r2.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/src/detail/</path>
    <filename>r2_8cpp</filename>
    <includes id="io_8h" name="io.h" local="no" imported="no">robowflex_library/io.h</includes>
    <includes id="hdf5_8h" name="hdf5.h" local="no" imported="no">robowflex_library/io/hdf5.h</includes>
    <includes id="r2_8h" name="r2.h" local="no" imported="no">robowflex_library/detail/r2.h</includes>
  </compound>
  <compound kind="file">
    <name>ur5.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/src/detail/</path>
    <filename>ur5_8cpp</filename>
    <includes id="ur5_8h" name="ur5.h" local="no" imported="no">robowflex_library/detail/ur5.h</includes>
  </compound>
  <compound kind="file">
    <name>geometry.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/src/</path>
    <filename>geometry_8cpp</filename>
    <includes id="util_8h" name="util.h" local="no" imported="no">robowflex_library/util.h</includes>
    <includes id="io_8h" name="io.h" local="no" imported="no">robowflex_library/io.h</includes>
    <includes id="geometry_8h" name="geometry.h" local="no" imported="no">robowflex_library/geometry.h</includes>
  </compound>
  <compound kind="file">
    <name>io.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/src/</path>
    <filename>io_8cpp</filename>
    <includes id="macros_8h" name="macros.h" local="no" imported="no">robowflex_library/macros.h</includes>
    <includes id="util_8h" name="util.h" local="no" imported="no">robowflex_library/util.h</includes>
    <includes id="io_8h" name="io.h" local="no" imported="no">robowflex_library/io.h</includes>
    <includes id="bag_8h" name="bag.h" local="no" imported="no">robowflex_library/io/bag.h</includes>
    <includes id="handler_8h" name="handler.h" local="no" imported="no">robowflex_library/io/handler.h</includes>
  </compound>
  <compound kind="file">
    <name>hdf5.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/src/io/</path>
    <filename>hdf5_8cpp</filename>
    <includes id="macros_8h" name="macros.h" local="no" imported="no">robowflex_library/macros.h</includes>
    <includes id="io_8h" name="io.h" local="no" imported="no">robowflex_library/io.h</includes>
    <includes id="hdf5_8h" name="hdf5.h" local="no" imported="no">robowflex_library/io/hdf5.h</includes>
  </compound>
  <compound kind="file">
    <name>visualization.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/src/io/</path>
    <filename>visualization_8cpp</filename>
    <includes id="robot_8h" name="robot.h" local="no" imported="no">robowflex_library/robot.h</includes>
    <includes id="scene_8h" name="scene.h" local="no" imported="no">robowflex_library/scene.h</includes>
    <includes id="visualization_8h" name="visualization.h" local="no" imported="no">robowflex_library/io/visualization.h</includes>
  </compound>
  <compound kind="file">
    <name>planning.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/src/</path>
    <filename>planning_8cpp</filename>
    <includes id="io_8h" name="io.h" local="no" imported="no">robowflex_library/io.h</includes>
    <includes id="robot_8h" name="robot.h" local="no" imported="no">robowflex_library/robot.h</includes>
    <includes id="scene_8h" name="scene.h" local="no" imported="no">robowflex_library/scene.h</includes>
    <includes id="planning_8h" name="planning.h" local="no" imported="no">robowflex_library/planning.h</includes>
  </compound>
  <compound kind="file">
    <name>pool.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/src/</path>
    <filename>pool_8cpp</filename>
    <includes id="pool_8h" name="pool.h" local="no" imported="no">robowflex_library/pool.h</includes>
  </compound>
  <compound kind="file">
    <name>robot.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/src/</path>
    <filename>robot_8cpp</filename>
    <includes id="macros_8h" name="macros.h" local="no" imported="no">robowflex_library/macros.h</includes>
    <includes id="io_8h" name="io.h" local="no" imported="no">robowflex_library/io.h</includes>
    <includes id="io_2yaml_8h" name="yaml.h" local="no" imported="no">robowflex_library/io/yaml.h</includes>
    <includes id="geometry_8h" name="geometry.h" local="no" imported="no">robowflex_library/geometry.h</includes>
    <includes id="tf_8h" name="tf.h" local="no" imported="no">robowflex_library/tf.h</includes>
    <includes id="scene_8h" name="scene.h" local="no" imported="no">robowflex_library/scene.h</includes>
    <includes id="robot_8h" name="robot.h" local="no" imported="no">robowflex_library/robot.h</includes>
  </compound>
  <compound kind="file">
    <name>scene.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/src/</path>
    <filename>scene_8cpp</filename>
    <includes id="io_8h" name="io.h" local="no" imported="no">robowflex_library/io.h</includes>
    <includes id="io_2yaml_8h" name="yaml.h" local="no" imported="no">robowflex_library/io/yaml.h</includes>
    <includes id="geometry_8h" name="geometry.h" local="no" imported="no">robowflex_library/geometry.h</includes>
    <includes id="robot_8h" name="robot.h" local="no" imported="no">robowflex_library/robot.h</includes>
    <includes id="scene_8h" name="scene.h" local="no" imported="no">robowflex_library/scene.h</includes>
    <class kind="class">robowflex::Scene::CollisionPluginLoader</class>
    <namespace>robowflex</namespace>
  </compound>
  <compound kind="file">
    <name>tf.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/src/</path>
    <filename>tf_8cpp</filename>
    <includes id="geometry_8h" name="geometry.h" local="no" imported="no">robowflex_library/geometry.h</includes>
    <includes id="tf_8h" name="tf.h" local="no" imported="no">robowflex_library/tf.h</includes>
  </compound>
  <compound kind="file">
    <name>util.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/src/</path>
    <filename>util_8cpp</filename>
    <includes id="macros_8h" name="macros.h" local="no" imported="no">robowflex_library/macros.h</includes>
    <includes id="util_8h" name="util.h" local="no" imported="no">robowflex_library/util.h</includes>
  </compound>
  <compound kind="file">
    <name>yaml.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_library/src/</path>
    <filename>yaml_8cpp</filename>
    <includes id="macros_8h" name="macros.h" local="no" imported="no">robowflex_library/macros.h</includes>
    <includes id="geometry_8h" name="geometry.h" local="no" imported="no">robowflex_library/geometry.h</includes>
    <includes id="io_8h" name="io.h" local="no" imported="no">robowflex_library/io.h</includes>
    <includes id="io_2yaml_8h" name="yaml.h" local="no" imported="no">robowflex_library/io/yaml.h</includes>
    <includes id="yaml_8h" name="yaml.h" local="no" imported="no">robowflex_library/yaml.h</includes>
    <namespace>YAML</namespace>
    <namespace>robowflex</namespace>
    <namespace>robowflex::IO</namespace>
    <member kind="function">
      <type>bool</type>
      <name>isNode</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a23927fb72a17ab8ce0d0654d06b5532e</anchor>
      <arglist>(const YAML::Node &amp;node)</arglist>
    </member>
    <member kind="function">
      <type>YAML::Node</type>
      <name>toNode</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>aed4974f62a649f4a6107ac3d6e919b33</anchor>
      <arglist>(const geometry_msgs::Pose &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>YAML::Node</type>
      <name>toNode</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a819245f223ff0eb589ad28aa153f2485</anchor>
      <arglist>(const moveit_msgs::PlanningScene &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>YAML::Node</type>
      <name>toNode</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a799d8a2989dee49fff0dd91d17a883be</anchor>
      <arglist>(const moveit_msgs::MotionPlanRequest &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>YAML::Node</type>
      <name>toNode</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a414861fda8e5df5e954e0f65a97b2ed7</anchor>
      <arglist>(const moveit_msgs::RobotTrajectory &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>fromYAMLFile</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a50e952949006a40e6b7b5d589958e01d</anchor>
      <arglist>(moveit_msgs::PlanningScene &amp;msg, const std::string &amp;file)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>fromYAMLFile</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a0500ed90fd25e45a7a0476fee615bcc3</anchor>
      <arglist>(moveit_msgs::MotionPlanRequest &amp;msg, const std::string &amp;file)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>services.h</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_movegroup/include/robowflex_movegroup/</path>
    <filename>services_8h</filename>
    <includes id="robot_8h" name="robot.h" local="no" imported="no">robowflex_library/robot.h</includes>
    <includes id="scene_8h" name="scene.h" local="no" imported="no">robowflex_library/scene.h</includes>
    <class kind="class">robowflex::movegroup::MoveGroupHelper</class>
    <class kind="struct">robowflex::movegroup::MoveGroupHelper::Action</class>
    <namespace>robowflex</namespace>
    <namespace>robowflex::movegroup</namespace>
  </compound>
  <compound kind="file">
    <name>mixtape.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_movegroup/scripts/</path>
    <filename>mixtape_8cpp</filename>
    <includes id="util_8h" name="util.h" local="no" imported="no">robowflex_library/util.h</includes>
    <includes id="io_8h" name="io.h" local="no" imported="no">robowflex_library/io.h</includes>
    <includes id="services_8h" name="services.h" local="no" imported="no">robowflex_movegroup/services.h</includes>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>mixtape_8cpp.html</anchorfile>
      <anchor>a3c04138a5bfe5d72780bb7e82a18e627</anchor>
      <arglist>(int argc, char **argv)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>tapedeck.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_movegroup/scripts/</path>
    <filename>tapedeck_8cpp</filename>
    <includes id="util_8h" name="util.h" local="no" imported="no">robowflex_library/util.h</includes>
    <includes id="services_8h" name="services.h" local="no" imported="no">robowflex_movegroup/services.h</includes>
    <member kind="function">
      <type>void</type>
      <name>callback</name>
      <anchorfile>tapedeck_8cpp.html</anchorfile>
      <anchor>a71a61eac1f6c474c38e7aa100cc8f341</anchor>
      <arglist>(movegroup::MoveGroupHelper::Action &amp;action)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>tapedeck_8cpp.html</anchorfile>
      <anchor>a3c04138a5bfe5d72780bb7e82a18e627</anchor>
      <arglist>(int argc, char **argv)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>services.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_movegroup/src/</path>
    <filename>services_8cpp</filename>
    <includes id="io_8h" name="io.h" local="no" imported="no">robowflex_library/io.h</includes>
    <includes id="io_2yaml_8h" name="yaml.h" local="no" imported="no">robowflex_library/io/yaml.h</includes>
    <includes id="yaml_8h" name="yaml.h" local="no" imported="no">robowflex_library/yaml.h</includes>
    <includes id="services_8h" name="services.h" local="no" imported="no">robowflex_movegroup/services.h</includes>
  </compound>
  <compound kind="file">
    <name>ompl.h</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_ompl/include/robowflex_ompl/</path>
    <filename>ompl_8h</filename>
    <includes id="class__forward_8h" name="class_forward.h" local="no" imported="no">robowflex_library/class_forward.h</includes>
    <class kind="class">robowflex::OMPL::OMPLInterfacePlanner</class>
    <namespace>robowflex</namespace>
    <namespace>robowflex::OMPL</namespace>
  </compound>
  <compound kind="file">
    <name>ur5_ompl_interface.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_ompl/scripts/</path>
    <filename>ur5__ompl__interface_8cpp</filename>
    <includes id="util_8h" name="util.h" local="no" imported="no">robowflex_library/util.h</includes>
    <includes id="geometry_8h" name="geometry.h" local="no" imported="no">robowflex_library/geometry.h</includes>
    <includes id="robot_8h" name="robot.h" local="no" imported="no">robowflex_library/robot.h</includes>
    <includes id="scene_8h" name="scene.h" local="no" imported="no">robowflex_library/scene.h</includes>
    <includes id="planning_8h" name="planning.h" local="no" imported="no">robowflex_library/planning.h</includes>
    <includes id="ur5_8h" name="ur5.h" local="no" imported="no">robowflex_library/detail/ur5.h</includes>
    <includes id="ompl_8h" name="ompl.h" local="no" imported="no">robowflex_ompl/ompl.h</includes>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>ur5__ompl__interface_8cpp.html</anchorfile>
      <anchor>a3c04138a5bfe5d72780bb7e82a18e627</anchor>
      <arglist>(int argc, char **argv)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>ompl_interface.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_ompl/src/</path>
    <filename>ompl__interface_8cpp</filename>
    <includes id="handler_8h" name="handler.h" local="no" imported="no">robowflex_library/io/handler.h</includes>
    <includes id="robot_8h" name="robot.h" local="no" imported="no">robowflex_library/robot.h</includes>
    <includes id="scene_8h" name="scene.h" local="no" imported="no">robowflex_library/scene.h</includes>
    <includes id="planning_8h" name="planning.h" local="no" imported="no">robowflex_library/planning.h</includes>
    <includes id="ompl_8h" name="ompl.h" local="no" imported="no">robowflex_ompl/ompl.h</includes>
  </compound>
  <compound kind="file">
    <name>conversions.h</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_tesseract/include/robowflex_tesseract/</path>
    <filename>conversions_8h</filename>
    <includes id="class__forward_8h" name="class_forward.h" local="no" imported="no">robowflex_library/class_forward.h</includes>
    <namespace>robowflex</namespace>
    <namespace>robowflex::hypercube</namespace>
    <member kind="function">
      <type>tesseract::tesseract_ros::KDLEnvPtr</type>
      <name>constructTesseractEnv</name>
      <anchorfile>namespacerobowflex_1_1hypercube.html</anchorfile>
      <anchor>ae1da7fda2f5b63b56339d1b31c45e6bb</anchor>
      <arglist>(const robowflex::SceneConstPtr &amp;scene, const robowflex::RobotConstPtr &amp;robot)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>tesseract_planners.h</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_tesseract/include/robowflex_tesseract/</path>
    <filename>tesseract__planners_8h</filename>
    <includes id="class__forward_8h" name="class_forward.h" local="no" imported="no">robowflex_library/class_forward.h</includes>
    <includes id="planning_8h" name="planning.h" local="no" imported="no">robowflex_library/planning.h</includes>
    <class kind="class">robowflex::hypercube::Settings</class>
    <class kind="class">robowflex::hypercube::OMPLChainPlanner</class>
    <namespace>robowflex</namespace>
    <namespace>robowflex::hypercube</namespace>
    <member kind="typedef">
      <type>std::function&lt; ompl::base::PlannerPtr(const ompl::base::SpaceInformationPtr &amp;si, const std::string &amp;name, const std::map&lt; std::string, std::string &gt; &amp;map)&gt;</type>
      <name>ConfiguredPlannerAllocator</name>
      <anchorfile>namespacerobowflex_1_1hypercube.html</anchorfile>
      <anchor>a7790860877268b79fd620562a9e3f6ba</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>tesseract_benchmark.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_tesseract/scripts/</path>
    <filename>tesseract__benchmark_8cpp</filename>
    <includes id="util_8h" name="util.h" local="no" imported="no">robowflex_library/util.h</includes>
    <includes id="geometry_8h" name="geometry.h" local="no" imported="no">robowflex_library/geometry.h</includes>
    <includes id="robot_8h" name="robot.h" local="no" imported="no">robowflex_library/robot.h</includes>
    <includes id="scene_8h" name="scene.h" local="no" imported="no">robowflex_library/scene.h</includes>
    <includes id="planning_8h" name="planning.h" local="no" imported="no">robowflex_library/planning.h</includes>
    <includes id="tesseract__planners_8h" name="tesseract_planners.h" local="no" imported="no">robowflex_tesseract/tesseract_planners.h</includes>
    <includes id="benchmarking_8h" name="benchmarking.h" local="no" imported="no">robowflex_library/benchmarking.h</includes>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>tesseract__benchmark_8cpp.html</anchorfile>
      <anchor>a3c04138a5bfe5d72780bb7e82a18e627</anchor>
      <arglist>(int argc, char **argv)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>ur5_tesseract.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_tesseract/scripts/</path>
    <filename>ur5__tesseract_8cpp</filename>
    <includes id="util_8h" name="util.h" local="no" imported="no">robowflex_library/util.h</includes>
    <includes id="geometry_8h" name="geometry.h" local="no" imported="no">robowflex_library/geometry.h</includes>
    <includes id="robot_8h" name="robot.h" local="no" imported="no">robowflex_library/robot.h</includes>
    <includes id="scene_8h" name="scene.h" local="no" imported="no">robowflex_library/scene.h</includes>
    <includes id="planning_8h" name="planning.h" local="no" imported="no">robowflex_library/planning.h</includes>
    <includes id="tesseract__planners_8h" name="tesseract_planners.h" local="no" imported="no">robowflex_tesseract/tesseract_planners.h</includes>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>ur5__tesseract_8cpp.html</anchorfile>
      <anchor>a3c04138a5bfe5d72780bb7e82a18e627</anchor>
      <arglist>(int argc, char **argv)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>conversions.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_tesseract/src/</path>
    <filename>conversions_8cpp</filename>
    <includes id="scene_8h" name="scene.h" local="no" imported="no">robowflex_library/scene.h</includes>
    <includes id="robot_8h" name="robot.h" local="no" imported="no">robowflex_library/robot.h</includes>
    <includes id="conversions_8h" name="conversions.h" local="no" imported="no">robowflex_tesseract/conversions.h</includes>
  </compound>
  <compound kind="file">
    <name>tesseract_planners.cpp</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_tesseract/src/</path>
    <filename>tesseract__planners_8cpp</filename>
    <includes id="scene_8h" name="scene.h" local="no" imported="no">robowflex_library/scene.h</includes>
    <includes id="robot_8h" name="robot.h" local="no" imported="no">robowflex_library/robot.h</includes>
    <includes id="planning_8h" name="planning.h" local="no" imported="no">robowflex_library/planning.h</includes>
    <includes id="tesseract__planners_8h" name="tesseract_planners.h" local="no" imported="no">robowflex_tesseract/tesseract_planners.h</includes>
    <includes id="conversions_8h" name="conversions.h" local="no" imported="no">robowflex_tesseract/conversions.h</includes>
  </compound>
  <compound kind="file">
    <name>README.md</name>
    <path>/home/zak/ros/melodic/dev/src/robowflex/robowflex_visualization/</path>
    <filename>README_8md</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::Benchmarker</name>
    <filename>classrobowflex_1_1Benchmarker.html</filename>
    <class kind="class">robowflex::Benchmarker::Options</class>
    <class kind="class">robowflex::Benchmarker::Results</class>
    <member kind="enumeration">
      <type></type>
      <name>MetricOptions</name>
      <anchorfile>classrobowflex_1_1Benchmarker.html</anchorfile>
      <anchor>adfbfc1b613a5b24a7e25b1b7a47da6fb</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>WAYPOINTS</name>
      <anchorfile>classrobowflex_1_1Benchmarker.html</anchorfile>
      <anchor>adfbfc1b613a5b24a7e25b1b7a47da6fbaf1c5f94d6d5d975d97c01f21b4af961e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PATH</name>
      <anchorfile>classrobowflex_1_1Benchmarker.html</anchorfile>
      <anchor>adfbfc1b613a5b24a7e25b1b7a47da6fba8cd7482c71bd953e29784e94a67ba58f</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>CORRECT</name>
      <anchorfile>classrobowflex_1_1Benchmarker.html</anchorfile>
      <anchor>adfbfc1b613a5b24a7e25b1b7a47da6fba225cc5b91cecd73f11cd392b609e7c52</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>LENGTH</name>
      <anchorfile>classrobowflex_1_1Benchmarker.html</anchorfile>
      <anchor>adfbfc1b613a5b24a7e25b1b7a47da6fbad40dad13bbb9082001f8e3d56ed2cf77</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>CLEARANCE</name>
      <anchorfile>classrobowflex_1_1Benchmarker.html</anchorfile>
      <anchor>adfbfc1b613a5b24a7e25b1b7a47da6fba155d1dc50cfde1b1b1e60ddcc56d3a3d</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SMOOTHNESS</name>
      <anchorfile>classrobowflex_1_1Benchmarker.html</anchorfile>
      <anchor>adfbfc1b613a5b24a7e25b1b7a47da6fbad9338f6edcb665f0868f47dc7b116218</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>WAYPOINTS</name>
      <anchorfile>classrobowflex_1_1Benchmarker.html</anchorfile>
      <anchor>adfbfc1b613a5b24a7e25b1b7a47da6fbaf1c5f94d6d5d975d97c01f21b4af961e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PATH</name>
      <anchorfile>classrobowflex_1_1Benchmarker.html</anchorfile>
      <anchor>adfbfc1b613a5b24a7e25b1b7a47da6fba8cd7482c71bd953e29784e94a67ba58f</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>CORRECT</name>
      <anchorfile>classrobowflex_1_1Benchmarker.html</anchorfile>
      <anchor>adfbfc1b613a5b24a7e25b1b7a47da6fba225cc5b91cecd73f11cd392b609e7c52</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>LENGTH</name>
      <anchorfile>classrobowflex_1_1Benchmarker.html</anchorfile>
      <anchor>adfbfc1b613a5b24a7e25b1b7a47da6fbad40dad13bbb9082001f8e3d56ed2cf77</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>CLEARANCE</name>
      <anchorfile>classrobowflex_1_1Benchmarker.html</anchorfile>
      <anchor>adfbfc1b613a5b24a7e25b1b7a47da6fba155d1dc50cfde1b1b1e60ddcc56d3a3d</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SMOOTHNESS</name>
      <anchorfile>classrobowflex_1_1Benchmarker.html</anchorfile>
      <anchor>adfbfc1b613a5b24a7e25b1b7a47da6fbad9338f6edcb665f0868f47dc7b116218</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addBenchmarkingRequest</name>
      <anchorfile>classrobowflex_1_1Benchmarker.html</anchorfile>
      <anchor>abaf5f16de50c8405f7bf3dc3756af26c</anchor>
      <arglist>(const std::string &amp;name, const ScenePtr &amp;scene, const PlannerPtr &amp;planner, const MotionRequestBuilderPtr &amp;request)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>benchmark</name>
      <anchorfile>classrobowflex_1_1Benchmarker.html</anchorfile>
      <anchor>ab4e3053043412c4eb35b456ccf07e1de</anchor>
      <arglist>(const std::vector&lt; BenchmarkOutputterPtr &gt; &amp;output, const Options &amp;options=Options())</arglist>
    </member>
    <member kind="typedef" protection="private">
      <type>std::tuple&lt; ScenePtr, PlannerPtr, MotionRequestBuilderPtr &gt;</type>
      <name>BenchmarkRequest</name>
      <anchorfile>classrobowflex_1_1Benchmarker.html</anchorfile>
      <anchor>a0e8e38b0c5bac7611292a2dc4cb9c97c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::map&lt; std::string, BenchmarkRequest &gt;</type>
      <name>requests_</name>
      <anchorfile>classrobowflex_1_1Benchmarker.html</anchorfile>
      <anchor>a90fafec21210fa7753aa4cd4b4fab522</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::Benchmarker::Options</name>
    <filename>classrobowflex_1_1Benchmarker_1_1Options.html</filename>
    <member kind="function">
      <type></type>
      <name>Options</name>
      <anchorfile>classrobowflex_1_1Benchmarker_1_1Options.html</anchorfile>
      <anchor>a568cd90a8ea479907aa5280d3a53fc17</anchor>
      <arglist>(unsigned int runs=100, uint32_t options=~0)</arglist>
    </member>
    <member kind="variable">
      <type>unsigned int</type>
      <name>runs</name>
      <anchorfile>classrobowflex_1_1Benchmarker_1_1Options.html</anchorfile>
      <anchor>a627afe71315dea80b603f15385bd4bb5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>options</name>
      <anchorfile>classrobowflex_1_1Benchmarker_1_1Options.html</anchorfile>
      <anchor>a9c2a09268069602ab200ec0f51fbb19a</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::Benchmarker::Results</name>
    <filename>classrobowflex_1_1Benchmarker_1_1Results.html</filename>
    <class kind="class">robowflex::Benchmarker::Results::Run</class>
    <member kind="function">
      <type></type>
      <name>Results</name>
      <anchorfile>classrobowflex_1_1Benchmarker_1_1Results.html</anchorfile>
      <anchor>a4df02ab8ee06bafa0471bb14db44d794</anchor>
      <arglist>(const std::string &amp;name, const SceneConstPtr scene, const PlannerConstPtr planner, const MotionRequestBuilderConstPtr builder, const Options &amp;options)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addRun</name>
      <anchorfile>classrobowflex_1_1Benchmarker_1_1Results.html</anchorfile>
      <anchor>a73d3f08bcbdc3b4ee53a03c2be217b06</anchor>
      <arglist>(int num, double time, planning_interface::MotionPlanResponse &amp;run)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>computeMetric</name>
      <anchorfile>classrobowflex_1_1Benchmarker_1_1Results.html</anchorfile>
      <anchor>a3a9eb42c6f96a2f58052a7593a1f7d4d</anchor>
      <arglist>(planning_interface::MotionPlanResponse &amp;run, Run &amp;metrics)</arglist>
    </member>
    <member kind="variable">
      <type>const std::string</type>
      <name>name</name>
      <anchorfile>classrobowflex_1_1Benchmarker_1_1Results.html</anchorfile>
      <anchor>a45867b61b7e1294c075043275bbb21ff</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const SceneConstPtr</type>
      <name>scene</name>
      <anchorfile>classrobowflex_1_1Benchmarker_1_1Results.html</anchorfile>
      <anchor>a02565770842974c5a12a97f4c0074a1c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const PlannerConstPtr</type>
      <name>planner</name>
      <anchorfile>classrobowflex_1_1Benchmarker_1_1Results.html</anchorfile>
      <anchor>a3e1635ac77cc75abeb02a6ce37d82ecf</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const MotionRequestBuilderConstPtr</type>
      <name>builder</name>
      <anchorfile>classrobowflex_1_1Benchmarker_1_1Results.html</anchorfile>
      <anchor>a91a2c77cf0c4075ebf92c536aeeafe7d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const Options</type>
      <name>options</name>
      <anchorfile>classrobowflex_1_1Benchmarker_1_1Results.html</anchorfile>
      <anchor>a4ec2acce88b08806a3509c28edf40ffd</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>boost::posix_time::ptime</type>
      <name>start</name>
      <anchorfile>classrobowflex_1_1Benchmarker_1_1Results.html</anchorfile>
      <anchor>af24a50716571c497d28d6b22709de731</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>boost::posix_time::ptime</type>
      <name>finish</name>
      <anchorfile>classrobowflex_1_1Benchmarker_1_1Results.html</anchorfile>
      <anchor>ac21037ad15db3b4f1db10dfb57200419</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>std::vector&lt; Run &gt;</type>
      <name>runs</name>
      <anchorfile>classrobowflex_1_1Benchmarker_1_1Results.html</anchorfile>
      <anchor>a661c4aa3ecd0ebc8f78d807f72053a57</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::Benchmarker::Results::Run</name>
    <filename>classrobowflex_1_1Benchmarker_1_1Results_1_1Run.html</filename>
    <class kind="class">robowflex::Benchmarker::Results::Run::toString</class>
    <member kind="typedef">
      <type>boost::variant&lt; bool, double, int &gt;</type>
      <name>MetricValue</name>
      <anchorfile>classrobowflex_1_1Benchmarker_1_1Results_1_1Run.html</anchorfile>
      <anchor>a5d4d7d1d4f3568acaf6b01520765a0dc</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Run</name>
      <anchorfile>classrobowflex_1_1Benchmarker_1_1Results_1_1Run.html</anchorfile>
      <anchor>afaf01f39005f8cfcb575dbdddd327512</anchor>
      <arglist>(int num, double time, bool success)</arglist>
    </member>
    <member kind="variable">
      <type>int</type>
      <name>num</name>
      <anchorfile>classrobowflex_1_1Benchmarker_1_1Results_1_1Run.html</anchorfile>
      <anchor>af0aa76d09770bc1ef09a3993a3edcf1a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>double</type>
      <name>time</name>
      <anchorfile>classrobowflex_1_1Benchmarker_1_1Results_1_1Run.html</anchorfile>
      <anchor>aa150758cf15b53efa456aca6e860dd59</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>bool</type>
      <name>success</name>
      <anchorfile>classrobowflex_1_1Benchmarker_1_1Results_1_1Run.html</anchorfile>
      <anchor>a992689f2cc5745164923632335309f87</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>moveit_msgs::RobotTrajectory</type>
      <name>path</name>
      <anchorfile>classrobowflex_1_1Benchmarker_1_1Results_1_1Run.html</anchorfile>
      <anchor>a08d96f2cb22beefbaca06cef19c2e66a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>std::map&lt; std::string, MetricValue &gt;</type>
      <name>metrics</name>
      <anchorfile>classrobowflex_1_1Benchmarker_1_1Results_1_1Run.html</anchorfile>
      <anchor>a0d32abcc17fc3d278953a3605ef69cff</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::Benchmarker::Results::Run::toString</name>
    <filename>classrobowflex_1_1Benchmarker_1_1Results_1_1Run_1_1toString.html</filename>
    <member kind="function">
      <type>const std::string</type>
      <name>operator()</name>
      <anchorfile>classrobowflex_1_1Benchmarker_1_1Results_1_1Run_1_1toString.html</anchorfile>
      <anchor>a178cc86d497527f923f3f804593d2b59</anchor>
      <arglist>(int value) const</arglist>
    </member>
    <member kind="function">
      <type>const std::string</type>
      <name>operator()</name>
      <anchorfile>classrobowflex_1_1Benchmarker_1_1Results_1_1Run_1_1toString.html</anchorfile>
      <anchor>a8803a9d674575d54491f296d338f87c0</anchor>
      <arglist>(double value) const</arglist>
    </member>
    <member kind="function">
      <type>const std::string</type>
      <name>operator()</name>
      <anchorfile>classrobowflex_1_1Benchmarker_1_1Results_1_1Run_1_1toString.html</anchorfile>
      <anchor>acff4d71177c44647d739d858a5a64c1d</anchor>
      <arglist>(bool value) const</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::BenchmarkerConstPtr</name>
    <filename>classrobowflex_1_1BenchmarkerConstPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::BenchmarkerPtr</name>
    <filename>classrobowflex_1_1BenchmarkerPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::BenchmarkOutputter</name>
    <filename>classrobowflex_1_1BenchmarkOutputter.html</filename>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~BenchmarkOutputter</name>
      <anchorfile>classrobowflex_1_1BenchmarkOutputter.html</anchorfile>
      <anchor>ada5ba264d079a2499accad312f64acfa</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>dumpResult</name>
      <anchorfile>classrobowflex_1_1BenchmarkOutputter.html</anchorfile>
      <anchor>a599222e7ede784a921c7cf2b7469a92d</anchor>
      <arglist>(const Benchmarker::Results &amp;results)=0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::BenchmarkOutputterConstPtr</name>
    <filename>classrobowflex_1_1BenchmarkOutputterConstPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::BenchmarkOutputterPtr</name>
    <filename>classrobowflex_1_1BenchmarkOutputterPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::Exception</name>
    <filename>classrobowflex_1_1Exception.html</filename>
    <base>std::exception</base>
    <member kind="function">
      <type></type>
      <name>Exception</name>
      <anchorfile>classrobowflex_1_1Exception.html</anchorfile>
      <anchor>a2ac289e0ea34588d9c8dd8e147482a09</anchor>
      <arglist>(int value, const std::string &amp;message)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>getValue</name>
      <anchorfile>classrobowflex_1_1Exception.html</anchorfile>
      <anchor>a7c364163d2aa2a01ba515d22b5535c5a</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const std::string &amp;</type>
      <name>getMessage</name>
      <anchorfile>classrobowflex_1_1Exception.html</anchorfile>
      <anchor>afe60a3198763685aa0e2846f7b9133ba</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual const char *</type>
      <name>what</name>
      <anchorfile>classrobowflex_1_1Exception.html</anchorfile>
      <anchor>a93d13fea53c850383744bbe0a243e6e3</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>const int</type>
      <name>value_</name>
      <anchorfile>classrobowflex_1_1Exception.html</anchorfile>
      <anchor>ab437c933e5130684a8069ab40983ace0</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>const std::string</type>
      <name>message_</name>
      <anchorfile>classrobowflex_1_1Exception.html</anchorfile>
      <anchor>a1fac8b580526bf15e9ad137d4416c916</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::FetchRobot</name>
    <filename>classrobowflex_1_1FetchRobot.html</filename>
    <base>robowflex::Robot</base>
    <member kind="function">
      <type></type>
      <name>FetchRobot</name>
      <anchorfile>classrobowflex_1_1FetchRobot.html</anchorfile>
      <anchor>a2f1dd0e56219abfeb970d12d6e8fafbc</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>initialize</name>
      <anchorfile>classrobowflex_1_1FetchRobot.html</anchorfile>
      <anchor>a3bed849dca843c559365c55df12f259c</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>addVirtualJointSRDF</name>
      <anchorfile>classrobowflex_1_1FetchRobot.html</anchorfile>
      <anchor>a143d8e0b64460e4dcc46cdce3e87fa90</anchor>
      <arglist>(tinyxml2::XMLDocument &amp;doc)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setBasePose</name>
      <anchorfile>classrobowflex_1_1FetchRobot.html</anchorfile>
      <anchor>aafcd98b9aa5069182adba35810c080db</anchor>
      <arglist>(double x, double y, double theta)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>pointHead</name>
      <anchorfile>classrobowflex_1_1FetchRobot.html</anchorfile>
      <anchor>ae815a3ed21e4c535f93e2c76c9135f24</anchor>
      <arglist>(const Eigen::Vector3d &amp;point)</arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static const std::string</type>
      <name>URDF</name>
      <anchorfile>classrobowflex_1_1FetchRobot.html</anchorfile>
      <anchor>a42b50b499939dade2dcb03ea4ea494e9</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static const std::string</type>
      <name>SRDF</name>
      <anchorfile>classrobowflex_1_1FetchRobot.html</anchorfile>
      <anchor>a59d660e348eedf45052df38ba2d77e59</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static const std::string</type>
      <name>LIMITS</name>
      <anchorfile>classrobowflex_1_1FetchRobot.html</anchorfile>
      <anchor>a62c4ef404db725a7afe65a0bc6f5ff18</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static const std::string</type>
      <name>KINEMATICS</name>
      <anchorfile>classrobowflex_1_1FetchRobot.html</anchorfile>
      <anchor>a2ef267cf2b4a5ecd539d2eacae960060</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::FetchRobotConstPtr</name>
    <filename>classrobowflex_1_1FetchRobotConstPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::FetchRobotPtr</name>
    <filename>classrobowflex_1_1FetchRobotPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::Geometry</name>
    <filename>classrobowflex_1_1Geometry.html</filename>
    <class kind="class">robowflex::Geometry::ShapeType</class>
    <member kind="function">
      <type></type>
      <name>Geometry</name>
      <anchorfile>classrobowflex_1_1Geometry.html</anchorfile>
      <anchor>aba0c0a75b6520a95cc337a83e685035e</anchor>
      <arglist>(ShapeType::Type type, const Eigen::Vector3d &amp;dimensions, const std::string &amp;resource=&quot;&quot;)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Geometry</name>
      <anchorfile>classrobowflex_1_1Geometry.html</anchorfile>
      <anchor>ab01e14aff00b6d2743b31e47fc848cdd</anchor>
      <arglist>(const Geometry &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>Geometry &amp;</type>
      <name>operator=</name>
      <anchorfile>classrobowflex_1_1Geometry.html</anchorfile>
      <anchor>a11af3cdc67ce0382365a9731672aea20</anchor>
      <arglist>(const Geometry &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>contains</name>
      <anchorfile>classrobowflex_1_1Geometry.html</anchorfile>
      <anchor>afeed3d5a783d0b6ae7d01c2706951289</anchor>
      <arglist>(const Eigen::Vector3d &amp;point) const</arglist>
    </member>
    <member kind="function">
      <type>std::pair&lt; bool, Eigen::Vector3d &gt;</type>
      <name>sample</name>
      <anchorfile>classrobowflex_1_1Geometry.html</anchorfile>
      <anchor>abceb50074b6e2cbcc990c1396098ea77</anchor>
      <arglist>(const unsigned int attempts=50) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>isMesh</name>
      <anchorfile>classrobowflex_1_1Geometry.html</anchorfile>
      <anchor>a90091f195278eaf211faaaa34da0cd78</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const shape_msgs::SolidPrimitive</type>
      <name>getSolidMsg</name>
      <anchorfile>classrobowflex_1_1Geometry.html</anchorfile>
      <anchor>a9718437f6931324f7187d85413075e39</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const shape_msgs::Mesh</type>
      <name>getMeshMsg</name>
      <anchorfile>classrobowflex_1_1Geometry.html</anchorfile>
      <anchor>a748ef0e6fd0ee8c617765204e0930952</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const shapes::ShapePtr &amp;</type>
      <name>getShape</name>
      <anchorfile>classrobowflex_1_1Geometry.html</anchorfile>
      <anchor>a1267d2f3c3226c04ada7936e8286b04a</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const bodies::BodyPtr &amp;</type>
      <name>getBody</name>
      <anchorfile>classrobowflex_1_1Geometry.html</anchorfile>
      <anchor>a0e8c7c3c948151dbf4f1aeee805561ae</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static GeometryPtr</type>
      <name>makeSphere</name>
      <anchorfile>classrobowflex_1_1Geometry.html</anchorfile>
      <anchor>a3b45cda54e08fbd87c67432dd44c1ee4</anchor>
      <arglist>(double radius)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static GeometryPtr</type>
      <name>makeBox</name>
      <anchorfile>classrobowflex_1_1Geometry.html</anchorfile>
      <anchor>a4b0635a1b21b2c98bf6f383199ed42f5</anchor>
      <arglist>(double x, double y, double z)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static GeometryPtr</type>
      <name>makeCylinder</name>
      <anchorfile>classrobowflex_1_1Geometry.html</anchorfile>
      <anchor>ad6fa7bfc9ea2360c86aff746799e85dc</anchor>
      <arglist>(double radius, double length)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static GeometryPtr</type>
      <name>makeCone</name>
      <anchorfile>classrobowflex_1_1Geometry.html</anchorfile>
      <anchor>a7aee3a61fc6ade3599cc82a4a449718b</anchor>
      <arglist>(double radius, double length)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static GeometryPtr</type>
      <name>makeMesh</name>
      <anchorfile>classrobowflex_1_1Geometry.html</anchorfile>
      <anchor>a05e5b0f2f7db3483263064b0eab47618</anchor>
      <arglist>(const std::string &amp;resource, const Eigen::Vector3d &amp;scale={1, 1, 1})</arglist>
    </member>
    <member kind="function" protection="private">
      <type>shapes::Shape *</type>
      <name>loadShape</name>
      <anchorfile>classrobowflex_1_1Geometry.html</anchorfile>
      <anchor>ac84f99fddb1e1f5eabac2d3663a489ea</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bodies::Body *</type>
      <name>loadBody</name>
      <anchorfile>classrobowflex_1_1Geometry.html</anchorfile>
      <anchor>acadeec2a7cd0f8b7ac6ac309937d390a</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ShapeType::Type</type>
      <name>type_</name>
      <anchorfile>classrobowflex_1_1Geometry.html</anchorfile>
      <anchor>ad89296c0ff755b3beb18caa9df99e6c5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const Eigen::Vector3d</type>
      <name>dimensions_</name>
      <anchorfile>classrobowflex_1_1Geometry.html</anchorfile>
      <anchor>ababeb23de988823a7aa4198b213c69dc</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::string</type>
      <name>resource_</name>
      <anchorfile>classrobowflex_1_1Geometry.html</anchorfile>
      <anchor>ad87f6f762768545896cd92430531ce6d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const shapes::ShapePtr</type>
      <name>shape_</name>
      <anchorfile>classrobowflex_1_1Geometry.html</anchorfile>
      <anchor>a5ca5785ff342faa9f8af554906223118</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const bodies::BodyPtr</type>
      <name>body_</name>
      <anchorfile>classrobowflex_1_1Geometry.html</anchorfile>
      <anchor>ad4d104f32ee944e7537644a07644510f</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::Geometry::ShapeType</name>
    <filename>classrobowflex_1_1Geometry_1_1ShapeType.html</filename>
    <member kind="enumeration">
      <type></type>
      <name>Type</name>
      <anchorfile>classrobowflex_1_1Geometry_1_1ShapeType.html</anchorfile>
      <anchor>a5046aea046e675696ac6c78bf273825b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>BOX</name>
      <anchorfile>classrobowflex_1_1Geometry_1_1ShapeType.html</anchorfile>
      <anchor>a5046aea046e675696ac6c78bf273825ba88dd34d51e5b1b255ead4bf3200a7d51</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SPHERE</name>
      <anchorfile>classrobowflex_1_1Geometry_1_1ShapeType.html</anchorfile>
      <anchor>a5046aea046e675696ac6c78bf273825bafaea4efab646e1956f00d187f1265f7f</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>CYLINDER</name>
      <anchorfile>classrobowflex_1_1Geometry_1_1ShapeType.html</anchorfile>
      <anchor>a5046aea046e675696ac6c78bf273825ba2c41d9a87b690b7afc5805b343d7a4a5</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>CONE</name>
      <anchorfile>classrobowflex_1_1Geometry_1_1ShapeType.html</anchorfile>
      <anchor>a5046aea046e675696ac6c78bf273825ba4abdeb43d35691fe96127109454a85f2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>MESH</name>
      <anchorfile>classrobowflex_1_1Geometry_1_1ShapeType.html</anchorfile>
      <anchor>a5046aea046e675696ac6c78bf273825ba40efcc8aff42316338c0c2b0355fb3ae</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>BOX</name>
      <anchorfile>classrobowflex_1_1Geometry_1_1ShapeType.html</anchorfile>
      <anchor>a5046aea046e675696ac6c78bf273825ba88dd34d51e5b1b255ead4bf3200a7d51</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SPHERE</name>
      <anchorfile>classrobowflex_1_1Geometry_1_1ShapeType.html</anchorfile>
      <anchor>a5046aea046e675696ac6c78bf273825bafaea4efab646e1956f00d187f1265f7f</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>CYLINDER</name>
      <anchorfile>classrobowflex_1_1Geometry_1_1ShapeType.html</anchorfile>
      <anchor>a5046aea046e675696ac6c78bf273825ba2c41d9a87b690b7afc5805b343d7a4a5</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>CONE</name>
      <anchorfile>classrobowflex_1_1Geometry_1_1ShapeType.html</anchorfile>
      <anchor>a5046aea046e675696ac6c78bf273825ba4abdeb43d35691fe96127109454a85f2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>MESH</name>
      <anchorfile>classrobowflex_1_1Geometry_1_1ShapeType.html</anchorfile>
      <anchor>a5046aea046e675696ac6c78bf273825ba40efcc8aff42316338c0c2b0355fb3ae</anchor>
      <arglist></arglist>
    </member>
    <member kind="function" static="yes">
      <type>static Type</type>
      <name>toType</name>
      <anchorfile>classrobowflex_1_1Geometry_1_1ShapeType.html</anchorfile>
      <anchor>a60f8d4aa261144317286b444a29f34d0</anchor>
      <arglist>(const std::string &amp;str)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static const std::string &amp;</type>
      <name>toString</name>
      <anchorfile>classrobowflex_1_1Geometry_1_1ShapeType.html</anchorfile>
      <anchor>a0e497d7082981764e599200bbda0c660</anchor>
      <arglist>(Type type)</arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static const unsigned int</type>
      <name>MAX</name>
      <anchorfile>classrobowflex_1_1Geometry_1_1ShapeType.html</anchorfile>
      <anchor>a7c68fb7f3e0768bd5744572f5e852f74</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static const std::vector&lt; std::string &gt;</type>
      <name>STRINGS</name>
      <anchorfile>classrobowflex_1_1Geometry_1_1ShapeType.html</anchorfile>
      <anchor>a7c0d841b0157d8b03435a2cf3751ccb2</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::GeometryConstPtr</name>
    <filename>classrobowflex_1_1GeometryConstPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::GeometryPtr</name>
    <filename>classrobowflex_1_1GeometryPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::HDF5DataConstPtr</name>
    <filename>classrobowflex_1_1HDF5DataConstPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::HDF5DataPtr</name>
    <filename>classrobowflex_1_1HDF5DataPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::hypercube::OMPLChainPlanner</name>
    <filename>classrobowflex_1_1hypercube_1_1OMPLChainPlanner.html</filename>
    <base>robowflex::Planner</base>
    <member kind="function">
      <type></type>
      <name>OMPLChainPlanner</name>
      <anchorfile>classrobowflex_1_1hypercube_1_1OMPLChainPlanner.html</anchorfile>
      <anchor>afc9f762e53aa4c2eb03cb4144ea7c8a2</anchor>
      <arglist>(const RobotPtr &amp;robot, const std::string &amp;name=&quot;&quot;)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>initialize</name>
      <anchorfile>classrobowflex_1_1hypercube_1_1OMPLChainPlanner.html</anchorfile>
      <anchor>a7c6c33e3c21618ac08bb6fb0af68144a</anchor>
      <arglist>(const std::string &amp;config_file, const Settings &amp;settings)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>registerPlannerAllocator</name>
      <anchorfile>classrobowflex_1_1hypercube_1_1OMPLChainPlanner.html</anchorfile>
      <anchor>aa8b103f14f471aeac48eea575a6aab5e</anchor>
      <arglist>(const std::string &amp;planner_id, const ConfiguredPlannerAllocator &amp;pa)</arglist>
    </member>
    <member kind="function">
      <type>planning_interface::MotionPlanResponse</type>
      <name>plan</name>
      <anchorfile>classrobowflex_1_1hypercube_1_1OMPLChainPlanner.html</anchorfile>
      <anchor>acc8d2bb82b41849fde6cc0022763a935</anchor>
      <arglist>(const SceneConstPtr &amp;scene, const planning_interface::MotionPlanRequest &amp;request) override</arglist>
    </member>
    <member kind="function">
      <type>const std::vector&lt; std::string &gt;</type>
      <name>getPlannerConfigs</name>
      <anchorfile>classrobowflex_1_1hypercube_1_1OMPLChainPlanner.html</anchorfile>
      <anchor>ac8b60f94d73f87f5acee86ca49d7036a</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; tesseract::tesseract_planning::ChainOmplInterface &gt;</type>
      <name>chain_interface_</name>
      <anchorfile>classrobowflex_1_1hypercube_1_1OMPLChainPlanner.html</anchorfile>
      <anchor>aa48985729e00c44f7ca7d992ca243dce</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>Settings</type>
      <name>settings_</name>
      <anchorfile>classrobowflex_1_1hypercube_1_1OMPLChainPlanner.html</anchorfile>
      <anchor>a4f922f947a9d8c8290b503c03513c39b</anchor>
      <arglist></arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>registerDefaultPlanners</name>
      <anchorfile>classrobowflex_1_1hypercube_1_1OMPLChainPlanner.html</anchorfile>
      <anchor>aad777aa682723a586a09766e4216997c</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::map&lt; std::string, ConfiguredPlannerAllocator &gt;</type>
      <name>known_planners_</name>
      <anchorfile>classrobowflex_1_1hypercube_1_1OMPLChainPlanner.html</anchorfile>
      <anchor>a51bf6cdbbd30d7bf72fd55e491d5be7d</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::hypercube::OMPLChainPlannerPtr</name>
    <filename>classrobowflex_1_1hypercube_1_1OMPLChainPlannerPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::hypercube::OMPLInterfacePlannerConstPtr</name>
    <filename>classrobowflex_1_1hypercube_1_1OMPLInterfacePlannerConstPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::hypercube::Settings</name>
    <filename>classrobowflex_1_1hypercube_1_1Settings.html</filename>
    <member kind="function">
      <type></type>
      <name>Settings</name>
      <anchorfile>classrobowflex_1_1hypercube_1_1Settings.html</anchorfile>
      <anchor>ab7169a6eefce79566dd07db3b1e5e967</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable">
      <type>bool</type>
      <name>simplify_solutions</name>
      <anchorfile>classrobowflex_1_1hypercube_1_1Settings.html</anchorfile>
      <anchor>a9c748d3cbc87b0db082e172a5fa67a81</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>double</type>
      <name>longest_valid_segment_fraction</name>
      <anchorfile>classrobowflex_1_1hypercube_1_1Settings.html</anchorfile>
      <anchor>a55fde30dc88fdb78e1eb5051a2febd78</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>bool</type>
      <name>use_continuous_validator</name>
      <anchorfile>classrobowflex_1_1hypercube_1_1Settings.html</anchorfile>
      <anchor>ae935f95a1db3a4d815c268222e678571</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::IO::Bag</name>
    <filename>classrobowflex_1_1IO_1_1Bag.html</filename>
    <member kind="enumeration">
      <type></type>
      <name>Mode</name>
      <anchorfile>classrobowflex_1_1IO_1_1Bag.html</anchorfile>
      <anchor>a6c815ce5d20042c1bcb1e8a96185399e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>READ</name>
      <anchorfile>classrobowflex_1_1IO_1_1Bag.html</anchorfile>
      <anchor>a6c815ce5d20042c1bcb1e8a96185399eaf0a100871108c22d8065e118bead6acd</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>WRITE</name>
      <anchorfile>classrobowflex_1_1IO_1_1Bag.html</anchorfile>
      <anchor>a6c815ce5d20042c1bcb1e8a96185399ea15d631bfb446e35c940b06ae983241c5</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>READ</name>
      <anchorfile>classrobowflex_1_1IO_1_1Bag.html</anchorfile>
      <anchor>a6c815ce5d20042c1bcb1e8a96185399eaf0a100871108c22d8065e118bead6acd</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>WRITE</name>
      <anchorfile>classrobowflex_1_1IO_1_1Bag.html</anchorfile>
      <anchor>a6c815ce5d20042c1bcb1e8a96185399ea15d631bfb446e35c940b06ae983241c5</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Bag</name>
      <anchorfile>classrobowflex_1_1IO_1_1Bag.html</anchorfile>
      <anchor>ac9720ad66eb2c47b99d168efc3460168</anchor>
      <arglist>(const std::string &amp;file, Mode mode=WRITE)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~Bag</name>
      <anchorfile>classrobowflex_1_1IO_1_1Bag.html</anchorfile>
      <anchor>a1195732a759aed17b6a0a2f0cc9f5055</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>addMessage</name>
      <anchorfile>classrobowflex_1_1IO_1_1Bag.html</anchorfile>
      <anchor>aeea42ab2e31c185453927c9fb1ec6069</anchor>
      <arglist>(const std::string &amp;topic, const T &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; T &gt;</type>
      <name>getMessages</name>
      <anchorfile>classrobowflex_1_1IO_1_1Bag.html</anchorfile>
      <anchor>af292c5404b7b96f0f1334e2770f6c57f</anchor>
      <arglist>(const std::vector&lt; std::string &gt; &amp;topics)</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const Mode</type>
      <name>mode_</name>
      <anchorfile>classrobowflex_1_1IO_1_1Bag.html</anchorfile>
      <anchor>ac430677e1235675d30949cc9a53e56d7</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const std::string</type>
      <name>file_</name>
      <anchorfile>classrobowflex_1_1IO_1_1Bag.html</anchorfile>
      <anchor>ac264fb5fdd126958a9bb5a7f4ce8b7f3</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>rosbag::Bag</type>
      <name>bag_</name>
      <anchorfile>classrobowflex_1_1IO_1_1Bag.html</anchorfile>
      <anchor>ad23060099e5c3afdd319de1b786b1713</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::IO::Handler</name>
    <filename>classrobowflex_1_1IO_1_1Handler.html</filename>
    <member kind="function">
      <type></type>
      <name>Handler</name>
      <anchorfile>classrobowflex_1_1IO_1_1Handler.html</anchorfile>
      <anchor>ae8dd424faaa9fe35333996a53cdbf1ae</anchor>
      <arglist>(const std::string &amp;name)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Handler</name>
      <anchorfile>classrobowflex_1_1IO_1_1Handler.html</anchorfile>
      <anchor>a7078fbad0ad270d2f5d275070bba8070</anchor>
      <arglist>(Handler const &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>operator=</name>
      <anchorfile>classrobowflex_1_1IO_1_1Handler.html</anchorfile>
      <anchor>a9bfb1d08271ad49c8c67d14558614232</anchor>
      <arglist>(Handler const &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Handler</name>
      <anchorfile>classrobowflex_1_1IO_1_1Handler.html</anchorfile>
      <anchor>a2f677c5376f80e5d397adedf37c622cc</anchor>
      <arglist>(const IO::Handler &amp;handler, const std::string &amp;name=&quot;&quot;)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~Handler</name>
      <anchorfile>classrobowflex_1_1IO_1_1Handler.html</anchorfile>
      <anchor>a44102c6477dd39913320d2f54cde6092</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>loadYAMLtoROS</name>
      <anchorfile>classrobowflex_1_1IO_1_1Handler.html</anchorfile>
      <anchor>ac2833ad1ff056153192e11258815595e</anchor>
      <arglist>(const YAML::Node &amp;node, const std::string &amp;prefix=&quot;&quot;)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setParam</name>
      <anchorfile>classrobowflex_1_1IO_1_1Handler.html</anchorfile>
      <anchor>a1ec8d085e19a898f3d70bd964dc8d045</anchor>
      <arglist>(const std::string &amp;key, const T &amp;value)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>hasParam</name>
      <anchorfile>classrobowflex_1_1IO_1_1Handler.html</anchorfile>
      <anchor>a44c6c6b8384d8e1d97f98190bdbad6bb</anchor>
      <arglist>(const std::string &amp;key) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getParam</name>
      <anchorfile>classrobowflex_1_1IO_1_1Handler.html</anchorfile>
      <anchor>aac6e13dd1c4ebfc715490d864ad9c77e</anchor>
      <arglist>(const std::string &amp;key, T &amp;value) const</arglist>
    </member>
    <member kind="function">
      <type>const ros::NodeHandle &amp;</type>
      <name>getHandle</name>
      <anchorfile>classrobowflex_1_1IO_1_1Handler.html</anchorfile>
      <anchor>a8663a3da14f2743ccacd0b7521e05087</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const std::string &amp;</type>
      <name>getName</name>
      <anchorfile>classrobowflex_1_1IO_1_1Handler.html</anchorfile>
      <anchor>a398cffcab454fe34c96bb79c23c680c7</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const std::string &amp;</type>
      <name>getNamespace</name>
      <anchorfile>classrobowflex_1_1IO_1_1Handler.html</anchorfile>
      <anchor>aca54f2aa4b4fd07b52c6f380e9da195c</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function" protection="private" static="yes">
      <type>static const std::string</type>
      <name>generateUUID</name>
      <anchorfile>classrobowflex_1_1IO_1_1Handler.html</anchorfile>
      <anchor>a1c33a06440b120c6a01421c7bc28e659</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const std::string</type>
      <name>name_</name>
      <anchorfile>classrobowflex_1_1IO_1_1Handler.html</anchorfile>
      <anchor>adce3f8d4454c4efbf6f920201cbbeaf5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const std::string</type>
      <name>namespace_</name>
      <anchorfile>classrobowflex_1_1IO_1_1Handler.html</anchorfile>
      <anchor>a9dc12e4fef4248535a407f3cde628064</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ros::NodeHandle</type>
      <name>nh_</name>
      <anchorfile>classrobowflex_1_1IO_1_1Handler.html</anchorfile>
      <anchor>aa2204200cb5ef2c362d39ec00d561d79</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; std::string &gt;</type>
      <name>params_</name>
      <anchorfile>classrobowflex_1_1IO_1_1Handler.html</anchorfile>
      <anchor>a7cdb461c7a29b3d6e7cbab1d6876294a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static const std::string</type>
      <name>UUID</name>
      <anchorfile>classrobowflex_1_1IO_1_1Handler.html</anchorfile>
      <anchor>afa2f8303d4182ccdbeb60cb0fa350a5a</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::IO::HDF5Data</name>
    <filename>classrobowflex_1_1IO_1_1HDF5Data.html</filename>
    <member kind="function">
      <type></type>
      <name>HDF5Data</name>
      <anchorfile>classrobowflex_1_1IO_1_1HDF5Data.html</anchorfile>
      <anchor>ac1491abe53cd26b64f99371e41f89dd7</anchor>
      <arglist>(const T &amp;location, const std::string &amp;name)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~HDF5Data</name>
      <anchorfile>classrobowflex_1_1IO_1_1HDF5Data.html</anchorfile>
      <anchor>aa51b2e7674204aa655c18e6159110e50</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const std::vector&lt; hsize_t &gt;</type>
      <name>getDims</name>
      <anchorfile>classrobowflex_1_1IO_1_1HDF5Data.html</anchorfile>
      <anchor>ae083d5c3ff58a1eec05eb1a046ea8f31</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const void *</type>
      <name>getData</name>
      <anchorfile>classrobowflex_1_1IO_1_1HDF5Data.html</anchorfile>
      <anchor>a23b45724262d818679402adc0e113c15</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const std::string</type>
      <name>getStatus</name>
      <anchorfile>classrobowflex_1_1IO_1_1HDF5Data.html</anchorfile>
      <anchor>a417bcd1b3122e0e52a34fd070b9461e5</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const T &amp;</type>
      <name>get</name>
      <anchorfile>classrobowflex_1_1IO_1_1HDF5Data.html</anchorfile>
      <anchor>ac9422297f7f4037b15f485e3f521186c</anchor>
      <arglist>(const std::vector&lt; hsize_t &gt; &amp;index) const</arglist>
    </member>
    <member kind="function" protection="private">
      <type>std::tuple&lt; H5::PredType, unsigned int, std::string &gt;</type>
      <name>getDataProperties</name>
      <anchorfile>classrobowflex_1_1IO_1_1HDF5Data.html</anchorfile>
      <anchor>af311153f4ac891fc022f918f444824f9</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const H5::DataSet</type>
      <name>dataset_</name>
      <anchorfile>classrobowflex_1_1IO_1_1HDF5Data.html</anchorfile>
      <anchor>a868568e8226f656c0aca79b56b8338f2</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const H5::DataSpace</type>
      <name>space_</name>
      <anchorfile>classrobowflex_1_1IO_1_1HDF5Data.html</anchorfile>
      <anchor>aee3e2fc2c48038144cce891bd31f1b7b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const H5T_class_t</type>
      <name>type_</name>
      <anchorfile>classrobowflex_1_1IO_1_1HDF5Data.html</anchorfile>
      <anchor>ae7bfe49968b526e104ce7116ceb9435e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const int</type>
      <name>rank_</name>
      <anchorfile>classrobowflex_1_1IO_1_1HDF5Data.html</anchorfile>
      <anchor>ab7aa327dcff28ad8b49eff9c8e4a4907</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const hsize_t *</type>
      <name>dims_</name>
      <anchorfile>classrobowflex_1_1IO_1_1HDF5Data.html</anchorfile>
      <anchor>a09d36053782f7ea9a557a589c08f6f2d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const void *</type>
      <name>data_</name>
      <anchorfile>classrobowflex_1_1IO_1_1HDF5Data.html</anchorfile>
      <anchor>a3174621eedf6aae2a81cb727a8589f46</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::IO::HDF5File</name>
    <filename>classrobowflex_1_1IO_1_1HDF5File.html</filename>
    <member kind="typedef">
      <type>boost::make_recursive_variant&lt; HDF5DataPtr, std::map&lt; std::string, boost::recursive_variant_ &gt; &gt;::type</type>
      <name>Node</name>
      <anchorfile>classrobowflex_1_1IO_1_1HDF5File.html</anchorfile>
      <anchor>a0691c40015573dc089f0a36d2574bc62</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::map&lt; std::string, Node &gt;</type>
      <name>NodeMap</name>
      <anchorfile>classrobowflex_1_1IO_1_1HDF5File.html</anchorfile>
      <anchor>ad736a2045e84ba0152d17c41ece933fa</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>HDF5File</name>
      <anchorfile>classrobowflex_1_1IO_1_1HDF5File.html</anchorfile>
      <anchor>aba0c7c690b6e20798c6cd9a228d340e5</anchor>
      <arglist>(const std::string &amp;filename)</arglist>
    </member>
    <member kind="function">
      <type>const HDF5DataPtr</type>
      <name>getData</name>
      <anchorfile>classrobowflex_1_1IO_1_1HDF5File.html</anchorfile>
      <anchor>ad85a0b44894e8226a7fd32be84eb4ec9</anchor>
      <arglist>(const std::vector&lt; std::string &gt; &amp;keys) const</arglist>
    </member>
    <member kind="function">
      <type>const std::vector&lt; std::vector&lt; std::string &gt; &gt;</type>
      <name>getKeys</name>
      <anchorfile>classrobowflex_1_1IO_1_1HDF5File.html</anchorfile>
      <anchor>a76502a090c1c8895b5aa75d098439f4f</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function" protection="private">
      <type>const std::vector&lt; std::string &gt;</type>
      <name>listObjects</name>
      <anchorfile>classrobowflex_1_1IO_1_1HDF5File.html</anchorfile>
      <anchor>a20b7ddae54affeb2dca0dff1811c844e</anchor>
      <arglist>(const T &amp;location) const</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>loadData</name>
      <anchorfile>classrobowflex_1_1IO_1_1HDF5File.html</anchorfile>
      <anchor>a4c430e8b3ad652179dbbdfbf7861a2a1</anchor>
      <arglist>(Node &amp;node, const T &amp;location, const std::string &amp;name)</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const H5::H5File</type>
      <name>file_</name>
      <anchorfile>classrobowflex_1_1IO_1_1HDF5File.html</anchorfile>
      <anchor>a8fe22f4075c602824a8bf7847b24ccd5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>Node</type>
      <name>data_</name>
      <anchorfile>classrobowflex_1_1IO_1_1HDF5File.html</anchorfile>
      <anchor>a0ac234fefd5ebd557a8020c6df093221</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::IO::RVIZHelper</name>
    <filename>classrobowflex_1_1IO_1_1RVIZHelper.html</filename>
    <member kind="function">
      <type></type>
      <name>RVIZHelper</name>
      <anchorfile>classrobowflex_1_1IO_1_1RVIZHelper.html</anchorfile>
      <anchor>aa38ab1ceea8b4a281399fd21362ae42b</anchor>
      <arglist>(const RobotConstPtr &amp;robot, const std::string &amp;name=&quot;robowflex&quot;)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>updateTrajectory</name>
      <anchorfile>classrobowflex_1_1IO_1_1RVIZHelper.html</anchorfile>
      <anchor>af15ed6584a83c9abbdaee0a37e13e448</anchor>
      <arglist>(const planning_interface::MotionPlanResponse &amp;response)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>updateTrajectories</name>
      <anchorfile>classrobowflex_1_1IO_1_1RVIZHelper.html</anchorfile>
      <anchor>aa1769a8fed2faf24be83dc9ca2e85577</anchor>
      <arglist>(const std::vector&lt; planning_interface::MotionPlanResponse &gt; &amp;responses)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>updateScene</name>
      <anchorfile>classrobowflex_1_1IO_1_1RVIZHelper.html</anchorfile>
      <anchor>ac7c28b2d8923603632231b7fee37e4b9</anchor>
      <arglist>(const SceneConstPtr &amp;scene)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>updateMarkers</name>
      <anchorfile>classrobowflex_1_1IO_1_1RVIZHelper.html</anchorfile>
      <anchor>a947e2ed5b8132f76bb3fd875f7d978b2</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addMarker</name>
      <anchorfile>classrobowflex_1_1IO_1_1RVIZHelper.html</anchorfile>
      <anchor>ac17baa538e19ad09c7eb7effa4eba1f4</anchor>
      <arglist>(float x, float y, float z)</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>RobotConstPtr</type>
      <name>robot_</name>
      <anchorfile>classrobowflex_1_1IO_1_1RVIZHelper.html</anchorfile>
      <anchor>a93145ba9f05b0aa27c925b25b095014c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ros::NodeHandle</type>
      <name>nh_</name>
      <anchorfile>classrobowflex_1_1IO_1_1RVIZHelper.html</anchorfile>
      <anchor>a789db3404b017326b5271e65b38e3c5a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ros::Publisher</type>
      <name>marker_pub_</name>
      <anchorfile>classrobowflex_1_1IO_1_1RVIZHelper.html</anchorfile>
      <anchor>a1ff5ca3b2a449f5b1f60e8e3432aeb0d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ros::Publisher</type>
      <name>trajectory_pub_</name>
      <anchorfile>classrobowflex_1_1IO_1_1RVIZHelper.html</anchorfile>
      <anchor>a69b398440a32dca875f3951d6a876c57</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ros::Publisher</type>
      <name>scene_pub_</name>
      <anchorfile>classrobowflex_1_1IO_1_1RVIZHelper.html</anchorfile>
      <anchor>a6714790347a2f536752a5fa2fe1e239a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::map&lt; std::string, visualization_msgs::Marker &gt;</type>
      <name>markers_</name>
      <anchorfile>classrobowflex_1_1IO_1_1RVIZHelper.html</anchorfile>
      <anchor>a7d8ac999887d3a2c9c5131b79e1b69ab</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::JSONBenchmarkOutputter</name>
    <filename>classrobowflex_1_1JSONBenchmarkOutputter.html</filename>
    <base>robowflex::BenchmarkOutputter</base>
    <member kind="function">
      <type></type>
      <name>JSONBenchmarkOutputter</name>
      <anchorfile>classrobowflex_1_1JSONBenchmarkOutputter.html</anchorfile>
      <anchor>a823f260391ad1f45ba35152d40f6aa47</anchor>
      <arglist>(const std::string &amp;file)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~JSONBenchmarkOutputter</name>
      <anchorfile>classrobowflex_1_1JSONBenchmarkOutputter.html</anchorfile>
      <anchor>a0af8cc2f77b9d5e71c84ba51e46a2f23</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>dumpResult</name>
      <anchorfile>classrobowflex_1_1JSONBenchmarkOutputter.html</anchorfile>
      <anchor>afbf711b65b647061c1abd04e10ec2575</anchor>
      <arglist>(const Benchmarker::Results &amp;results) override</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>bool</type>
      <name>is_init_</name>
      <anchorfile>classrobowflex_1_1JSONBenchmarkOutputter.html</anchorfile>
      <anchor>a92cdea83a7f7a3886c62545ff5c400f8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const std::string</type>
      <name>file_</name>
      <anchorfile>classrobowflex_1_1JSONBenchmarkOutputter.html</anchorfile>
      <anchor>ab6ac8f6200afce4d306ad5b38e9e48cc</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::ofstream</type>
      <name>outfile_</name>
      <anchorfile>classrobowflex_1_1JSONBenchmarkOutputter.html</anchorfile>
      <anchor>af8379a7e1fcc2084e8588c082da7e302</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::MotionRequestBuilder</name>
    <filename>classrobowflex_1_1MotionRequestBuilder.html</filename>
    <member kind="function">
      <type></type>
      <name>MotionRequestBuilder</name>
      <anchorfile>classrobowflex_1_1MotionRequestBuilder.html</anchorfile>
      <anchor>ac53028afafd5ede152427e00b51ad8e0</anchor>
      <arglist>(const PlannerConstPtr &amp;planner, const std::string &amp;group_name)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setWorkspaceBounds</name>
      <anchorfile>classrobowflex_1_1MotionRequestBuilder.html</anchorfile>
      <anchor>ab3668987415bf45930a2438ae3935c4b</anchor>
      <arglist>(const moveit_msgs::WorkspaceParameters &amp;wp)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setStartConfiguration</name>
      <anchorfile>classrobowflex_1_1MotionRequestBuilder.html</anchorfile>
      <anchor>a85ba3502e0ccadad2226415c84153d3e</anchor>
      <arglist>(const std::vector&lt; double &gt; &amp;joints)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setStartConfiguration</name>
      <anchorfile>classrobowflex_1_1MotionRequestBuilder.html</anchorfile>
      <anchor>a74bd83627cc405702c14f890884475f5</anchor>
      <arglist>(const robot_state::RobotStatePtr &amp;state)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setGoalConfiguration</name>
      <anchorfile>classrobowflex_1_1MotionRequestBuilder.html</anchorfile>
      <anchor>ae3ff76382121eb61c41931a6bfa17cef</anchor>
      <arglist>(const std::vector&lt; double &gt; &amp;joints)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setGoalConfiguration</name>
      <anchorfile>classrobowflex_1_1MotionRequestBuilder.html</anchorfile>
      <anchor>a4da8877fbbfc74bba310760e93c13412</anchor>
      <arglist>(const robot_state::RobotStatePtr &amp;state)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setGoalRegion</name>
      <anchorfile>classrobowflex_1_1MotionRequestBuilder.html</anchorfile>
      <anchor>a465c8c99f090dcb934b8764a10cfa393</anchor>
      <arglist>(const std::string &amp;ee_name, const std::string &amp;base_name, const Eigen::Affine3d &amp;pose, const GeometryConstPtr &amp;geometry, const Eigen::Quaterniond &amp;orientation, const Eigen::Vector3d &amp;tolerances)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addPathPoseConstraint</name>
      <anchorfile>classrobowflex_1_1MotionRequestBuilder.html</anchorfile>
      <anchor>ab23251a765c8ec2e20d8a3f7a208de2f</anchor>
      <arglist>(const std::string &amp;ee_name, const std::string &amp;base_name, const Eigen::Affine3d &amp;pose, const GeometryConstPtr &amp;geometry, const Eigen::Quaterniond &amp;orientation, const Eigen::Vector3d &amp;tolerances)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addPathPositionConstraint</name>
      <anchorfile>classrobowflex_1_1MotionRequestBuilder.html</anchorfile>
      <anchor>a1452324b16693c58069fca06c094cd29</anchor>
      <arglist>(const std::string &amp;ee_name, const std::string &amp;base_name, const Eigen::Affine3d &amp;pose, const GeometryConstPtr &amp;geometry)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addPathOrientationConstraint</name>
      <anchorfile>classrobowflex_1_1MotionRequestBuilder.html</anchorfile>
      <anchor>a528a97f8c4b17b92803ccd4fe39d52e6</anchor>
      <arglist>(const std::string &amp;ee_name, const std::string &amp;base_name, const Eigen::Quaterniond &amp;orientation, const Eigen::Vector3d &amp;tolerances)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setAllowedPlanningTime</name>
      <anchorfile>classrobowflex_1_1MotionRequestBuilder.html</anchorfile>
      <anchor>a2124a6c488484ef2ec0097e06e56e6ad</anchor>
      <arglist>(double allowed_planning_time)</arglist>
    </member>
    <member kind="function">
      <type>const planning_interface::MotionPlanRequest &amp;</type>
      <name>getRequest</name>
      <anchorfile>classrobowflex_1_1MotionRequestBuilder.html</anchorfile>
      <anchor>a716058f146f91d6039ef08bff8d97027</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>moveit_msgs::Constraints &amp;</type>
      <name>getPathConstraints</name>
      <anchorfile>classrobowflex_1_1MotionRequestBuilder.html</anchorfile>
      <anchor>a3c0124b3c45dc3087508a83fff616182</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>toYAMLFile</name>
      <anchorfile>classrobowflex_1_1MotionRequestBuilder.html</anchorfile>
      <anchor>a4b56c303fe7a31df60f9ea36d1ba19f8</anchor>
      <arglist>(const std::string &amp;file)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>fromYAMLFile</name>
      <anchorfile>classrobowflex_1_1MotionRequestBuilder.html</anchorfile>
      <anchor>a8a7016a3777f7843da0738157a17a362</anchor>
      <arglist>(const std::string &amp;file)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>setConfig</name>
      <anchorfile>classrobowflex_1_1MotionRequestBuilder.html</anchorfile>
      <anchor>a786e67b08f9873bab228de0fe759816b</anchor>
      <arglist>(const std::string &amp;requested_config)</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const PlannerConstPtr</type>
      <name>planner_</name>
      <anchorfile>classrobowflex_1_1MotionRequestBuilder.html</anchorfile>
      <anchor>a023efd5def6cf1e935e62a57f075d6ce</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const RobotConstPtr</type>
      <name>robot_</name>
      <anchorfile>classrobowflex_1_1MotionRequestBuilder.html</anchorfile>
      <anchor>a4609dd3215c6c06624730c1c97a1bbb3</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const std::string</type>
      <name>group_name_</name>
      <anchorfile>classrobowflex_1_1MotionRequestBuilder.html</anchorfile>
      <anchor>af70ca71975795907ce8f1b09f5fd0c94</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const robot_model::JointModelGroup *</type>
      <name>jmg_</name>
      <anchorfile>classrobowflex_1_1MotionRequestBuilder.html</anchorfile>
      <anchor>a69934a62a3c1c8bfca4e55d216ed567b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>planning_interface::MotionPlanRequest</type>
      <name>request_</name>
      <anchorfile>classrobowflex_1_1MotionRequestBuilder.html</anchorfile>
      <anchor>a11d30567db3d5170584c2c1b20b37a8e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static const std::vector&lt; std::string &gt;</type>
      <name>DEFAULT_CONFIGS</name>
      <anchorfile>classrobowflex_1_1MotionRequestBuilder.html</anchorfile>
      <anchor>a51a7c337cdacc40e4c4a722648f2d31f</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::MotionRequestBuilderConstPtr</name>
    <filename>classrobowflex_1_1MotionRequestBuilderConstPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::MotionRequestBuilderPtr</name>
    <filename>classrobowflex_1_1MotionRequestBuilderPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::movegroup::MoveGroupHelper</name>
    <filename>classrobowflex_1_1movegroup_1_1MoveGroupHelper.html</filename>
    <class kind="struct">robowflex::movegroup::MoveGroupHelper::Action</class>
    <member kind="typedef">
      <type>std::function&lt; void(Action &amp;)&gt;</type>
      <name>ResultCallback</name>
      <anchorfile>classrobowflex_1_1movegroup_1_1MoveGroupHelper.html</anchorfile>
      <anchor>a371a7fb1b1b5a1adf4e6995dd457b859</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>MoveGroupHelper</name>
      <anchorfile>classrobowflex_1_1movegroup_1_1MoveGroupHelper.html</anchorfile>
      <anchor>a0c14dc5175bb82a10f94ac2068e9ee10</anchor>
      <arglist>(const std::string &amp;move_group=MOVE_GROUP)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~MoveGroupHelper</name>
      <anchorfile>classrobowflex_1_1movegroup_1_1MoveGroupHelper.html</anchorfile>
      <anchor>ab3ad0daffd247637d701a3a84422df1d</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setResultCallback</name>
      <anchorfile>classrobowflex_1_1movegroup_1_1MoveGroupHelper.html</anchorfile>
      <anchor>a970bf941c0e5568294a078a750d1bce1</anchor>
      <arglist>(const ResultCallback &amp;callback)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>executeTrajectory</name>
      <anchorfile>classrobowflex_1_1movegroup_1_1MoveGroupHelper.html</anchorfile>
      <anchor>a674191a847e98283753bc0d23f7e251e</anchor>
      <arglist>(const robot_trajectory::RobotTrajectory &amp;path)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>pullState</name>
      <anchorfile>classrobowflex_1_1movegroup_1_1MoveGroupHelper.html</anchorfile>
      <anchor>aba8785f28b04fedbd3bef240eadfe15b</anchor>
      <arglist>(RobotPtr &amp;robot)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>pullScene</name>
      <anchorfile>classrobowflex_1_1movegroup_1_1MoveGroupHelper.html</anchorfile>
      <anchor>ae0b71e14c39e515e6191861f02c2fc6c</anchor>
      <arglist>(ScenePtr &amp;scene)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>pushScene</name>
      <anchorfile>classrobowflex_1_1movegroup_1_1MoveGroupHelper.html</anchorfile>
      <anchor>a10b21dcf26d5bc6ac007303a4a9be265</anchor>
      <arglist>(const SceneConstPtr &amp;scene)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>moveGroupGoalCallback</name>
      <anchorfile>classrobowflex_1_1movegroup_1_1MoveGroupHelper.html</anchorfile>
      <anchor>a64ea8f29252c8def019fec0518e51db0</anchor>
      <arglist>(const moveit_msgs::MoveGroupActionGoal &amp;msg)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>moveGroupResultCallback</name>
      <anchorfile>classrobowflex_1_1movegroup_1_1MoveGroupHelper.html</anchorfile>
      <anchor>aa460b8e53190d65264b1550ad97f0df4</anchor>
      <arglist>(const moveit_msgs::MoveGroupActionResult &amp;msg)</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ros::NodeHandle</type>
      <name>nh_</name>
      <anchorfile>classrobowflex_1_1movegroup_1_1MoveGroupHelper.html</anchorfile>
      <anchor>abc96e5c11b5b338f5b21d5ff1c64ec79</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ros::Subscriber</type>
      <name>goal_sub_</name>
      <anchorfile>classrobowflex_1_1movegroup_1_1MoveGroupHelper.html</anchorfile>
      <anchor>a8529bc15498cb313ecf4c65f63e6e93c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ros::Subscriber</type>
      <name>result_sub_</name>
      <anchorfile>classrobowflex_1_1movegroup_1_1MoveGroupHelper.html</anchorfile>
      <anchor>a70fc91aadc3960867b1cfee4306ec57c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ros::ServiceClient</type>
      <name>gpsc_</name>
      <anchorfile>classrobowflex_1_1movegroup_1_1MoveGroupHelper.html</anchorfile>
      <anchor>a20fbb3989810a7811793ffea77072fc5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ros::ServiceClient</type>
      <name>apsc_</name>
      <anchorfile>classrobowflex_1_1movegroup_1_1MoveGroupHelper.html</anchorfile>
      <anchor>af9be882e19c6a472a4c1a4b7e7da1815</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>actionlib::SimpleActionClient&lt; moveit_msgs::ExecuteTrajectoryAction &gt;</type>
      <name>eac_</name>
      <anchorfile>classrobowflex_1_1movegroup_1_1MoveGroupHelper.html</anchorfile>
      <anchor>abf97f68cb1383db63ed8e30a5a7b7d4a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ResultCallback</type>
      <name>callback_</name>
      <anchorfile>classrobowflex_1_1movegroup_1_1MoveGroupHelper.html</anchorfile>
      <anchor>a1f9719a7523dbb1009b5bde28c1a2ee8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::map&lt; std::string, Action &gt;</type>
      <name>requests_</name>
      <anchorfile>classrobowflex_1_1movegroup_1_1MoveGroupHelper.html</anchorfile>
      <anchor>ae086348eb164103f1e08f51bf18b67c3</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>RobotPtr</type>
      <name>robot_</name>
      <anchorfile>classrobowflex_1_1movegroup_1_1MoveGroupHelper.html</anchorfile>
      <anchor>a39020060a55295de1599ee75ec3535ec</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static const std::string</type>
      <name>MOVE_GROUP</name>
      <anchorfile>classrobowflex_1_1movegroup_1_1MoveGroupHelper.html</anchorfile>
      <anchor>af785f6ae98abc7954c79263a787a7fa5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static const std::string</type>
      <name>GET_SCENE</name>
      <anchorfile>classrobowflex_1_1movegroup_1_1MoveGroupHelper.html</anchorfile>
      <anchor>acd02e61e67c5e2b126739359ac1d6312</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static const std::string</type>
      <name>APPLY_SCENE</name>
      <anchorfile>classrobowflex_1_1movegroup_1_1MoveGroupHelper.html</anchorfile>
      <anchor>ae2293a330de6858a7b26a9b1f1c8bdf6</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static const std::string</type>
      <name>EXECUTE</name>
      <anchorfile>classrobowflex_1_1movegroup_1_1MoveGroupHelper.html</anchorfile>
      <anchor>a046fe8f5c7cfb168bdc361a5a6d3a745</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>robowflex::movegroup::MoveGroupHelper::Action</name>
    <filename>structrobowflex_1_1movegroup_1_1MoveGroupHelper_1_1Action.html</filename>
    <member kind="function">
      <type>bool</type>
      <name>fromYAMLFile</name>
      <anchorfile>structrobowflex_1_1movegroup_1_1MoveGroupHelper_1_1Action.html</anchorfile>
      <anchor>a2a35ff6d912acf00e56404ded224181d</anchor>
      <arglist>(const std::string &amp;file)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>toYAMLFile</name>
      <anchorfile>structrobowflex_1_1movegroup_1_1MoveGroupHelper_1_1Action.html</anchorfile>
      <anchor>a41585d8b8edc08261c1282a17d466877</anchor>
      <arglist>(const std::string &amp;file)</arglist>
    </member>
    <member kind="variable">
      <type>std::string</type>
      <name>id</name>
      <anchorfile>structrobowflex_1_1movegroup_1_1MoveGroupHelper_1_1Action.html</anchorfile>
      <anchor>ae5d945b5cfd8c56dabad9da028311a8a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>ScenePtr</type>
      <name>scene</name>
      <anchorfile>structrobowflex_1_1movegroup_1_1MoveGroupHelper_1_1Action.html</anchorfile>
      <anchor>ac6ad3e11d4f50edf883c5f7676524617</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>moveit_msgs::MotionPlanRequest</type>
      <name>request</name>
      <anchorfile>structrobowflex_1_1movegroup_1_1MoveGroupHelper_1_1Action.html</anchorfile>
      <anchor>a372dab226d581d9b0b5a77d224625077</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>bool</type>
      <name>success</name>
      <anchorfile>structrobowflex_1_1movegroup_1_1MoveGroupHelper_1_1Action.html</anchorfile>
      <anchor>a049f6ff314c303872ee1df012e1446b2</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>double</type>
      <name>time</name>
      <anchorfile>structrobowflex_1_1movegroup_1_1MoveGroupHelper_1_1Action.html</anchorfile>
      <anchor>abca8672dbd6ad06ec7397ce09a00dfa4</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>moveit_msgs::RobotTrajectory</type>
      <name>trajectory</name>
      <anchorfile>structrobowflex_1_1movegroup_1_1MoveGroupHelper_1_1Action.html</anchorfile>
      <anchor>ade9c29ff8d841d06b2c63a4547b38844</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::OMPL::FetchOMPLPipelinePlanner</name>
    <filename>classrobowflex_1_1OMPL_1_1FetchOMPLPipelinePlanner.html</filename>
    <base>robowflex::OMPL::OMPLPipelinePlanner</base>
    <member kind="function">
      <type></type>
      <name>FetchOMPLPipelinePlanner</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1FetchOMPLPipelinePlanner.html</anchorfile>
      <anchor>ae1cf6fd4c1252bfd48aca0b7e98ffd15</anchor>
      <arglist>(const RobotPtr &amp;robot, const std::string &amp;name=&quot;&quot;)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>initialize</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1FetchOMPLPipelinePlanner.html</anchorfile>
      <anchor>a554e388d3753191ee63f1cfc2a41c828</anchor>
      <arglist>(const Settings &amp;settings=Settings(), const std::string &amp;config_file=CONFIG, const std::string &amp;plugin=DEFAULT_PLUGIN, const std::vector&lt; std::string &gt; &amp;adapters=DEFAULT_ADAPTERS)</arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static const std::string</type>
      <name>CONFIG</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1FetchOMPLPipelinePlanner.html</anchorfile>
      <anchor>ac16a9c1b129d6fcd73cfd4d89104ec36</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::OMPL::FetchOMPLPipelinePlannerConstPtr</name>
    <filename>classrobowflex_1_1OMPL_1_1FetchOMPLPipelinePlannerConstPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::OMPL::FetchOMPLPipelinePlannerPtr</name>
    <filename>classrobowflex_1_1OMPL_1_1FetchOMPLPipelinePlannerPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::OMPL::OMPLInterfacePlanner</name>
    <filename>classrobowflex_1_1OMPL_1_1OMPLInterfacePlanner.html</filename>
    <base>robowflex::Planner</base>
    <member kind="function">
      <type></type>
      <name>OMPLInterfacePlanner</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1OMPLInterfacePlanner.html</anchorfile>
      <anchor>aed3cb24ee55046ce3033bba0256a0203</anchor>
      <arglist>(const RobotPtr &amp;robot, const std::string &amp;name=&quot;&quot;)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>OMPLInterfacePlanner</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1OMPLInterfacePlanner.html</anchorfile>
      <anchor>a34f4005c3c3e4f25cd883ef1d5d44ac0</anchor>
      <arglist>(OMPLInterfacePlanner const &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>operator=</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1OMPLInterfacePlanner.html</anchorfile>
      <anchor>acf43c03a5153b614b1e6217ebd386c68</anchor>
      <arglist>(OMPLInterfacePlanner const &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>initialize</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1OMPLInterfacePlanner.html</anchorfile>
      <anchor>a7a89f91ee9543fe0e075f1bbffd6922a</anchor>
      <arglist>(const std::string &amp;config_file=&quot;&quot;, const OMPL::Settings settings=Settings())</arglist>
    </member>
    <member kind="function">
      <type>planning_interface::MotionPlanResponse</type>
      <name>plan</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1OMPLInterfacePlanner.html</anchorfile>
      <anchor>a87749080b4e88fbeb0d4169121ebde90</anchor>
      <arglist>(const SceneConstPtr &amp;scene, const planning_interface::MotionPlanRequest &amp;request) override</arglist>
    </member>
    <member kind="function">
      <type>ompl_interface::ModelBasedPlanningContextPtr</type>
      <name>getPlanningContext</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1OMPLInterfacePlanner.html</anchorfile>
      <anchor>afd9d99ef99d732011efa58e0e053436b</anchor>
      <arglist>(const SceneConstPtr &amp;scene, const planning_interface::MotionPlanRequest &amp;request)</arglist>
    </member>
    <member kind="function">
      <type>const std::vector&lt; std::string &gt;</type>
      <name>getPlannerConfigs</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1OMPLInterfacePlanner.html</anchorfile>
      <anchor>a723be26e5fed9d74efb4cb297adf4c75</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ompl_interface::OMPLInterface</type>
      <name>interface_</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1OMPLInterfacePlanner.html</anchorfile>
      <anchor>a537c4cc9a4f2fd61b6b8e716e7f169c6</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; std::string &gt;</type>
      <name>configs_</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1OMPLInterfacePlanner.html</anchorfile>
      <anchor>a0e28fc8eb79d051b27bd2d027680d683</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::OMPL::OMPLInterfacePlannerConstPtr</name>
    <filename>classrobowflex_1_1OMPL_1_1OMPLInterfacePlannerConstPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::OMPL::OMPLInterfacePlannerPtr</name>
    <filename>classrobowflex_1_1OMPL_1_1OMPLInterfacePlannerPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::OMPL::OMPLPipelinePlanner</name>
    <filename>classrobowflex_1_1OMPL_1_1OMPLPipelinePlanner.html</filename>
    <base>robowflex::PipelinePlanner</base>
    <member kind="function">
      <type></type>
      <name>OMPLPipelinePlanner</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1OMPLPipelinePlanner.html</anchorfile>
      <anchor>a226afb6ec94fbdf09e23cdebe0226d18</anchor>
      <arglist>(const RobotPtr &amp;robot, const std::string &amp;name=&quot;&quot;)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>OMPLPipelinePlanner</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1OMPLPipelinePlanner.html</anchorfile>
      <anchor>af4afedd4884271f255cd1b808638a491</anchor>
      <arglist>(OMPLPipelinePlanner const &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>operator=</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1OMPLPipelinePlanner.html</anchorfile>
      <anchor>a608b8216b621842274b4c9fc2d8b46d4</anchor>
      <arglist>(OMPLPipelinePlanner const &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>initialize</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1OMPLPipelinePlanner.html</anchorfile>
      <anchor>ac47f87595636ca0f7b6c0e189ad58a9d</anchor>
      <arglist>(const std::string &amp;config_file=&quot;&quot;, const Settings &amp;settings=Settings(), const std::string &amp;plugin=DEFAULT_PLUGIN, const std::vector&lt; std::string &gt; &amp;adapters=DEFAULT_ADAPTERS)</arglist>
    </member>
    <member kind="function">
      <type>const std::vector&lt; std::string &gt;</type>
      <name>getPlannerConfigs</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1OMPLPipelinePlanner.html</anchorfile>
      <anchor>a237ed251659a5395a3d20718720010d3</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="variable" protection="protected" static="yes">
      <type>static const std::string</type>
      <name>DEFAULT_PLUGIN</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1OMPLPipelinePlanner.html</anchorfile>
      <anchor>a66c19012c5ad5fb6cafdefde47b9bc99</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected" static="yes">
      <type>static const std::vector&lt; std::string &gt;</type>
      <name>DEFAULT_ADAPTERS</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1OMPLPipelinePlanner.html</anchorfile>
      <anchor>a6aff3c16c7c1c62d37be46d2dc4f9003</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; std::string &gt;</type>
      <name>configs_</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1OMPLPipelinePlanner.html</anchorfile>
      <anchor>a2379db965cd98f3b0860fce49eaf016c</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::OMPL::OMPLPipelinePlannerConstPtr</name>
    <filename>classrobowflex_1_1OMPL_1_1OMPLPipelinePlannerConstPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::OMPL::OMPLPipelinePlannerPtr</name>
    <filename>classrobowflex_1_1OMPL_1_1OMPLPipelinePlannerPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::OMPL::R2OMPLPipelinePlanner</name>
    <filename>classrobowflex_1_1OMPL_1_1R2OMPLPipelinePlanner.html</filename>
    <base>robowflex::OMPL::OMPLPipelinePlanner</base>
    <member kind="function">
      <type></type>
      <name>R2OMPLPipelinePlanner</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1R2OMPLPipelinePlanner.html</anchorfile>
      <anchor>a13df7922cabce0d97263049642fd457f</anchor>
      <arglist>(const R2RobotPtr &amp;robot, const std::string &amp;name=&quot;&quot;)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>initialize</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1R2OMPLPipelinePlanner.html</anchorfile>
      <anchor>a0ef59e418332414ebb78e1407e3d0577</anchor>
      <arglist>(const std::string &amp;config_file=CONFIG, const Settings &amp;settings=Settings(), const std::string &amp;plugin=PLUGIN, const std::vector&lt; std::string &gt; &amp;adapters=DEFAULT_ADAPTERS)</arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static const std::string</type>
      <name>CONFIG</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1R2OMPLPipelinePlanner.html</anchorfile>
      <anchor>a4f9474af2a88b6f640296ccd51c07686</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static const std::string</type>
      <name>PLUGIN</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1R2OMPLPipelinePlanner.html</anchorfile>
      <anchor>a0d439b6ee1e3c9c4688baefdf0114010</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::OMPL::R2OMPLPipelinePlannerConstPtr</name>
    <filename>classrobowflex_1_1OMPL_1_1R2OMPLPipelinePlannerConstPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::OMPL::R2OMPLPipelinePlannerPtr</name>
    <filename>classrobowflex_1_1OMPL_1_1R2OMPLPipelinePlannerPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::OMPL::Settings</name>
    <filename>classrobowflex_1_1OMPL_1_1Settings.html</filename>
    <member kind="function">
      <type></type>
      <name>Settings</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1Settings.html</anchorfile>
      <anchor>ac4c80ad8b016c623a473a2511df89922</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setParam</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1Settings.html</anchorfile>
      <anchor>a2f4447a6c4afc913a7ba8d1f928af795</anchor>
      <arglist>(IO::Handler &amp;handler) const</arglist>
    </member>
    <member kind="variable">
      <type>int</type>
      <name>max_goal_samples</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1Settings.html</anchorfile>
      <anchor>a5f65003be8a97f6bef473c69320a4bee</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>int</type>
      <name>max_goal_sampling_attempts</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1Settings.html</anchorfile>
      <anchor>ad9643e6653230ed8dfdcdf7b859566f7</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>int</type>
      <name>max_planning_threads</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1Settings.html</anchorfile>
      <anchor>a6a65421d3030b5829688f0b98945393f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>double</type>
      <name>max_solution_segment_length</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1Settings.html</anchorfile>
      <anchor>ab132ef5502cac543057055470524aa6d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>int</type>
      <name>max_state_sampling_attempts</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1Settings.html</anchorfile>
      <anchor>a652200d2ce72f7513ad33b9ac7e64e97</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>int</type>
      <name>minimum_waypoint_count</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1Settings.html</anchorfile>
      <anchor>a558849374a0a7e54ae4f21e381ed1987</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>bool</type>
      <name>simplify_solutions</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1Settings.html</anchorfile>
      <anchor>ad2121bb6c9d309f91daab07860bce595</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>bool</type>
      <name>use_constraints_approximations</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1Settings.html</anchorfile>
      <anchor>a2bebaa65d6082c72e0de5fcca2566af8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>bool</type>
      <name>display_random_valid_states</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1Settings.html</anchorfile>
      <anchor>a31fe8edf43e6932f0cbf6dd994e99c8e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>std::string</type>
      <name>link_for_exploration_tree</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1Settings.html</anchorfile>
      <anchor>ae8df3ed47993dc14f26944bf770cc3c8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>double</type>
      <name>maximum_waypoint_distance</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1Settings.html</anchorfile>
      <anchor>ac1b01bfbcbd3513340802f3949a65dd9</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::OMPL::UR5OMPLPipelinePlanner</name>
    <filename>classrobowflex_1_1OMPL_1_1UR5OMPLPipelinePlanner.html</filename>
    <base>robowflex::OMPL::OMPLPipelinePlanner</base>
    <member kind="function">
      <type></type>
      <name>UR5OMPLPipelinePlanner</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1UR5OMPLPipelinePlanner.html</anchorfile>
      <anchor>a513b56c348890de224d76191380064c5</anchor>
      <arglist>(const RobotPtr &amp;robot, const std::string &amp;name=&quot;&quot;)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>initialize</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1UR5OMPLPipelinePlanner.html</anchorfile>
      <anchor>a8b88ebc1ed22cc6a4f71e8cb5eaaaa92</anchor>
      <arglist>(const Settings &amp;settings=Settings(), const std::string &amp;config_file=CONFIG, const std::string &amp;plugin=DEFAULT_PLUGIN, const std::vector&lt; std::string &gt; &amp;adapters=DEFAULT_ADAPTERS)</arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static const std::string</type>
      <name>CONFIG</name>
      <anchorfile>classrobowflex_1_1OMPL_1_1UR5OMPLPipelinePlanner.html</anchorfile>
      <anchor>a0809148d9dedf3268d2f5917d7ab9226</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::OMPL::UR5OMPLPipelinePlannerConstPtr</name>
    <filename>classrobowflex_1_1OMPL_1_1UR5OMPLPipelinePlannerConstPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::OMPL::UR5OMPLPipelinePlannerPtr</name>
    <filename>classrobowflex_1_1OMPL_1_1UR5OMPLPipelinePlannerPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::OMPLBenchmarkOutputter</name>
    <filename>classrobowflex_1_1OMPLBenchmarkOutputter.html</filename>
    <base>robowflex::BenchmarkOutputter</base>
    <member kind="function">
      <type></type>
      <name>OMPLBenchmarkOutputter</name>
      <anchorfile>classrobowflex_1_1OMPLBenchmarkOutputter.html</anchorfile>
      <anchor>a149c1f9bbd120917cbb6932c858e34a6</anchor>
      <arglist>(const std::string &amp;prefix)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~OMPLBenchmarkOutputter</name>
      <anchorfile>classrobowflex_1_1OMPLBenchmarkOutputter.html</anchorfile>
      <anchor>a33c49c23a6fe5acdbe5c82a96161d461</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>dumpResult</name>
      <anchorfile>classrobowflex_1_1OMPLBenchmarkOutputter.html</anchorfile>
      <anchor>af94db46b8767da9621c59338e7b30398</anchor>
      <arglist>(const Benchmarker::Results &amp;results) override</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const std::string</type>
      <name>prefix_</name>
      <anchorfile>classrobowflex_1_1OMPLBenchmarkOutputter.html</anchorfile>
      <anchor>ac6c9d83c4ecbea83eda898615ed29012</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::ParamRobot</name>
    <filename>classrobowflex_1_1ParamRobot.html</filename>
    <base>robowflex::Robot</base>
    <member kind="function">
      <type></type>
      <name>ParamRobot</name>
      <anchorfile>classrobowflex_1_1ParamRobot.html</anchorfile>
      <anchor>add98ad8cab1c8680a034acb9d7eb2511</anchor>
      <arglist>(const std::string &amp;name=&quot;DEFAULT&quot;)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::ParamRobotConstPtr</name>
    <filename>classrobowflex_1_1ParamRobotConstPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::ParamRobotPtr</name>
    <filename>classrobowflex_1_1ParamRobotPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::PipelinePlanner</name>
    <filename>classrobowflex_1_1PipelinePlanner.html</filename>
    <base>robowflex::Planner</base>
    <member kind="function">
      <type></type>
      <name>PipelinePlanner</name>
      <anchorfile>classrobowflex_1_1PipelinePlanner.html</anchorfile>
      <anchor>a192de26dd90b1fc380df49e350c0e032</anchor>
      <arglist>(const RobotPtr &amp;robot, const std::string &amp;name=&quot;&quot;)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>PipelinePlanner</name>
      <anchorfile>classrobowflex_1_1PipelinePlanner.html</anchorfile>
      <anchor>a1a68eaa392c3d7fc12592ab1c1f7bf12</anchor>
      <arglist>(PipelinePlanner const &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>operator=</name>
      <anchorfile>classrobowflex_1_1PipelinePlanner.html</anchorfile>
      <anchor>aa6b0ec72daacfca53494f8ff313b185c</anchor>
      <arglist>(PipelinePlanner const &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>planning_interface::MotionPlanResponse</type>
      <name>plan</name>
      <anchorfile>classrobowflex_1_1PipelinePlanner.html</anchorfile>
      <anchor>af45699d4dc7608422e667632b61bd57d</anchor>
      <arglist>(const SceneConstPtr &amp;scene, const planning_interface::MotionPlanRequest &amp;request) override</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>planning_pipeline::PlanningPipelinePtr</type>
      <name>pipeline_</name>
      <anchorfile>classrobowflex_1_1PipelinePlanner.html</anchorfile>
      <anchor>a247ba3b8df0aaa415604b1c27472df1a</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::PipelinePlannerConstPtr</name>
    <filename>classrobowflex_1_1PipelinePlannerConstPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::PipelinePlannerPtr</name>
    <filename>classrobowflex_1_1PipelinePlannerPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::Planner</name>
    <filename>classrobowflex_1_1Planner.html</filename>
    <member kind="function">
      <type></type>
      <name>Planner</name>
      <anchorfile>classrobowflex_1_1Planner.html</anchorfile>
      <anchor>ac33eb030232bc7c062fa5e314f377797</anchor>
      <arglist>(const RobotPtr &amp;robot, const std::string &amp;name=&quot;&quot;)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Planner</name>
      <anchorfile>classrobowflex_1_1Planner.html</anchorfile>
      <anchor>a40932b451c2f36646ec79186517008d0</anchor>
      <arglist>(Planner const &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>operator=</name>
      <anchorfile>classrobowflex_1_1Planner.html</anchorfile>
      <anchor>a438413f9f58cb0f53586825b80ed55e2</anchor>
      <arglist>(Planner const &amp;)=delete</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual planning_interface::MotionPlanResponse</type>
      <name>plan</name>
      <anchorfile>classrobowflex_1_1Planner.html</anchorfile>
      <anchor>a7d960d5d92a79e9be51c6f97518635b6</anchor>
      <arglist>(const SceneConstPtr &amp;scene, const planning_interface::MotionPlanRequest &amp;request)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual const std::vector&lt; std::string &gt;</type>
      <name>getPlannerConfigs</name>
      <anchorfile>classrobowflex_1_1Planner.html</anchorfile>
      <anchor>a89918d3df60622c648acba1a9f6e547d</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function">
      <type>const RobotPtr</type>
      <name>getRobot</name>
      <anchorfile>classrobowflex_1_1Planner.html</anchorfile>
      <anchor>a0dc851c5cdc24e5cb9656066ff232ad2</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const std::string &amp;</type>
      <name>getName</name>
      <anchorfile>classrobowflex_1_1Planner.html</anchorfile>
      <anchor>a6613fb90892f07bc96047a11126d370a</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>RobotPtr</type>
      <name>robot_</name>
      <anchorfile>classrobowflex_1_1Planner.html</anchorfile>
      <anchor>a863cfac02101ae20c367a3db489af1a1</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>IO::Handler</type>
      <name>handler_</name>
      <anchorfile>classrobowflex_1_1Planner.html</anchorfile>
      <anchor>a731773cb15cb42bf5d79c94075980a74</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>const std::string</type>
      <name>name_</name>
      <anchorfile>classrobowflex_1_1Planner.html</anchorfile>
      <anchor>a6eed2da0765a0c26ea54ed62728c9c01</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::PlannerConstPtr</name>
    <filename>classrobowflex_1_1PlannerConstPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::PlannerPtr</name>
    <filename>classrobowflex_1_1PlannerPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::Pool</name>
    <filename>classrobowflex_1_1Pool.html</filename>
    <class kind="class">robowflex::Pool::Job</class>
    <class kind="class">robowflex::Pool::Joblet</class>
    <member kind="function">
      <type></type>
      <name>Pool</name>
      <anchorfile>classrobowflex_1_1Pool.html</anchorfile>
      <anchor>a2272e59defd3e3d447eb52563daacc5d</anchor>
      <arglist>(unsigned int n=std::thread::hardware_concurrency())</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~Pool</name>
      <anchorfile>classrobowflex_1_1Pool.html</anchorfile>
      <anchor>a2157e55ddcb79eec5d74468ad49e6324</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>unsigned int</type>
      <name>getThreadCount</name>
      <anchorfile>classrobowflex_1_1Pool.html</anchorfile>
      <anchor>a0b2265b607414ab380e6fbfeadac969b</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; Job&lt; RT &gt; &gt;</type>
      <name>submit</name>
      <anchorfile>classrobowflex_1_1Pool.html</anchorfile>
      <anchor>af9e55c05b0d527d5d618cf21b034f15a</anchor>
      <arglist>(const std::function&lt; RT(Args...)&gt; &amp;&amp;function, Args &amp;&amp;... args) const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>run</name>
      <anchorfile>classrobowflex_1_1Pool.html</anchorfile>
      <anchor>ac89bbf31e9157a51bafb676d3669852c</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>bool</type>
      <name>active_</name>
      <anchorfile>classrobowflex_1_1Pool.html</anchorfile>
      <anchor>a180ace568cec016b9b8467e27ca8fe66</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::mutex</type>
      <name>mutex_</name>
      <anchorfile>classrobowflex_1_1Pool.html</anchorfile>
      <anchor>a08b629e2be02fcee63a8e84d27a9e137</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::condition_variable</type>
      <name>cv_</name>
      <anchorfile>classrobowflex_1_1Pool.html</anchorfile>
      <anchor>a78e5921e6eb7fc9830cdebaaa1ba49da</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; std::thread &gt;</type>
      <name>threads_</name>
      <anchorfile>classrobowflex_1_1Pool.html</anchorfile>
      <anchor>ac113aa3b99af5ee27916ddf514e8b06b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::queue&lt; std::shared_ptr&lt; Joblet &gt; &gt;</type>
      <name>jobs_</name>
      <anchorfile>classrobowflex_1_1Pool.html</anchorfile>
      <anchor>af077037bc6dfd6c0b17b5a65404cf4c9</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::Pool::Job</name>
    <filename>classrobowflex_1_1Pool_1_1Job.html</filename>
    <templarg></templarg>
    <base>robowflex::Pool::Joblet</base>
    <member kind="function">
      <type></type>
      <name>Job</name>
      <anchorfile>classrobowflex_1_1Pool_1_1Job.html</anchorfile>
      <anchor>abd55ca16a4741e2501d25cb9151a90d7</anchor>
      <arglist>(const std::function&lt; RT(Args...)&gt; &amp;&amp;function, Args &amp;&amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>execute</name>
      <anchorfile>classrobowflex_1_1Pool_1_1Job.html</anchorfile>
      <anchor>af619f4fcea3fc3c1c16e8b8cedce0432</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>RT</type>
      <name>get</name>
      <anchorfile>classrobowflex_1_1Pool_1_1Job.html</anchorfile>
      <anchor>a02d9ac474d09b9b963ef20a6a4841eba</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>wait</name>
      <anchorfile>classrobowflex_1_1Pool_1_1Job.html</anchorfile>
      <anchor>a9fb274d5f66026562c90a0b85315466f</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::function&lt; RT()&gt;</type>
      <name>function_</name>
      <anchorfile>classrobowflex_1_1Pool_1_1Job.html</anchorfile>
      <anchor>a57a6a66b3c7f0c8e223e800a5c9df4e4</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::packaged_task&lt; RT()&gt;</type>
      <name>task_</name>
      <anchorfile>classrobowflex_1_1Pool_1_1Job.html</anchorfile>
      <anchor>ae8a61a423bb305cc4b7cbc0b917c3a0a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::future&lt; RT &gt;</type>
      <name>future_</name>
      <anchorfile>classrobowflex_1_1Pool_1_1Job.html</anchorfile>
      <anchor>a3f9e8b0a0c282bdb11653d1e64488bac</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::Pool::Joblet</name>
    <filename>classrobowflex_1_1Pool_1_1Joblet.html</filename>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>execute</name>
      <anchorfile>classrobowflex_1_1Pool_1_1Joblet.html</anchorfile>
      <anchor>a0e6b92a92c4599a81225019c17fbebcc</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>cancel</name>
      <anchorfile>classrobowflex_1_1Pool_1_1Joblet.html</anchorfile>
      <anchor>af0147d96c2d056fa2feeb2fd9c416314</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>isCancled</name>
      <anchorfile>classrobowflex_1_1Pool_1_1Joblet.html</anchorfile>
      <anchor>ab06d4bda05a0ebb0215b24538b819ead</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>bool</type>
      <name>canceled</name>
      <anchorfile>classrobowflex_1_1Pool_1_1Joblet.html</anchorfile>
      <anchor>adf02f7be92d21e53128e42db925db8b8</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::PoolPlanner</name>
    <filename>classrobowflex_1_1PoolPlanner.html</filename>
    <base>robowflex::Planner</base>
    <member kind="function">
      <type></type>
      <name>PoolPlanner</name>
      <anchorfile>classrobowflex_1_1PoolPlanner.html</anchorfile>
      <anchor>af99ff1f3b0f797a6531065ef253ce340</anchor>
      <arglist>(const RobotPtr &amp;robot, unsigned int n=std::thread::hardware_concurrency(), const std::string &amp;name=&quot;&quot;)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>PoolPlanner</name>
      <anchorfile>classrobowflex_1_1PoolPlanner.html</anchorfile>
      <anchor>a64d21ec5d0936635d8d2915cb7fea1da</anchor>
      <arglist>(PoolPlanner const &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>operator=</name>
      <anchorfile>classrobowflex_1_1PoolPlanner.html</anchorfile>
      <anchor>a9d7f128ad2eadf33c0cc12a9fd25b0e1</anchor>
      <arglist>(PoolPlanner const &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>initialize</name>
      <anchorfile>classrobowflex_1_1PoolPlanner.html</anchorfile>
      <anchor>ae2b4ff45a4701cd36bb90c6d00f34ae7</anchor>
      <arglist>(Args &amp;&amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; Pool::Job&lt; planning_interface::MotionPlanResponse &gt; &gt;</type>
      <name>submit</name>
      <anchorfile>classrobowflex_1_1PoolPlanner.html</anchorfile>
      <anchor>a09696a9e660b3128c62fe0c0fefcac88</anchor>
      <arglist>(const SceneConstPtr &amp;scene, const planning_interface::MotionPlanRequest &amp;request)</arglist>
    </member>
    <member kind="function">
      <type>planning_interface::MotionPlanResponse</type>
      <name>plan</name>
      <anchorfile>classrobowflex_1_1PoolPlanner.html</anchorfile>
      <anchor>a9aa99599129d083dd229affb8be8ba2d</anchor>
      <arglist>(const SceneConstPtr &amp;scene, const planning_interface::MotionPlanRequest &amp;request) override</arglist>
    </member>
    <member kind="function">
      <type>const std::vector&lt; std::string &gt;</type>
      <name>getPlannerConfigs</name>
      <anchorfile>classrobowflex_1_1PoolPlanner.html</anchorfile>
      <anchor>a9c6b1d4df283de96bae93522726e7237</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>Pool</type>
      <name>pool_</name>
      <anchorfile>classrobowflex_1_1PoolPlanner.html</anchorfile>
      <anchor>a69c8891cf824d19d89edec57c296bbdc</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::queue&lt; PlannerPtr &gt;</type>
      <name>planners_</name>
      <anchorfile>classrobowflex_1_1PoolPlanner.html</anchorfile>
      <anchor>ac8122ce06d9cc41b02b3878f1aecef39</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::mutex</type>
      <name>mutex_</name>
      <anchorfile>classrobowflex_1_1PoolPlanner.html</anchorfile>
      <anchor>a5c8165adc36be613dc5421723ec917f6</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::condition_variable</type>
      <name>cv_</name>
      <anchorfile>classrobowflex_1_1PoolPlanner.html</anchorfile>
      <anchor>a9e09ba8ecf61daf165f37256df7ff402</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::R2Robot</name>
    <filename>classrobowflex_1_1R2Robot.html</filename>
    <base>robowflex::Robot</base>
    <member kind="function">
      <type></type>
      <name>R2Robot</name>
      <anchorfile>classrobowflex_1_1R2Robot.html</anchorfile>
      <anchor>a2b90dcea5765e94c26f1e9424424a721</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>initialize</name>
      <anchorfile>classrobowflex_1_1R2Robot.html</anchorfile>
      <anchor>a6472ac39b20712da48f1da9080135c9e</anchor>
      <arglist>(const std::vector&lt; std::string &gt; kinematics)</arglist>
    </member>
    <member kind="function">
      <type>robot_trajectory::RobotTrajectoryPtr</type>
      <name>loadSMTData</name>
      <anchorfile>classrobowflex_1_1R2Robot.html</anchorfile>
      <anchor>a9a83c7b1c959a990f02f7df513e5377d</anchor>
      <arglist>(const std::string &amp;filename)</arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static const std::string</type>
      <name>URDF</name>
      <anchorfile>classrobowflex_1_1R2Robot.html</anchorfile>
      <anchor>ad8d95d0fa60449fa49ae2ec076dcd2bb</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static const std::string</type>
      <name>SRDF</name>
      <anchorfile>classrobowflex_1_1R2Robot.html</anchorfile>
      <anchor>aaabe755da865cc4f1683bf3e1422ddde</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static const std::string</type>
      <name>LIMITS</name>
      <anchorfile>classrobowflex_1_1R2Robot.html</anchorfile>
      <anchor>aa4ed2c5eeb4246dc7b9faa8aeac89186</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static const std::string</type>
      <name>KINEMATICS</name>
      <anchorfile>classrobowflex_1_1R2Robot.html</anchorfile>
      <anchor>a68e4c9311caa52f170345cfc4072b9bb</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static const std::string</type>
      <name>CACHED</name>
      <anchorfile>classrobowflex_1_1R2Robot.html</anchorfile>
      <anchor>a4f3dff9ddd28f7f77232946c84aee487</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static const std::vector&lt; std::string &gt;</type>
      <name>SAMPLERS</name>
      <anchorfile>classrobowflex_1_1R2Robot.html</anchorfile>
      <anchor>aaf01a98b45b45146eb9fe53f516b45a9</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static const std::map&lt; std::string, std::string &gt;</type>
      <name>CREEPY</name>
      <anchorfile>classrobowflex_1_1R2Robot.html</anchorfile>
      <anchor>ae01b9425cade14d21885de56075539a6</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::R2RobotConstPtr</name>
    <filename>classrobowflex_1_1R2RobotConstPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::R2RobotPtr</name>
    <filename>classrobowflex_1_1R2RobotPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::Robot</name>
    <filename>classrobowflex_1_1Robot.html</filename>
    <member kind="typedef">
      <type>std::function&lt; bool(YAML::Node &amp;)&gt;</type>
      <name>PostProcessYAMLFunction</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a4b594d34ed151710ce19beba02ad143b</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; bool(tinyxml2::XMLDocument &amp;)&gt;</type>
      <name>PostProcessXMLFunction</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>ad9b7a293d3bec2f0ddc962461b561fae</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Robot</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a457bcf0ddf445edee175402066a74e04</anchor>
      <arglist>(const std::string &amp;name)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Robot</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>ab053bceb0c7c7154e45c8dfbcf3ae14c</anchor>
      <arglist>(Robot const &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>operator=</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a7c783ce042a65a6ed27e1a36eb89f1b0</anchor>
      <arglist>(Robot const &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>initialize</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a933a7af02c404aba03212afe35df0cee</anchor>
      <arglist>(const std::string &amp;urdf_file, const std::string &amp;srdf_file, const std::string &amp;limits_file, const std::string &amp;kinematics_file)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>loadYAMLFile</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a9939fdcbb03c7e6cbdfa40248a8e7ebc</anchor>
      <arglist>(const std::string &amp;name, const std::string &amp;file)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>loadYAMLFile</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>ad4da99659c50e025b4e8342d960984b0</anchor>
      <arglist>(const std::string &amp;name, const std::string &amp;file, const PostProcessYAMLFunction &amp;function)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>loadXMLFile</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a495d035b0828de2fb1355176560a87e5</anchor>
      <arglist>(const std::string &amp;name, const std::string &amp;file)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>loadXMLFile</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a2ab32d2a3264031c7d1b2e55cdd380ec</anchor>
      <arglist>(const std::string &amp;name, const std::string &amp;file, const PostProcessXMLFunction &amp;function)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setURDFPostProcessFunction</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a5ea995b274007a74b8f6181a87e2d5b5</anchor>
      <arglist>(const PostProcessXMLFunction &amp;function)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setSRDFPostProcessFunction</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a57cd4903ff4d8bb6b4702e6c96bce28d</anchor>
      <arglist>(const PostProcessXMLFunction &amp;function)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setLimitsPostProcessFunction</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a0ccb737d3ba5c625e9e86a0531e10cc0</anchor>
      <arglist>(const PostProcessYAMLFunction &amp;function)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setKinematicsPostProcessFunction</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a5de9e5a10617c7fc4e4fe1db787410ee</anchor>
      <arglist>(const PostProcessYAMLFunction &amp;function)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>loadKinematics</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a6d9e07b52b4f4bc9616fca43f09b0837</anchor>
      <arglist>(const std::string &amp;group)</arglist>
    </member>
    <member kind="function">
      <type>const std::string &amp;</type>
      <name>getModelName</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>accf7eea7633af92ccb9b3efdcedb2597</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const std::string &amp;</type>
      <name>getName</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>aa5958035cdbf65441a811b72817ef139</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const robot_model::RobotModelPtr &amp;</type>
      <name>getModelConst</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>ad9bd958af6eca8e2c14a76dc132273e0</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>robot_model::RobotModelPtr &amp;</type>
      <name>getModel</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>acab870ea49f0a952a4568ec88e5b8b58</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>urdf::ModelInterfaceConstSharedPtr</type>
      <name>getURDF</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a802a1e10317966ad1a4b9e8df41e3ddd</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>srdf::ModelConstSharedPtr</type>
      <name>getSRDF</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a5708cab292f3d08ccd9df464b98d301a</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const robot_model::RobotStatePtr &amp;</type>
      <name>getScratchState</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>ae0c9c91c1d39f470a6405f5b93f80c1d</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>robot_model::RobotStatePtr &amp;</type>
      <name>getScratchState</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a7f040576e15e86ee2fd84d15653bdf0b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const IO::Handler &amp;</type>
      <name>getHandlerConst</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a58e6c221f9e3e5039f0a532e31f466b6</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>IO::Handler &amp;</type>
      <name>getHandler</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a4b36261cf7dfdb58d98ec44e79ae7489</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setState</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>ace2a3be0a96fa9459e5aa5b305fb3192</anchor>
      <arglist>(const std::vector&lt; double &gt; &amp;positions)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setState</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>ad6649b9a456ca41d0a5cd7e438d6e3c3</anchor>
      <arglist>(const std::map&lt; std::string, double &gt; &amp;variable_map)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setState</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a64952e20207bed8ef1384e684e3cb6d4</anchor>
      <arglist>(const std::vector&lt; std::string &gt; &amp;variable_names, const std::vector&lt; double &gt; &amp;variable_position)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setState</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>af11e3a72ba5eae5048b2bc93fbbff1cb</anchor>
      <arglist>(const moveit_msgs::RobotState &amp;state)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setGroupState</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a890a77440810c04f5ecd44511d713df1</anchor>
      <arglist>(const std::string &amp;name, const std::vector&lt; double &gt; &amp;positions)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>setFromIK</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a349bbb89bb456954688f3a4db9156190</anchor>
      <arglist>(const std::string &amp;group, const GeometryConstPtr &amp;region, const Eigen::Affine3d &amp;pose, const Eigen::Quaterniond &amp;orientation, const Eigen::Vector3d &amp;tolerances)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; double &gt;</type>
      <name>getState</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a15219846c3e3b04b1449c7e4ff9e08cb</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>getJointNames</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a09a162f5080f2264ad96eaf7ed84569f</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const Eigen::Affine3d &amp;</type>
      <name>getLinkTF</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a64c361a1ce3b9273f7fa4af109203913</anchor>
      <arglist>(const std::string &amp;name) const</arglist>
    </member>
    <member kind="function">
      <type>const Eigen::Affine3d</type>
      <name>getRelativeLinkTF</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a48c6a8846e571f2f191106b6578f181f</anchor>
      <arglist>(const std::string &amp;base, const std::string &amp;target) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>inCollision</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a4ec010b144418cdb8d60976a6915b9f2</anchor>
      <arglist>(const SceneConstPtr &amp;scene) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>dumpGeometry</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a04eb1e0b01ffe651204548001059c28a</anchor>
      <arglist>(const std::string &amp;file) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>dumpPathTransforms</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a221604ce88a58e3082f7e51a5cdc86bd</anchor>
      <arglist>(const robot_trajectory::RobotTrajectory &amp;path, const std::string &amp;filename, double fps=30, double threshold=0.0)</arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static const std::string</type>
      <name>ROBOT_DESCRIPTION</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>ab03e40f41fc386d844838a39f78cbea7</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static const std::string</type>
      <name>ROBOT_SEMANTIC</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>ac577a49c2378411cb1b221e911d2c8ac</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static const std::string</type>
      <name>ROBOT_PLANNING</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>ae8f9a313cf54ed88b470d5e2bd4c36e8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static const std::string</type>
      <name>ROBOT_KINEMATICS</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a10221fa82796ed88e7468abaa9727db8</anchor>
      <arglist></arglist>
    </member>
    <member kind="function" protection="protected">
      <type>bool</type>
      <name>loadRobotDescription</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>aed214621a0f7da30df77b84003fcdfb4</anchor>
      <arglist>(const std::string &amp;urdf_file, const std::string &amp;srdf_file, const std::string &amp;limits_file, const std::string &amp;kinematics_file)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>loadRobotModel</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>ab6591172d7a47d0c5560f0808275edd7</anchor>
      <arglist>(bool namespaced=true)</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>const std::string</type>
      <name>name_</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a5ec5301565f76ffe0c5ce3653249c6ea</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>IO::Handler</type>
      <name>handler_</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a85c721ef13c1e5d3d3febc191febbf95</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>PostProcessXMLFunction</type>
      <name>urdf_function_</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a3aa602cf845075674d69a62cc6f8837f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>PostProcessXMLFunction</type>
      <name>srdf_function_</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a571ff5f19f80e45754e481dff58ab251</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>PostProcessYAMLFunction</type>
      <name>limits_function_</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a5ca41a76660e44d95d046cb8eb9713a8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>PostProcessYAMLFunction</type>
      <name>kinematics_function_</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a75f5ff9171927216c9833d64ba293827</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; robot_model_loader::RobotModelLoader &gt;</type>
      <name>loader_</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>ace593f019215c2eb4db1369db4e69153</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>robot_model::RobotModelPtr</type>
      <name>model_</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>ad31a1bb919e4323b8bf127f43c6f2779</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::map&lt; std::string, robot_model::SolverAllocatorFn &gt;</type>
      <name>imap_</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>acc7912155338a96a81ea6930aa8f65f9</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>kinematics_plugin_loader::KinematicsPluginLoaderPtr</type>
      <name>kinematics_</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a1d2adc884177ed0b6ba42b471bf9bfb1</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>robot_state::RobotStatePtr</type>
      <name>scratch_</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>acdf39eaeec13987878ad792cf5b7517a</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>initialize</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a933a7af02c404aba03212afe35df0cee</anchor>
      <arglist>(const std::string &amp;urdf_file, const std::string &amp;srdf_file, const std::string &amp;limits_file, const std::string &amp;kinematics_file)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>loadYAMLFile</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a9939fdcbb03c7e6cbdfa40248a8e7ebc</anchor>
      <arglist>(const std::string &amp;name, const std::string &amp;file)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>loadYAMLFile</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>ad4da99659c50e025b4e8342d960984b0</anchor>
      <arglist>(const std::string &amp;name, const std::string &amp;file, const PostProcessYAMLFunction &amp;function)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>loadXMLFile</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a495d035b0828de2fb1355176560a87e5</anchor>
      <arglist>(const std::string &amp;name, const std::string &amp;file)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>loadXMLFile</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a2ab32d2a3264031c7d1b2e55cdd380ec</anchor>
      <arglist>(const std::string &amp;name, const std::string &amp;file, const PostProcessXMLFunction &amp;function)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setURDFPostProcessFunction</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a5ea995b274007a74b8f6181a87e2d5b5</anchor>
      <arglist>(const PostProcessXMLFunction &amp;function)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setSRDFPostProcessFunction</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a57cd4903ff4d8bb6b4702e6c96bce28d</anchor>
      <arglist>(const PostProcessXMLFunction &amp;function)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setLimitsPostProcessFunction</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a0ccb737d3ba5c625e9e86a0531e10cc0</anchor>
      <arglist>(const PostProcessYAMLFunction &amp;function)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setKinematicsPostProcessFunction</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a5de9e5a10617c7fc4e4fe1db787410ee</anchor>
      <arglist>(const PostProcessYAMLFunction &amp;function)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>loadKinematics</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a6d9e07b52b4f4bc9616fca43f09b0837</anchor>
      <arglist>(const std::string &amp;group)</arglist>
    </member>
    <member kind="function">
      <type>const std::string &amp;</type>
      <name>getModelName</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>accf7eea7633af92ccb9b3efdcedb2597</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const std::string &amp;</type>
      <name>getName</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>aa5958035cdbf65441a811b72817ef139</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const robot_model::RobotModelPtr &amp;</type>
      <name>getModelConst</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>ad9bd958af6eca8e2c14a76dc132273e0</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>robot_model::RobotModelPtr &amp;</type>
      <name>getModel</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>acab870ea49f0a952a4568ec88e5b8b58</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>urdf::ModelInterfaceConstSharedPtr</type>
      <name>getURDF</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a802a1e10317966ad1a4b9e8df41e3ddd</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>srdf::ModelConstSharedPtr</type>
      <name>getSRDF</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a5708cab292f3d08ccd9df464b98d301a</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const robot_model::RobotStatePtr &amp;</type>
      <name>getScratchState</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>ae0c9c91c1d39f470a6405f5b93f80c1d</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>robot_model::RobotStatePtr &amp;</type>
      <name>getScratchState</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a7f040576e15e86ee2fd84d15653bdf0b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const IO::Handler &amp;</type>
      <name>getHandlerConst</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a58e6c221f9e3e5039f0a532e31f466b6</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>IO::Handler &amp;</type>
      <name>getHandler</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a4b36261cf7dfdb58d98ec44e79ae7489</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setState</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>ace2a3be0a96fa9459e5aa5b305fb3192</anchor>
      <arglist>(const std::vector&lt; double &gt; &amp;positions)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setState</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>ad6649b9a456ca41d0a5cd7e438d6e3c3</anchor>
      <arglist>(const std::map&lt; std::string, double &gt; &amp;variable_map)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setState</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a64952e20207bed8ef1384e684e3cb6d4</anchor>
      <arglist>(const std::vector&lt; std::string &gt; &amp;variable_names, const std::vector&lt; double &gt; &amp;variable_position)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setState</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>af11e3a72ba5eae5048b2bc93fbbff1cb</anchor>
      <arglist>(const moveit_msgs::RobotState &amp;state)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setGroupState</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a890a77440810c04f5ecd44511d713df1</anchor>
      <arglist>(const std::string &amp;name, const std::vector&lt; double &gt; &amp;positions)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>setFromIK</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a349bbb89bb456954688f3a4db9156190</anchor>
      <arglist>(const std::string &amp;group, const GeometryConstPtr &amp;region, const Eigen::Affine3d &amp;pose, const Eigen::Quaterniond &amp;orientation, const Eigen::Vector3d &amp;tolerances)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; double &gt;</type>
      <name>getState</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a15219846c3e3b04b1449c7e4ff9e08cb</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>getJointNames</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a09a162f5080f2264ad96eaf7ed84569f</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const Eigen::Affine3d &amp;</type>
      <name>getLinkTF</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a64c361a1ce3b9273f7fa4af109203913</anchor>
      <arglist>(const std::string &amp;name) const</arglist>
    </member>
    <member kind="function">
      <type>const Eigen::Affine3d</type>
      <name>getRelativeLinkTF</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a48c6a8846e571f2f191106b6578f181f</anchor>
      <arglist>(const std::string &amp;base, const std::string &amp;target) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>inCollision</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a4ec010b144418cdb8d60976a6915b9f2</anchor>
      <arglist>(const SceneConstPtr &amp;scene) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>dumpGeometry</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a04eb1e0b01ffe651204548001059c28a</anchor>
      <arglist>(const std::string &amp;file) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>dumpPathTransforms</name>
      <anchorfile>classrobowflex_1_1Robot.html</anchorfile>
      <anchor>a221604ce88a58e3082f7e51a5cdc86bd</anchor>
      <arglist>(const robot_trajectory::RobotTrajectory &amp;path, const std::string &amp;filename, double fps=30, double threshold=0.0)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::RobotConstPtr</name>
    <filename>classrobowflex_1_1RobotConstPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::RobotPtr</name>
    <filename>classrobowflex_1_1RobotPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::ROS</name>
    <filename>classrobowflex_1_1ROS.html</filename>
    <member kind="function">
      <type></type>
      <name>ROS</name>
      <anchorfile>classrobowflex_1_1ROS.html</anchorfile>
      <anchor>a568777a2eaae9960669a6ab59cd0444c</anchor>
      <arglist>(int argc, char **argv, const std::string &amp;name=&quot;robowflex&quot;, unsigned int threads=1)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~ROS</name>
      <anchorfile>classrobowflex_1_1ROS.html</anchorfile>
      <anchor>abb984791d23332628d190bc35e03679c</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>getArgs</name>
      <anchorfile>classrobowflex_1_1ROS.html</anchorfile>
      <anchor>a714cc3a488fed9013f4448634318fa20</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>wait</name>
      <anchorfile>classrobowflex_1_1ROS.html</anchorfile>
      <anchor>a9687b5c346f331645e9af8418e8882a2</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>int</type>
      <name>argc_</name>
      <anchorfile>classrobowflex_1_1ROS.html</anchorfile>
      <anchor>ae56aa0b489efd8edb4e78881663c9915</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>char **</type>
      <name>argv_</name>
      <anchorfile>classrobowflex_1_1ROS.html</anchorfile>
      <anchor>a02a4d3fab0fbfadb5cefbeb107e728c6</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::Scene</name>
    <filename>classrobowflex_1_1Scene.html</filename>
    <class kind="class">robowflex::Scene::CollisionPluginLoader</class>
    <member kind="function">
      <type></type>
      <name>Scene</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>ab17fdf219c3ee8f1d77541ee3b4115f0</anchor>
      <arglist>(const RobotConstPtr &amp;robot)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Scene</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>aae99632f6bac1360a8c7463c9ab0f613</anchor>
      <arglist>(const Scene &amp;scene)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>operator=</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>aa2771dc5c436b1fb4fe38e5943613741</anchor>
      <arglist>(const Scene &amp;scene)</arglist>
    </member>
    <member kind="function">
      <type>const planning_scene::PlanningScenePtr &amp;</type>
      <name>getSceneConst</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>a7e3ec5011e09fb8c42614d6af5973990</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>planning_scene::PlanningScenePtr &amp;</type>
      <name>getScene</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>a946e2f3150209cd07d7bddb0d2d3e303</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>moveit_msgs::PlanningScene</type>
      <name>getMessage</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>a6728fbc58d809906d33186cb2447b562</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>robot_state::RobotState &amp;</type>
      <name>getCurrentState</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>a9ec3b6fee58cfdac9d7579e241eedc75</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>collision_detection::AllowedCollisionMatrix &amp;</type>
      <name>getACM</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>a6946c43ac70d64d1beb54e408d90378e</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>useMessage</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>a46e691ddb85f60a851cf1a5d079e5f75</anchor>
      <arglist>(const moveit_msgs::PlanningScene &amp;msg, bool diff=false)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>updateCollisionObject</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>ac3e73f9467511359255edaa473e03936</anchor>
      <arglist>(const std::string &amp;name, const GeometryConstPtr &amp;geometry, const Eigen::Affine3d &amp;pose)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>removeCollisionObject</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>a66550f4e75ccb87860fc2b3ff43e73d3</anchor>
      <arglist>(const std::string &amp;name)</arglist>
    </member>
    <member kind="function">
      <type>Eigen::Affine3d</type>
      <name>getObjectPose</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>a324e933beb17041a118af94628e44f32</anchor>
      <arglist>(const std::string &amp;name)</arglist>
    </member>
    <member kind="function">
      <type>Eigen::Affine3d</type>
      <name>getFramePose</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>a43d730ceb1b2c8a2a8441b6e02205f98</anchor>
      <arglist>(const std::string &amp;id) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>setCollisionDetector</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>ae1b2ce6565ce7288b73d69e9ce1f3a3e</anchor>
      <arglist>(const std::string &amp;detector_name) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>attachObject</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>acf80ca513d91260f818ffd09ac33b102</anchor>
      <arglist>(const std::string &amp;name)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>attachObject</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>a2ce1723844b8797a3ffcbd764e041821</anchor>
      <arglist>(const std::string &amp;name, const std::string &amp;ee_link, const std::vector&lt; std::string &gt; &amp;touch_links)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>detachObject</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>a597071696f8ae08885be81390738380c</anchor>
      <arglist>(const std::string &amp;name)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>toYAMLFile</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>a180d072af873ec9bfcafbca8fd9d1ad4</anchor>
      <arglist>(const std::string &amp;file)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>fromYAMLFile</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>a575b1a28985a1a90bd5b2588c2a8fe6a</anchor>
      <arglist>(const std::string &amp;file)</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>CollisionPluginLoaderPtr</type>
      <name>loader_</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>a4155bec4e60824349316776c81120f4a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>planning_scene::PlanningScenePtr</type>
      <name>scene_</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>a1fd81e46bebc8bf34f2bfe33df4ee880</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>const planning_scene::PlanningScenePtr &amp;</type>
      <name>getSceneConst</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>a7e3ec5011e09fb8c42614d6af5973990</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>planning_scene::PlanningScenePtr &amp;</type>
      <name>getScene</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>a946e2f3150209cd07d7bddb0d2d3e303</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>moveit_msgs::PlanningScene</type>
      <name>getMessage</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>a6728fbc58d809906d33186cb2447b562</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>robot_state::RobotState &amp;</type>
      <name>getCurrentState</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>a9ec3b6fee58cfdac9d7579e241eedc75</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>collision_detection::AllowedCollisionMatrix &amp;</type>
      <name>getACM</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>a6946c43ac70d64d1beb54e408d90378e</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>useMessage</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>a46e691ddb85f60a851cf1a5d079e5f75</anchor>
      <arglist>(const moveit_msgs::PlanningScene &amp;msg, bool diff=false)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>updateCollisionObject</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>ac3e73f9467511359255edaa473e03936</anchor>
      <arglist>(const std::string &amp;name, const GeometryConstPtr &amp;geometry, const Eigen::Affine3d &amp;pose)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>removeCollisionObject</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>a66550f4e75ccb87860fc2b3ff43e73d3</anchor>
      <arglist>(const std::string &amp;name)</arglist>
    </member>
    <member kind="function">
      <type>Eigen::Affine3d</type>
      <name>getObjectPose</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>a324e933beb17041a118af94628e44f32</anchor>
      <arglist>(const std::string &amp;name)</arglist>
    </member>
    <member kind="function">
      <type>Eigen::Affine3d</type>
      <name>getFramePose</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>a43d730ceb1b2c8a2a8441b6e02205f98</anchor>
      <arglist>(const std::string &amp;id) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>setCollisionDetector</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>ae1b2ce6565ce7288b73d69e9ce1f3a3e</anchor>
      <arglist>(const std::string &amp;detector_name) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>attachObject</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>acf80ca513d91260f818ffd09ac33b102</anchor>
      <arglist>(const std::string &amp;name)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>attachObject</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>a2ce1723844b8797a3ffcbd764e041821</anchor>
      <arglist>(const std::string &amp;name, const std::string &amp;ee_link, const std::vector&lt; std::string &gt; &amp;touch_links)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>detachObject</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>a597071696f8ae08885be81390738380c</anchor>
      <arglist>(const std::string &amp;name)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>toYAMLFile</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>a180d072af873ec9bfcafbca8fd9d1ad4</anchor>
      <arglist>(const std::string &amp;file)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>fromYAMLFile</name>
      <anchorfile>classrobowflex_1_1Scene.html</anchorfile>
      <anchor>a575b1a28985a1a90bd5b2588c2a8fe6a</anchor>
      <arglist>(const std::string &amp;file)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::Scene::CollisionPluginLoader</name>
    <filename>classrobowflex_1_1Scene_1_1CollisionPluginLoader.html</filename>
    <member kind="function">
      <type></type>
      <name>CollisionPluginLoader</name>
      <anchorfile>classrobowflex_1_1Scene_1_1CollisionPluginLoader.html</anchorfile>
      <anchor>ab6ecf75433eff83d73f80b39ee4cc2bd</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>collision_detection::CollisionPluginPtr</type>
      <name>load</name>
      <anchorfile>classrobowflex_1_1Scene_1_1CollisionPluginLoader.html</anchorfile>
      <anchor>a5d547ed0c8656390a74438d7ba1508a6</anchor>
      <arglist>(const std::string &amp;name)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>activate</name>
      <anchorfile>classrobowflex_1_1Scene_1_1CollisionPluginLoader.html</anchorfile>
      <anchor>ace999b7a66fcef277d0124dad060aff5</anchor>
      <arglist>(const std::string &amp;name, const planning_scene::PlanningScenePtr &amp;scene, bool exclusive)</arglist>
    </member>
    <member kind="typedef" protection="private">
      <type>pluginlib::ClassLoader&lt; collision_detection::CollisionPlugin &gt;</type>
      <name>PluginLoader</name>
      <anchorfile>classrobowflex_1_1Scene_1_1CollisionPluginLoader.html</anchorfile>
      <anchor>a3221828521635118d90a91a8ea2e5a2a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::shared_ptr&lt; PluginLoader &gt;</type>
      <name>loader_</name>
      <anchorfile>classrobowflex_1_1Scene_1_1CollisionPluginLoader.html</anchorfile>
      <anchor>aca5722fa74fb631420c18b0900a50bec</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::map&lt; std::string, collision_detection::CollisionPluginPtr &gt;</type>
      <name>plugins_</name>
      <anchorfile>classrobowflex_1_1Scene_1_1CollisionPluginLoader.html</anchorfile>
      <anchor>a8ecae50b45f691ebe3eddc3264238e99</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::SceneConstPtr</name>
    <filename>classrobowflex_1_1SceneConstPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::ScenePtr</name>
    <filename>classrobowflex_1_1ScenePtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::TrajectoryBenchmarkOutputter</name>
    <filename>classrobowflex_1_1TrajectoryBenchmarkOutputter.html</filename>
    <base>robowflex::BenchmarkOutputter</base>
    <member kind="function">
      <type></type>
      <name>TrajectoryBenchmarkOutputter</name>
      <anchorfile>classrobowflex_1_1TrajectoryBenchmarkOutputter.html</anchorfile>
      <anchor>ac3761d968ab1360d60b1a12ba4568bfb</anchor>
      <arglist>(const std::string &amp;file)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>dumpResult</name>
      <anchorfile>classrobowflex_1_1TrajectoryBenchmarkOutputter.html</anchorfile>
      <anchor>a5a83be0bb147d485ec7274e3fa957704</anchor>
      <arglist>(const Benchmarker::Results &amp;results) override</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>const std::string</type>
      <name>file_</name>
      <anchorfile>classrobowflex_1_1TrajectoryBenchmarkOutputter.html</anchorfile>
      <anchor>a7eec1a10896901d2082fd7ff902c5b2d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>IO::Bag</type>
      <name>bag_</name>
      <anchorfile>classrobowflex_1_1TrajectoryBenchmarkOutputter.html</anchorfile>
      <anchor>ac6ca980984887d408f698ed04d5bda91</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::UR5Robot</name>
    <filename>classrobowflex_1_1UR5Robot.html</filename>
    <base>robowflex::Robot</base>
    <member kind="function">
      <type></type>
      <name>UR5Robot</name>
      <anchorfile>classrobowflex_1_1UR5Robot.html</anchorfile>
      <anchor>a2058d324ada342d643405e2e12618f2c</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>initialize</name>
      <anchorfile>classrobowflex_1_1UR5Robot.html</anchorfile>
      <anchor>a024aa71f34ed15fc01f43feae85cb63f</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static const std::string</type>
      <name>URDF</name>
      <anchorfile>classrobowflex_1_1UR5Robot.html</anchorfile>
      <anchor>a44d26d1356e8389e7ed54f5765660021</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static const std::string</type>
      <name>SRDF</name>
      <anchorfile>classrobowflex_1_1UR5Robot.html</anchorfile>
      <anchor>a903514eaf8d3dc6a03c839bbb2ec825c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static const std::string</type>
      <name>LIMITS</name>
      <anchorfile>classrobowflex_1_1UR5Robot.html</anchorfile>
      <anchor>a52f6a9d42f4c7e8de6e5ef48e58cbfdf</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static const std::string</type>
      <name>KINEMATICS</name>
      <anchorfile>classrobowflex_1_1UR5Robot.html</anchorfile>
      <anchor>aa1f172b91d61a1be6084ec8243ffa66e</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>robowflex::UR5RobotConstPtr</name>
    <filename>classrobowflex_1_1UR5RobotConstPtr.html</filename>
  </compound>
  <compound kind="class">
    <name>robowflex::UR5RobotPtr</name>
    <filename>classrobowflex_1_1UR5RobotPtr.html</filename>
  </compound>
  <compound kind="namespace">
    <name>robowflex</name>
    <filename>namespacerobowflex.html</filename>
    <namespace>robowflex::hypercube</namespace>
    <namespace>robowflex::IO</namespace>
    <namespace>robowflex::movegroup</namespace>
    <namespace>robowflex::OMPL</namespace>
    <namespace>robowflex::TF</namespace>
    <class kind="class">robowflex::Benchmarker</class>
    <class kind="class">robowflex::BenchmarkerConstPtr</class>
    <class kind="class">robowflex::BenchmarkerPtr</class>
    <class kind="class">robowflex::BenchmarkOutputter</class>
    <class kind="class">robowflex::BenchmarkOutputterConstPtr</class>
    <class kind="class">robowflex::BenchmarkOutputterPtr</class>
    <class kind="class">robowflex::Exception</class>
    <class kind="class">robowflex::FetchRobot</class>
    <class kind="class">robowflex::FetchRobotConstPtr</class>
    <class kind="class">robowflex::FetchRobotPtr</class>
    <class kind="class">robowflex::Geometry</class>
    <class kind="class">robowflex::GeometryConstPtr</class>
    <class kind="class">robowflex::GeometryPtr</class>
    <class kind="class">robowflex::HDF5DataConstPtr</class>
    <class kind="class">robowflex::HDF5DataPtr</class>
    <class kind="class">robowflex::JSONBenchmarkOutputter</class>
    <class kind="class">robowflex::MotionRequestBuilder</class>
    <class kind="class">robowflex::MotionRequestBuilderConstPtr</class>
    <class kind="class">robowflex::MotionRequestBuilderPtr</class>
    <class kind="class">robowflex::OMPLBenchmarkOutputter</class>
    <class kind="class">robowflex::ParamRobot</class>
    <class kind="class">robowflex::ParamRobotConstPtr</class>
    <class kind="class">robowflex::ParamRobotPtr</class>
    <class kind="class">robowflex::PipelinePlanner</class>
    <class kind="class">robowflex::PipelinePlannerConstPtr</class>
    <class kind="class">robowflex::PipelinePlannerPtr</class>
    <class kind="class">robowflex::Planner</class>
    <class kind="class">robowflex::PlannerConstPtr</class>
    <class kind="class">robowflex::PlannerPtr</class>
    <class kind="class">robowflex::Pool</class>
    <class kind="class">robowflex::PoolPlanner</class>
    <class kind="class">robowflex::R2Robot</class>
    <class kind="class">robowflex::R2RobotConstPtr</class>
    <class kind="class">robowflex::R2RobotPtr</class>
    <class kind="class">robowflex::Robot</class>
    <class kind="class">robowflex::RobotConstPtr</class>
    <class kind="class">robowflex::RobotPtr</class>
    <class kind="class">robowflex::ROS</class>
    <class kind="class">robowflex::Scene</class>
    <class kind="class">robowflex::SceneConstPtr</class>
    <class kind="class">robowflex::ScenePtr</class>
    <class kind="class">robowflex::TrajectoryBenchmarkOutputter</class>
    <class kind="class">robowflex::UR5Robot</class>
    <class kind="class">robowflex::UR5RobotConstPtr</class>
    <class kind="class">robowflex::UR5RobotPtr</class>
    <member kind="function">
      <type>std::map&lt; std::string, double &gt;</type>
      <name>getFinalJointPositions</name>
      <anchorfile>namespacerobowflex.html</anchorfile>
      <anchor>acf413cf9ebc78076a09d96a90fade076</anchor>
      <arglist>(const planning_interface::MotionPlanResponse &amp;response)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>explode</name>
      <anchorfile>namespacerobowflex.html</anchorfile>
      <anchor>aa3c6c6a0a52b8ce690f26c87aad1103e</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>robowflex::hypercube</name>
    <filename>namespacerobowflex_1_1hypercube.html</filename>
    <class kind="class">robowflex::hypercube::OMPLChainPlanner</class>
    <class kind="class">robowflex::hypercube::OMPLChainPlannerPtr</class>
    <class kind="class">robowflex::hypercube::OMPLInterfacePlannerConstPtr</class>
    <class kind="class">robowflex::hypercube::Settings</class>
    <member kind="typedef">
      <type>std::function&lt; ompl::base::PlannerPtr(const ompl::base::SpaceInformationPtr &amp;si, const std::string &amp;name, const std::map&lt; std::string, std::string &gt; &amp;map)&gt;</type>
      <name>ConfiguredPlannerAllocator</name>
      <anchorfile>namespacerobowflex_1_1hypercube.html</anchorfile>
      <anchor>a7790860877268b79fd620562a9e3f6ba</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>tesseract::tesseract_ros::KDLEnvPtr</type>
      <name>constructTesseractEnv</name>
      <anchorfile>namespacerobowflex_1_1hypercube.html</anchorfile>
      <anchor>ae1da7fda2f5b63b56339d1b31c45e6bb</anchor>
      <arglist>(const robowflex::SceneConstPtr &amp;scene, const robowflex::RobotConstPtr &amp;robot)</arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>robowflex::IO</name>
    <filename>namespacerobowflex_1_1IO.html</filename>
    <class kind="class">robowflex::IO::Bag</class>
    <class kind="class">robowflex::IO::Handler</class>
    <class kind="class">robowflex::IO::HDF5Data</class>
    <class kind="class">robowflex::IO::HDF5File</class>
    <class kind="class">robowflex::IO::RVIZHelper</class>
    <member kind="function">
      <type>bool</type>
      <name>isNode</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a23927fb72a17ab8ce0d0654d06b5532e</anchor>
      <arglist>(const YAML::Node &amp;node)</arglist>
    </member>
    <member kind="function">
      <type>YAML::Node</type>
      <name>toNode</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>aed4974f62a649f4a6107ac3d6e919b33</anchor>
      <arglist>(const geometry_msgs::Pose &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>YAML::Node</type>
      <name>toNode</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a819245f223ff0eb589ad28aa153f2485</anchor>
      <arglist>(const moveit_msgs::PlanningScene &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>YAML::Node</type>
      <name>toNode</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a799d8a2989dee49fff0dd91d17a883be</anchor>
      <arglist>(const moveit_msgs::MotionPlanRequest &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>YAML::Node</type>
      <name>toNode</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a414861fda8e5df5e954e0f65a97b2ed7</anchor>
      <arglist>(const moveit_msgs::RobotTrajectory &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>fromYAMLFile</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a50e952949006a40e6b7b5d589958e01d</anchor>
      <arglist>(moveit_msgs::PlanningScene &amp;msg, const std::string &amp;file)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>fromYAMLFile</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a0500ed90fd25e45a7a0476fee615bcc3</anchor>
      <arglist>(moveit_msgs::MotionPlanRequest &amp;msg, const std::string &amp;file)</arglist>
    </member>
    <member kind="function">
      <type>const std::string</type>
      <name>resolvePackage</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a8eeeed7331bc650b06edd7303bffdaee</anchor>
      <arglist>(const std::string &amp;path)</arglist>
    </member>
    <member kind="function">
      <type>const std::string</type>
      <name>resolvePath</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a826655a9fb6eb0e25f130038bd6d7b3e</anchor>
      <arglist>(const std::string &amp;path)</arglist>
    </member>
    <member kind="function">
      <type>const std::string</type>
      <name>loadXMLToString</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a5dd541346890d73d822ef60dd2f613a1</anchor>
      <arglist>(const std::string &amp;path)</arglist>
    </member>
    <member kind="function">
      <type>const std::string</type>
      <name>loadXacroToString</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a25f75b885a3fa3f1394d5635d1a39c0e</anchor>
      <arglist>(const std::string &amp;path)</arglist>
    </member>
    <member kind="function">
      <type>const std::string</type>
      <name>loadFileToString</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a88a45ee4c89766f0c209b379b1beb82a</anchor>
      <arglist>(const std::string &amp;path)</arglist>
    </member>
    <member kind="function">
      <type>const std::string</type>
      <name>runCommand</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>aabd3ba6574462927b5aac495a6a1e032</anchor>
      <arglist>(const std::string &amp;cmd)</arglist>
    </member>
    <member kind="function">
      <type>const std::pair&lt; bool, YAML::Node &gt;</type>
      <name>loadFileToYAML</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a5dc8b1061b726b65ba86762af283f776</anchor>
      <arglist>(const std::string &amp;path)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>createFile</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>ad82de1798068dd05831c5eb168fe285e</anchor>
      <arglist>(std::ofstream &amp;out, const std::string &amp;file)</arglist>
    </member>
    <member kind="function">
      <type>const std::pair&lt; bool, std::vector&lt; std::string &gt; &gt;</type>
      <name>listDirectory</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a7bce336f927ad6de91f66790fdf9efab</anchor>
      <arglist>(const std::string &amp;directory)</arglist>
    </member>
    <member kind="function">
      <type>const std::string</type>
      <name>getHostname</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>aef6d4bc3a2b8ab2c6b2385d936f3a2b5</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>boost::posix_time::ptime</type>
      <name>getDate</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a407d1992227eef3b0031d5c8e74aa278</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>tokenize</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>aad1c0d0a27f1920ec75babb52490faac</anchor>
      <arglist>(const std::string &amp;string, const std::string &amp;separators)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>YAMLToFile</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>a14b1fd382262138158a5adbad8691ea0</anchor>
      <arglist>(const YAML::Node &amp;node, const std::string &amp;file)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>messageToYAMLFile</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>ab412be03d8a95eb75be650509931ab80</anchor>
      <arglist>(T &amp;msg, const std::string &amp;file)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>YAMLFileToMessage</name>
      <anchorfile>namespacerobowflex_1_1IO.html</anchorfile>
      <anchor>afc0e96b30b4528708e238b3635addda7</anchor>
      <arglist>(T &amp;msg, const std::string &amp;file)</arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>robowflex::movegroup</name>
    <filename>namespacerobowflex_1_1movegroup.html</filename>
    <class kind="class">robowflex::movegroup::MoveGroupHelper</class>
  </compound>
  <compound kind="namespace">
    <name>robowflex::OMPL</name>
    <filename>namespacerobowflex_1_1OMPL.html</filename>
    <class kind="class">robowflex::OMPL::FetchOMPLPipelinePlanner</class>
    <class kind="class">robowflex::OMPL::FetchOMPLPipelinePlannerConstPtr</class>
    <class kind="class">robowflex::OMPL::FetchOMPLPipelinePlannerPtr</class>
    <class kind="class">robowflex::OMPL::OMPLInterfacePlanner</class>
    <class kind="class">robowflex::OMPL::OMPLInterfacePlannerConstPtr</class>
    <class kind="class">robowflex::OMPL::OMPLInterfacePlannerPtr</class>
    <class kind="class">robowflex::OMPL::OMPLPipelinePlanner</class>
    <class kind="class">robowflex::OMPL::OMPLPipelinePlannerConstPtr</class>
    <class kind="class">robowflex::OMPL::OMPLPipelinePlannerPtr</class>
    <class kind="class">robowflex::OMPL::R2OMPLPipelinePlanner</class>
    <class kind="class">robowflex::OMPL::R2OMPLPipelinePlannerConstPtr</class>
    <class kind="class">robowflex::OMPL::R2OMPLPipelinePlannerPtr</class>
    <class kind="class">robowflex::OMPL::Settings</class>
    <class kind="class">robowflex::OMPL::UR5OMPLPipelinePlanner</class>
    <class kind="class">robowflex::OMPL::UR5OMPLPipelinePlannerConstPtr</class>
    <class kind="class">robowflex::OMPL::UR5OMPLPipelinePlannerPtr</class>
    <member kind="function">
      <type>bool</type>
      <name>loadOMPLConfig</name>
      <anchorfile>namespacerobowflex_1_1OMPL.html</anchorfile>
      <anchor>a31492e70aba96a5bf04e48eb9840f6be</anchor>
      <arglist>(IO::Handler &amp;handler, const std::string &amp;config_file, std::vector&lt; std::string &gt; &amp;configs)</arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>robowflex::TF</name>
    <filename>namespacerobowflex_1_1TF.html</filename>
    <member kind="function">
      <type>Eigen::Vector3d</type>
      <name>vectorMsgToEigen</name>
      <anchorfile>namespacerobowflex_1_1TF.html</anchorfile>
      <anchor>a1ed7fbaac296f875a3dfc136ff80bc99</anchor>
      <arglist>(const geometry_msgs::Vector3 &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>geometry_msgs::Vector3</type>
      <name>vectorEigenToMsg</name>
      <anchorfile>namespacerobowflex_1_1TF.html</anchorfile>
      <anchor>a1708cf70a26bb21cbdeea7df8e7dec9b</anchor>
      <arglist>(const Eigen::Vector3d &amp;vector)</arglist>
    </member>
    <member kind="function">
      <type>Eigen::Affine3d</type>
      <name>poseMsgToEigen</name>
      <anchorfile>namespacerobowflex_1_1TF.html</anchorfile>
      <anchor>af53d77db21f3736f386c805cbbdd37ba</anchor>
      <arglist>(const geometry_msgs::Pose &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>geometry_msgs::Pose</type>
      <name>poseEigenToMsg</name>
      <anchorfile>namespacerobowflex_1_1TF.html</anchorfile>
      <anchor>a306bde67c676d1879f5579c96f4f0a24</anchor>
      <arglist>(const Eigen::Affine3d &amp;pose)</arglist>
    </member>
    <member kind="function">
      <type>Eigen::Quaterniond</type>
      <name>quaternionMsgToEigen</name>
      <anchorfile>namespacerobowflex_1_1TF.html</anchorfile>
      <anchor>aa36db77a74d939b2d1c6eac8579f6921</anchor>
      <arglist>(const geometry_msgs::Quaternion &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>geometry_msgs::Quaternion</type>
      <name>quaternionEigenToMsg</name>
      <anchorfile>namespacerobowflex_1_1TF.html</anchorfile>
      <anchor>ae84c5fbe81f21d64a9caa67bad89563d</anchor>
      <arglist>(const Eigen::Quaterniond &amp;quaternion)</arglist>
    </member>
    <member kind="function">
      <type>moveit_msgs::BoundingVolume</type>
      <name>getBoundingVolume</name>
      <anchorfile>namespacerobowflex_1_1TF.html</anchorfile>
      <anchor>a69a9ae40884d2c53e418bd56b80bfd1a</anchor>
      <arglist>(const Eigen::Affine3d &amp;pose, const GeometryConstPtr &amp;geometry)</arglist>
    </member>
    <member kind="function">
      <type>moveit_msgs::PositionConstraint</type>
      <name>getPositionConstraint</name>
      <anchorfile>namespacerobowflex_1_1TF.html</anchorfile>
      <anchor>a3fb881f1dd59704d6f6237825161774f</anchor>
      <arglist>(const std::string &amp;ee_name, const std::string &amp;base_name, const Eigen::Affine3d &amp;pose, const GeometryConstPtr &amp;geometry)</arglist>
    </member>
    <member kind="function">
      <type>moveit_msgs::OrientationConstraint</type>
      <name>getOrientationConstraint</name>
      <anchorfile>namespacerobowflex_1_1TF.html</anchorfile>
      <anchor>a1519d7a4836023d69ae0cd587a527906</anchor>
      <arglist>(const std::string &amp;ee_name, const std::string &amp;base_name, const Eigen::Quaterniond &amp;orientation, const Eigen::Vector3d &amp;tolerances)</arglist>
    </member>
    <member kind="function">
      <type>Eigen::Quaterniond</type>
      <name>sampleOrientation</name>
      <anchorfile>namespacerobowflex_1_1TF.html</anchorfile>
      <anchor>a6e8c3dcd0a542c8923fe4eea954d6948</anchor>
      <arglist>(const Eigen::Quaterniond &amp;orientation, const Eigen::Vector3d &amp;tolerances)</arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>YAML</name>
    <filename>namespaceYAML.html</filename>
  </compound>
  <compound kind="page">
    <name>md__home_zak_ros_melodic_dev_src_robowflex_robowflex_visualization_README</name>
    <title>Robowflex Visualization</title>
    <filename>md__home_zak_ros_melodic_dev_src_robowflex_robowflex_visualization_README</filename>
  </compound>
  <compound kind="page">
    <name>md__home_zak_ros_melodic_dev_src_robowflex_robowflex_doc_doc_markdown_design</name>
    <title>Robowflex Design Notes</title>
    <filename>md__home_zak_ros_melodic_dev_src_robowflex_robowflex_doc_doc_markdown_design</filename>
  </compound>
  <compound kind="page">
    <name>md__home_zak_ros_melodic_dev_src_robowflex_robowflex_doc_doc_markdown_scripts</name>
    <title>Scripts</title>
    <filename>md__home_zak_ros_melodic_dev_src_robowflex_robowflex_doc_doc_markdown_scripts</filename>
  </compound>
  <compound kind="page">
    <name>index</name>
    <title>Robowflex</title>
    <filename>index</filename>
  </compound>
</tagfile>
