/* Author: Carlos Quintero Pena*/

#include <robowflex_library/util.h>
#include <robowflex_library/log.h>
#include <robowflex_tesseract/stretch.h>
#include <robowflex_tesseract/utils.h>
#include <robowflex_library/io.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/yaml.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/random.h>
#include <robowflex_library/builder.h>
#include <fstream>

using namespace robowflex;

static const std::string GROUP = "mobile_base_manipulator";
static const std::string GROUP_MANIP = "stretch_head";
static const std::string END_EFFECTOR = "link_head_tilt";
static const std::string ROBOT_FOV = "fov_geom";

std::vector<double> sampleBasePose()
{
    std::vector<double> base_pose;

    if (RNG::uniformBool())
    {
        double x = 0.0;
        // if (RNG::uniformBool())
            x = RNG::uniformReal(0.0, 0.3);
        // else
            // x = RNG::uniformReal(1.74, 2.04);

        base_pose.push_back(x);
        base_pose.push_back(RNG::uniformReal(-1.5, 1.5));
    }
    else
    {
        base_pose.push_back(RNG::uniformReal(0.0, 2.04));
        double y = 0.0;
        if (RNG::uniformBool())
            y = RNG::uniformReal(-1.4, -1.1);
        else
            y = RNG::uniformReal(1.1, 1.4);

        base_pose.push_back(y);
    }

    base_pose.push_back(RNG::uniformReal(-1.57, 1.57));

    return base_pose;
}

std::string randomlySelectObject()
{
    if (RNG::uniformBool())
        return "Can1";
    return "Can2";
}

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    bool visualize = false;
    std::string planner_name;
    if (argc > 1 and std::string(argv[1]) == "visualize")
        visualize = true;
    if (argc > 2)
        planner_name = std::string(argv[2]);

    // Enable mobile base.
    bool enable_mobile_base = false;
    if (GROUP == "mobile_base_manipulator")
        enable_mobile_base = true;

    int num_experiments = 1000, cnt = 0, monit_cnt = 0;
    std::ofstream fout;
    const std::string &path = IO::resolvePackage("package://robowflex_tesseract/result/results_" + planner_name + ".txt");
    fout.open(path);
    fout << planner_name << std::endl;

    // Create the default Stretch robot.
    auto stretch = std::make_shared<StretchRobot>();
    stretch->initialize(false, enable_mobile_base, GROUP, GROUP_MANIP);
    stretch->setGroupState(GROUP, {0.0, 0.0, 0.0, 0.0, 0.0});

    // Load a geometric model of the fov and place it at the tip of the chain.
    const auto &fov_geom = Geometry::makeMesh("package://robowflex_tesseract/scenes/table/camera_view.stl");

    // const auto &props = octo_generator_->getCameraProperties();
    // EigenSTL::vector_Vector3d vertices;
    // double z_far = 2.0;
    // // These are the equations from somewhere
    // double cx = 1.0;
    // double cy = 1.0;
    // vertices.emplace_back(Eigen::Vector3d(0, 0, 0));
    // vertices.emplace_back(Eigen::Vector3d(cx, cy, z_far));
    // vertices.emplace_back(Eigen::Vector3d(-cx, cy, z_far));
    // vertices.emplace_back(Eigen::Vector3d(0, 0, 0));
    // vertices.emplace_back(Eigen::Vector3d(cx, -cy, z_far));
    // vertices.emplace_back(Eigen::Vector3d(cx, cy, z_far));
    // vertices.emplace_back(Eigen::Vector3d(0, 0, 0));
    // vertices.emplace_back(Eigen::Vector3d(-cx, cy, z_far));
    // vertices.emplace_back(Eigen::Vector3d(-cx, -cy, z_far));
    // vertices.emplace_back(Eigen::Vector3d(0, 0, 0));
    // vertices.emplace_back(Eigen::Vector3d(-cx, -cy, z_far));
    // vertices.emplace_back(Eigen::Vector3d(cx, -cy, z_far));
    // vertices.emplace_back(Eigen::Vector3d(cx, cy, z_far));
    // vertices.emplace_back(Eigen::Vector3d(cx, -cy, z_far));
    // vertices.emplace_back(Eigen::Vector3d(-cx, -cy, z_far));
    // vertices.emplace_back(Eigen::Vector3d(-cx, -cy, z_far));
    // vertices.emplace_back(Eigen::Vector3d(-cx, cy, z_far));
    // vertices.emplace_back(Eigen::Vector3d(cx, cy, z_far));
    // const auto &fov_geom = Geometry::makeMesh(vertices);


    const auto &ee_tf = stretch->getLinkTF(END_EFFECTOR);
    auto fov_pose = RobotPose::Identity();
    fov_pose.translate(ee_tf.translation());
    fov_pose.rotate(ee_tf.rotation());

    // Load scene and attach fov to robot.
    auto scene = std::make_shared<Scene>(stretch);
    scene->fromYAMLFile("package://robowflex_tesseract/scenes/table/modified_scene.yaml");
    scene->getCurrentState() = *stretch->getScratchState();
    scene->updateCollisionObject(ROBOT_FOV, fov_geom, fov_pose);
    scene->attachObjectToState(scene->getCurrentState(), ROBOT_FOV, END_EFFECTOR, stretch->getModel()->getLinkModelNames());
    scene->removeCollisionObject(ROBOT_FOV);
    *stretch->getScratchState() = scene->getCurrentState();

    // Allow fov to collide with all the environment.
    auto &acm = scene->getACM();
    for (const auto &obj : scene->getCollisionObjects())
        acm.setEntry(ROBOT_FOV, obj, true);

    // auto robot_offset = TF::createPoseQ(Eigen::Vector3d{0.5, 0.0, 0}, Eigen::Quaterniond{0.707, 0, 0, -0.707});//-90 z
    // auto robot_offset = TF::createPoseQ(Eigen::Vector3d{-0.5, 0.0, 0}, Eigen::Quaterniond{1,0,0,0});
    // scene->moveAllObjectsGlobal(robot_offset);

    // RVIZ helper.
    IO::RVIZHelperPtr rviz;
    if (visualize)
    {
        rviz = std::make_shared<IO::RVIZHelper>(stretch);
        rviz->updateScene(scene);
        std::cin.ignore();
    }

    // Setup planner and request.
    PlannerPtr planner;
    configurePlanner(planner_name, stretch, planner);
    const auto &request = std::make_shared<MotionRequestBuilder>(stretch, GROUP);
    request->setWorkspaceBounds(Eigen::Vector3d{-5, -5, -5}, Eigen::Vector3d{5, 5, 5});


    ////////////////
    auto const &model = stretch->getModelConst();
    auto const &gmodel = model->getJointModelGroup(GROUP);
    for (auto const &joint : gmodel->getJointModels())
    {
        for (auto const &var : joint->getVariableBounds())
        {
            std::cout << joint->getName() << ": min->" << var.min_position_ << ", max->" << var.max_position_ << std::endl;
        }
    }

    const std::vector<double> pan_joint_bnds = {gmodel->getJointModel("joint_head_pan")->getVariableBounds("joint_head_pan").min_position_, gmodel->getJointModel("joint_head_pan")->getVariableBounds("joint_head_pan").max_position_};
    const std::vector<double> tilt_joint_bnds = {gmodel->getJointModel("joint_head_tilt")->getVariableBounds("joint_head_tilt").min_position_, gmodel->getJointModel("joint_head_tilt")->getVariableBounds("joint_head_tilt").max_position_};
    //////////////////


    while(cnt < num_experiments)
    {
        if (visualize)
        {
            // Clean RViz.
            rviz->removeAllMarkers();
            rviz->updateMarkers();
        }

        // Copy original scene.
        auto new_scene = scene->deepCopy();

        // Set start configuration
        stretch->getScratchState()->enforceBounds();
        new_scene->getCurrentState() = *stretch->getScratchState();
        request->useSceneStateAsStart(new_scene);

        // Randomly choose an object to monitor (Can1 or Can2) and sample location..
        auto const &object = randomlySelectObject();
        const auto &object_noise = TF::samplePositionGaussian(Eigen::Vector3d{0.05, 0.1, 0.0});
        const auto &new_position = object_noise + new_scene->getObjectPose(object).translation();
        if ((new_position[0] < 0.44 or new_position[0] > 1.64) and (new_position[1] > -1.1 and new_position[1] < 1.1))
            continue;

        // Actually move object.
        auto transform = TF::identity();
        transform.translate(object_noise);
        new_scene->moveObjectGlobal(object, transform);

        if (visualize)
        {
            // Add arrow for visualization.
            auto marker_pose = TF::identity();
            marker_pose.translate(new_position);
            marker_pose.translate(Eigen::Vector3d{0.0, 0.0, 0.15});
            marker_pose.rotate(Eigen::Quaterniond{0.707, 0.0, 0.707, 0.0});
            rviz->removeAllMarkers();
            rviz->addArrowMarker("object_arrow", "map", marker_pose, {1, 0, 0, 1}, {0.06, 0.01, 0.01});
            rviz->updateMarkers();

            // Show start.
            rviz->updateScene(new_scene);
            RBX_INFO("Showing start. Press Enter to continue");
            std::cin.ignore();
        }

        // Find a valid goal configuration.
        std::vector<double> base_pose;
        double pan, tilt;
        bool valid = false;
        while (not valid)
        {
            // Sample robot base.
            base_pose = sampleBasePose();

            // Compute pan and tilt for goal
            stretch->setGroupState(GROUP, {base_pose[0], base_pose[1], base_pose[2], 0.0, 0.0});
            const auto &point_pose = new_scene->getObjectPose(object);
            const auto &point_pan = stretch->getLinkTF("link_head_pan").inverse() * point_pose;
            const auto &point_tilt = stretch->getLinkTF("link_head_tilt").inverse() * point_pose;
            pan = atan2(point_pan.translation().y(), point_pan.translation().x());
            tilt = 1.57 - atan2(point_tilt.translation().x(), point_tilt.translation().y());
            // std::cout << "pan = " << pan << ", tilt = " << tilt << std::endl;
            if (pan < pan_joint_bnds[0] or pan > pan_joint_bnds[1] or tilt < tilt_joint_bnds[0] or tilt > tilt_joint_bnds[1])
                continue;

            // Set the goal (to check monitoring constraint)
            stretch->setGroupState(GROUP, {base_pose[0], base_pose[1], base_pose[2], pan, tilt});
            stretch->getScratchState()->enforceBounds();
            new_scene->getCurrentState() = *stretch->getScratchState();

            // Create a new acm allowing everything but object and fov.
            auto tacm = new_scene->getScene()->getAllowedCollisionMatrixNonConst();
            new_scene->clearACM(tacm);
            tacm.setEntry(ROBOT_FOV, object, false);

            // Check for visibility collision.
            collision_detection::CollisionRequest req;
            collision_detection::CollisionResult res;
            new_scene->getScene()->getCollisionEnv()->checkRobotCollision(req, res, new_scene->getCurrentState(), tacm);
            if (res.collision)
                valid = true;
        }

        // Set goal.
        request->setGoalConfiguration(stretch->getScratchState());

        if (visualize)
        {
            // Show valid goal.
            rviz->updateScene(new_scene);
            RBX_INFO("Showing goal. Press Enter to continue");
            std::cin.ignore();
        }

        // std::cout << "pan " << stretch->getScratchState()->getVariablePosition("joint_head_pan");
        // std::cout << "tilt " << stretch->getScratchState()->getVariablePosition("joint_head_tilt");
        // std::cout << "position of end effector" << std::endl;
        // std::cout << stretch->getLinkTF(END_EFFECTOR).translation() << std::endl;

        // Plan.
        const auto &res = planner->plan(new_scene, request->getRequest());
        if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            RBX_ERROR("Trajectory not found. Aborting!");
            continue;
        }
        Trajectory trajectory(res.trajectory_);

        // monit_cnt = 0;
        // for (int index = 0; index < trajectory.getNumWaypoints();++index)
        // {
        //     const auto &wp = trajectory.getTrajectoryConst()->getWayPoint(index);
        //
        //     auto tacm = new_scene->getScene()->getAllowedCollisionMatrixNonConst();
        //     new_scene->clearACM(tacm);
        //     tacm.setEntry(ROBOT_FOV, object, false);
        //
        //     // Check for visibility collision.
        //     collision_detection::CollisionRequest req;
        //     collision_detection::CollisionResult res;
        //     new_scene->getScene()->getCollisionEnv()->checkRobotCollision(req, res, wp, tacm);
        //
        //     if (res.collision)
        //         monit_cnt++;
        // }

        double monit_rate = (double)monit_cnt / trajectory.getNumWaypoints();
        // fout << monit_rate << " ";
        // std::cout << "Before " << monit_rate << " " << std::endl;

        if (planner_name == "RRTConnectPost")
        {
            for (int index = 0; index < trajectory.getNumWaypoints();++index)
            {
                // Compute pan and tilt to monitor object at current wp.
                auto &wp = trajectory.getTrajectory()->getWayPointPtr(index);

                const auto &point_pose = new_scene->getObjectPose(object);
                const auto &point_pan = wp->getGlobalLinkTransform("link_head_pan").inverse() * point_pose;
                const auto &point_tilt = wp->getGlobalLinkTransform("link_head_tilt").inverse() * point_pose;
                pan = atan2(point_pan.translation().y(), point_pan.translation().x());
                tilt = 1.57 - atan2(point_tilt.translation().x(), point_tilt.translation().y());
                wp->setVariablePosition("joint_head_pan", pan);
                wp->setVariablePosition("joint_head_tilt", tilt);
                wp->enforceBounds();
                wp->update(true);
            }
        }

        // Visualize trajectory.
        monit_cnt = 0;
        for (int index = 0; index < trajectory.getNumWaypoints();++index)
        {
            const auto &wp = trajectory.getTrajectoryConst()->getWayPoint(index);

            auto tacm = new_scene->getScene()->getAllowedCollisionMatrixNonConst();
            new_scene->clearACM(tacm);
            tacm.setEntry(ROBOT_FOV, object, false);

            // Check for visibility collision.
            collision_detection::CollisionRequest req;
            collision_detection::CollisionResult res;
            new_scene->getScene()->getCollisionEnv()->checkRobotCollision(req, res, wp, tacm);

            if (res.collision)
                monit_cnt++;
        }

        monit_rate = (double)monit_cnt / trajectory.getNumWaypoints();
        fout << monit_rate << " ";
        std::cout << monit_rate << " " << std::endl;

        if (visualize)
        {
            rviz->updateTrajectory(trajectory);
            RBX_INFO("Press Enter to continue");
            std::cin.ignore();
        }

        cnt++;
    }

    fout << std::endl;
    fout.close();

    return 0;
}
