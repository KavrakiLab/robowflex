/* Author: Zachary Kingston */

#include <chrono>
#include <thread>

#include <robowflex_dart/gui.h>
#include <robowflex_dart/io.h>
#include <robowflex_dart/robot.h>
#include <robowflex_dart/space.h>
#include <robowflex_dart/tsr.h>
#include <robowflex_dart/world.h>

using namespace robowflex;

int main(int /*argc*/, char ** /*argv*/)
{
    bool ik = true;

    auto world = std::make_shared<darts::World>();

    auto fetch1 = darts::loadMoveItRobot("fetch1",                                         //
                                         "package://fetch_description/robots/fetch.urdf",  //
                                         "package://fetch_moveit_config/config/fetch.srdf");
    auto start = fetch1->getSkeleton()->getState();

    auto scene = std::make_shared<darts::Structure>("object");

    dart::dynamics::FreeJoint::Properties joint;
    joint.mName = "box";
    joint.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0.5, 0, 0.3);
    scene->addFreeFrame(joint, darts::makeBox(0.05, 0.05, 0.1));

    world->addRobot(fetch1);
    world->addStructure(scene);

    darts::Window window(world);

    //
    // Setup End-Effector IK
    //
    darts::TSR::Specification ik_spec;
    ik_spec.setFrame("fetch1", "wrist_roll_link");
    auto ik_tsr = std::make_shared<darts::TSR>(world, ik_spec);
    ik_tsr->useGroup("arm_with_torso");
    ik_tsr->initialize();

    darts::Window::InteractiveOptions options_ik;
    options_ik.name = "arm_ik";
    options_ik.size = 0.4;
    options_ik.thickness = 1;
    options_ik.pose = fetch1->getFrame("wrist_roll_link")->getWorldTransform();

    options_ik.callback = [&](const auto &frame) {
        auto &spec = ik_tsr->getSpecification();
        spec.setPose(frame->getWorldTransform());
        ik_tsr->updatePose();

        if (ik)
            ik_tsr->solveWorld();
    };

    auto ik_ret = window.createInteractiveMarker(options_ik);

    //
    // Setup Head-Look IK
    //
    darts::TSR::Specification look_spec;
    look_spec.setFrame("fetch1", "head_camera_link");
    auto look_tsr = std::make_shared<darts::TSR>(world, look_spec);
    look_tsr->initialize();

    darts::Window::InteractiveOptions options_look;
    options_look.name = "head_look";
    options_look.pose = fetch1->getFrame("head_camera_link")->getWorldTransform();
    options_look.pose.translation()[1] += 0.5;

    options_look.linear[0] = false;  // disable linear control
    options_look.linear[1] = false;
    options_look.linear[2] = false;
    options_look.rotation[0] = false;  // no roll
    options_look.planar[0] = false;    // disable planar control
    options_look.planar[1] = false;
    options_look.planar[2] = false;

    options_look.callback = [&](const auto &frame) {
        auto &spec = look_tsr->getSpecification();
        spec.setPose(frame->getWorldTransform());
        look_tsr->updatePose();

        if (ik)
            look_tsr->solveWorld();
    };

    auto look_ret = window.createInteractiveMarker(options_look);

    // Enable/Disable IK
    window.getWidget()->addCheckbox("Enable IK", ik);

    // Pick / Place Button
    bool picked = false;
    auto *cube = scene->getFrame("box");
    window.getWidget()->addButton("Pick/Place", [&] {
        if (not picked)
            fetch1->reparentFreeFrame(cube, "wrist_roll_link");
        else
            scene->reparentFreeFrame(cube);

        picked = not picked;
    });

    // Setup reset button
    window.getWidget()->addText("Press button to reset robot state!");
    window.getWidget()->addButton("Reset", [&] {
        // do it a few times since the IK tries to update
        for (std::size_t i = 0; i < 3; ++i)
        {
            fetch1->getSkeleton()->setState(start);
            ik_ret.target->setTransform(fetch1->getFrame("wrist_roll_link")->getWorldTransform());
            auto tf = fetch1->getFrame("head_camera_link")->getWorldTransform();
            tf.translation()[1] += 0.5;
            look_ret.target->setTransform(tf);
        }
    });

    window.run();
    return 0;
}
