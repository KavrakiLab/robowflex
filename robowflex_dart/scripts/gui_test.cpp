// #include <dart/dart.hpp>
// #include <robowflex_dart/structure.h>
// #include <dart/gui/osg/osg.hpp>

// int main()
// {
//   // Create a world
//   dart::simulation::WorldPtr world(new dart::simulation::World);

//   // Add a target object to the world
//   dart::gui::osg::InteractiveFramePtr target(
//         new dart::gui::osg::InteractiveFrame(dart::dynamics::Frame::World()));
//   world->addSimpleFrame(target);

//   dart::dynamics::SimpleFramePtr frame(new dart::dynamics::SimpleFrame);
//   frame->setShape(robowflex::darts::makeSphere(0.1));
//   frame->createVisualAspect();
//   world->addSimpleFrame(frame);

//   // Wrap a WorldNode around it
//   osg::ref_ptr<CustomWorldNode> node = new CustomWorldNode(world);

//   // Create a Viewer and set it up with the WorldNode
//   dart::gui::osg::ImGuiViewer viewer;
//   viewer.addWorldNode(node);

//   // Active the drag-and-drop feature for the target
//   viewer.enableDragAndDrop(target.get());
//   viewer.enableDragAndDrop(frame.get());

//   // Pass in the custom event handler
//   viewer.addEventHandler(new CustomEventHandler);

//   // Set up the window to be 640x480
//   viewer.setUpViewInWindow(0, 0, 640, 480);

//   // Adjust the viewpoint of the Viewer
//   viewer.getCameraManipulator()->setHomePosition(
//         ::osg::Vec3( 2.57f,  3.14f, 1.64f),
//         ::osg::Vec3( 0.00f,  0.00f, 0.00f),
//         ::osg::Vec3(-0.24f, -0.25f, 0.94f));
//   // We need to re-dirty the CameraManipulator by passing it into the viewer
//   // again, so that the viewer knows to update its HomePosition setting
//   viewer.setCameraManipulator(viewer.getCameraManipulator());

//   // Begin running the application loop
//   viewer.run();
// }

/* Author: Zachary Kingston */

#include <thread>
#include <chrono>

#include <robowflex_dart/io.h>
#include <robowflex_dart/robot.h>
#include <robowflex_dart/world.h>
#include <robowflex_dart/space.h>
#include <robowflex_dart/tsr.h>
#include <robowflex_dart/gui.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    auto world = std::make_shared<darts::World>();

    auto fetch1 = darts::loadMoveItRobot("fetch1",                                         //
                                         "package://fetch_description/robots/fetch.urdf",  //
                                         "package://fetch_moveit_config/config/fetch.srdf");

    world->addRobot(fetch1);

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
    options_ik.name = "head_look";
    options_ik.size = 0.5;
    options_ik.thickness = 1;
    options_ik.pose = fetch1->getFrame("wrist_roll_link")->getWorldTransform();
    options_ik.callback = [&](const auto &frame) {
        auto &spec = ik_tsr->getSpecification();
        spec.setPose(frame->getWorldTransform());
        ik_tsr->updatePose();
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
    options_look.linear[0] = false;
    options_look.linear[1] = false;
    options_look.linear[2] = false;
    options_look.rotation[0] = false;
    options_look.planar[0] = false;
    options_look.planar[1] = false;
    options_look.planar[2] = false;
    options_look.callback = [&](const auto &frame) {
        auto &spec = look_tsr->getSpecification();
        spec.setPose(frame->getWorldTransform());
        look_tsr->updatePose();
        look_tsr->solveWorld();
    };

    auto look_ret = window.createInteractiveMarker(options_look);

    window.run();
    return 0;
}
