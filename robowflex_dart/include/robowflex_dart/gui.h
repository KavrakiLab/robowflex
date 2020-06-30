/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_DART_GUI_
#define ROBOWFLEX_DART_GUI_

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>

#include <robowflex_library/class_forward.h>
#include <robowflex_dart/world.h>
#include <robowflex_dart/space.h>
#include <robowflex_dart/planning.h>

namespace robowflex
{
    namespace darts
    {
        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(Window)
        /** \endcond */

        class Window : public dart::gui::osg::WorldNode
        {
        public:
            Window(const WorldPtr &world) : dart::gui::osg::WorldNode(world->getSim()), world_(world)
            {
                node_ = this;
                viewer_.addWorldNode(node_);
                viewer_.setUpViewInWindow(0, 0, 1080, 1080);
                viewer_.getCameraManipulator()->setHomePosition(  //
                    ::osg::Vec3(2.57, 3.14, 1.64),                //
                    ::osg::Vec3(0.00, 0.00, 0.00),                //
                    ::osg::Vec3(-0.24, -0.25, 0.94));
                viewer_.setCameraManipulator(viewer_.getCameraManipulator());
            }

            // void customPreRefresh() override;
            // void customPostRefresh() override;
            // void customPreStep() override;
            // void customPostStep() override;

            void setCameraPose(const RobotPose &pose)
            {
                ::osg::Matrixd m(pose.data());
                auto cm = viewer_.getCameraManipulator();
                cm->setByMatrix(m);
            }

            using InteractiveCallback = std::function<void(const dart::gui::osg::InteractiveFrame *)>;

            struct InteractiveOptions
            {
                std::string name{"interactive_marker"};
                Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
                InteractiveCallback callback{};
                dart::dynamics::Frame *parent{dart::dynamics::Frame::World()};
                double size{0.2};
                double thickness{2.};

                bool linear[3]{true, true, true};
                bool rotation[3]{true, true, true};
                bool planar[3]{true, true, true};
            };

            struct InteractiveReturn
            {
                dart::gui::osg::InteractiveFramePtr target;
                dart::gui::osg::InteractiveFrameDnD *dnd;
                dart::common::Connection signal;
            };

            InteractiveReturn createInteractiveMarker(const InteractiveOptions &options)
            {
                InteractiveReturn r;
                r.target = std::make_shared<dart::gui::osg::InteractiveFrame>(  //
                    options.parent, options.name, options.pose, options.size, options.thickness);
                world_->getSim()->addSimpleFrame(r.target);

                for (std::size_t i = 0; i < 3; ++i)
                {
                    auto lt = r.target->getTool(dart::gui::osg::InteractiveTool::Type::LINEAR, i);
                    lt->setEnabled(options.linear[i]);
                    auto rt = r.target->getTool(dart::gui::osg::InteractiveTool::Type::ANGULAR, i);
                    rt->setEnabled(options.rotation[i]);
                    auto pt = r.target->getTool(dart::gui::osg::InteractiveTool::Type::PLANAR, i);
                    pt->setEnabled(options.planar[i]);
                }

                r.dnd = viewer_.enableDragAndDrop(r.target.get());

                auto callback = options.callback;
                r.signal =
                    r.target->onTransformUpdated.connect([callback](const dart::dynamics::Entity *entity) {
                        callback(dynamic_cast<const dart::gui::osg::InteractiveFrame *>(entity));
                    });
            }

            using DnDCallback = std::function<void(const dart::dynamics::BodyNode *)>;

            struct DnDReturn
            {
                dart::gui::osg::BodyNodeDnD *dnd;
                dart::common::Connection signal;
            };

            DnDReturn enableNodeDragNDrop(dart::dynamics::BodyNode *node, const DnDCallback &callback = {})
            {
                DnDReturn r;
                auto dnd = viewer_.enableDragAndDrop(node, true, true);
                r.dnd = dnd;

                r.signal =
                    node->onTransformUpdated.connect([dnd, callback](const dart::dynamics::Entity *entity) {
                        if (dnd->isMoving())
                            callback(dynamic_cast<const dart::dynamics::BodyNode *>(entity));
                    });

                return r;
            }

            void run()
            {
                viewer_.run();
            }

            void animatePath(const PlanBuilder &builder, const ompl::geometric::PathGeometric &path,
                             std::size_t times = 1, double fps = 60)
            {
                std::size_t n = path.getStateCount();
                std::size_t i = times;
                while ((times == 0) ? true : i--)
                {
                    builder.rspace->setWorldState(world_, builder.getStateConst(path.getState(0)));
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

                    for (std::size_t j = 0; j < n; ++j)
                    {
                        if (not builder.info->isValid(path.getState(j)))
                            std::cout << "State " << j << " is invalid!" << std::endl;

                        builder.rspace->setWorldState(world_, builder.getStateConst(path.getState(j)));

                        std::this_thread::sleep_for(std::chrono::milliseconds((unsigned int)(1000 / fps)));
                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                }
            }

        private:
            WorldPtr world_;

            ::osg::ref_ptr<Window> node_;
            dart::gui::osg::ImGuiViewer viewer_;
        };
    }  // namespace darts
}  // namespace robowflex

#endif
