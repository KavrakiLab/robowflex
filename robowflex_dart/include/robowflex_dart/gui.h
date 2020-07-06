/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_DART_GUI_
#define ROBOWFLEX_DART_GUI_

#include <mutex>
#include <condition_variable>

#include <dart/dart.hpp>
#include <dart/external/imgui/imgui.h>
#include <dart/gui/osg/ImGuiWidget.hpp>
#include <dart/gui/osg/osg.hpp>

#include <robowflex_library/class_forward.h>
#include <robowflex_library/adapter.h>

#include <robowflex_dart/world.h>
#include <robowflex_dart/space.h>
#include <robowflex_dart/planning.h>

namespace robowflex
{
    namespace darts
    {
        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(Window)
        ROBOWFLEX_CLASS_FORWARD(WindowWidget)
        /** \endcond */

        class WindowWidget : public dart::gui::osg::ImGuiWidget
        {
        public:
            ROBOWFLEX_CLASS_FORWARD(Element)
            class Element
            {
            public:
                virtual void render() const = 0;
            };

            class TextElement : public Element
            {
            public:
                TextElement(const std::string &text) : text(text)
                {
                }

                void render() const override
                {
                    ImGui::Text(text.c_str());
                }

            private:
                const std::string text;
            };

            class CheckboxElement : public Element
            {
            public:
                CheckboxElement(const std::string &text, bool &boolean) : text(text), boolean(boolean)
                {
                }

                void render() const override
                {
                    ImGui::Checkbox(text.c_str(), &boolean);
                }

            private:
                const std::string text;
                bool &boolean;
            };

            ROBOWFLEX_CLASS_FORWARD(ButtonElement)
            using ButtonCallback = std::function<void()>;
            class ButtonElement : public Element
            {
            public:
                ButtonElement(const std::string &text, const ButtonCallback &callback)
                  : text(text), callback(callback)
                {
                }

                void render() const override
                {
                    const auto &button = ImGui::Button(text.c_str());
                    if (button)
                        callback();
                }

            private:
                const std::string text;
                const ButtonCallback callback;
            };

            using RenderCallback = std::function<void()>;
            class RenderElement : public Element
            {
            public:
                RenderElement(const RenderCallback &callback) : callback(callback)
                {
                }

                void render() const override
                {
                    if (callback)
                        callback();
                }

            private:
                const RenderCallback callback;
            };

            WindowWidget()
            {
            }

            void render() override
            {
                if (elements_.empty())
                    return;

                ImGui::SetNextWindowPos(ImVec2(10, 20));
                ImGui::SetNextWindowBgAlpha(0.5f);
                if (!ImGui::Begin("Robowflex DART", nullptr,
                                  ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_HorizontalScrollbar))
                {
                    // Early out if the window is collapsed, as an optimization.
                    ImGui::End();
                    return;
                }

                for (const auto &element : elements_)
                    element->render();
            }

            void addText(const std::string &text)
            {
                elements_.push_back(std::make_shared<TextElement>(text));
            }

            void addButton(const std::string &text, const ButtonCallback &callback)
            {
                elements_.push_back(std::make_shared<ButtonElement>(text, callback));
            }

            void addCheckbox(const std::string &text, bool &boolean)
            {
                elements_.push_back(std::make_shared<CheckboxElement>(text, boolean));
            }

            void addCallback(const RenderCallback &callback)
            {
                elements_.push_back(std::make_shared<RenderElement>(callback));
            }

        private:
            std::vector<ElementPtr> elements_;
        };

        class Window : public dart::gui::osg::WorldNode
        {
        public:
            Window(const WorldPtr &world) : dart::gui::osg::WorldNode(world->getSim()), world_(world)
            {
                node_ = this;
                viewer_.addWorldNode(node_);
                viewer_.setUpViewInWindow(0, 0, 1080, 1080);
                auto cm = viewer_.getCameraManipulator();
                cm->setHomePosition(                //
                    ::osg::Vec3(2.57, 3.14, 1.64),  //
                    ::osg::Vec3(0.00, 0.00, 0.00),  //
                    ::osg::Vec3(-0.24, -0.25, 0.94));
                viewer_.setCameraManipulator(cm);
                viewer_.setVerticalFieldOfView(30);

                widget_ = std::make_shared<WindowWidget>();
                viewer_.getImGuiHandler()->addWidget(widget_);
            }

            // void customPreRefresh() override;
            // void customPostRefresh() override;
            // void customPreStep() override;
            // void customPostStep() override;

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

                return r;
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

            void run(std::function<void()> thread = {})
            {
                if (thread)
                {
                    std::thread t(thread);
                    viewer_.run();
                }
                else
                    viewer_.run();
            }

            void animatePath(const StateSpacePtr &space, const ompl::geometric::PathGeometric &path,
                             std::size_t times = 1, double fps = 60, bool block = true)
            {
                bool active = true;

                std::mutex m;
                std::condition_variable cv;

                if (animation_)
                {
                    animation_->join();
                    animation_.reset();
                }
                auto thread = std::make_shared<std::thread>([&] {
                    std::size_t n = path.getStateCount();
                    std::size_t i = times;

                    std::unique_lock<std::mutex> lk(m);
                    while ((times == 0) ? true : i--)
                    {
                        space->setWorldState(world_, path.getState(0));
                        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

                        for (std::size_t j = 0; j < n; ++j)
                        {
                            space->setWorldState(world_, path.getState(j));

                            std::this_thread::sleep_for(
                                std::chrono::milliseconds((unsigned int)(1000 / fps)));
                        }

                        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    }

                    active = false;
                    cv.notify_one();
                });

                animation_ = thread;

                if (block)
                {
                    std::unique_lock<std::mutex> lk(m);
                    cv.wait(lk, [&] { return not active; });
                }
            }

            void animatePath(const PlanBuilder &builder, const ompl::geometric::PathGeometric &path,
                             std::size_t times = 1, double fps = 60, bool block = true)
            {
                ompl::geometric::PathGeometric extract(builder.rinfo);
                for (std::size_t i = 0; i < path.getStateCount(); ++i)
                    extract.append(builder.getStateConst(path.getState(i)));

                animatePath(builder.rspace, extract, times, fps, block);
            }

            WindowWidgetPtr getWidget()
            {
                return widget_;
            }

        private:
            WorldPtr world_;
            WindowWidgetPtr widget_;

            std::shared_ptr<std::thread> animation_{nullptr};

            ::osg::ref_ptr<Window> node_;
            dart::gui::osg::ImGuiViewer viewer_;
        };
    }  // namespace darts
}  // namespace robowflex

#endif
