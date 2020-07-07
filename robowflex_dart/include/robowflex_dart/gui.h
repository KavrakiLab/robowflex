/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_DART_GUI_
#define ROBOWFLEX_DART_GUI_

#include <mutex>

#include <dart/external/imgui/imgui.h>
#include <dart/gui/osg/ImGuiWidget.hpp>
#include <dart/gui/osg/osg.hpp>

#include <robowflex_library/class_forward.h>
#include <robowflex_library/adapter.h>

#include <robowflex_dart/world.h>
#include <robowflex_dart/space.h>
#include <robowflex_dart/tsr.h>
#include <robowflex_dart/planning.h>

namespace robowflex
{
    namespace darts
    {
        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(Window)
        ROBOWFLEX_CLASS_FORWARD(Widget)
        ROBOWFLEX_CLASS_FORWARD(WindowWidget)
        ROBOWFLEX_CLASS_FORWARD(TSRWidget)
        /** \endcond */

        /** \class robowflex::darts::WindowPtr
            \brief A shared pointer wrapper for robowflex::darts::Window. */

        /** \class robowflex::darts::WindowConstPtr
            \brief A const shared pointer wrapper for robowflex::darts::Window. */

        /** \class robowflex::darts::WindowWidgetPtr
            \brief A shared pointer wrapper for robowflex::darts::WindowWidget. */

        /** \class robowflex::darts::WindowWidgetConstPtr
            \brief A const shared pointer wrapper for robowflex::darts::WindowWidget. */

        /** \brief Open Scene Graph GUI for DART visualization.
         */
        class Window : public dart::gui::osg::WorldNode
        {
            friend TSRWidget;

        public:
            /** \brief Constructor.
             *  \param[in] world World to visualize.
             */
            Window(const WorldPtr &world);
            void customPreRefresh() override;

            /** \name GUI Interaction
                \{ */

            /** \brief Callback function on an interactive frame moving.
             */
            using InteractiveCallback = std::function<void(const dart::gui::osg::InteractiveFrame *)>;

            /** \brief Options for creating an interactive marker.
             */
            struct InteractiveOptions
            {
                std::string name{"interactive_marker"};                 ///< Name of marker.
                Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};  ///< Relative pose of marker.
                InteractiveCallback callback{};                         ///< Callback function on motion.
                dart::dynamics::Frame *parent{dart::dynamics::Frame::World()};  ///< Parent frame.
                double size{0.2};                                               ///< Size of marker.
                double thickness{2.};      ///< Thickness of marker arrows.
                bool obstructable{false};  ///< Is this frame obstructable?

                bool linear[3]{true, true, true};    ///< Linear position controls enabled.
                bool rotation[3]{true, true, true};  ///< Rotation ring controls enabled.
                bool planar[3]{true, true, true};    ///< Planar translation controls enabled.
            };

            /** \brief Return from creating an interactive marker.
             */
            struct InteractiveReturn
            {
                dart::gui::osg::InteractiveFramePtr target;  ///< Interactive Frame generated.
                dart::gui::osg::InteractiveFrameDnD *dnd;    ///< Drag 'n Drop object.
                dart::common::Connection signal;             ///< Connection from motion to callback.
            };

            /** \brief Create a new interactive marker in the GUI.
             *  \param[in] options Options for creating marker.
             *  \return The new marker.
             */
            InteractiveReturn createInteractiveMarker(const InteractiveOptions &options);

            /** \brief Callback function on a drag 'n drop frame moving.
             */
            using DnDCallback = std::function<void(const dart::dynamics::BodyNode *)>;

            /** \brief Return from creating a movable frame.
             */
            struct DnDReturn
            {
                dart::gui::osg::BodyNodeDnD *dnd;  ///< Drag 'n Drop object.
                dart::common::Connection signal;   ///< Connection from motion to callback.
            };

            /** \brief Enable drag 'n drop functionality on a body node being visualized.
             *  With DnD, the body node will automatically use IK to move wherever dragged.
             *  \param[in] node Node to enable Drag 'n Drop on.
             *  \param[in] callback Callback function to call when node is dragged.
             *  \return The new drag 'n drop node.
             */
            DnDReturn enableNodeDragNDrop(dart::dynamics::BodyNode *node, const DnDCallback &callback = {});

            /** \} */

            /** \name Animation
                \{ */

            /** \brief Animate a motion plan using the world.
             *  \param[in] space State space of the plan.
             *  \param[in] path The plan to visualize.
             *  \param[in] times Number of times to loop through animation.
             *  \param[in] fps Update rate.
             *  \param[in] block If true, blocks until animation is done. Otherwise, immediately returns.
             */
            void animatePath(const StateSpacePtr &space, const ompl::geometric::PathGeometric &path,
                             std::size_t times = 1, double fps = 60, bool block = true);

            /** \brief Animate a motion plan using the world.
             *  This version of the call will automatically visualize either constrained or unconstrained
             * paths.
             *  \param[in] builder Plan builder structure. \param[in] path The plan to visualize.
             *  \param[in] times Number of times to loop through animation.
             *  \param[in] fps Update rate.
             *  \param[in] block If true, blocks until animation is done. Otherwise, immediately returns.
             */
            void animatePath(const PlanBuilder &builder, const ompl::geometric::PathGeometric &path,
                             std::size_t times = 1, double fps = 60, bool block = true);

            /** \} */

            /** \brief Run the GUI. Blocks.
             *  \param[in] thread Function to run in a separate thread from the GUI's visualization.
             */
            void run(std::function<void()> thread = {});

            /** \brief Get the IMGUI configurable widget.
             */
            WindowWidgetPtr getWidget();

            /** \brief Add a new IMGUI widget.
             *  \param[in] widget Widget to add.
             */
            void addWidget(const WidgetPtr &widget);

        private:
            WorldPtr world_;                  ///< World to visualize.
            WindowWidgetPtr widget_;          ///< IMGUI widget.
            std::vector<WidgetPtr> widgets_;  ///< Other widgets;

            std::shared_ptr<std::thread> animation_{nullptr};  ///< Animation thread.

            ::osg::ref_ptr<Window> node_;         ///< OSG Node.
            dart::gui::osg::ImGuiViewer viewer_;  ///< Viewer
        };

        /** \brief Abstract class for IMGUI Widget.
         */
        class Widget : public dart::gui::osg::ImGuiWidget
        {
        public:
            /** \brief Initialization with window context.
             *  \param[in] window GUI window.
             */
            virtual void initialize(const Window *window);

            /** \brief Called before window refresh.
             */
            virtual void prerefresh();
        };

        /** \brief IMGUI widget to add interactive GUI elements programmatically.
         */
        class WindowWidget : public Widget
        {
        public:
            /** \cond IGNORE */
            ROBOWFLEX_CLASS_FORWARD(Element)
            /** \endcond */

            /** \class robowflex::darts::WindowWidget::ElementPtr
                \brief A shared pointer wrapper for robowflex::darts::WindowWidget::Element. */

            /** \class robowflex::darts::WindowWidget::ElementConstPtr
                \brief A const shared pointer wrapper for robowflex::darts::WindowWidget::Element. */

            /** \brief Abstract GUI element.
             */
            class Element
            {
            public:
                /** \brief Render method. Renders IMGUI contents.
                 */
                virtual void render() const = 0;
            };

            /** \brief A basic text element.
             */
            class TextElement : public Element
            {
            public:
                /** \brief Constructor.
                 *  \param[in] text Text to display.
                 */
                TextElement(const std::string &text);
                void render() const override;

            private:
                const std::string text;  ///< Text to display.
            };

            /** \brief A checkbox element that modifies a boolean.
             */
            class CheckboxElement : public Element
            {
            public:
                /** \brief Constructor.
                 *  \param[in] text Text to display.
                 *  \param[in] boolean Boolean to modify.
                 */
                CheckboxElement(const std::string &text, bool &boolean);
                void render() const override;

            private:
                const std::string text;  ///< Display text.
                bool &boolean;           ///< Associated boolean.
            };

            /** \brief Callback function upon a button press.
             */
            using ButtonCallback = std::function<void()>;

            /** \brief A basic push-button element.
             */
            class ButtonElement : public Element
            {
            public:
                /** \brief Constructor.
                 *  \param[in] text Text to display on button.
                 *  \param[in] callback Callback upon press.
                 */
                ButtonElement(const std::string &text, const ButtonCallback &callback);
                void render() const override;

            private:
                const std::string text;         ///< Display text.
                const ButtonCallback callback;  ///< Callback.
            };

            /** \brief Callback upon a render call.
             */
            using RenderCallback = std::function<void()>;

            /** \brief Generic rendered element. Use callback to display whatever GUI elements needed.
             */
            class RenderElement : public Element
            {
            public:
                /** \brief Constructor.
                 *  \param[in] callback Callback to render.
                 */
                RenderElement(const RenderCallback &callback);
                void render() const override;

            private:
                const RenderCallback callback;  ///< Callback.
            };

            /** \brief Constructor.
             */
            WindowWidget();

            /** \brief Render GUI.
             */
            void render() override;

            /** \name Element Addition
                \{ */

            /** \brief Add a new text element to the GUI.
             *  \param[in] text Text to display.
             */
            void addText(const std::string &text);

            /** \brief Add a new checkbox element to the GUI.
             *  \param[in] text Text to display on checkbox.
             *  \param[in] boolean Boolean to modify upon checkbox change.
             */
            void addCheckbox(const std::string &text, bool &boolean);

            /** \brief Add a new button element to the GUI.
             *  \param[in] text Text to display on the button.
             *  \param[in] callback Callback upon button press.
             */
            void addButton(const std::string &text, const ButtonCallback &callback);

            /** \brief Add a generic render callback for the GUI.
             *  \param[in] callback Callback to render.
             */
            void addCallback(const RenderCallback &callback);

            /** \} */

        private:
            std::vector<ElementPtr> elements_;  ///< GUI elements.
        };

        /** \brief IMGUI widget to design TSRs.
         */
        class TSRWidget : public Widget
        {
        public:
            /** \brief Constructor.
             *  \param[in] name Name of TSR.
             *  \param[in] spec Base specification of the TSR
             */
            TSRWidget(const std::string &name = "TSR", const TSR::Specification &spec = {});
            void initialize(const Window *window) override;
            void prerefresh() override;

            /** \brief Render GUI.
             */
            void render() override;

            /** \brief Solve for a solution to the current TSR.
             */
            void solve();

            /** \brief Get current TSR specification.
             *  \return The current TSR specification.
             */
            const TSR::Specification &getSpecification() const;

        private:
            void updateFrameCB(const dart::gui::osg::InteractiveFrame *frame);
            void updateLLCB(const dart::gui::osg::InteractiveFrame *frame);
            void updateUUCB(const dart::gui::osg::InteractiveFrame *frame);
            void updateMirror();
            void updateShape();

            void syncTSR();
            void syncSpec();
            void syncGUI();
            void syncFrame();

            Window *window_;
            WorldPtr world_;

            const std::string name_;
            const TSR::Specification original_;
            TSR::Specification spec_;
            TSR::Specification prev_;

            dart::dynamics::SimpleFramePtr shape_;
            dart::dynamics::SimpleFramePtr offset_;
            Window::InteractiveReturn frame_;
            Window::InteractiveReturn ll_frame_;
            Window::InteractiveReturn uu_frame_;

            bool gui_{false};

            float position_[3];
            float rotation_[3];

            // bounds
            bool sync_{true};
            bool show_{true};

            float xp_[2];
            float yp_[2];
            float zp_[2];

            float xr_[2];
            float yr_[2];
            float zr_[2];

            // result
            TSRPtr tsr_;
            std::mutex mutex_;
            float tolerance_;
            int maxIter_;
            bool track_{false};
            bool grad_{false};
            bool last_{false};

            static const std::size_t n_times_{100};
            std::size_t o_times_{0};
            float times_[n_times_] = {0.};
        };
    }  // namespace darts
}  // namespace robowflex

#endif
