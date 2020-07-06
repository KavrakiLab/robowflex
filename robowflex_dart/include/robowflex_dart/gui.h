/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_DART_GUI_
#define ROBOWFLEX_DART_GUI_

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

        /** \class robowflex::darts::WindowPtr
            \brief A shared pointer wrapper for robowflex::darts::Window. */

        /** \class robowflex::darts::WindowConstPtr
            \brief A const shared pointer wrapper for robowflex::darts::Window. */

        /** \class robowflex::darts::WindowWidgetPtr
            \brief A shared pointer wrapper for robowflex::darts::WindowWidget. */

        /** \class robowflex::darts::WindowWidgetConstPtr
            \brief A const shared pointer wrapper for robowflex::darts::WindowWidget. */

        /** \brief IMGUI widget to add interactive GUI elements programmatically.
         */
        class WindowWidget : public dart::gui::osg::ImGuiWidget
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
                const RenderCallback callback; ///< Callback.
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
            std::vector<ElementPtr> elements_; ///< GUI elements.
        };

        /** \brief Open Scene Graph GUI for DART visualization.
         */
        class Window : public dart::gui::osg::WorldNode
        {
        public:
            /** \brief Constructor.
             *  \param[in] world World to visualize.
             */
            Window(const WorldPtr &world);

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
                double thickness{2.};  ///< Thickness of marker arrows.

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

            /** \brief Get the IMGUI widget.
             */
            WindowWidgetPtr getWidget();

        private:
            WorldPtr world_;          ///< World to visualize.
            WindowWidgetPtr widget_;  ///< IMGUI widget.

            std::shared_ptr<std::thread> animation_{nullptr};  ///< Animation thread.

            ::osg::ref_ptr<Window> node_;         ///< OSG Node.
            dart::gui::osg::ImGuiViewer viewer_;  ///< Viewer
        };
    }  // namespace darts
}  // namespace robowflex

#endif
