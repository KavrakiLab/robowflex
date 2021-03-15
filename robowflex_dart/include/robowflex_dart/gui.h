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
        ROBOWFLEX_CLASS_FORWARD(TSREditWidget)
        /** \endcond */

        /** \class robowflex::darts::WindowPtr
            \brief A shared pointer wrapper for robowflex::darts::Window. */

        /** \class robowflex::darts::WindowConstPtr
            \brief A const shared pointer wrapper for robowflex::darts::Window. */

        /** \class robowflex::darts::WindowWidgetPtr
            \brief A shared pointer wrapper for robowflex::darts::WindowWidget. */

        /** \class robowflex::darts::WindowWidgetConstPtr
            \brief A const shared pointer wrapper for robowflex::darts::WindowWidget. */

        /** \brief Generate a unique identifier.
         *  \return A random unique ID.
         */
        std::string generateUUID();

        /** \brief Viewer class.
         */
        class Viewer : public dart::gui::osg::ImGuiViewer
        {
        public:
            /** \brief Constructor.
             *  \param[in] world World viewed.
             */
            Viewer(const WorldPtr &world);

            void updateTraversal() override;

        private:
            WorldPtr world_;  ///< World.
        };

        /** \brief Open Scene Graph GUI for DART visualization.
         */
        class Window : public dart::gui::osg::WorldNode
        {
            friend TSREditWidget;

        public:
            /** \brief Constructor.
             *  \param[in] world World to visualize.
             */
            Window(const WorldPtr &world);
            void customPreRefresh() override;
            void customPostRefresh() override;

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

                /** \brief Disables all linear controls.
                 */
                void disableLinearControls();

                /** \brief Disables all rotation controls.
                 */
                void disableRotationControls();

                /** \brief Disables all planar controls.
                 */
                void disablePlanarControls();

                /** \brief Disables all controls.
                 */
                void disableControls();
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

            /** \brief Get world used for visualization.
             *  \return World used by window.
             */
            WorldPtr getWorld();

            /** \brief Get world used for visualization.
             *  \return World used by window.
             */
            const WorldPtr &getWorldConst() const;

        private:
            WorldPtr world_;                  ///< World to visualize.
            WindowWidgetPtr widget_;          ///< IMGUI widget.
            std::vector<WidgetPtr> widgets_;  ///< Other widgets;

            std::shared_ptr<std::thread> animation_{nullptr};  ///< Animation thread.

            ::osg::ref_ptr<Window> node_;  ///< OSG Node.
            Viewer viewer_;                ///< Viewer
        };

        /** \brief Abstract class for IMGUI Widget.
         */
        class Widget : public dart::gui::osg::ImGuiWidget
        {
        public:
            /** \brief Initialization with window context.
             *  \param[in] window GUI window.
             */
            virtual void initialize(Window *window);

            /** \brief Called before window refresh.
             */
            virtual void prerefresh();
        };

        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(ImGuiElement)
        /** \endcond */

        /** \class robowflex::darts::ImGuiElementPtr
            \brief A shared pointer wrapper for robowflex::darts::WindowWidget::Element. */

        /** \class robowflex::darts::ImGuiElementConstPtr
            \brief A const shared pointer wrapper for robowflex::darts::WindowWidget::Element. */

        /** \brief Abstract GUI element.
         */
        class ImGuiElement
        {
        public:
            /** \brief Render method. Renders IMGUI contents.
             */
            virtual void render() const = 0;
        };

        /** \brief A basic text element.
         */
        class TextElement : public ImGuiElement
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
        class CheckboxElement : public ImGuiElement
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
        class ButtonElement : public ImGuiElement
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
        class RenderElement : public ImGuiElement
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

        /** \brief Line plot element. Displays an updated line graph of data.
         */
        class LinePlotElement : public ImGuiElement
        {
        public:
            std::string id{"##" + generateUUID()};  ///< Unique ID.
            std::string label{""};                  ///< Plot Label.
            std::string units{""};                  ///< Plot Units.
            bool show_min{false};                   ///< Display minimum value under plot.
            bool show_max{false};                   ///< Display maximum value under plot.
            bool show_avg{false};                   ///< Display average value under plot.
            bool recent{true};                      ///< Display most recent value on plot.
            std::size_t max_size{100};              ///< Maximum size of plot data.
            Eigen::Vector3d color{1., 1., 1.};      ///< Color of plot.

            /** \brief Add a point to the plot data.
             *  \param[in] x Point to add.
             */
            void addPoint(float x);
            void render() const override;

        private:
            std::size_t index{0};           ///< Index in data.
            std::size_t total_elements{0};  ///< Total elements inserted in data.
            std::vector<float> elements;    ///< All data points.
            float latest;                   ///< Last input data point.

            /** \brief Compute the average over the data.
             *  \return The average.
             */
            float average() const;

            /** \brief Compute the minimum over the data.
             *  \return The minimum.
             */
            float minimum() const;

            /** \brief Compute the maximum over the data.
             *  \return The maximum.
             */
            float maximum() const;
        };

        /** \brief IMGUI widget to add interactive GUI elements programmatically.
         */
        class WindowWidget : public Widget
        {
        public:
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

            /** \brief Add a generic element to the GUI.
             *  \param[in] element Element to render.
             */
            void addElement(const ImGuiElementPtr &element);

            /** \} */

        private:
            std::vector<ImGuiElementPtr> elements_;  ///< GUI elements.
        };

        /** \brief IMGUI widget to design TSRs.
         */
        class TSREditWidget : public Widget
        {
        public:
            /** \brief Constructor.
             *  \param[in] name Name of TSR.
             *  \param[in] spec Base specification of the TSR
             */
            TSREditWidget(const std::string &name = "TSR", const TSR::Specification &spec = {});
            void initialize(Window *window) override;
            void prerefresh() override;

            /** \brief Render GUI.
             */
            void render() override;

            /** \brief Get current TSR specification.
             *  \return The current TSR specification.
             */
            const TSR::Specification &getSpecification() const;

            /** \brief Get the current TSR.
             *  \return The current TSR.
             */
            const TSRPtr &getTSR() const;

        private:
            /** \name Element Synchronization
                \{ */

            /** \brief Frame update callback on moving the main interactive frame.
             *  \param[in] frame The interactive frame.
             */
            void updateFrameCB(const dart::gui::osg::InteractiveFrame *frame);

            /** \brief Frame update callback on moving the lower bound control.
             *  \param[in] frame The interactive frame.
             */
            void updateLLCB(const dart::gui::osg::InteractiveFrame *frame);

            /** \brief Frame update callback on moving the upper bound control.
             *  \param[in] frame The interactive frame.
             */
            void updateUUCB(const dart::gui::osg::InteractiveFrame *frame);

            /** \brief If synchronizing bounds, mirrors updates on other bound.
             */
            void updateMirror();

            /** \brief Updates the TSR to the specification.
             */
            void syncTSR();

            /** \brief Updates the specification to the GUI.
             */
            void syncSpec();

            /** \brief Updates the GUI to the specification.
             */
            void syncGUI();

            /** \brief Updates the display frames to the specification.
             */
            void syncFrame();

            bool gui_{false};  ///< True if a synchronize call is coming from the GUI update loop.

            /** \} */

            /** \name Interactive/Display Frames
                \{ */

            /** \brief Get the volume for the bounds.
             *  \return The volume of bounds.
             */
            Eigen::Vector3d getVolume() const;

            /** \brief Updates the displayed shape volume for the bounds.
             */
            void updateShape();

            dart::dynamics::SimpleFramePtr offset_;      ///< Offset frame for bounds.
            dart::dynamics::SimpleFramePtr shape_;       ///< Display boundary shape frame.
            dart::dynamics::SimpleFramePtr rbounds_[3];  ///< Display rotation bounds.
            Window::InteractiveReturn frame_;            ///< Main interactive frame.
            Window::InteractiveReturn ll_frame_;         ///< Lower bound interactive frame.
            Window::InteractiveReturn uu_frame_;         ///< Upper bound interactive frame.

            /** \} */

            /** \name TSR Specification
                \{ */

            const std::string name_;             ///< Name of this window.
            const TSR::Specification original_;  ///< Original specification provided to the window.
            TSR::Specification spec_;            ///< Current specification.
            TSR::Specification prev_;            ///< Prior iteration specification.
            TSRPtr tsr_;                         ///< Corresponding TSR.
            std::mutex mutex_;                   ///< Solving mutex.

            /** \} */

            /** \name GUI Options
                \{ */

            bool sync_bounds_{true};  ///< Synchronize changes in volume on other bound.
            bool show_volume_{true};  ///< Show TSR volume.
            bool show_bounds_{true};  ///< Show TSR rotation bounds.

            /** \} */

            /** \name GUI Values
                \{ */

            const double volume_alpha_{0.2};     ///< Volume alpha.
            const double rotation_alpha_{0.6};   ///< Rotation bound alpha.
            const double rotation_width_{0.05};  ///< Rotation bound width.

            const float max_position_{5.0f};  ///< Max position value.
            const float drag_step_{0.01f};    ///< Slider drag amount.

            float position_[3];  ///< GUI frame position.
            float rotation_[3];  ///< GUI frame rotation.

            float xp_[2];  ///< GUI X position bounds.
            float yp_[2];  ///< GUI Y position bounds.
            float zp_[2];  ///< GUI Z position bounds.

            float xr_[2];  ///< GUI X orientation bounds.
            float yr_[2];  ///< GUI Y orientation bounds.
            float zr_[2];  ///< GUI Z orientation bounds.

            float inner_radius{0.2};  ///< GUI Rotation bound inner radius.

            /** \} */
        };

        /** \brief Class for solving a set of TSRs.
         */
        class TSRSolveWidget : public Widget
        {
        public:
            /** \brief Constructor.
             *  \param[in] world World to use.
             *  \param[in] tsrs Set of TSRs to consider.
             */
            TSRSolveWidget(const WorldPtr &world, const std::vector<TSRPtr> &tsrs);

            /** \brief Constructor.
             *  \param[in] tsrs Set of TSRs to consider.
             */
            TSRSolveWidget(const TSRSetPtr &tsrs);

            void initialize(Window *window) override;
            void prerefresh() override;

            /** \brief Render GUI.
             */
            void render() override;

            /** \brief Solve for a solution to the current TSR.
             */
            void solve();

        private:
            TSRSetPtr tsrs_;  ///< TSR set.

            /** \name GUI Values
                \{ */

            const float max_tolerance_{0.1f};    ///< Max tolerance value.
            const int max_iteration_{200};       ///< Max iteration value.
            const float drag_tolerance_{0.01f};  ///< Slider drag for tolerance.

            bool track_tsr_{false};     ///< Track the TSR by solving IK.
            bool use_gradient_{false};  ///< Use gradient solving instead of built-in.
            bool need_solve_{false};    ///< A solve is requested.

            float step_;       ///< GUI gradient step size.
            float limit_;      ///< GUI gradient limit.
            float damping_;    ///< GUI SVD damping.
            float tolerance_;  ///< GUI solver tolerance
            int maxIter_;      ///< GUI maximum allowed iterations.
            int item_{0};      ///< GUI solver.

            /** \} */

            /** \name GUI Plots
                \{ */

            bool last_solve_{false};      ///< Result of last TSR solve.
            LinePlotElement solve_time_;  ///< Plot of TSR solve times.

            /** \brief Error plots for TSRs.
             */
            struct ErrorLines
            {
                LinePlotElement xpd;  ///< X position error.
                LinePlotElement ypd;  ///< Y position error.
                LinePlotElement zpd;  ///< Z position error.
                LinePlotElement xrd;  ///< X orientation error.
                LinePlotElement yrd;  ///< Y orientation error.
                LinePlotElement zrd;  ///< Z orientation error.
            };

            std::vector<ErrorLines> errors_;

            /** \} */
        };
    }  // namespace darts
}  // namespace robowflex

#endif
