/* Author: Zachary Kingston */

#include <condition_variable>
#include <mutex>

#include <boost/uuid/uuid.hpp>             // for UUID generation
#include <boost/uuid/uuid_generators.hpp>  // for UUID generation
#include <boost/uuid/uuid_io.hpp>          // for UUID generationinclude <condition_variable>

#include <robowflex_library/constants.h>

#include <robowflex_dart/gui.h>

namespace constants = robowflex::constants;
using namespace robowflex::darts;

std::string robowflex::darts::generateUUID()
{
    boost::uuids::random_generator gen;
    boost::uuids::uuid u = gen();
    return boost::lexical_cast<std::string>(u);
}

//
// Viewer
//

Viewer::Viewer(const WorldPtr &world)
  : dart::gui::osg::ImGuiViewer(::osg::Vec4(1.0, 1.0, 1.0, 1.0)), world_(world)
{
}

void Viewer::updateTraversal()
{
    // world_->lock();
    osgViewer::Viewer::updateTraversal();
    // world_->unlock();
}

//
// Window::InteractiveOptions
//

void Window::InteractiveOptions::disableLinearControls()
{
    linear[0] = false;
    linear[1] = false;
    linear[2] = false;
}

void Window::InteractiveOptions::disableRotationControls()
{
    rotation[0] = false;
    rotation[1] = false;
    rotation[2] = false;
}

void Window::InteractiveOptions::disablePlanarControls()
{
    planar[0] = false;
    planar[1] = false;
    planar[2] = false;
}

void Window::InteractiveOptions::disableControls()
{
    disableLinearControls();
    disableRotationControls();
    disablePlanarControls();
}

//
// Window
//

Window::Window(const WorldPtr &world)
  : dart::gui::osg::WorldNode(world->getSim()), world_(world), viewer_(world)
{
    node_ = this;
    viewer_.addWorldNode(node_);
    viewer_.setUpViewInWindow(0, 0, 1080, 720);
    auto *cm = viewer_.getCameraManipulator();
    cm->setHomePosition(                //
        ::osg::Vec3(5.00, 5.00, 2.00),  //
        ::osg::Vec3(0.00, 0.00, 0.00),  //
        ::osg::Vec3(-0.24, -0.25, 0.94));
    viewer_.setCameraManipulator(cm);
    viewer_.setVerticalFieldOfView(15);

    widget_ = std::make_shared<WindowWidget>();
    addWidget(widget_);
}

void Window::customPreRefresh()
{
    // world_->lock();
    for (const auto &widget : widgets_)
        widget->prerefresh();
}

void Window::customPostRefresh()
{
    // world_->unlock();
}

void Window::addWidget(const WidgetPtr &widget)
{
    widgets_.emplace_back(widget);
    widget->initialize(const_cast<Window *>(this));
    viewer_.getImGuiHandler()->addWidget(widget);
}

Window::InteractiveReturn Window::createInteractiveMarker(const InteractiveOptions &options)
{
    InteractiveReturn r;
    r.target = std::make_shared<dart::gui::osg::InteractiveFrame>(  //
        options.parent, options.name, options.pose, options.size, options.thickness);
    world_->getSim()->addSimpleFrame(r.target);

    for (std::size_t i = 0; i < 3; ++i)
    {
        auto *lt = r.target->getTool(dart::gui::osg::InteractiveTool::Type::LINEAR, i);
        lt->setEnabled(options.linear[i]);
        auto *rt = r.target->getTool(dart::gui::osg::InteractiveTool::Type::ANGULAR, i);
        rt->setEnabled(options.rotation[i]);
        auto *pt = r.target->getTool(dart::gui::osg::InteractiveTool::Type::PLANAR, i);
        pt->setEnabled(options.planar[i]);
    }

    r.dnd = viewer_.enableDragAndDrop(r.target.get());
    r.dnd->setObstructable(options.obstructable);

    auto callback = options.callback;
    r.signal = r.target->onTransformUpdated.connect([callback](const dart::dynamics::Entity *entity) {
        if (entity)
        {
            const auto *cast = dynamic_cast<const dart::gui::osg::InteractiveFrame *>(entity);
            if (cast and callback)
                callback(cast);
        }
    });

    return r;
}

Window::DnDReturn Window::enableNodeDragNDrop(dart::dynamics::BodyNode *node, const DnDCallback &callback)
{
    DnDReturn r;
    auto *dnd = viewer_.enableDragAndDrop(node, true, true);
    r.dnd = dnd;

    r.signal = node->onTransformUpdated.connect([dnd, callback](const dart::dynamics::Entity *entity) {
        if (dnd->isMoving())
            callback(dynamic_cast<const dart::dynamics::BodyNode *>(entity));
    });

    return r;
}

void Window::animatePath(const StateSpacePtr &space, const ompl::geometric::PathGeometric &path,
                         std::size_t times, double fps, bool block)
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

                std::this_thread::sleep_for(std::chrono::milliseconds((unsigned int)(1000 / fps)));
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

void Window::animatePath(const PlanBuilder &builder, const ompl::geometric::PathGeometric &path,
                         std::size_t times, double fps, bool block)
{
    ompl::geometric::PathGeometric extract(builder.rinfo);
    for (std::size_t i = 0; i < path.getStateCount(); ++i)
        extract.append(builder.toStateConst(path.getState(i)));

    animatePath(builder.rspace, extract, times, fps, block);
}

WindowWidgetPtr Window::getWidget()
{
    return widget_;
}

WorldPtr Window::getWorld()
{
    return world_;
}

const WorldPtr &Window::getWorldConst() const
{
    return world_;
}

void Window::run(std::function<void()> thread)
{
    if (thread)
    {
        std::thread t(thread);
        viewer_.run();
    }
    else
        viewer_.run();
}

//
// Widget
//

void Widget::initialize(Window * /*window*/)
{
}

void Widget::prerefresh()
{
}

//
// Window Widget
//

TextElement::TextElement(const std::string &text) : text(text)
{
}

void TextElement::render() const
{
    ImGui::Text("%s", text.c_str());
}

CheckboxElement::CheckboxElement(const std::string &text, bool &boolean) : text(text), boolean(boolean)
{
}

void CheckboxElement::render() const
{
    ImGui::Checkbox(text.c_str(), &boolean);
}

ButtonElement::ButtonElement(const std::string &text, const ButtonCallback &callback)
  : text(text), callback(callback)
{
}

void ButtonElement::render() const
{
    const auto &button = ImGui::Button(text.c_str());
    if (button)
        callback();
}

RenderElement::RenderElement(const RenderCallback &callback) : callback(callback)
{
}

void RenderElement::render() const
{
    if (callback)
        callback();
}

float LinePlotElement::average() const
{
    float avg = 0.;
    for (std::size_t i = 0; i < total_elements; ++i)
        avg += elements[i];
    avg /= (float)max_size;
    return avg;
}

float LinePlotElement::minimum() const
{
    float min = (total_elements) ? std::numeric_limits<float>::max() : 0;
    for (std::size_t i = 0; i < total_elements; ++i)
        min = (min > elements[i]) ? elements[i] : min;
    return min;
}

float LinePlotElement::maximum() const
{
    float max = 0.;
    for (std::size_t i = 0; i < total_elements; ++i)
        max = (max < elements[i]) ? elements[i] : max;
    return max;
}

void LinePlotElement::addPoint(float x)
{
    elements.resize(max_size);
    if (index >= max_size)
        index = 0;

    if (total_elements > max_size)
        total_elements = max_size;

    latest = elements[index++] = x;
    index = index % max_size;

    if (total_elements < max_size)
        total_elements++;
}

void LinePlotElement::render() const
{
    ImGui::PushID(id.c_str());
    ImGui::PushStyleColor(ImGuiCol_PlotLines,
                          (ImVec4)ImColor((float)color[0], (float)color[1], (float)color[2]));

    char overlay[64] = "N/A";
    if (total_elements)
        sprintf(overlay, "%.3f %s", latest, units.c_str());

    ImGui::PlotLines(label.c_str(), elements.data(), total_elements, index, overlay);

    ImGui::PopStyleColor(1);
    ImGui::PopID();

    if (show_min)
        ImGui::Text("min: %.3f %s", minimum(), units.c_str());
    if (show_avg)
        ImGui::Text("avg: %.3f %s", average(), units.c_str());
    if (show_max)
        ImGui::Text("max: %.3f %s", maximum(), units.c_str());
}

WindowWidget::WindowWidget()
{
}

void WindowWidget::render()
{
    if (elements_.empty())
        return;

    ImGui::SetNextWindowBgAlpha(0.5f);
    if (!ImGui::Begin("Robowflex DART", nullptr,
                      ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_HorizontalScrollbar))
    {
        ImGui::End();
        return;
    }

    for (const auto &element : elements_)
        element->render();
}

void WindowWidget::addText(const std::string &text)
{
    addElement(std::make_shared<TextElement>(text));
}

void WindowWidget::addButton(const std::string &text, const ButtonCallback &callback)
{
    addElement(std::make_shared<ButtonElement>(text, callback));
}

void WindowWidget::addCheckbox(const std::string &text, bool &boolean)
{
    addElement(std::make_shared<CheckboxElement>(text, boolean));
}

void WindowWidget::addCallback(const RenderCallback &callback)
{
    addElement(std::make_shared<RenderElement>(callback));
}

void WindowWidget::addElement(const ImGuiElementPtr &element)
{
    elements_.push_back(element);
}

//
// TSR Widget
//
TSREditWidget::TSREditWidget(const std::string &name, const TSR::Specification &spec)
  : name_(name), original_(spec), spec_(spec)
{
}

void TSREditWidget::initialize(Window *window)
{
    auto world = window->world_;

    // Create main interactive marker.
    Window::InteractiveOptions frame_opt;
    frame_opt.name = "TSREditWidget-" + name_ + "-frame";
    frame_opt.pose = spec_.pose;
    if (spec_.base.frame != magic::ROOT_FRAME)
        frame_opt.parent = world->getRobot(spec_.base.structure)->getFrame(spec_.base.frame);
    frame_opt.callback = [&](const dart::gui::osg::InteractiveFrame *frame) { updateFrameCB(frame); };
    frame_ = window->createInteractiveMarker(frame_opt);

    // Create offset frame for bound frames.
    offset_ = std::make_shared<dart::dynamics::SimpleFrame>(dart::dynamics::Frame::World(),
                                                            "TSREditWidget-" + name_ + "-offset");
    world->getSim()->addSimpleFrame(offset_);

    // Create lower bound control.
    Window::InteractiveOptions ll_opt;
    ll_opt.name = "TSREditWidget-" + name_ + "-ll";
    ll_opt.callback = [&](const dart::gui::osg::InteractiveFrame *frame) { updateLLCB(frame); };
    ll_opt.parent = offset_.get();
    ll_opt.rotation[0] = false;
    ll_opt.rotation[1] = false;
    ll_opt.rotation[2] = false;
    ll_opt.planar[0] = false;
    ll_opt.planar[1] = false;
    ll_opt.planar[2] = false;
    ll_opt.size = 0.1;
    ll_opt.thickness = 2;
    ll_frame_ = window->createInteractiveMarker(ll_opt);

    // Create upper bound control.
    Window::InteractiveOptions uu_opt;
    uu_opt.name = "TSREditWidget-" + name_ + "-uu";
    uu_opt.callback = [&](const dart::gui::osg::InteractiveFrame *frame) { updateUUCB(frame); };
    uu_opt.parent = offset_.get();
    uu_opt.rotation[0] = false;
    uu_opt.rotation[1] = false;
    uu_opt.rotation[2] = false;
    uu_opt.planar[0] = false;
    uu_opt.planar[1] = false;
    uu_opt.planar[2] = false;
    uu_opt.size = 0.1;
    uu_opt.thickness = 2;
    uu_frame_ = window->createInteractiveMarker(uu_opt);

    // Create shape frame. Shape is separate since it can be offset from main bound TF.
    shape_ =
        std::make_shared<dart::dynamics::SimpleFrame>(offset_.get(), "TSREditWidget-" + name_ + "-shape");
    world->getSim()->addSimpleFrame(shape_);

    // Create rotation bounds.
    for (std::size_t i = 0; i < 3; ++i)
    {
        rbounds_[i] = std::make_shared<dart::dynamics::SimpleFrame>(
            offset_.get(), "TSREditWidget-" + name_ + "-rb" + std::to_string(i));

        Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
        if (i == 1)
            tf.rotate(Eigen::AngleAxisd(-constants::half_pi, Eigen::Vector3d::UnitZ()));
        else if (i == 2)
            tf.rotate(Eigen::AngleAxisd(constants::half_pi, Eigen::Vector3d::UnitY()));

        rbounds_[i]->setRelativeTransform(tf);
        world->getSim()->addSimpleFrame(rbounds_[i]);
    }

    // Create TSR.
    tsr_ = std::make_shared<darts::TSR>(world, spec_);
    tsr_->initialize();

    // Synchronize elements.
    syncGUI();
    syncFrame();
    updateShape();
}

void TSREditWidget::syncFrame()
{
    frame_.target->setRelativeTransform(spec_.pose);
    offset_->setTranslation(frame_.target->getWorldTransform().translation());
    ll_frame_.target->setRelativeTranslation(spec_.position.lower);
    uu_frame_.target->setRelativeTranslation(spec_.position.upper);
}

void TSREditWidget::syncTSR()
{
    tsr_->getSpecification() = spec_;
    tsr_->updatePose();
    tsr_->updateBounds();
    tsr_->updateSolver();
}

void TSREditWidget::syncSpec()
{
    spec_.setPosition(position_[0], position_[1], position_[2]);
    spec_.setRotation(rotation_[0], rotation_[1], rotation_[2]);

    spec_.setXPosTolerance(xp_[0], xp_[1]);
    spec_.setYPosTolerance(yp_[0], yp_[1]);
    spec_.setZPosTolerance(zp_[0], zp_[1]);

    spec_.setXRotTolerance(xr_[0], xr_[1]);
    spec_.setYRotTolerance(yr_[0], yr_[1]);
    spec_.setZRotTolerance(zr_[0], zr_[1]);

    updateMirror();
    syncTSR();
}

void TSREditWidget::syncGUI()
{
    auto pos = spec_.getPosition();
    position_[0] = pos[0];
    position_[1] = pos[1];
    position_[2] = pos[2];

    auto rot = spec_.getEulerRotation();
    rotation_[0] = rot[0];
    rotation_[1] = rot[1];
    rotation_[2] = rot[2];

    xp_[0] = spec_.position.lower[0];
    xp_[1] = spec_.position.upper[0];
    yp_[0] = spec_.position.lower[1];
    yp_[1] = spec_.position.upper[1];
    zp_[0] = spec_.position.lower[2];
    zp_[1] = spec_.position.upper[2];

    xr_[0] = spec_.orientation.lower[0];
    xr_[1] = spec_.orientation.upper[0];
    yr_[0] = spec_.orientation.lower[1];
    yr_[1] = spec_.orientation.upper[1];
    zr_[0] = spec_.orientation.lower[2];
    zr_[1] = spec_.orientation.upper[2];
}

void TSREditWidget::updateFrameCB(const dart::gui::osg::InteractiveFrame *frame)
{
    if (gui_)
        return;

    prev_ = spec_;
    const auto &tf = frame->getRelativeTransform();
    spec_.pose = tf;

    updateMirror();
    syncGUI();
    syncTSR();
}

void TSREditWidget::updateLLCB(const dart::gui::osg::InteractiveFrame *frame)
{
    if (gui_)
        return;

    Eigen::Vector3d t = frame->getRelativeTransform().translation();
    Eigen::Vector3d diff =
        (sync_bounds_) ? Eigen::Vector3d(t - spec_.position.lower) : Eigen::Vector3d::Zero();

    spec_.setXPosTolerance(t[0], spec_.position.upper[0] - diff[0]);
    spec_.setYPosTolerance(t[1], spec_.position.upper[1] - diff[1]);
    spec_.setZPosTolerance(t[2], spec_.position.upper[2] - diff[2]);

    syncGUI();
    syncTSR();
}

void TSREditWidget::updateUUCB(const dart::gui::osg::InteractiveFrame *frame)
{
    if (gui_)
        return;

    Eigen::Vector3d t = frame->getRelativeTransform().translation();
    Eigen::Vector3d diff =
        (sync_bounds_) ? Eigen::Vector3d(t - spec_.position.upper) : Eigen::Vector3d::Zero();

    spec_.setXPosTolerance(spec_.position.lower[0] - diff[0], t[0]);
    spec_.setYPosTolerance(spec_.position.lower[1] - diff[1], t[1]);
    spec_.setZPosTolerance(spec_.position.lower[2] - diff[2], t[2]);

    syncGUI();
    syncTSR();
}

void TSREditWidget::updateMirror()
{
    if (not sync_bounds_)
        return;

    {
        auto ld = spec_.position.lower - prev_.position.lower;
        auto ud = spec_.position.upper - prev_.position.upper;

        spec_.position.lower -= ud;
        spec_.position.upper -= ld;
    }

    {
        auto ld = spec_.orientation.lower - prev_.orientation.lower;
        auto ud = spec_.orientation.upper - prev_.orientation.upper;

        spec_.orientation.lower -= ud;
        spec_.orientation.upper -= ld;
    }
}

Eigen::Vector3d TSREditWidget::getVolume() const
{
    return spec_.position.upper - spec_.position.lower;
}

void TSREditWidget::updateShape()
{
    // Position volume
    {
        Eigen::Vector3d abs = getVolume();

        shape_->setShape(makeBox(abs));
        auto *va = shape_->getVisualAspect(true);
        va->setColor(dart::Color::Gray(volume_alpha_));
        va->setShadowed(false);
        va->setHidden(not show_volume_);

        shape_->setRelativeTranslation(spec_.position.lower + abs / 2);
    }

    // Rotation bounds
    for (std::size_t i = 0; i < 3; ++i)
    {
        rbounds_[i]->setShape(makeArcsegment(spec_.orientation.lower[i], spec_.orientation.upper[i],
                                             inner_radius, inner_radius + rotation_width_));
        auto *va = rbounds_[i]->getVisualAspect(true);
        va->setShadowed(false);
        va->setHidden(not show_bounds_);

        if (i == 0)
            va->setColor(dart::Color::Red(rotation_alpha_));
        if (i == 1)
            va->setColor(dart::Color::Green(rotation_alpha_));
        if (i == 2)
            va->setColor(dart::Color::Blue(rotation_alpha_));
    }
}

void TSREditWidget::render()
{
    prev_ = spec_;  // save previous spec

    // Draw main window.
    ImGui::SetNextWindowBgAlpha(0.5f);
    std::string title = "TSR Editor - " + name_;
    if (!ImGui::Begin(title.c_str(), nullptr, ImGuiWindowFlags_HorizontalScrollbar))
    {
        ImGui::End();
        return;
    }

    // Draw frame position and orientation values.
    if (ImGui::TreeNodeEx("Frame", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::DragFloat3("Position", position_, drag_step_, -max_position_, max_position_, "%0.2f");
        ImGui::DragFloat3("Rotation", rotation_, drag_step_, -constants::pi, constants::pi, "%0.2f");

        if (ImGui::Button("Reset Position"))
        {
            spec_.setPosition(original_.getPosition());
            syncGUI();
        }

        ImGui::SameLine();

        if (ImGui::Button("Reset Rotation"))
        {
            spec_.setRotation(original_.getRotation());
            syncGUI();
        }

        ImGui::TreePop();
    }

    ImGui::Separator();
    ImGui::Spacing();

    // Draw TSR bound interface.
    if (ImGui::TreeNodeEx("Bounds", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Checkbox("Sync. Bounds", &sync_bounds_);
        ImGui::Spacing();

        ImGui::Text("Position Bounds");
        auto half = max_position_ / 2;
        ImGui::DragFloat2("X##1", xp_, drag_step_, -half, half, "%0.4f");
        ImGui::DragFloat2("Y##1", yp_, drag_step_, -half, half, "%0.4f");
        ImGui::DragFloat2("Z##1", zp_, drag_step_, -half, half, "%0.4f");
        ImGui::Checkbox("Show Volume", &show_volume_);
        ImGui::Spacing();
        ImGui::Spacing();

        ImGui::Text("Rotation Bounds");
        ImGui::DragFloat2("X##2", xr_, drag_step_, -constants::pi, constants::pi, "%0.4f");
        ImGui::DragFloat2("Y##2", yr_, drag_step_, -constants::pi, constants::pi, "%0.4f");
        ImGui::DragFloat2("Z##2", zr_, drag_step_, -constants::pi, constants::pi, "%0.4f");

        ImGui::Checkbox("Show Rot. Bounds", &show_bounds_);
        ImGui::DragFloat("Bound Rad.", &inner_radius, drag_step_, 0., 0.5, "%0.2f");

        if (ImGui::Button("Reset Bounds"))
        {
            spec_.position = original_.position;
            spec_.orientation = original_.orientation;
            prev_ = spec_;
            syncGUI();
        }

        ImGui::TreePop();
    }

    if (ImGui::Button("Print TSR"))
        spec_.print(std::cout);

    gui_ = true;
    syncSpec();
    syncGUI();
    syncFrame();
    gui_ = false;
}

void TSREditWidget::prerefresh()
{
    updateShape();
}

const TSR::Specification &TSREditWidget::getSpecification() const
{
    return spec_;
}

const TSRPtr &TSREditWidget::getTSR() const
{
    return tsr_;
}

//
// TSRSolveWidget
//

TSRSolveWidget::TSRSolveWidget(const WorldPtr &world, const std::vector<TSRPtr> &tsrs)
  : TSRSolveWidget(std::make_shared<TSRSet>(world, tsrs, false))
{
}

TSRSolveWidget::TSRSolveWidget(const TSRSetPtr &tsrs) : tsrs_(tsrs)
{
}

void TSRSolveWidget::render()
{
    // Draw main window.
    ImGui::SetNextWindowBgAlpha(0.5f);
    std::string title = "TSR Solver";
    if (!ImGui::Begin(title.c_str(), nullptr, ImGuiWindowFlags_HorizontalScrollbar))
    {
        ImGui::End();
        return;
    }

    // Draw TSR solve interface.
    if (ImGui::TreeNodeEx("Solving", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Checkbox("Track TSR", &track_tsr_);
        ImGui::SameLine();
        ImGui::Checkbox("Use Gradient?", &use_gradient_);

        ImGui::Columns(2);
        if (ImGui::SliderInt("Max Iter.", &maxIter_, 1, max_iteration_))
        {
            tsrs_->setMaxIterations(maxIter_);
            tsrs_->updateSolver();
        }
        if (ImGui::SliderFloat("Step", &step_, 0.001f, 1.f))
            tsrs_->setStep(step_);

        if (ImGui::SliderFloat("Limit", &limit_, 0.001f, 1.f))
            tsrs_->setLimit(limit_);

        ImGui::NextColumn();
        if (ImGui::SliderFloat("Tol.", &tolerance_, 1e-5f, max_tolerance_, "< %.5f", 3.f))
        {
            tsrs_->setMaxIterations(tolerance_);
            tsrs_->updateSolver();
        }

        const char *items[] = {"dSVD", "SVD", "QR"};
        if (ImGui::Combo("combo", &item_, items, IM_ARRAYSIZE(items)))
        {
            if (item_ == 0)
            {
                tsrs_->useSVD();
                tsrs_->useDamping(true);
            }
            else if (item_ == 1)
            {
                tsrs_->useSVD();
                tsrs_->useDamping(false);
            }
            else
                tsrs_->useQR();
        }

        if (ImGui::SliderFloat("Damp.", &damping_, 1e-8f, 1e-3f, "< %.8f", 10.f))
            tsrs_->setDamping(damping_);

        ImGui::Columns(1);

        if (ImGui::Button("Solve TSR"))
            need_solve_ = true;

        ImGui::SameLine();

        if (last_solve_)
            ImGui::TextColored((ImVec4)ImColor(0.0f, 1.0f, 0.0f), "Success!");
        else
            ImGui::TextColored((ImVec4)ImColor(1.0f, 0.0f, 0.0f), "Failure!");

        solve_time_.render();

        ImGui::TreePop();
    }

    ImGui::Separator();
    ImGui::Spacing();

    // Draw distance tracking information.

    const auto &tsrs = tsrs_->getTSRs();
    for (std::size_t i = 0; i < tsrs.size(); ++i)
    {
        const auto &tsr = tsrs[i];
        const auto &spec = tsr->getSpecification();

        const std::string title = "B:" + spec.base.frame + ":" + spec.base.structure +  //
                                  " > T:" + spec.target.frame + ":" + spec.target.structure;

        if (ImGui::TreeNodeEx(title.c_str(), ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::Columns(2);
            ImGui::Text("Pos. Error");
            errors_[i].xpd.render();
            errors_[i].ypd.render();
            errors_[i].zpd.render();
            ImGui::NextColumn();

            ImGui::Text("Rot. Error");
            errors_[i].xrd.render();
            errors_[i].yrd.render();
            errors_[i].zrd.render();

            ImGui::Columns(1);

            ImGui::TreePop();
        }
    }
}

void TSRSolveWidget::solve()
{
    auto start = std::chrono::steady_clock::now();

    if (use_gradient_)
    {
        Eigen::VectorXd world(tsrs_->getWorldIndices().size());
        tsrs_->getPositionsWorldState(world);

        last_solve_ = tsrs_->solveGradientWorldState(world);
    }
    else
        last_solve_ = tsrs_->solveWorld();

    auto end = std::chrono::steady_clock::now();

    auto time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    solve_time_.addPoint(time);
}

void TSRSolveWidget::initialize(Window * /*window*/)
{
    // Compute world indices for all TSRs
    const auto &tsrs = tsrs_->getTSRs();
    std::set<std::pair<std::size_t, std::size_t>> wis;
    for (const auto &tsr : tsrs)
    {
        std::vector<std::pair<std::size_t, std::size_t>> wts =
            (tsr->getNumWorldDofs()) ? tsr->getWorldIndices() : tsr->computeWorldIndices();

        for (const auto &index : wts)
            wis.emplace(index);
    }

    std::vector<std::pair<std::size_t, std::size_t>> wts;
    wts.reserve(wis.size());
    for (const auto &index : wis)
        wts.emplace_back(index);

    tsrs_->useWorldIndices(wts);
    tsrs_->setWorldIndices(wts);
    tsrs_->computeLimits();

    tsrs_->initialize();

    // Setup solve parameters.
    maxIter_ = tsrs_->getMaxIterations();
    tolerance_ = tsrs_->getTolerance();
    step_ = tsrs_->getStep();
    limit_ = tsrs_->getLimit();
    damping_ = tsrs_->getDamping();

    // Setup solve time plots.
    solve_time_.label = "Solve Time";
    solve_time_.units = "microsec.";
    solve_time_.show_min = true;
    solve_time_.show_max = true;
    solve_time_.show_avg = true;

    // Setup error plots.
    const std::size_t n = tsrs_->numTSRs();
    errors_.resize(n);

    for (std::size_t i = 0; i < n; ++i)
    {
        errors_[i].xpd.color = Eigen::Vector3d(1, 0, 0);
        errors_[i].ypd.color = Eigen::Vector3d(0, 1, 0);
        errors_[i].zpd.color = Eigen::Vector3d(0, 0, 1);

        errors_[i].xrd.label = "X";
        errors_[i].xrd.color = Eigen::Vector3d(1, 0, 0);
        errors_[i].yrd.label = "Y";
        errors_[i].yrd.color = Eigen::Vector3d(0, 1, 0);
        errors_[i].zrd.label = "Z";
        errors_[i].zrd.color = Eigen::Vector3d(0, 0, 1);
    }
}

void TSRSolveWidget::prerefresh()
{
    if (track_tsr_ or need_solve_)
        solve();

    need_solve_ = false;

    const auto &tsrs = tsrs_->getTSRs();
    for (std::size_t i = 0; i < tsrs.size(); ++i)
    {
        Eigen::VectorXd e = Eigen::VectorXd::Zero(6);
        tsrs[i]->getErrorWorldRaw(e);

        errors_[i].xrd.addPoint(e[0]);
        errors_[i].yrd.addPoint(e[1]);
        errors_[i].zrd.addPoint(e[2]);
        errors_[i].xpd.addPoint(e[3]);
        errors_[i].ypd.addPoint(e[4]);
        errors_[i].zpd.addPoint(e[5]);
    }
}
