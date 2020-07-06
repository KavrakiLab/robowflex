#include <mutex>
#include <condition_variable>

#include <robowflex_dart/gui.h>

using namespace robowflex::darts;

//
// Widget
//

void Widget::initialize(const Window *window)
{
}

//
// Window Widget
//

WindowWidget::TextElement::TextElement(const std::string &text) : text(text)
{
}

void WindowWidget::TextElement::render() const
{
    ImGui::Text("%s", text.c_str());
}

WindowWidget::CheckboxElement::CheckboxElement(const std::string &text, bool &boolean)
  : text(text), boolean(boolean)
{
}

void WindowWidget::CheckboxElement::render() const
{
    ImGui::Checkbox(text.c_str(), &boolean);
}

WindowWidget::ButtonElement::ButtonElement(const std::string &text, const ButtonCallback &callback)
  : text(text), callback(callback)
{
}

void WindowWidget::ButtonElement::render() const
{
    const auto &button = ImGui::Button(text.c_str());
    if (button)
        callback();
}

WindowWidget::RenderElement::RenderElement(const RenderCallback &callback) : callback(callback)
{
}

void WindowWidget::RenderElement::render() const
{
    if (callback)
        callback();
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
    elements_.push_back(std::make_shared<TextElement>(text));
}

void WindowWidget::addButton(const std::string &text, const ButtonCallback &callback)
{
    elements_.push_back(std::make_shared<ButtonElement>(text, callback));
}

void WindowWidget::addCheckbox(const std::string &text, bool &boolean)
{
    elements_.push_back(std::make_shared<CheckboxElement>(text, boolean));
}

void WindowWidget::addCallback(const RenderCallback &callback)
{
    elements_.push_back(std::make_shared<RenderElement>(callback));
}

//
// TSR Widget
//
TSRWidget::TSRWidget(const std::string &name, const TSR::Specification &spec)
  : name_(name), original_(spec), spec_(spec)
{
}

void TSRWidget::initialize(const Window *window)
{
    window_ = const_cast<Window *>(window);
    world_ = window->world_;

    // main pose control
    Window::InteractiveOptions frame_opt;
    frame_opt.name = "TSRWidget-" + name_ + "-frame";
    frame_opt.pose = spec_.pose;
    frame_opt.callback = [&](const dart::gui::osg::InteractiveFrame *frame) { updateFrameCB(frame); };

    frame_ = window_->createInteractiveMarker(frame_opt);

    offset_ = std::make_shared<dart::dynamics::SimpleFrame>(dart::dynamics::Frame::World(),
                                                            "TSRWidget-" + name_ + "-offset");

    // lower bound control
    Window::InteractiveOptions ll_opt;
    ll_opt.name = "TSRWidget-" + name_ + "-ll";
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
    ll_frame_ = window_->createInteractiveMarker(ll_opt);

    // upper bound control
    Window::InteractiveOptions uu_opt;
    uu_opt.name = "TSRWidget-" + name_ + "-uu";
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
    uu_frame_ = window_->createInteractiveMarker(uu_opt);

    shape_ = std::make_shared<dart::dynamics::SimpleFrame>(offset_.get(), "TSRWidget-" + name_ + "-shape");
    world_->getSim()->addSimpleFrame(shape_);

    syncGUI();
    syncFrame();

    updateShape();
}

void TSRWidget::syncFrame()
{
    frame_.target->setRelativeTransform(spec_.pose);
    offset_->setTranslation(frame_.target->getWorldTransform().translation());

    ll_frame_.target->setRelativeTranslation(spec_.position.lower);
    uu_frame_.target->setRelativeTranslation(spec_.position.upper);
}

void TSRWidget::syncSpec()
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
}

void TSRWidget::syncGUI()
{
    auto pos = spec_.getPosition();
    position_[0] = pos[0];
    position_[1] = pos[1];
    position_[2] = pos[2];

    auto rot = spec_.getRotation().toRotationMatrix().eulerAngles(0, 1, 2);
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

void TSRWidget::updateFrameCB(const dart::gui::osg::InteractiveFrame *frame)
{
    if (gui_)
        return;

    prev_ = spec_;
    auto tf = frame->getRelativeTransform();
    spec_.pose = tf;
    offset_->setRotation(Eigen::Matrix3d::Identity());

    syncGUI();
    updateMirror();
}

void TSRWidget::updateLLCB(const dart::gui::osg::InteractiveFrame *frame)
{
    if (gui_)
        return;

    Eigen::Vector3d t = frame->getRelativeTransform().translation();
    Eigen::Vector3d diff = (sync_) ? Eigen::Vector3d(t - spec_.position.lower) : Eigen::Vector3d::Zero();

    spec_.setXPosTolerance(t[0], spec_.position.upper[0] - diff[0]);
    spec_.setYPosTolerance(t[1], spec_.position.upper[1] - diff[1]);
    spec_.setZPosTolerance(t[2], spec_.position.upper[2] - diff[2]);

    syncGUI();
    updateShape();
}

void TSRWidget::updateUUCB(const dart::gui::osg::InteractiveFrame *frame)
{
    if (gui_)
        return;

    Eigen::Vector3d t = frame->getRelativeTransform().translation();
    Eigen::Vector3d diff = (sync_) ? Eigen::Vector3d(t - spec_.position.upper) : Eigen::Vector3d::Zero();

    spec_.setXPosTolerance(spec_.position.lower[0] - diff[0], t[0]);
    spec_.setYPosTolerance(spec_.position.lower[1] - diff[1], t[1]);
    spec_.setZPosTolerance(spec_.position.lower[2] - diff[2], t[2]);

    syncGUI();
    updateShape();
}

void TSRWidget::updateMirror()
{
    if (not sync_)
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

void TSRWidget::updateShape()
{
    Eigen::Vector3d abs = (spec_.position.upper - spec_.position.lower);
    Eigen::Vector3d mid = abs / 2;

    shape_->setRelativeTranslation(spec_.position.lower + mid);
    shape_->setShape(makeBox(abs[0], abs[1], abs[2]));

    auto va = shape_->createVisualAspect();
    va->setHidden(not show_);
    va->setShadowed(false);
    va->setColor(dart::Color::Gray(0.1));
}

void TSRWidget::render()
{
    prev_ = spec_;

    ImGui::SetNextWindowBgAlpha(0.5f);
    std::string title = "TSR Helper - " + name_;
    if (!ImGui::Begin(title.c_str(), nullptr, ImGuiWindowFlags_HorizontalScrollbar))
    {
        ImGui::End();
        return;
    }

    bool update = false;
    if (ImGui::TreeNodeEx("Frame", ImGuiTreeNodeFlags_DefaultOpen))
    {
        update |= ImGui::DragFloat3("Position", position_, 0.01f, -5.0f, 5.0f, "%0.2f");
        update |= ImGui::DragFloat3("Rotation", rotation_, 0.01f, -dart::math::constants<double>::pi(),
                                    dart::math::constants<double>::pi(), "%0.2f");

        if (ImGui::Button("Reset Position"))
        {
            update |= true;
            spec_.setPosition(original_.getPosition());
            syncGUI();
        }

        ImGui::SameLine();

        if (ImGui::Button("Reset Rotation"))
        {
            update |= true;
            spec_.setRotation(original_.getRotation());
            syncGUI();
        }

        ImGui::TreePop();
    }

    ImGui::Separator();

    if (ImGui::TreeNodeEx("Bounds", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Checkbox("Visualize", &show_);
        ImGui::SameLine();
        ImGui::Checkbox("Sync. Bounds", &sync_);

        update |= ImGui::DragFloat2("X Pos. Bounds", xp_, 0.01f, -2.5f, 2.5f, "%0.4f");
        update |= ImGui::DragFloat2("Y Pos. Bounds", yp_, 0.01f, -2.5f, 2.5f, "%0.4f");
        update |= ImGui::DragFloat2("Z Pos. Bounds", zp_, 0.01f, -2.5f, 2.5f, "%0.4f");

        update |= ImGui::DragFloat2("X Rot. Bounds", xr_, 0.01f, -2.5f, 2.5f, "%0.4f");
        update |= ImGui::DragFloat2("Y Rot. Bounds", yr_, 0.01f, -2.5f, 2.5f, "%0.4f");
        update |= ImGui::DragFloat2("Z Rot. Bounds", zr_, 0.01f, -2.5f, 2.5f, "%0.4f");

        if (ImGui::Button("Reset Bounds"))
        {
            update |= true;
            spec_.position = original_.position;
            spec_.orientation = original_.orientation;
            prev_ = spec_;
            syncGUI();
        }

        ImGui::TreePop();
    }

    if (ImGui::TreeNodeEx("Solving", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Checkbox("Track TSR", &track_);

        if (track_)
            solve();

        if (ImGui::Button("Solve TSR"))
            solve();

        ImGui::SameLine();

        if (last_)
            ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "Success!");
        else
            ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "Failure!");

        ImGui::TreePop();
    }

    {
        gui_ = true;
        syncSpec();
        syncGUI();
        syncFrame();
        updateShape();
        gui_ = false;
    }
}

void TSRWidget::solve()
{
    auto tsr = std::make_shared<darts::TSR>(world_, spec_);
    tsr->initialize();
    // tsr->getSpecification().print(std::cout);
    last_ = tsr->solveWorld();
}

const TSR::Specification &TSRWidget::getSpecification() const
{
    return spec_;
}

//
// Window
//

Window::Window(const WorldPtr &world) : dart::gui::osg::WorldNode(world->getSim()), world_(world)
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
    addWidget(widget_);
}

void Window::addWidget(const WidgetPtr &widget)
{
    widget->initialize(this);
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
        auto lt = r.target->getTool(dart::gui::osg::InteractiveTool::Type::LINEAR, i);
        lt->setEnabled(options.linear[i]);
        auto rt = r.target->getTool(dart::gui::osg::InteractiveTool::Type::ANGULAR, i);
        rt->setEnabled(options.rotation[i]);
        auto pt = r.target->getTool(dart::gui::osg::InteractiveTool::Type::PLANAR, i);
        pt->setEnabled(options.planar[i]);
    }

    r.dnd = viewer_.enableDragAndDrop(r.target.get());
    r.dnd->setObstructable(options.obstructable);

    auto callback = options.callback;
    r.signal = r.target->onTransformUpdated.connect([callback](const dart::dynamics::Entity *entity) {
        callback(dynamic_cast<const dart::gui::osg::InteractiveFrame *>(entity));
    });

    return r;
}

Window::DnDReturn Window::enableNodeDragNDrop(dart::dynamics::BodyNode *node, const DnDCallback &callback)
{
    DnDReturn r;
    auto dnd = viewer_.enableDragAndDrop(node, true, true);
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
        extract.append(builder.getStateConst(path.getState(i)));

    animatePath(builder.rspace, extract, times, fps, block);
}

WindowWidgetPtr Window::getWidget()
{
    return widget_;
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
