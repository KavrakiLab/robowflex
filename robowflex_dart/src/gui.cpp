#include <mutex>
#include <condition_variable>

#include <robowflex_dart/gui.h>

using namespace robowflex::darts;

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
    viewer_.getImGuiHandler()->addWidget(widget_);
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
