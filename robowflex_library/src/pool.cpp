/* Author: Zachary Kingston */

#include <robowflex_library/pool.h>

using namespace robowflex;

///
/// Joblet
///

void Pool::Joblet::cancel()
{
    canceled = true;
}

bool Pool::Joblet::isCancled() const
{
    return canceled;
}

///
/// Pool
///

Pool::Pool(unsigned int n) : active_(true)
{
    for (unsigned int i = 0; i < n; ++i)
        threads_.emplace_back(std::bind(&Pool::run, this));
}

Pool::~Pool()
{
    active_ = false;
    cv_.notify_all();

    for (auto &thread : threads_)
        thread.join();
}

unsigned int Pool::getThreadCount() const
{
    return threads_.size();
}

void Pool::run()
{
    while (active_)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [&] { return (active_ && !jobs_.empty()) || !active_; });

        if (!active_)
            break;

        auto job = jobs_.front();
        jobs_.pop();

        lock.unlock();

        // Ignore canceled jobs.
        if (!job->isCancled())
            job->execute();
    }
}
