/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_POOL_
#define ROBOWFLEX_POOL_

#include <memory>      // for std::shared_ptr
#include <thread>      // for std::thread
#include <future>      // for std::future / std::promise
#include <functional>  // for std::function
#include <vector>      // for std::vector
#include <queue>       // for std::queue

namespace robowflex
{
    /**
     * make_function implementation taken from:
     * https://stackoverflow.com/questions/27825559/why-is-there-no-stdmake-function/27826081#27826081
     * TODO: Investigate other APIs for the submit() function in robowflex::Pool.
     */

    /** \cond IGNORE */

    template <typename T>
    struct function_traits : public function_traits<decltype(&T::operator())>
    {
    };

    // for pointers to member function
    template <typename C, typename RT, typename... Args>
    struct function_traits<RT (C::*)(Args...) const>
    {
        enum
        {
            arity = sizeof...(Args)
        };
        typedef std::function<RT(Args...)> f_type;
    };

    // for pointers to member function
    template <typename C, typename RT, typename... Args>
    struct function_traits<RT (C::*)(Args...)>
    {
        enum
        {
            arity = sizeof...(Args)
        };
        typedef std::function<RT(Args...)> f_type;
    };

    // for function pointers
    template <typename RT, typename... Args>
    struct function_traits<RT (*)(Args...)>
    {
        enum
        {
            arity = sizeof...(Args)
        };
        typedef std::function<RT(Args...)> f_type;
    };

    template <typename L>
    static typename function_traits<L>::f_type make_function(L l)
    {
        return (typename function_traits<L>::f_type)(l);
    }

    // handles bind & multiple function call operator()'s
    template <typename RT, typename... Args, class T>
    auto make_function(T &&t) -> std::function<decltype(RT(t(std::declval<Args>()...)))(Args...)>
    {
        return {std::forward<T>(t)};
    }

    // handles explicit overloads
    template <typename RT, typename... Args>
    auto make_function(RT (*p)(Args...)) -> std::function<RT(Args...)>
    {
        return {p};
    }

    // handles explicit overloads
    template <typename RT, typename... Args, typename C>
    auto make_function(RT (C::*p)(Args...)) -> std::function<RT(Args...)>
    {
        return {p};
    }

    /* \endcond */

    /** \brief A thread pool that can execute arbitrary functions asynchronously.
     *  Functions with arguments to be executed are put in the queue through submit(). This returns a
     *  Pool::Job that can be used to retrieve the result or cancel the job if the result is no longer needed.
     */
    class Pool
    {
    public:
        /** \brief Interface class for Pool::Job so template parameters are not needed for the queue.
         */
        class Joblet
        {
        public:
            /** \brief Execute the underlying function.
             */
            virtual void execute() = 0;

            /** \brief Cancels this job.
             */
            void cancel();

            /** \brief Checks if this job has been cancled.
             *  \return True if the job is cancled, false otherwise.
             */
            bool isCancled() const;

        protected:
            bool canceled{false};  ///< Whether the job is cancled or not.
        };

        /** \brief A job that returns \a RT.
         *  \tparam RT Return type of function to be executed.
         */
        template <typename RT>
        class Job : public Joblet
        {
        public:
            /** \brief Constructor.
             *  \param[in] function Function to execute.
             *  \param[in] args Arguments to function.
             *  \tparam Args Types of the function arguments.
             */
            template <typename... Args>
            Job(const std::function<RT(Args...)> &&function, Args &&... args)
              : function_(std::bind(function, args...)), task_(function_), future_(task_.get_future())
            {
            }

            /** \brief Executes the task and stores the result in \a future_
             */
            void execute() override
            {
                task_();
            }

            /** \brief Blocking call to retrieve the result of the function.
             *  Note that if the job was canceled it is not guaranteed that this function will return, as the
             *  job might never be executed.
             *  \return The result of the job.
             */
            RT get()
            {
                return future_.get();
            }

            /** \brief Waits until result of the job is available.
             */
            void wait() const
            {
                future_.wait();
            }

            /** \brief Returns true if the task is done, false otherwise.
             *  \return True if task is done, false otherwise.
             */
            bool isDone() const
            {
                return waitFor(0);
            }

            /** \brief Waits for a number of seconds to see if the task completes.
             *  \return True if task is complete, false otherwise.
             */
            bool waitFor(double time) const
            {
                return future_.wait_for(std::chrono::duration<double>(time)) == std::future_status::ready;
            }

        private:
            std::function<RT()> function_;   ///< Bound function to execute.
            std::packaged_task<RT()> task_;  ///< Task of function.
            std::future<RT> future_;         ///< Future of function result.
        };

        /** \brief Constructor.
         *  \param[in] n The number of threads to use. By default uses available hardware threads.
         */
        Pool(unsigned int n = std::thread::hardware_concurrency());

        /** \brief Destructor.
         *  Cancels all threads and joins them.
         */
        ~Pool();

        /** \brief Get the number of threads.
         *  \return The number of threads.
         */
        unsigned int getThreadCount() const;

        /** \brief Submit a function with arguments to be processed by the thread pool.
         *  Submitted functions must be wrapped with robowflex::make_function() or be a std::function type so
         *  argument template deduction works.
         *  \param[in] function Function to execute.
         *  \param[in] args Arguments to the function.
         *  \tparam RT Return type of function.
         *  \tparam Args Types of the arguments to the function.
         *  \return A job that contains information about the submitted function. This job can be canceled,
         *  which results in no execution of the function by the queue.
         */
        template <typename RT, typename... Args>
        std::shared_ptr<Job<RT>> submit(const std::function<RT(Args...)> &&function, Args &&... args) const
        {
            auto job = std::make_shared<Job<RT>>(std::forward<const std::function<RT(Args...)>>(function),
                                                 std::forward<Args>(args)...);

            {
                std::unique_lock<std::mutex> lock(mutex_);
                jobs_.emplace(job);

                cv_.notify_one();
            }

            return job;
        }

        /** \brief Background thread process.
         *  Executes jobs submitted from submit().
         */
        void run();

    private:
        bool active_{false};                  ///< Is thread pool active?
        mutable std::mutex mutex_;            ///< Job queue mutex.
        mutable std::condition_variable cv_;  ///< Job queue condition variable.

        std::vector<std::thread> threads_;                  ///< Threads.
        mutable std::queue<std::shared_ptr<Joblet>> jobs_;  ///< Jobs to execute.
    };
}  // namespace robowflex

#endif
