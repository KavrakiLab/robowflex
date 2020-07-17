/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_IO_BAG_
#define ROBOWFLEX_IO_BAG_

#include <robowflex_library/macros.h>

// clang-format off
ROBOWFLEX_PUSH_DISABLE_GCC_WARNING(-Wcast-qual)
// clang-format on
#include <rosbag/bag.h>
ROBOWFLEX_POP_GCC

#include <rosbag/view.h>

namespace robowflex
{
    namespace IO
    {
        /** \brief `rosbag` management class to ease message saving and loading.
         */
        class Bag
        {
        public:
            /** \brief File modes
             */
            enum Mode
            {
                READ,  ///< Read-only
                WRITE  ///< Write-only
            };

            /** \brief Constructor.
             *  \param[in] file File to open or create.
             *  \param[in] mode Mode to open file in.
             */
            Bag(const std::string &file, Mode mode = WRITE);

            /** \brief Destructor.
             *  Closes opened bag.
             */
            ~Bag();

            /** \brief Adds a message to the bag under \a topic.
             *  \param[in] topic Topic to save message under.
             *  \param[in] msg Message to write.
             *  \tparam T Type of message.
             */
            template <typename T>
            bool addMessage(const std::string &topic, const T &msg)
            {
                if (mode_ == WRITE)
                {
                    bag_.write(topic, ros::Time::now(), msg);
                    return true;
                }

                return false;
            }

            /** \brief Gets messages from an opened bag. Returns all messages of type \a T from a list of
             *  topics \a topics.
             *  \param[in] topics List of topics to load messages from.
             *  \tparam T type of messages to load from topics.
             */
            template <typename T>
            std::vector<T> getMessages(const std::vector<std::string> &topics)
            {
                std::vector<T> msgs;

                if (mode_ != READ)
                    return msgs;

                rosbag::View view(bag_, rosbag::TopicQuery(topics));
                for (auto &msg : view)
                {
                    typename T::ConstPtr ptr = msg.instantiate<T>();
                    if (ptr != nullptr)
                        msgs.emplace_back(*ptr);
                }

                return msgs;
            }

        private:
            const Mode mode_;         ///< Mode to open file in.
            const std::string file_;  ///< File opened.
            rosbag::Bag bag_;         ///< `rosbag` opened.
        };
    }  // namespace IO
}  // namespace robowflex

#endif
