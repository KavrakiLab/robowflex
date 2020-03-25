/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_DART_ACM_
#define ROBOWFLEX_DART_ACM_

#include <set>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/collision/CollisionFilter.hpp>

#include <robowflex_library/class_forward.h>

namespace robowflex
{
    namespace darts
    {
        ROBOWFLEX_CLASS_FORWARD(Structure)
        ROBOWFLEX_CLASS_FORWARD(ACM)

        class ACM
        {
        public:
            ACM(const Structure *robot);

            void disableCollision(const std::string &a, const std::string &b);
            void enableCollision(const std::string &a, const std::string &b);

            std::shared_ptr<dart::collision::BodyNodeCollisionFilter> getFilter();
            const std::shared_ptr<dart::collision::BodyNodeCollisionFilter> &getFilterConst() const;

            const Structure *getStructure() const;

            std::set<std::pair<std::string, std::string>> &getDisabledPairs();
            const std::set<std::pair<std::string, std::string>> &getDisabledPairsConst() const;

        private:
            std::pair<std::string, std::string> makeKey(const std::string &a, const std::string &b) const;
            dart::dynamics::BodyNode *getBodyNode(const std::string &key);

            const Structure *structure_;
            std::set<std::pair<std::string, std::string>> acm_;
            std::shared_ptr<dart::collision::BodyNodeCollisionFilter> filter_;
        };

    }  // namespace darts
}  // namespace robowflex

#endif
