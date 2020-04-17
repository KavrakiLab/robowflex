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
        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(Structure)
        /** \endcond */

        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(ACM)
        /** \endcond */

        /** \class robowflex::darts::ACMPtr
            \brief A shared pointer wrapper for robowflex::darts::ACM. */

        /** \class robowflex::darts::ACMConstPtr
            \brief A const shared pointer wrapper for robowflex::darts::ACM. */

        /** \brief Allowable collision matrix for robowflex::darts::Structure.
         */
        class ACM
        {
        public:
            /** \brief Constructor.
             *  \param[in] structure The structure the ACM is for.
             */
            ACM(const Structure *structure);

            /** \name Enabling / Disabling Collisions
                \{ */

            /** \brief Disable collisions between body frames \a a and \a b.
             *  \param[in] a Body frame A.
             *  \param[in] b Body frame B.
             */
            void disableCollision(const std::string &a, const std::string &b);

            /** \brief Enable collisions between body frames \a a and \a b.
             *  \param[in] a Body frame A.
             *  \param[in] b Body frame B.
             */
            void enableCollision(const std::string &a, const std::string &b);

            /** \} */

            /** \name ACM Information
                \{ */

            /** \brief Get the pairs of frames that have collision disabled.
             *  \return The set of pairs with collision disabled.
             */
            std::set<std::pair<std::string, std::string>> &getDisabledPairs();

            /** \brief Get the pairs of frames that have collision disabled.
             *  \return The set of pairs with collision disabled.
             */
            const std::set<std::pair<std::string, std::string>> &getDisabledPairsConst() const;

            /** \brief Get the Dart collision filter corresponding to this ACM.
             *  \return The collision filter.
             */
            std::shared_ptr<dart::collision::BodyNodeCollisionFilter> getFilter();

            /** \brief Get the Dart collision filter corresponding to this ACM.
             *  \return The collision filter.
             */
            const std::shared_ptr<dart::collision::BodyNodeCollisionFilter> &getFilterConst() const;

            /** \} */

            /** \brief Get the structure this ACM is for.
             *  \return The structure for the ACM.
             */
            const Structure *getStructure() const;

        private:
            /** \brief Create a unique key for two frame names.
             *  \param[in] a Body frame A.
             *  \param[in] b Body frame B.
             *  \return Key for disabled collision in ACM.
             */
            std::pair<std::string, std::string> makeKey(const std::string &a, const std::string &b) const;

            /** \brief Get the body node in the structure corresponding to the body frame name.
             *  \param[in] key Name of body frame.
             *  \return Body frame in structure.
             */
            dart::dynamics::BodyNode *getBodyNode(const std::string &key);

            const Structure *structure_;                         ///< Structure this ACM is for.
            std::set<std::pair<std::string, std::string>> acm_;  ///< Disabled collision pairs.
            std::shared_ptr<dart::collision::BodyNodeCollisionFilter> filter_;  ///< Collision filter
        };

    }  // namespace darts
}  // namespace robowflex

#endif
