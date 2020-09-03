/*  Author Zachary Kingston */

#ifndef ROBOWFLEX_ID_
#define ROBOWFLEX_ID_

#include <atomic>

#include <robowflex_library/class_forward.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(ID);
    /** \endcond */

    /** \class robowflex::IDPtr
        \brief A shared pointer wrapper for robowflex::ID. */

    /** \class robowflex::IDConstPtr
        \brief A const shared pointer wrapper for robowflex::ID. */

    /** \brief Adds functionality to uniquely ID a specific class as well as the "version" of that class,
     * managed by an incrementing counter.
     */
    class ID
    {
    public:
        /** \brief Constructor.
         */
        ID();

        /** \brief Get the unique ID for this object.
         *  \return The unique ID.
         */
        const std::string &getID() const;

        /** \brief Get the current version of this object.
         *  \return The version number.
         */
        std::size_t getVersion() const;

        /** \brief Compare with another ID object.
         *  \param[in] b Object to compare against.
         *  \return True if the same, false otherwise.
         */
        bool operator==(const ID &b) const;

    protected:
        /** \brief Increment the version number of this object.
         */
        void incrementVersion();

    private:
        const std::string id_;        ///< Unique object ID
        std::atomic_size_t version_;  ///< Version number.
    };

    /** \brief Compare to ID objects.
     *  \param[in] a First object to compare.
     *  \param[in] b Second object to compare.
     *  \return True if \a a and \a b are the same, false otherwise.
     */
    bool compareIDs(const ID &a, const ID &b);

    /** \brief Compare to ID objects.
     *  \param[in] a First object to compare.
     *  \param[in] b Second object to compare.
     *  \return True if \a a and \a b are the same, false otherwise.
     */
    bool compareIDs(const IDPtr &a, const IDPtr &b);

    /** \brief Compare to ID objects.
     *  \param[in] a First object to compare.
     *  \param[in] b Second object to compare.
     *  \return True if \a a and \a b are the same, false otherwise.
     */
    bool compareIDs(const IDConstPtr &a, const IDConstPtr &b);
}  // namespace robowflex

#endif
