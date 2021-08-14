/*  Author Zachary Kingston */

#ifndef ROBOWFLEX_ID_
#define ROBOWFLEX_ID_

#include <atomic>
#include <string>

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
        /** \brief A snapshot of the state of an ID. Can be compared against another ID.
         */
        using Key = std::pair<std::string, std::size_t>;

        /** \brief Get a null key for initialization.
         *  \return The null key.
         */
        static Key getNullKey();

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

        /** \brief Get this ID as a Key.
         *  \return The ID as a Key.
         */
        Key getKey() const;

        /** \brief Compare with another ID object.
         *  \param[in] b Object to compare against.
         *  \return True if the same, false otherwise.
         */
        bool operator==(const ID &b) const;

        /** \brief Compare with an ID Key.
         *  \param[in] b Key to compare against.
         *  \return True if the same, false otherwise.
         */
        bool operator==(const Key &b) const;

    protected:
        /** \brief Increment the version number of this object.
         */
        void incrementVersion();

    private:
        const std::string id_;        ///< Unique object ID
        std::atomic_size_t version_;  ///< Version number.
    };

    /** \brief Compare two ID objects.
     *  \param[in] a First object to compare.
     *  \param[in] b Second object to compare.
     *  \return True if \a a and \a b are the same, false otherwise.
     */
    bool compareIDs(const ID &a, const ID &b);

    /** \brief Compare two ID objects.
     *  \param[in] a First object to compare.
     *  \param[in] b Second object to compare.
     *  \return True if \a a and \a b are the same, false otherwise.
     */
    bool compareIDs(const IDPtr &a, const IDPtr &b);

    /** \brief Compare two ID objects.
     *  \param[in] a First object to compare.
     *  \param[in] b Second object to compare.
     *  \return True if \a a and \a b are the same, false otherwise.
     */
    bool compareIDs(const IDConstPtr &a, const IDConstPtr &b);

    /** \brief Compare an ID object to a key.
     *  \param[in] a Object to compare.
     *  \param[in] b Key to compare against.
     *  \return True if \a a and \a b are the same, false otherwise.
     */
    bool compareIDs(const ID &a, const ID::Key &b);

    /** \brief Compare an ID object to a key.
     *  \param[in] a Object to compare.
     *  \param[in] b Key to compare against.
     *  \return True if \a a and \a b are the same, false otherwise.
     */
    bool compareIDs(const IDPtr &a, const ID::Key &b);

    /** \brief Compare an ID object to a key.
     *  \param[in] a Object to compare.
     *  \param[in] b Key to compare against.
     *  \return True if \a a and \a b are the same, false otherwise.
     */
    bool compareIDs(const IDConstPtr &a, const ID::Key &b);

    /** \brief Compare an ID object to a key.
     *  \param[in] a Key to compare.
     *  \param[in] b Key to compare against.
     *  \return True if \a a and \a b are the same, false otherwise.
     */
    bool compareIDs(const ID::Key &a, const ID::Key &b);
}  // namespace robowflex

#endif
