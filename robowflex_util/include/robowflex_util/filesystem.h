/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_MOVEIT_FILESYSTEM_
#define ROBOWFLEX_MOVEIT_FILESYSTEM_

#include <string>   // for std::string
#include <utility>  // for std::pair
#include <fstream>  // for std::ofstream
#include <set>      // for std::set
#include <vector>   // for std::vector

namespace robowflex
{
    /** \brief File and ROS Input / Output operations.
     */
    namespace IO
    {
        /** \brief Resolves `package://` URLs to their canonical form.
         *  The path does not need to exist, but the package does. Can be used to write new files in packages.
         *  \param[in] path Path to resolve.
         *  \return The canonical path, or "" on failure.
         */
        std::string resolvePackage(const std::string &path);

        /** \brief Finds all package URIs within a string.
         *  \param[in] string String to search.
         *  \return List of package URIs.
         */
        std::set<std::string> findPackageURIs(const std::string &string);

        /** \brief Checks if an extension is the file extension of a path.
         *  \param[in] path_string Path to check extension on.
         *  \param[in] extension Extension to check.
         *  \return True if path is of the extension.
         */
        bool isExtension(const std::string &path_string, const std::string &extension);

        /** \brief Resolves `package://` URLs and relative file paths to their canonical form.
         *  \param[in] path Path to resolve.
         *  \return The canonical path, or "" on failure.
         */
        std::string resolvePath(const std::string &path);

        /** \brief Resolves `package://` URLs to get the directory this path is in.
         *  \param[in] path Path to get the parent of.
         *  \return The directory that this path is contained in, or "" on failure.
         */
        std::string resolveParent(const std::string &path);

        /** \brief Concatenates two elements of a path, a directory and a filename.
         *  \param[in] directory Path to use as directory. If there are other elements at the end of the
         *                       directory path, they will be removed.
         *  \param[in] filename Filename to add.
         *  \return The canonical path for this path.
         */
        std::string makeFilepath(const std::string &directory, const std::string &filename);

        /** \brief Loads an XML or .xacro file to a string.
         *  \param[in] path File to load.
         *  \return The loaded file, or "" on failure (file does not exist or .xacro is malformed).
         */
        std::string loadXMLToString(const std::string &path);

        /** \brief Loads a .xacro file to a string.
         *  \param[in] path File to load.
         *  \return The loaded file, or "" on failure (file does not exist or .xacro is malformed).
         */
        std::string loadXacroToString(const std::string &path);

        /** \brief Loads a file to a string.
         *  \param[in] path File to load.
         *  \return The loaded file, or "" on failure (file does not exist).
         */
        std::string loadFileToString(const std::string &path);

        /** \brief Creates a file and opens an output stream. Creates directories if they do not exist.
         *  \param[out] out Output stream to initialize.
         *  \param[in] file File to create and open.
         */
        void createFile(std::ofstream &out, const std::string &file);

        /** \brief Creates a temporary file and opens an output stream.
         *  \param[out] out Output stream to initialize.
         *  \return Filename of temporary file.
         */
        std::string createTempFile(std::ofstream &out);

        /** \brief Deletes a file.
         *  \param[in] file File to delete.
         */
        void deleteFile(const std::string &file);

        /** \brief Lists of the contents of a directory.
         *  \param[in] directory Directory to list.
         *  \return A pair of a bool and a vector of strings of filenames of the directories contents. The
         * first element will be true on success, false on failure. These filenames are absolute paths.
         */
        std::pair<bool, std::vector<std::string>> listDirectory(const std::string &directory);
    }  // namespace IO
}  // namespace robowflex

#endif
