using namespace robowflex;

namespace
{
    boost::filesystem::path expandHome(const boost::filesystem::path &in)
    {
        const char *home = std::getenv("HOME");
        if (home == nullptr)
        {
            XROS_WARN("HOME Environment variable is not set! Cannot resolve ~ in path.");
            return in;
        }

        boost::filesystem::path out;
        for (const auto &p : in)
            out /= (p.string() == "~") ? home : p;

        return out;
    }

    boost::filesystem::path expandSymlinks(const boost::filesystem::path &in)
    {
        // Check if the path has a symlink before expansion to avoid error.
        boost::filesystem::path out;
        for (const auto &p : in)
        {
            auto tmp = out / p;
            if (boost::filesystem::is_symlink(tmp))
                return boost::filesystem::canonical(in);
        }

        return in;
    }

    boost::filesystem::path expandPath(const boost::filesystem::path &in)
    {
        boost::filesystem::path out = in;
        out = expandHome(out);
        out = expandSymlinks(out);

        return boost::filesystem::absolute(out);
    }

    // is lhs a prefix of rhs?
    bool isPrefix(const std::string &lhs, const std::string &rhs)
    {
        return std::equal(lhs.begin(), lhs.begin() + std::min(lhs.size(), rhs.size()), rhs.begin());
    }

    // is lhs a suffix? of rhs?
    bool isSuffix(const std::string &lhs, const std::string &rhs)
    {
        return std::equal(lhs.rbegin(), lhs.rbegin() + std::min(lhs.size(), rhs.size()), rhs.rbegin());
    }

    bool isExtension(const std::string &path_string, const std::string &extension)
    {
        boost::filesystem::path path(path_string);
        const std::string last = boost::filesystem::extension(path);
        return isSuffix(extension, last);
    }
}  // namespace

std::string IO::loadFileToString(const std::string &path)
{
    const std::string full_path = resolvePath(path);
    if (full_path.empty())
        return "";

    std::ifstream ifs(full_path.c_str(), std::ios::in | std::ios::binary | std::ios::ate);

    std::ifstream::pos_type size = ifs.tellg();
    ifs.seekg(0, std::ios::beg);

    std::vector<char> bytes(size);
    ifs.read(bytes.data(), size);

    return std::string(bytes.data(), size);
}

std::string IO::runCommand(const std::string &cmd)
{
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd.c_str(), "r"), pclose);
    if (!pipe)
    {
        XROS_ERROR("Failed to run command `%s`!", cmd);
        return "";
    }

    while (!feof(pipe.get()))
    {
        if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
            result += buffer.data();
    }

    return result;
}
std::pair<bool, YAML::Node> IO::loadFileToYAML(const std::string &path)
{
    YAML::Node file;
    const std::string full_path = resolvePath(path);
    if (full_path.empty())
        return std::make_pair(false, file);

    if (!isExtension(full_path, "yml") && !isExtension(full_path, "yaml"))
        return std::make_pair(false, file);

    try
    {
        return std::make_pair(true, YAML::LoadFile(full_path));
    }
    catch (std::exception &e)
    {
        return std::make_pair(false, file);
    }
}

std::pair<bool, std::vector<YAML::Node>> IO::loadAllFromFileToYAML(const std::string &path)
{
    std::vector<YAML::Node> file;
    const std::string full_path = resolvePath(path);
    if (full_path.empty())
        return std::make_pair(false, file);

    if (!isExtension(full_path, "yml") && !isExtension(full_path, "yaml"))
        return std::make_pair(false, file);

    try
    {
        return std::make_pair(true, YAML::LoadAllFromFile(full_path));
    }
    catch (std::exception &e)
    {
        return std::make_pair(false, file);
    }
}

bool IO::YAMLToFile(const YAML::Node &node, const std::string &file)
{
    YAML::Emitter out;
    out << node;

    std::ofstream fout;
    IO::createFile(fout, file);

    fout << out.c_str();
    fout.close();

    return true;
}

std::string IO::generateUUID()
{
    boost::uuids::random_generator gen;
    boost::uuids::uuid u = gen();

    std::string s = boost::lexical_cast<std::string>(u);

    std::replace(s.begin(), s.end(), '-', '_');

    return s;
}

void IO::createFile(std::ofstream &out, const std::string &file)
{
    boost::filesystem::path path(file);
    path = expandHome(path);
    path = expandSymlinks(path);

    const auto parent = path.parent_path().string();

    if (!parent.empty())
        boost::filesystem::create_directories(parent);

    out.open(path.string(), std::ofstream::out | std::ofstream::trunc);
}

std::string IO::createTempFile(std::ofstream &out)
{
    auto temp = boost::filesystem::unique_path();
    auto filename = "/tmp/" + temp.string();
    createFile(out, filename);

    return filename;
}

void IO::deleteFile(const std::string &file)
{
    boost::filesystem::path path(file);
    path = expandHome(path);
    path = expandSymlinks(path);

    boost::filesystem::remove(path);
}

std::pair<bool, std::vector<std::string>> IO::listDirectory(const std::string &directory)
{
    std::vector<std::string> contents;

    const std::string full_path = resolvePath(directory);
    if (full_path.empty())
        return std::make_pair(false, contents);

    boost::filesystem::path path(full_path);
    if (!boost::filesystem::is_directory(path))
        return std::make_pair(false, contents);

    for (auto it = boost::filesystem::directory_iterator(path); it != boost::filesystem::directory_iterator();
         ++it)
        contents.emplace_back(expandPath(it->path()).string());

    return std::make_pair(true, contents);
}

std::string IO::getHostname()
{
    return boost::asio::ip::host_name();
}

std::size_t IO::getProcessID()
{
    return boost::interprocess::ipcdetail::get_current_process_id();
}

std::size_t IO::getThreadID()
{
    return boost::interprocess::ipcdetail::get_current_thread_id();
}

boost::posix_time::ptime IO::getDate()
{
    return boost::posix_time::microsec_clock::local_time();
}

double IO::getSeconds(boost::posix_time::ptime start, boost::posix_time::ptime finish)
{
    auto duration = finish - start;
    return duration.total_microseconds() / 1000000.;
}

void IO::threadSleep(double seconds)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long int>(seconds * 1000)));
}

std::size_t IO::hashYAML(const YAML::Node &node)
{
    std::stringstream sout;
    sout << node;

    return hashString(sout.str());
}

std::size_t IO::hashString(const std::string &string)
{
    return std::hash<std::string>{}(string);
}

template <typename T>
std::vector<T> IO::tokenize(const std::string &s, const std::string &separators)
{
    boost::char_separator<char> seps(separators.c_str());
    boost::tokenizer<boost::char_separator<char>> tokenizer(s, seps);

    std::vector<T> values;
    std::transform(tokenizer.begin(), tokenizer.end(), std::back_inserter(values),
                   [](const std::string &s) { return boost::lexical_cast<T>(s); });

    return std::vector<T>();
}

template std::vector<std::string> IO::tokenize(const std::string &, const std::string &);
template std::vector<double> IO::tokenize(const std::string &, const std::string &);
