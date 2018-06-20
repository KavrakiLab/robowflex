#ifndef ROBOWFLEX_R2_
#define ROBOWFLEX_R2_

namespace robowflex
{
    class R2Robot : public Robot
    {
    public:
        R2Robot();
        bool initialize(const std::vector<std::string> kinematics);

    private:
        static const std::string URDF;
        static const std::string SRDF;
        static const std::string LIMITS;
        static const std::string KINEMATICS;
        static const std::string CACHED;
        static const std::vector<std::string> SAMPLERS;
    };

    namespace OMPL
    {
        ROBOWFLEX_CLASS_FORWARD(R2OMPLPipelinePlanner);
        class R2OMPLPipelinePlanner : public OMPLPipelinePlanner
        {
        public:
            R2OMPLPipelinePlanner(R2Robot &robot);

            bool initialize(const std::string &config_file = CONFIG, const Settings settings = Settings(),
                            const std::string &plugin = PLUGIN,
                            const std::vector<std::string> &adapters = DEFAULT_ADAPTERS);

        private:
            static const std::string CONFIG;
            static const std::string PLUGIN;
        };
    }  // namespace OMPL
}  // namespace robowflex

#endif
