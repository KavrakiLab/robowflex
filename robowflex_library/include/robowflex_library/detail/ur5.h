#ifndef ROBOWFLEX_UR5_
#define ROBOWFLEX_UR5_

namespace robowflex
{
    class UR5Robot : public Robot
    {
    public:
        UR5Robot();
        bool initialize();

    private:
        static const std::string URDF;
        static const std::string SRDF;
        static const std::string LIMITS;
        static const std::string KINEMATICS;
    };

    namespace OMPL
    {
        class UR5OMPLPipelinePlanner : public OMPLPipelinePlanner
        {
        public:
            UR5OMPLPipelinePlanner(UR5Robot &robot);

            bool initialize(const std::string &config_file = CONFIG, const Settings settings = Settings(),
                            const std::string &plugin = DEFAULT_PLUGIN,
                            const std::vector<std::string> &adapters = DEFAULT_ADAPTERS);

        private:
            static const std::string CONFIG;
        };
    }  // namespace OMPL
}  // namespace robowflex

#endif
