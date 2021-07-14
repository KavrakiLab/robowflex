# Benchmarking Planners in Robowflex {#benchmarking}

Robowflex makes it easy to profile and benchmark motion planners.
The primary tool used is `robowflex::Profiler`, which profiles and collects metric data on a motion planning run, producing `robowflex::PlanData`.
`robowflex::Experiment` uses `robowflex::Profiler` on a collection of motion planning queries (`robowflex::PlanningQuery`) to generate a `robowflex::PlanDataSet`.
All relevant classes are found in `benchmarking.h`.

For the following documentation, assume we have loaded a Fetch robot and a basic scene and planner, such as this:
```cpp
// ...
#include <robowflex_library/benchmarking.h>
#include <robowflex_ompl/ompl_interface.h>
// ...

auto fetch = std::make_shared<FetchRobot>();
fetch->initialize();

auto scene = std::make_shared<Scene>(fetch);

// Create the OMPL interface planner (from robowflex_ompl), as it has access to planner progress properties
auto planner = std::make_shared<OMPL::OMPLInterfacePlanner>(fetch);
planner->initialize("package://robowflex_resources/fetch/config/ompl_planning.yaml");
```

## Profiler

`robowflex::Profiler` is used with `robowflex::Experiment`, but can be used standalone to manually profile a single run.
Here is a high-level example of creating a profiler and then profiling a plan:
```cpp
// Create the profiler
Profiler profiler;

// ... customize profiler here ...

// Setup options for profiling run, look at documentation for full list
Profiler::Options options;

// What built-in metrics to compute at the end of the run
options.metrics = Profiler::WAYPOINTS | Profiler::CORRECT | Profiler::LENGTH | Profiler::SMOOTHNESS;

// Profiler results are stored in this PlanData.
PlanData result;

// Run and profile the plan.
if (profiler.profilePlan(planner, scene, request->getRequest(), options, result))
    RBX_INFO("Planning success!");
else
    RBX_ERROR("Planning failure!");
    
// ... look at results here ...
```
Take a look at the documentation for `robowflex::Profiler` for further options.
Additionally, look at `robowflex::PlanData` for what information is computed by default by the profiler, such as planning time.
For an in-depth example of how to use `robowflex::Profiler`, look at `robowflex_ompl/scripts/fetch_profile.cpp`.

### Planning Metrics

An important feature of the profiler is that you can add custom metrics to compute at the end of a run.
After adding a custom metric, these are automatically computed and added to the `robowflex::PlanData`.
These metrics can also be added to the profiler used inside of a `robowflex::Experiment` (using `robowflex::Experiment::getProfiler()`), to add metrics in a benchmark.

Here is an example of a metric that returns distance-to-go to the goal (using `robowflex::OMPL::OMPLInterfacePlanner`, as above).
The resulting metric will be stored in `robowflex::PlanData::metrics` under the name "goal_distance".

```cpp
// ... create profiler ...

profiler.addMetricCallback("goal_distance",                          //
    [](const PlannerPtr &planner,                                    //
              const SceneConstPtr &scene,                            //
              const planning_interface::MotionPlanRequest &request,  //
              const PlanData &run) -> PlannerMetric {
        const auto &ompl_planner = std::dynamic_pointer_cast<const OMPL::OMPLInterfacePlanner>(planner);
        const auto &pdef = ompl_planner->getLastSimpleSetup()->getProblemDefinition();
        return pdef->getSolutionDifference();
    };
);

// ... profile plan ....
```

Note that planning metrics are stored as a boost variant type.
In order to extract the underlying data, you will have to do something like the following:
```cpp
PlanData result;

// ... profile goes here ...

double goal_distance = boost::get<double>(result.metrics["goal_distance"]);
```

### Planner Progress Properties

Some planners also expose _progress properties_, which are planner metrics that change over the course of a planning run.
A common use-case for these properties is to profile asymptotically optimal planners (such as RRT*) with properties such as the best path cost so far or number of iterations.
The profiler automatically includes whatever progress properties are exposed through the planner itself, that is, through the `robowflex::Planner::getProgressProperties()` function.
If you are using `robowflex_ompl` `robowflex::OMPL::OMPLInterfacePlanner`, this will return the underlying OMPL planner's progress properties.
The profiler captures progress properties by spinning up a separate thread that queries the planner for each property at a specified update rate.

There are a number of options associated with the progress properties, look at the documentation for more information:
```cpp
Profiler::Options options;
options.progress = true;
options.progress_at_least_once = true;
options.progress_update_rate = 0.1;
```

You can also specify custom progress properties for the profiler to use.
Progress properties are specified through progress property allocator functions, which generate the query at the start of the profiling run, so that the function can be customized to the specifics of the query.
Here is an example of a progress property that counts the number of vertices in the OMPL planning graph:
```cpp
// ... create profiler ...

profiler.addProgressAllocator("num vertices INTEGER",  //
    [](const PlannerPtr &planner,                      //
              const SceneConstPtr &scene,              //
              const planning_interface::MotionPlanRequest &request) -> Planner::ProgressProperty {
        const auto &ompl_planner = std::dynamic_pointer_cast<const OMPL::OMPLInterfacePlanner>(planner);
        const auto &ss = ompl_planner->getLastSimpleSetup();
        const auto &op = ss->getPlanner();

        return [op] {
            ompl::base::PlannerData pd(op->getSpaceInformation());
            op->getPlannerData(pd);

            return std::to_string(pd.numVertices());
        };
    });
    
// ... profile plan ....
```

There are a few things to note.
First, if there are any progress properties captured, there is also a progress property called "time REAL" that is captured in tandem, which is the time that the progress property was captured in seconds since planning started.
Second, all progress properties are strings.
Third, all progress property names must include their datatype at the end of their name.
These are datatypes supported by SQL, e.g., "INTEGER", "VARCHAR(128)", etc.
This is to tell the benchmark experiment output functions how to encode the data.

Say you wanted to extract the "num vertices INTEGER" in a usable format, such as a time series.
You could do something like the following:
```cpp
PlanData result;

// ... profile goes here ...

std::vector<double> times;
std::vector<int> verts;
for (const auto &sample : result.progress)
{
    times.emplace_back(boost::lexical_cast<double>(sample["time REAL"]));
    verts.emplace_back(boost::lexical_cast<int>(sample["num vertices INTEGER"]));
}
```

You can also add simple callback functions to the profiler that are called at the same rate as the progress properties, e.g.,
```cpp
profiler.addProgressCallback([](const PlannerPtr &planner,                             //
                                const SceneConstPtr &scene,                            //
                                const planning_interface::MotionPlanRequest &request,  //
                                const PlanData &result) {
        std::cout << "Wow! A callback!" << std::endl;
    });
```

## Experiment

`robowflex::Experiment` is a wrapper that profiles a number of motion planning queries and collects all data into a `robowflex::PlanDataSet`.
It is simple to setup and run an experiment:
```cpp
Profiler::Options options;
// ... setup options ...

Experiment experiment( //
  "demo",  // Name of the experiment. This is encoded in output files so experiments can be told apart.
  options, // Options used by the underlying profiler. Can be modifed later by using Experiment::getOptions().
  60.0,    // Total time allotted to each query. This overrides whatever is in the request, unless you call
           //   Experiment::overridePlanningTime().
  100,     // Total number of trials that each query should be profiled for.
  false    // (Optional, defaults to false) If true, the benchmarker will re-run each query until the *total* 
           //   time has exceeded the allotted time. That is, say 5 seconds are allotted to a query. The 
           //   planner solves the problem in 3 seconds. The benchmarker will then re-run the same query but 
           //   only with a time budget of 2 seconds.
);

// Add a named query to the experiment.
// These consist of the planning scene, the motion planner, and the request.
// Note that you can add *multiple* queries under the same name - the experiment will collect the data under
// the same name if this is the case.
// This is useful if you have variations on a planning scene.
experiment.addQuery("basic", scene, planner, request);

// ... add pre & post run callbacks to experiment ...
experiment.setPreRunCallback([&](const PlannerQuery &query) {
    std::cout << "Running query " << query.name << std::endl;
 });

Profiler &profiler = experiment.getProfiler();
// ... modify profiler, e.g., add custom metrics ...

// Run Benchmark!
PlanDataSetPtr dataset = experiment.benchmark(1);
```

Note that you can add pre- and post-run callbacks to the experiment that run for every trial.
There is also post-query callback that is called after a run's data has been entered into the dataset.

Finally, you can output `robowflex::PlanDataSet` to a number of output file types, all of which inherit from `robowflex::PlanDataSetOutputter`, e.g.,
```cpp
OMPLPlanDataSetOutputter output("results");
output.dump(*dataset);
```
The resulting log files from `robowflex::OMPLPlanDataSetOutputter` can be used with OMPL's `ompl_benchmark_statistics.py` script to generate benchmarking databases.
See [the OMPL documentation for benchmarking](http://ompl.kavrakilab.org/benchmark.html) for more information.
The resulting database can be viewed online at [plannerarena.org](http://plannerarena.org/).

For an in-depth example of how to use `robowflex::Experiment`, look at:
- `robowflex_library/scripts/fetch_benchmark.cpp`.
- `robowflex_library/scripts/fetch_scenes_benchmark.cpp`.
- `robowflex_ompl/scripts/fetch_ompl_benchmark.cpp`.
- `robowflex_ompl/scripts/fetch_ompl_scenes_benchmark.cpp`.

**IMPORTANT** If you are using the `robowflex_ompl` `robowflex::OMPL::OMPLInterfacePlanner` you cannot benchmark with multiple threads, as the underlying planner is context sensitive.
Use only one thread if profiling queries that contain this planner.
