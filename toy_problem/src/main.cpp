#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/Profile.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/schedule/Viewer.hpp>
#include <rmf_traffic/schedule/Database.hpp>

#include <rmf_utils/optional.hpp>

#include <deque>
#include <eigen3/Eigen/Eigen>

namespace {

struct DeliveryTask
{
  uint64_t id; // task id
  rmf_traffic::Time request_time;
  std::size_t pickup_waypoint;
  std::size_t dropoff_waypoint;
};

struct Robot
{
  uint64_t id; // participant id
  std::size_t start_waypoint;
  std::deque<uint64_t> task_queue = {};
};

class TaskPlanner
{
public:
  TaskPlanner(
    std::vector<DeliveryTask> tasks,
    std::vector<Robot> robots,
    rmf_traffic::agv::Planner& planner)
  : _planner(std::move(planner))
  {
    _num_tasks = tasks.size();
    _num_robots = robots.size();

    for (const auto& task : tasks)
      _tasks.insert({task.id, task});
    for (const auto& robot : robots)
      _robots.insert({robot.id, robot});

    _graph = _planner.get_configuration().graph();
  }

  void solve()
  {
    // Fill up the dqueues for each robot with task-ids 
  }

  void display_solution()
  {

  }

private:
  std::size_t _num_tasks = 0;
  std::size_t _num_robots = 0;
  std::unordered_map<uint64_t, DeliveryTask> _tasks;
  std::unordered_map<uint64_t, Robot> _robots;
  rmf_traffic::agv::Graph _graph;
  rmf_traffic::agv::Planner _planner;
};

rmf_utils::clone_ptr<rmf_traffic::agv::ScheduleRouteValidator>
make_test_schedule_validator(
  const rmf_traffic::schedule::Viewer& viewer,
  rmf_traffic::Profile profile)
{
  return rmf_utils::make_clone<rmf_traffic::agv::ScheduleRouteValidator>(
    viewer,
    std::numeric_limits<rmf_traffic::schedule::ParticipantId>::max(),
    std::move(profile));
}

} // anonymous namespace


int main(int argc, char* argv[])
{  
  // Graph
  // 00-01-02-03
  // |  |  |  |
  // 04-05-06-07
  // |  |  |  |
  // 08-09-10-11
  // |  |  |  | 
  // 12-13-14-15

  rmf_traffic::agv::Graph graph;
  auto add_bidir_lane = [&](const std::size_t w0, const std::size_t w1)
    {
      graph.add_lane(w0, w1);
      graph.add_lane(w1, w0);
    };
  const double edge_length = 100; //metres
  const std::string map_name = "test_map";
  for (int i = 0; i < 4; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      graph.add_waypoint(map_name, {j*edge_length, -i*edge_length});
    }
  }

  for (int i = 0; i < 16; ++i)
  {
    if ((i+1) % 4 != 0)
      add_bidir_lane(i,i+1);
    if (i + 4 < 16)
      add_bidir_lane(i,i+4);
  }


  const auto shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(1.0);
  const rmf_traffic::Profile profile{shape, shape};
  const rmf_traffic::agv::VehicleTraits traits(
    {1.0, 0.7}, {0.6, 0.5}, profile);
  rmf_traffic::schedule::Database database;
  const auto default_options = rmf_traffic::agv::Planner::Options{
    make_test_schedule_validator(database, profile)};
  rmf_traffic::agv::Planner planner{
    rmf_traffic::agv::Planner::Configuration{graph, traits},
    default_options    
  };

  // TODO: parse yaml to obtain list of tasks and robots
  std::vector<Robot> robots;
  std::vector<DeliveryTask> tasks;
  Robot robot1{1, 13, {}};
  Robot robot2{2, 2, {}};
  robots.emplace_back(robot1);
  robots.emplace_back(robot2);
  
  const auto start_time = std::chrono::steady_clock::now();
  DeliveryTask task1{1, start_time, 0, 3};
  DeliveryTask task2{2, start_time, 15, 2};
  DeliveryTask task3{3, start_time, 7, 9};
  tasks.emplace_back(task1);
  tasks.emplace_back(task2);
  tasks.emplace_back(task3);

  TaskPlanner task_planner{
    tasks,
    robots,
    planner
  };

  task_planner.solve();
  task_planner.display_solution();
  
  return 1;
}

