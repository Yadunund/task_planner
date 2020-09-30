#ifndef RMF_TASKS__TASK_ALLOCATION_HPP
#define RMF_TASKS__TASK_ALLOCATION_HPP

#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/Profile.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/schedule/Viewer.hpp>
#include <rmf_traffic/schedule/Database.hpp>

#include <rmf_battery/agv/SimpleDevicePowerSink.hpp>
#include <rmf_battery/agv/SimpleMotionPowerSink.hpp>
#include <rmf_battery/agv/BatterySystem.hpp>

#include <rmf_utils/optional.hpp>

#include <queue>
#include <vector>
#include <eigen3/Eigen/Eigen>
#include <unordered_map>
#include <map>
#include <iostream>
#include <limits>
#include <cstdlib>
#include <cassert>
#include <set>
#include <unordered_set>
#include <algorithm>

namespace {

const double segmentation_threshold = 1.0;
// ============================================================================
struct RobotState
{
  std::size_t waypoint;
  std::size_t charging_waypoint;
  double finish_time = 0.0;
  double battery_soc = 1.0;
  double threshold_soc = 0.20;
  static RobotState make(
    std::size_t wp_, std::size_t c_wp_)
  {
    return RobotState{wp_, c_wp_};
  }
};

// ============================================================================
struct Invariant
{
  std::size_t task_id;
  double invariant_cost;
};

struct InvariantLess
{
  bool operator()(const Invariant& a, const Invariant& b) const
  {
    return a.invariant_cost < b.invariant_cost;
  }
};

// ============================================================================
class TaskRequest
{
public:

  struct Estimate
  {
    RobotState finish_state;
    double wait_until;
  };

  // Get the id of the task
  virtual std::size_t id() const = 0;

  // Estimate the state of the robot when the task is finished along with the
  // time the robot has to wait before commencing the task
  virtual rmf_utils::optional<Estimate> estimate_finish(
    const RobotState& initial_state) const = 0;

  // Estimate the invariant component of the task's duration
  virtual double invariant_duration() const = 0;

  // Get the earliest start time that this task may begin
  virtual double earliest_start_time() const = 0;

};

using ConstTaskRequestPtr = std::shared_ptr<TaskRequest>;

// ============================================================================
class ChargeBatteryTaskRequest : public TaskRequest
{
public:
  ChargeBatteryTaskRequest(
    rmf_battery::agv::BatterySystem battery_system,
    std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink,
    std::shared_ptr<rmf_battery::DevicePowerSink> device_sink,
    std::shared_ptr<rmf_traffic::agv::Planner> planner,
    bool drain_battery)
  : _battery_system(battery_system),
    _motion_sink(motion_sink),
    _device_sink(device_sink),
    _planner(planner),
    _drain_battery(drain_battery)
  {
    _invariant_duration = 0.0;
  }

  static ConstTaskRequestPtr make(
    rmf_battery::agv::BatterySystem battery_system,
    std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink,
    std::shared_ptr<rmf_battery::DevicePowerSink> device_sink,
    std::shared_ptr<rmf_traffic::agv::Planner> planner,
    bool drain_battery = true)
  {
    return std::make_shared<ChargeBatteryTaskRequest>(
      battery_system, motion_sink, device_sink, planner, drain_battery);
  }

  std::size_t id() const final
  {
    return _id;
  }

  rmf_utils::optional<Estimate> estimate_finish(
    const RobotState& initial_state) const final
  {

    // TODO: should we consider adding a cost to the finish_time instead of 
    // returning nullopt here?
    if (abs(initial_state.battery_soc - _charge_soc) < 1e-3)
    {
      std::cout << " -- Charge battery: Battery full" << std::endl;
      return rmf_utils::nullopt;
    }

    auto state = initial_state;

    // Compute time taken to reach charging waypoint from current location
    state.waypoint = initial_state.charging_waypoint;
    const auto start_time = std::chrono::steady_clock::now() +
      rmf_traffic::time::from_seconds(initial_state.finish_time);

    double battery_soc = initial_state.battery_soc;
    double variant_duration = 0.0;

    if (initial_state.waypoint != initial_state.charging_waypoint)
    {
      // Compute plan to charging waypoint along with battery drain
      rmf_traffic::agv::Planner::Start start{
        start_time,
        initial_state.waypoint,
        0.0};

      rmf_traffic::agv::Planner::Goal goal{initial_state.charging_waypoint};

      const auto result = _planner->plan(start, goal);
      const auto& trajectory = result->get_itinerary().back().trajectory();
      const auto& finish_time = *trajectory.finish_time();
      const double variant_duration = rmf_traffic::time::to_seconds(
        finish_time - start_time);

      if (_drain_battery)
      {
        const double dSOC_motion = _motion_sink->compute_change_in_charge(
          trajectory);
        const double dSOC_device = _device_sink->compute_change_in_charge(
          variant_duration);
        battery_soc = battery_soc - dSOC_motion - dSOC_device;
      }

      if (battery_soc <= state.threshold_soc)
      {
        // If a robot cannot reach its charging dock given its initial battery soc
        std::cout << " -- Charge battery: Unable to reach charger" << std::endl;
        return rmf_utils::nullopt;
      }
    }

    // Default _charge_soc = 1.0
    double delta_soc = _charge_soc - battery_soc;
    assert(delta_soc >= 0.0);
    double time_to_charge =
      (3600 * delta_soc * _battery_system.nominal_capacity()) /
      _battery_system.charging_current();

    const double wait_until = initial_state.finish_time;

    state.finish_time = 
      wait_until +
      variant_duration +
      time_to_charge;

    state.battery_soc = _charge_soc;

    rmf_utils::optional<Estimate> estimate =
      Estimate{state, wait_until};

    return estimate;

  }

  double invariant_duration() const final
  {
    return _invariant_duration;
  }

  double earliest_start_time() const final
  {
    return 0.0;
  }

private:
  // fixed id for now
  std::size_t _id = 101;
  rmf_battery::agv::BatterySystem _battery_system;
  std::shared_ptr<rmf_battery::MotionPowerSink> _motion_sink;
  std::shared_ptr<rmf_battery::DevicePowerSink> _device_sink;
  std::shared_ptr<rmf_traffic::agv::Planner> _planner;
  bool _drain_battery;
  // soc to always charge the battery up to
  double _charge_soc = 1.0;
  double _invariant_duration; 
};

// ============================================================================
class DeliveryTaskRequest : public TaskRequest
{
public:
  using Planner = rmf_traffic::agv::Planner;
  using MotionPowerSink = rmf_battery::MotionPowerSink;
  using DevicePowerSink = rmf_battery::DevicePowerSink;
  DeliveryTaskRequest(
    std::size_t id,
    std::size_t pickup,
    std::size_t dropoff,
    std::shared_ptr<MotionPowerSink> motion_sink,
    std::shared_ptr<DevicePowerSink> device_sink,
    std::shared_ptr<Planner> planner,
    bool drain_battery,
    double start_time = 0.0)
  : _id(id),
    _pickup_waypoint(pickup),
    _dropoff_waypoint(dropoff),
    _motion_sink(motion_sink),
    _device_sink(device_sink),
    _planner(planner),
    _drain_battery(drain_battery),
    _start_time(start_time)
  {
    // Calculate duration of invariant component of task
    const auto plan_start_time = std::chrono::steady_clock::now();
    Planner::Start start{
      plan_start_time,
      _pickup_waypoint,
      0.0};

    Planner::Goal goal{_dropoff_waypoint};
    const auto result_to_dropoff = _planner->plan(start, goal);

    const auto trajectory = result_to_dropoff->get_itinerary().back().trajectory();
    const auto& finish_time = *trajectory.finish_time();
    _invariant_duration = rmf_traffic::time::to_seconds(
      finish_time - plan_start_time);

    _invariant_battery_drain = 0.0;

    if (_drain_battery)
    {
      // Compute battery drain
      const double dSOC_motion = _motion_sink->compute_change_in_charge(trajectory);
      const double dSOC_device = _device_sink->compute_change_in_charge(_invariant_duration);
      _invariant_battery_drain = dSOC_motion + dSOC_device;
    }
    
  }

  static ConstTaskRequestPtr make(
    std::size_t id,
    std::size_t pickup,
    std::size_t dropoff,
    std::shared_ptr<MotionPowerSink> motion_sink,
    std::shared_ptr<DevicePowerSink> device_sink,
    std::shared_ptr<Planner> planner,
    bool drain_battery = true,
    double start_time = 0.0)
  {
    return std::make_shared<DeliveryTaskRequest>(
      id, pickup, dropoff, motion_sink, device_sink, planner, drain_battery, start_time);
  }

  std::size_t id() const final
  {
    return _id;
  }

  rmf_utils::optional<Estimate> estimate_finish(
    const RobotState& initial_state) const final
  {
    auto state = initial_state;
    state.waypoint = _dropoff_waypoint;

    double variant_duration = 0.0;

    const auto now = std::chrono::steady_clock::now();
    auto start_time = now +
      rmf_traffic::time::from_seconds(initial_state.finish_time);

    double battery_soc = initial_state.battery_soc;
    double dSOC_motion = 0.0;
    double dSOC_device = 0.0;

    if (initial_state.waypoint != _pickup_waypoint)
    {
      // Compute plan to pickup waypoint along with battery drain
      Planner::Start start{
      start_time,
      initial_state.waypoint,
      0.0};

      Planner::Goal goal{_pickup_waypoint};

      const auto result_to_pickup = _planner->plan(start, goal);
      // We assume we can always compute a plan
      const auto& trajectory = result_to_pickup->get_itinerary().back().trajectory();
      const auto& finish_time = *trajectory.finish_time();
      variant_duration = rmf_traffic::time::to_seconds(
        finish_time - start_time);

      if(_drain_battery)
      {
        // Compute battery drain
        dSOC_motion = _motion_sink->compute_change_in_charge(trajectory);
        dSOC_device = _device_sink->compute_change_in_charge(variant_duration);
        battery_soc = battery_soc - dSOC_motion - dSOC_device;
      }

      if (battery_soc <= state.threshold_soc)
      {
        std::cout << " -- Delivery: Unable to reach pickup" << std::endl;
        return rmf_utils::nullopt;
      }

      start_time = finish_time;
    }

    const double ideal_start = _start_time - variant_duration;
    const double wait_until = std::max(initial_state.finish_time, ideal_start);

    // Factor in invariants
    state.finish_time =
      wait_until +
      variant_duration +
      _invariant_duration;

    battery_soc -= _invariant_battery_drain;

    if (battery_soc <= state.threshold_soc)
    {
      std::cout << " -- Delivery: Unable to reach dropoff" << std::endl;
      return rmf_utils::nullopt;
    }
    
    state.battery_soc = battery_soc;

    // TODO: Check if we have enough charge to head back to nearest charger

    rmf_utils::optional<TaskRequest::Estimate> estimate =
      Estimate{state, wait_until};

    return estimate;
  }

  double invariant_duration() const final
  {
    return _invariant_duration;
  }

  double earliest_start_time() const final
  {
    return _start_time;
  }

private:
  std::size_t _id; // task id
  std::size_t _pickup_waypoint;
  std::size_t _dropoff_waypoint;
  std::shared_ptr<MotionPowerSink> _motion_sink;
  std::shared_ptr<DevicePowerSink> _device_sink;
  std::shared_ptr<Planner> _planner;
  bool _drain_battery;
  double _invariant_duration;
  double _invariant_battery_drain;
  double _start_time;
};

// ============================================================================
class Candidates
{
public:

  struct Entry
  {
    std::size_t candidate;
    RobotState state;
    double wait_until;
  };

  // Map finish time to Entry
  using Map = std::multimap<double, Entry>;

  static Candidates make(
      const std::vector<RobotState>& initial_states,
      const TaskRequest& request);

  Candidates(const Candidates& other)
  {
    _value_map = other._value_map;
    _update_map();
  }

  Candidates& operator=(const Candidates& other)
  {
    _value_map = other._value_map;
    _update_map();
    return *this;
  }

  Candidates(Candidates&&) = default;
  Candidates& operator=(Candidates&&) = default;

  // We may have more than one best candidate so we store their iterators in
  // a Range
  struct Range
  {
    Map::const_iterator begin;
    Map::const_iterator end;
  };

  Range best_candidates() const
  {
    assert(!_value_map.empty());

    Range range;
    range.begin = _value_map.begin();
    auto it = range.begin;
    while (it->first == range.begin->first)
      ++it;

    range.end = it;
    return range;
  }

  double best_finish_time() const
  {
    assert(!_value_map.empty());
    return _value_map.begin()->first;
  }

  void update_candidate(
    std::size_t candidate, RobotState state, double wait_until)
  {
    const auto it = _candidate_map.at(candidate);
    _value_map.erase(it);
    _candidate_map[candidate] = _value_map.insert(
      {state.finish_time, Entry{candidate, state, wait_until}});
  }


private:
  Map _value_map;
  std::vector<Map::iterator> _candidate_map;

  Candidates(Map candidate_values)
    : _value_map(std::move(candidate_values))
  {
    _update_map();
  }

  void _update_map()
  {
    for (auto it = _value_map.begin(); it != _value_map.end(); ++it)
    {
      const auto c = it->second.candidate;
      if (_candidate_map.size() <= c)
        _candidate_map.resize(c+1);

      _candidate_map[c] = it;
    }
  }
};

Candidates Candidates::make(
    const std::vector<RobotState>& initial_states,
    const TaskRequest& request)
{
  Map initial_map;
  for (std::size_t i = 0; i < initial_states.size(); ++i)
  {
    const auto& state = initial_states[i];
    const auto finish = request.estimate_finish(state);
    if (finish.has_value())
    {
      initial_map.insert(
        {finish.value().finish_state.finish_time,
        Entry{i, finish.value().finish_state, finish.value().wait_until}});
    }
  }

  return Candidates(std::move(initial_map));
}

// ============================================================================
struct PendingTask
{
  PendingTask(
      std::vector<RobotState> initial_states,
      ConstTaskRequestPtr request_)
    : request(std::move(request_)),
      candidates(Candidates::make(initial_states, *request)),
      earliest_start_time(request->earliest_start_time())
  {
    // Do nothing
  }

  ConstTaskRequestPtr request;
  Candidates candidates;
  double earliest_start_time;
};

// ============================================================================
struct Assignment
{
  std::size_t task_id;
  RobotState state;
  double earliest_start_time;
};

// ============================================================================
struct Node
{
  using AssignedTasks =
  std::vector<std::vector<Assignment>>;
  using UnassignedTasks =
    std::unordered_map<std::size_t, PendingTask>;
  using InvariantSet = std::multiset<Invariant, InvariantLess>;

  AssignedTasks assigned_tasks;
  UnassignedTasks unassigned_tasks;
  double cost_estimate;
  double latest_time;
  InvariantSet unassigned_invariants;

  void sort_invariants()
  {
    unassigned_invariants.clear();
    for (const auto& u : unassigned_tasks)
    {
      unassigned_invariants.insert(
            Invariant{u.first, u.second.request->invariant_duration()});
    }
  }

  void pop_unassigned(std::size_t task_id)
  {
    unassigned_tasks.erase(task_id);

    bool popped_invariant = false;
    InvariantSet::iterator erase_it;
    for (auto it = unassigned_invariants.begin();
         it != unassigned_invariants.end(); ++it)
    {
      if (it->task_id == task_id)
      {
        popped_invariant = true;
        erase_it = it;
        break;
      }
    }
    unassigned_invariants.erase(erase_it);

    assert(popped_invariant);
  }
};

using NodePtr = std::shared_ptr<Node>;
using ConstNodePtr = std::shared_ptr<const Node>;

struct LowestCostEstimate
{
  bool operator()(const ConstNodePtr& a, const ConstNodePtr& b)
  {
    return b->cost_estimate < a->cost_estimate;
  }
};

// ============================================================================
class InvariantHeuristicQueue
{
public:

  InvariantHeuristicQueue(std::vector<double> initial_values)
  {
    assert(!initial_values.empty());
    std::sort(initial_values.begin(), initial_values.end());

    for (const auto value : initial_values)
      _stacks.push_back({value});
  }

  void add(double new_value)
  {
    // Add the new value to the smallest stack
    const double value = _stacks[0].back() + new_value;
    _stacks[0].push_back(value);

    // Find the largest stack that is still smaller than the current front
    const auto next_it = _stacks.begin() + 1;
    auto end_it = next_it;
    for (; end_it != _stacks.end(); ++end_it)
    {
      if (value <= end_it->back())
        break;
    }

    if (next_it != end_it)
    {
      // Rotate the vector elements to move the front stack to its new place
      // in the order
      std::rotate(_stacks.begin(), next_it, end_it);
    }
  }

  double compute_cost() const
  {
    double total_cost = 0.0;
    for (const auto& stack : _stacks)
    {
      // NOTE: We start iterating from i=1 because i=0 represents a component of
      // the cost that is already accounted for by g(n) and the variant
      // component of h(n)
      for (std::size_t i=1; i < stack.size(); ++i)
        total_cost += stack[i];
    }

    return total_cost;
  }

private:
  std::vector<std::vector<double>> _stacks;
};

// ============================================================================
class Filter
{
public:

  enum class Type
  {
    Passthrough,
    Trie,
    Hash
  };

  Filter(Type type, const std::size_t N_tasks)
    : _type(type),
      _set(N_tasks, AssignmentHash(N_tasks))
  {
    // Do nothing
  }

  bool ignore(const Node& node);

private:

  struct TaskTable;

  struct AgentTable
  {
    std::unordered_map<std::size_t, std::unique_ptr<TaskTable>> agent;
  };

  struct TaskTable
  {
    std::unordered_map<std::size_t, std::unique_ptr<AgentTable>> task;
  };
  
  struct AssignmentHash
  {
    AssignmentHash(std::size_t N)
    {
      // We add 1 to N because
      _shift = std::ceil(std::log2(N+1));
    }

    std::size_t operator()(const Node::AssignedTasks& assignments) const
    {
      std::size_t output = 0;
      std::size_t count = 0;
      for (const auto& a : assignments)
      {
        for (const auto& s : a)
        {
          // We add 1 to the task_id to differentiate between task_id == 0 and
          // a task being unassigned.
          const std::size_t id = s.task_id + 1;
          output += id << (_shift * (count++));
        }
      }

      return output;
    }

    std::size_t _shift;
  };

  struct AssignmentEqual
  {
    bool operator()(
      const Node::AssignedTasks& A, const Node::AssignedTasks& B) const
    {
      if (A.size() != B.size())
        return false;

      for (std::size_t i=0; i < A.size(); ++i)
      {
        const auto& a = A[i];
        const auto& b = B[i];

        if (a.size() != b.size())
          return false;

        for (std::size_t j=0; j < a.size(); ++j)
        {
          if (a[j].task_id != b[j].task_id)
            return false;
        }
      }

      return true;
    }
  };

  using Set = std::unordered_set<Node::AssignedTasks, AssignmentHash, AssignmentEqual>;

  Type _type;
  AgentTable _root;
  Set _set;
};

bool Filter::ignore(const Node& node)
{
  if (_type == Type::Passthrough)
    return false;

  if (_type == Type::Hash)
    return !_set.insert(node.assigned_tasks).second;

  bool new_node = false;

  // TODO(MXG): Consider replacing this tree structure with a hash set

  AgentTable* agent_table = &_root;
  std::size_t a = 0;
  std::size_t t = 0;
  while(a < node.assigned_tasks.size())
  {
    const auto& current_agent = node.assigned_tasks.at(a);

    if (t < current_agent.size())
    {
      const auto& task_id = current_agent[t].task_id;
      const auto agent_insertion = agent_table->agent.insert({a, nullptr});
      if (agent_insertion.second)
        agent_insertion.first->second = std::make_unique<TaskTable>();

      auto* task_table = agent_insertion.first->second.get();

      const auto task_insertion = task_table->task.insert({task_id, nullptr});
      if (task_insertion.second)
      {
        new_node = true;
        task_insertion.first->second = std::make_unique<AgentTable>();
      }

      agent_table = task_insertion.first->second.get();
      ++t;
    }
    else
    {
      t = 0;
      ++a;
    }
  }

  return !new_node;
}

// ============================================================================
class TaskPlanner
{
public:
  using PriorityQueue = std::priority_queue<
      ConstNodePtr,
      std::vector<ConstNodePtr>,
      LowestCostEstimate>;
  using BatterySystem = rmf_battery::agv::BatterySystem;

  TaskPlanner(
    ConstTaskRequestPtr charge_battery,
    const Filter::Type filter_type,
    const bool debug)
  : _charge_battery(charge_battery),
    _filter_type(filter_type),
    _debug(debug)
  {

  }

  double compute_g(const Node::AssignedTasks& assigned_tasks)
  {
    double cost = 0.0;
    for (const auto& agent : assigned_tasks)
    {
      for (const auto& assignment : agent)
      {
        cost += assignment.state.finish_time - assignment.earliest_start_time;
      }
    }

    return cost;
  }

  ConstNodePtr solve(
    ConstNodePtr initial_node,
    const std::vector<RobotState> initial_states,
    const std::size_t num_tasks)
  {
    const auto start = std::chrono::steady_clock::now();
    _priority_queue = PriorityQueue{};
    _priority_queue.push(std::move(initial_node));

    Filter filter{_filter_type, num_tasks};
    _total_queue_entries = 1;
    _total_queue_expansions = 0;
    ConstNodePtr top = nullptr;

    const auto print_conclusion = [&]()
    {
      const auto finish = std::chrono::steady_clock::now();
      const auto elapsed = std::chrono::duration_cast<
          std::chrono::duration<double>>(finish - start);

      if (_debug)
      {
        std::cout << " ====================== \n";
        std::cout << "Time elapsed: " << elapsed.count() << std::endl;
        std::cout << "Winning cost: " << top->cost_estimate << std::endl;
        std::cout << "Final queue size: " << _priority_queue.size() << std::endl;
        std::cout << "Total queue expansions: " << _total_queue_expansions << std::endl;
        std::cout << "Total queue entries: " << _total_queue_entries << std::endl;
      }
    };

    while (!_priority_queue.empty())
    {
      // Print the nodes if debug is true
      // if (_debug)
      // {
      //   auto copy = _priority_queue;
      //   while (!copy.empty())
      //   {
      //     const auto top = copy.top();
      //     copy.pop();
      //     print_node(*top);
      //   }
      // }

      top = _priority_queue.top();

      // Pop the top of the priority queue
      _priority_queue.pop();

      // Check if unassigned tasks is empty -> solution found
      if (finished(*top))
      {
        _goal_node = *top;
        print_conclusion();
        std::cout << "Battery SOC:" << std::endl;
        for (std::size_t i = 0; i < top->assigned_tasks.size(); ++i)
        {
          std::cout << "  Agent: " << i << std::endl;
          for (const auto& assignment : top->assigned_tasks[i])
          {
            std::cout << "    " << assignment.task_id 
                      << " : " << 100 * assignment.state.battery_soc 
                      << "% at time " << assignment.state.finish_time << "s" 
                      << std::endl;
          }
        }
        return top;
      }

      // Apply possible actions to expand the node
      const auto new_nodes = expand(top, filter, initial_states);
      ++_total_queue_expansions;
      _total_queue_entries += new_nodes.size();

      // Add copies and with a newly assigned task to queue
      for (const auto&n : new_nodes)
        _priority_queue.push(n);
      
    }

    return nullptr;
  }

  ConstNodePtr greedy_solve(
    ConstNodePtr node,
    const std::vector<RobotState> initial_states)
  {
    while (!finished(*node))
    {
      ConstNodePtr next_node = nullptr;
      for (const auto& u : node->unassigned_tasks)
      {
        const auto& range = u.second.candidates.best_candidates();
        for (auto it = range.begin; it != range.end; ++it)
        {
          if (auto n = expand_candidate(it, u, node, nullptr))
          {
            if (!next_node || (n->cost_estimate < next_node->cost_estimate))
              {
                next_node = std::move(n);
              }
          }
          else
          {
            // expand_candidate returned nullptr either due to start time
            // segmentation or insufficient charge to complete task. For the 
            // later case, we assign a charging task to the agent
            if (node->latest_time + segmentation_threshold > it->second.wait_until)
            {
              const auto charge_node = expand_charger(
                node, it->second.candidate, initial_states);
              if (charge_node)
              {
                next_node = std::move(charge_node);
              }
              else
              {
                // agent has either full battery or insufficient charge to reach 
                // its charger. If later, we pop assigned task until
                // we can make it to the charger
                auto parent_node = std::make_shared<Node>(*node);
                RobotState state;
                if (parent_node->assigned_tasks[it->second.candidate].empty())
                {
                  state = initial_states[it->second.candidate];
                }
                else
                {
                  state = parent_node->assigned_tasks[it->second.candidate].back().state;
                }
                
                if (state.battery_soc < 0.99)
                {
                  while (!parent_node->assigned_tasks[it->second.candidate].empty())
                  {
                    auto& assignments = parent_node->assigned_tasks[it->second.candidate];
                    assignments.pop_back();
                    auto new_charge_node = expand_charger(
                      parent_node, it->second.candidate, initial_states);
                    if (new_charge_node)
                    {
                      next_node = std::move(new_charge_node);
                      break;
                    }
                  }
                }
              } 
            }
          }
        }
      }

      node = next_node;
      if (!node)
        std::cout << "Node is nullptr. Expect segfault" <<std::endl;
    }

    return node;
  }

  Node::AssignedTasks complete_solve(
    std::vector<RobotState> initial_states,
    std::vector<ConstTaskRequestPtr> tasks,
    bool greedy)
  {

    auto node = make_initial_node(initial_states, tasks);

    Node::AssignedTasks complete_assignments;
    complete_assignments.resize(node->assigned_tasks.size());

    while (node)
    {
      if (greedy)
        node = greedy_solve(node, initial_states);
      else
        node = solve(node, initial_states, tasks.size());

      if (!node)
      {
        std::cout << "No solution found!" << std::endl;
        return {};
      }

      if(_debug)
        print_node(*node);

      assert(complete_assignments.size() == node->assigned_tasks.size());
      for (std::size_t i = 0; i < complete_assignments.size(); ++i)
      {
        auto& all_assignments = complete_assignments[i];
        const auto& new_assignments = node->assigned_tasks[i];
        for (const auto& a : new_assignments)
        {
          all_assignments.push_back(a);
          // all_assignments.back().task_id = task_id_map.at(a.task_id);
        }
      }

      if (node->unassigned_tasks.empty())
        return complete_assignments;

      // std::unordered_map<std::size_t, std::size_t> new_task_id_map;
      std::vector<ConstTaskRequestPtr> new_tasks;
      // std::size_t task_counter = 0;
      for (const auto& u : node->unassigned_tasks)
      {
        new_tasks.push_back(u.second.request);
        // new_task_id_map[task_counter++] = task_id_map[u.first];
      }

      // task_id_map = std::move(new_task_id_map);

      // copy final state estimates 
      std::vector<RobotState> estimates;
      estimates.resize(node->assigned_tasks.size());
      for (std::size_t i = 0; i < node->assigned_tasks.size(); ++i)
      {
        const auto& assignments = node->assigned_tasks[i];
        if (assignments.empty())
          estimates[i] = initial_states[i];
        else
          estimates[i] = assignments.back().state;        
      }

      if (_debug)
      {
        std::cout << "Updating node with states and requests: " << std::endl;
        std::cout << "  *Initial States: wp, soc, finish_time" << std::endl;
        for (std::size_t i = 0; i < estimates.size(); ++i)
        {
          const auto& state = estimates[i];
          std::cout << "    " << i << " : " << state.waypoint << " , " 
                    <<  state.battery_soc * 100 <<  "% , "
                    <<  state.finish_time << "s" << std::endl;
        }
        std::cout << "  *Tasks: ";
        for (std::size_t i = 0; i < new_tasks.size(); ++i)
        {
          std::cout << " " << new_tasks[i]->id() << " ";
        }
        std::cout << std::endl;
      }

      node = make_initial_node(estimates, new_tasks);
      initial_states = estimates;
    }

    return complete_assignments;

  }

private:
  std::size_t _total_queue_entries = 0;
  std::size_t _total_queue_expansions = 0;

  ConstTaskRequestPtr _charge_battery;
  Filter::Type _filter_type;
  bool _debug;
  Node _goal_node;
  PriorityQueue _priority_queue;

  ConstNodePtr make_initial_node(
    std::vector<RobotState> initial_states,
    std::vector<ConstTaskRequestPtr> tasks)
  {
    auto initial_node = std::make_shared<Node>();

    initial_node->assigned_tasks.resize(initial_states.size());

    for (const auto& task : tasks)
      initial_node->unassigned_tasks.insert(
        {task->id(), PendingTask(initial_states, task)});

    initial_node->cost_estimate = compute_f(*initial_node);

    initial_node->sort_invariants();

    initial_node->latest_time = [&]() -> double
    {
      double latest = -std::numeric_limits<double>::infinity();
      for (const auto& s : initial_states)
      {
        if (latest < s.finish_time)
          latest = s.finish_time;
      }

      assert(!std::isinf(latest));
      return latest;
    }();

    double wait_until = std::numeric_limits<double>::infinity();
    for (const auto& u : initial_node->unassigned_tasks)
    {
      const auto& range = u.second.candidates.best_candidates();
      for (auto it = range.begin; it != range.end; ++it)
      {
        if (it->second.wait_until < wait_until)
          wait_until = it->second.wait_until;
      }
    }
    assert(!std::isinf(wait_until));

    if (initial_node->latest_time < wait_until)
      initial_node->latest_time = wait_until;

    return initial_node;
  }


  double get_latest_time(const Node& node)
  {
    double latest = -std::numeric_limits<double>::infinity();
    for (const auto& a : node.assigned_tasks)
    {
      if (a.empty())
        continue;

      const double f = a.back().state.finish_time;
      if (latest < f)
        latest = f;
    }

    assert(!std::isinf(latest));

    return latest;
  }

  double compute_g(const Node& node)
  {
    return compute_g(node.assigned_tasks);
  }

  double compute_h(const Node& node)
  {
    std::vector<double> initial_queue_values;
    initial_queue_values.resize(
          node.assigned_tasks.size(), std::numeric_limits<double>::infinity());

    for (const auto& u : node.unassigned_tasks)
    {
      // We subtract the invariant duration here because otherwise its
      // contribution to the cost estimate will be duplicated in the next section,
      // which could result in an overestimate.
      const double variant_value =
          u.second.candidates.best_finish_time()
          - u.second.request->invariant_duration();

      const auto& range = u.second.candidates.best_candidates();
      for (auto it = range.begin; it != range.end; ++it)
      {
        const std::size_t candidate = it->second.candidate;
        if (variant_value < initial_queue_values[candidate])
          initial_queue_values[candidate] = variant_value;
      }
    }

    for (std::size_t i=0; i < initial_queue_values.size(); ++i)
    {
      auto& value = initial_queue_values[i];
      if (std::isinf(value))
      {
        // Clear out any infinity placeholders. Those candidates simply don't have
        // any unassigned tasks that want to use it.
        const auto& assignments = node.assigned_tasks[i];
        if (assignments.empty())
          value = 0.0;
        else
          value = assignments.back().state.finish_time;
      }
    }

    InvariantHeuristicQueue queue(std::move(initial_queue_values));
    // NOTE: It is crucial that we use the ordered set of unassigned_invariants
    // here. The InvariantHeuristicQueue expects the invariant costs to be passed
    // to it in order of smallest to largest. If that assumption is not met, then
    // the final cost that's calculated may be invalid.
    for (const auto& u : node.unassigned_invariants)
      queue.add(u.invariant_cost);

    return queue.compute_cost();
  }

  double compute_f(const Node& n)
  {
    return compute_g(n) + compute_h(n);
  }

  ConstNodePtr expand_candidate(
    const Candidates::Map::const_iterator& it,
    const Node::UnassignedTasks::value_type& u,
    const ConstNodePtr& parent,
    Filter* filter)

  {
    const auto& entry = it->second;

    if (parent->latest_time + segmentation_threshold < entry.wait_until)
    {

      // No need to assign task as timeline is not relevant
      return nullptr;
    }

    auto new_node = std::make_shared<Node>(*parent);

    // Assign the unassigned task
    bool add_charger = false;
    new_node->assigned_tasks[entry.candidate].push_back(
      Assignment{u.first, entry.state, u.second.earliest_start_time});
    
    // Erase the assigned task from unassigned tasks
    new_node->pop_unassigned(u.first);

    // Update states of unassigned tasks for the candidate
    for (auto& new_u : new_node->unassigned_tasks)
    {
      const auto finish =
        new_u.second.request->estimate_finish(entry.state);
      if (finish.has_value())
      {
        new_u.second.candidates.update_candidate(
          entry.candidate,
          finish.value().finish_state,
          finish.value().wait_until);
      }
      else
      {
        add_charger = true;
        break;
        // return nullptr;

      }
    }

    if (add_charger)
    {
      auto battery_estimate = _charge_battery->estimate_finish(entry.state);
      if (battery_estimate.has_value())
      {
        new_node->assigned_tasks[entry.candidate].push_back(
          Assignment
          {
            _charge_battery->id(),
            battery_estimate.value().finish_state,
            battery_estimate.value().wait_until
          });
        for (auto& new_u : new_node->unassigned_tasks)
        {
          const auto finish =
            new_u.second.request->estimate_finish(battery_estimate.value().finish_state);
          if (finish.has_value())
          {
            new_u.second.candidates.update_candidate(
              entry.candidate, finish.value().finish_state, finish.value().wait_until);
          }
          else
          {
            // we should stop expanding this node
            return nullptr;
          }
        }
        
      }
      else
      {
        // agent cannot make it back to the charger
        return nullptr;
      }
      
    }

    // Update the cost estimate for new_node
    new_node->cost_estimate = compute_f(*new_node);
    new_node->latest_time = get_latest_time(*new_node);

    // Apply filter
    if (filter && filter->ignore(*new_node))
    {
      if (_debug)
      {
        std::cout << "Ignoring node: " << std::endl;
        print_node(*new_node);
        std::cout << "==============================================" << std::endl;
      }
      return nullptr;
    }

    return new_node;

  }

  ConstNodePtr expand_charger(
    ConstNodePtr parent,
    const std::size_t agent,
    const std::vector<RobotState> initial_states)
  {
    auto new_node = std::make_shared<Node>(*parent);
    // Assign charging task to an agent
    const auto& assignments = new_node->assigned_tasks[agent];
    RobotState state;
    if (!assignments.empty())
    {
      state = assignments.back().state;
    }
    else
    {
      // We use the initial state of the robot
      state = initial_states[agent];
    }

    auto estimate = _charge_battery->estimate_finish(state);
    if (estimate.has_value())
    {
      new_node->assigned_tasks[agent].push_back(
        Assignment{
          _charge_battery->id(),
          estimate.value().finish_state,
          estimate.value().wait_until});

      for (auto& new_u : new_node->unassigned_tasks)
      {
        const auto finish =
          new_u.second.request->estimate_finish(estimate.value().finish_state);
        if (finish.has_value())
        {
          new_u.second.candidates.update_candidate(
            agent, finish.value().finish_state, finish.value().wait_until);
        }
        else
        {
          return nullptr;
        }
      }

      new_node->cost_estimate = compute_f(*new_node);
      new_node->latest_time = get_latest_time(*new_node);
      return new_node;
    }

    return nullptr;
  }

  std::vector<ConstNodePtr> expand(
    ConstNodePtr parent,
    Filter& filter,
    const std::vector<RobotState> initial_states)
  {
    std::vector<ConstNodePtr> new_nodes;
    new_nodes.reserve(
      parent->unassigned_tasks.size() + parent->assigned_tasks.size());
    for (const auto& u : parent->unassigned_tasks)
    {
      const auto& range = u.second.candidates.best_candidates();
      for (auto it = range.begin; it!= range.end; it++)
      {
        if (auto new_node = expand_candidate(it, u, parent, &filter))
          new_nodes.push_back(std::move(new_node));
      }
    }

    // Assign charging task to each robot
    for (std::size_t i = 0; i < parent->assigned_tasks.size(); ++i)
    {
      if (auto n = expand_charger(parent, i, initial_states))
        new_nodes.push_back(n);
    }

    return new_nodes;
  }

  bool finished(const Node& node)
  {
    for (const auto& u : node.unassigned_tasks)
    {
      const auto range = u.second.candidates.best_candidates();
      for (auto it = range.begin; it!= range.end; ++it)
      {
        const double wait_time = it->second.wait_until;
        if (wait_time <= node.latest_time + segmentation_threshold)
          return false;
      }
    }

    return true;
  }

  void print_node(const Node& node)
  {
    std::cout << " -- " << node.cost_estimate << ": <";
    for (std::size_t a=0; a < node.assigned_tasks.size(); ++a)
    {
      if (a > 0)
        std::cout << ", ";

      std::cout << a << ": [";
      for (const auto i : node.assigned_tasks[a])
        std::cout << " " << i.task_id;
      std::cout << " ]";
    }

    std::cout << " -- ";
    bool first = true;
    for (const auto& u : node.unassigned_tasks)
    {
      if (first)
        first = false;
      else
        std::cout << ", ";

      std::cout << u.first << ":";
      const auto& range = u.second.candidates.best_candidates();
      for (auto it = range.begin; it != range.end; ++it)
        std::cout << " " << it->second.candidate;
    }

    std::cout << ">" << std::endl;
    std::cout << "----------------------------------------------" << std::endl;
  }

  void print_solution_node(
      const Node& node,
      const std::unordered_map<std::size_t, std::size_t>& task_id_map)
  {
    std::cout << "\nAssignments:\n";
    for (std::size_t i=0; i < node.assigned_tasks.size(); ++i)
    {
      std::cout << i;
      if (node.assigned_tasks[i].empty())
        std::cout << " (null):";
      else
        std::cout << " (" << node.assigned_tasks[i].back().state.finish_time << "):";

      for (const auto& t : node.assigned_tasks[i])
        std::cout << "  " << task_id_map.at(t.task_id);
      std::cout << "\n";
    }
    std::cout << std::endl;

    std::cout << "Unassigned:\n";
    for (const auto& u : node.unassigned_tasks)
    {
      std::cout << "Task " << task_id_map.at(u.first) << " candidates:";
      const auto& range = u.second.candidates.best_candidates();
      for (auto it = range.begin; it != range.end; ++it)
      {
        std::cout << " (" << it->second.candidate << ": "
                  << it->second.wait_until << ")";
      }
      std::cout << "\n";
    }
    std::cout << std::endl;
  }


};

} // anonymous namespace

#endif // RMF_TASKS__TASK_ALLOCATION_HPP
