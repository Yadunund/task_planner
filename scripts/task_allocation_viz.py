
import math
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as mpath
import matplotlib.patches as mpatches
import matplotlib.collections as mcoll

import random
import yaml

################################################################################

class RequestTask:
  class Profile:
    def __init__(self, task_id, waypoints):
      self.task_id = task_id
      self.waypoints = waypoints
      self.speed = 1.0
      self.priority = False
      self.time_window = (0, 0)

  class Plan:
    def __init__(self):
      self.travel_distance = 0.0  # location_T-1 to locatoin_T
      self.duration = 0.0         # travel_distance/speed
      self.finish_time = 0.0      # n_sum(durations)

  def __init__(self, task_id, waypoints):
    self.profile = self.Profile(task_id, waypoints)
    self.plan = self.Plan()

class Agent:
  def __init__(self, agent_id, waypoint):
    self.agent_id = agent_id
    self.waypoint = waypoint
    self.speed = 1.0
    self.assignments = []

  def assign_task(self, task):
    assert isinstance(task, RequestTask)
    self.assignments.append(task)

################################################################################


def plot_task_allocation(agents, graph):
  """Plot task allocated agents """
  print("\n--------------- plotting Graph for validation --------------")
  
  _, (ax1, ax2) = plt.subplots(1, 2)
  ax1.set_title("Agent Task Routes")
  ax2.set_title("Agent Task Durations")
  
  sum_cost = 0.0
  sum_accum = 0.0
  all_task_costs = {}

  for agt_id, agt in agents.items():
    start_wp = graph[agt.waypoint]
    ax1.annotate("Agent" + str(agt_id), start_wp) # todo
    verts=[start_wp]
    for tsk in agt.assignments:
          for wp in tsk.profile.waypoints:
            verts.append(graph[wp])
    colorline(ax1, verts)

    # TODO Clean all these below
    # dumb way to validate and cal accum cost again
    p_x = verts[0]
    cost = 0
    accum_cost = 0
    all_task_costs[agt_id] = []
    for i, x in enumerate(verts):
      if (i == 0):  # ignore agent's current position
        continue
      d_cost = math.hypot(x[0] - p_x[0], x[1] - p_x[1])
      all_task_costs[agt_id].append(d_cost)
      accum_cost += cost + d_cost
      cost += d_cost
      p_x = x
    print(" Total cost: {}, Accumulate cost {}".format(cost, accum_cost))
    sum_cost += cost
    sum_accum += accum_cost
  print(" Grand sum ==> cost: {}, Accum cost {}".format(sum_cost, sum_accum))

  # plot gantt chart here
  for y_axis, (_, costs) in enumerate(all_task_costs.items()):
    sum_cost = 0
    for cost in costs:
      ax2.barh(y_axis, width=cost, left=sum_cost)
      sum_cost += cost

  ax2.set_yticks(range(len(agents)))
  ax2.set_yticklabels([f'agent{id}' for id, _ in agents.items()])

  ax1.set_facecolor((0.9, 0.9, 0.9))
  ax1.autoscale()
  ax1.grid()
  ax2.grid()
  plt.show()

def colorline(ax, waypoints, linewidth=4, alpha=1.0):
  """ Function to plot color gradient lines on graph """
  codes = [mpath.Path.MOVETO] + [mpath.Path.LINETO]*(len(waypoints)-1)
  path = mpath.Path(waypoints, codes)
  verts = path.interpolated(steps=10).vertices
  x, y = verts[:, 0], verts[:, 1]
  z = np.linspace(0, 1, len(x))
  cmaps = ['Oranges', 'Greens', 'Purples', 'Blues', 'Reds',
            'YlOrBr', 'YlOrRd', 'OrRd', 'PuRd', 'RdPu', 'BuPu',
            'GnBu', 'PuBu', 'YlGnBu', 'PuBuGn', 'BuGn', 'YlGn']

  z = np.asarray(z)
  points = np.array([x, y]).T.reshape(-1, 1, 2)
  norm = plt.Normalize(0.0, 0.8)
  segments = np.concatenate([points[:-1], points[1:]], axis=1)
  lc = mcoll.LineCollection(segments, array=z, cmap=random.choice(cmaps), norm=norm,
                            linewidth=linewidth, alpha=alpha)

  ax.add_collection(lc)

################################################################################

def load_task_yaml(task_path):
  """ Load Task Yaml  """
  agents = {} # key: id, value: agent
  tasks = {}  # key: id, value: task
  graph = []

  with open(task_path, 'r') as stream:
    task_config = yaml.safe_load(stream)
    grid_size = task_config["grid_size"]
    grid_length = task_config["grid_length"]

    # Construct Grid World Graph
    for i in range(grid_size):
      for j in range(grid_size):
        graph.append((j*grid_length, i*grid_length))
    assert len(graph) != 0

    # add agents
    for agt in task_config["agents"]:
      agents[agt["id"]] = Agent(agt["id"], agt["wp"])
  
    # add deliveries
    for tsk in task_config["tasks"]["delivery"]:
      waypoints = (tsk["pickup"], tsk["dropoff"])
      tasks[tsk["id"]] = RequestTask(tsk["id"], waypoints)

  print(" Added {} agents, {} tasks".format(len(agents), len(tasks)))
  return agents, tasks, graph

def allocate_tasks(agents, tasks, allocation_path):
  """ 
  Load Task allocation Yaml, and allocate all tasks to designated agents 
  """
  with open(allocation_path, 'r') as stream:
    allocation_config = yaml.safe_load(stream)
    for allocation in allocation_config:
      agt_id = allocation["id"]
      for task_id in allocation["tasks"]:
        agents[agt_id].assign_task(tasks[task_id])
      print(" Allocated {} tasks to agent {}".format(
        len(agents[agt_id].assignments), agt_id))
  return agents

################################################################################

if __name__ == '__main__':
  agents, tasks, graph = load_task_yaml("task_config.yaml")
  agents = allocate_tasks(agents, tasks, "allocation.yaml")
  # agents = calculate_cost(agents); # todo
  plot_task_allocation(agents, graph)
