from PIL import Image, ImageOps
import numpy as np
import matplotlib.pyplot as plt

# load img of map

img = Image.open('/home/trungle/catkin_ws/src/navigation/src/testmap.png')

img = ImageOps.grayscale(img)

np_img = np.array(img)
np_img = - np_img # invert loaded img, because 0 = free, 1 = obj
np_img[np_img > 0] = 1
# plt.set_cmap('binary')
# plt.imshow(np_img)

# save image
np.save('cspace.npy', np_img)

grid = np.load('cspace.npy')
# plt.imshow(grid)
# plt.tight_layout()
# plt.show()

class treeNode():
  def __init__(self, locationX, locationY):
    self.locationX = locationX
    self.locationY = locationY
    self.children = []
    self.parent = None

class RRTAlgorithm():
  def __init__(self, start, goal, num_iterations, grid, step_size, near_goal_dist):
    self.randomTree = treeNode(start[0], start[1])
    self.goal = treeNode(goal[0], goal[1])
    self.nearestNode = None
    self.iterations = num_iterations
    self.grid = grid
    self.rho = step_size
    self.near_goal_dist = near_goal_dist
    self.path_distance = 0
    self.nearest_dist = 10000
    self.num_waypoints = 0
    self.waypoints = []

  def add_child(self, locationX, locationY):
    if self.goal_found(np.array([locationX, locationY])):
      self.nearestNode.children.append(self.goal)
      self.goal.parent = self.nearestNode
    else:
      temp_node = treeNode(locationX, locationY)
      self.nearestNode.children.append(temp_node)
      temp_node.parent = self.nearestNode

  def sample_point(self):
    point = np.random.random(2) * np.array([self.grid.shape[1], self.grid.shape[0]])
    return point

  def steer_point(self, location_start: treeNode, location_end):
    length, u_vect = self.unit_vector(location_start, location_end)
    if 5 <= length <= self.rho:
      point = location_end
    else:
      offset = self.rho * u_vect
      point = np.array([location_start.locationX + offset[0], location_start.locationY + offset[1]])

    if point[0] >= self.grid.shape[1]:
      point[0] = self.grid.shape[1]
    if point[1] >= self.grid.shape[0]:
      point[1] = self.grid.shape[0]

    return point

  def is_in_obstacle(self, location_start: treeNode, location_end):
    length, u_vect = self.unit_vector(location_start, location_end)
    temp_point = np.array([0.0, 0.0])
    
    for i in range(round(length)):
      temp_point[0] = location_start.locationX + i * u_vect[0]
      temp_point[1] = location_start.locationY + i * u_vect[1]

      if grid[round(temp_point[1]), round(temp_point[0])] == 0:
        return True

    return False

  def unit_vector(self, location_start: treeNode, location_end):
    curr_location = np.array([location_start.locationX, location_start.locationY])
    vect = location_end - curr_location
    length = np.linalg.norm(vect)
    u_vect = vect/length

    return length, u_vect

  def find_nearest(self, root: treeNode, point):
    if not root:
      return
    dist = self.calc_distance(root, point)
    if dist <= self.nearest_dist:
      self.nearestNode = root
      self.nearest_dist = dist

    for child in root.children:
      self.find_nearest(child, point)

  def calc_distance(self, node1, point):
    dist = np.sqrt((node1.locationX - point[0])**2 + (node1.locationY - point[1])**2)
    return dist

  def goal_found(self, point):
    goal_dist = self.calc_distance(self.goal, point)

    if goal_dist <= self.near_goal_dist:
      return True
    return False

  def reset_nearest_values(self):
    self.nearestNode = None
    self.nearest_dist = 10000

  def retrace_rrt_path(self, goal):
    if goal.locationX == self.randomTree.locationX and goal.locationY == self.randomTree.locationY:
      return

    self.num_waypoints += 1
    current_point = np.array([goal.locationX, goal.locationY])
    self.waypoints.insert(0, current_point)
    self.retrace_rrt_path(goal.parent)

start = np.array([20.0, 22.4])
goal = np.array([10.0, 74.4])
num_iterations = 400
step_size = 8
near_goal_dist = 8

fig = plt.figure("RRT Algorithm")
plt.imshow(255-grid, cmap='binary')
plt.plot(start[0], start[1], 'ro')
plt.plot(goal[0], goal[1], 'bo')
goal_region = plt.Circle((goal[0], goal[1]), near_goal_dist, color='b', fill = False)
ax = fig.gca()
ax.add_patch(goal_region)
plt.xlabel('X-axis $(m)$')
plt.ylabel('Y-axis $(m)$')

def process_rrt(start, goal, num_iterations, step_size, near_goal_dist):
  rrt = RRTAlgorithm(start, goal, num_iterations, grid, step_size, near_goal_dist)

  for i in range(rrt.iterations):
    rrt.reset_nearest_values()
    # print("Iteration: ", i)

    point = rrt.sample_point()
    rrt.find_nearest(rrt.randomTree, point)
    new = rrt.steer_point(rrt.nearestNode, point)
    
    if rrt.is_in_obstacle(rrt.nearestNode, new):
      continue

    rrt.add_child(new[0], new[1])
    # plt.pause(0.10)
    # plt.plot([rrt.nearestNode.locationX, new[0]], [rrt.nearestNode.locationY, new[1]], 'go', linestyle='--')

    if rrt.goal_found(new):
      rrt.add_child(goal[0], goal[1])
      print("Goal found!")
      print(rrt.goal.locationX, rrt.goal.locationY)
      print(new[0], new[1])
      break

  rrt.retrace_rrt_path(rrt.goal)
  rrt.waypoints.insert(0, start)
  print("Number of waypoints: ", rrt.num_waypoints)

  return rrt.waypoints

waypoints = process_rrt(start, goal, num_iterations, step_size, near_goal_dist)
for i in range(len(waypoints) - 1):
    plt.plot([waypoints[i][0], waypoints[i+1][0]], [waypoints[i][1], waypoints[i+1][1]], 'ro', linestyle='--')
    plt.pause(0.10)

plt.show()