"""
This code is adapted from search.py in the AIMA Python implementation, which is published with the license below:

	The MIT License (MIT)

	Copyright (c) 2016 aima-python contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Search (Chapters 3-4)

The way to use this code is to subclass Problem to create a class of problems,
then create problem instances and solve them with calls to the various search
functions."""

import sys
from collections import deque
import subway
import heapq
from subway import straight_line_distance
import math

#______________________________________________________________________________

'''DO NOT MODIFY THIS CLASS'''

class Problem:
	"""The abstract class for a formal problem.  You should subclass this and
	implement the method successor, and possibly __init__, goal_test, and
	path_cost. Then you will create instances of your subclass and solve them
	with the various search functions."""
	
	

	def __init__(self, initial, goal=None):
		"""The constructor specifies the initial state, and possibly a goal
		state, if there is a unique goal.  Your subclass's constructor can add
		other arguments."""
		self.initial = initial; self.goal = goal
		
	def successor(self, state):
		"""Given a state, return a sequence of (action, state) pairs reachable
		from this state. If there are many successors, consider an iterator
		that yields the successors one at a time, rather than building them
		all at once. Iterators will work fine within the framework."""
		raise NotImplementedError("successor() must be implemented in subclass")
	
	def goal_test(self, state):
		"""Return True if the state is a goal. The default method compares the
		state to self.goal, as specified in the constructor. Implement this
		method if checking against a single self.goal is not enough."""
		return state == self.goal
	
	def path_cost(self, c, state1, action, state2):
		"""Return the cost of a solution path that arrives at state2 from
		state1 via action, assuming cost c to get up to state1. If the problem
		is such that the path doesn't matter, this function will only look at
		state2.  If the path does matter, it will consider c and maybe state1
		and action. The default method costs 1 for every step in the path."""
		return c + 1
		
	def h(self, node):
		"""Return the heuristic function value for a particular node. Implement
		this if using informed (heuristic) search."""
		return 0
#______________________________________________________________________________
class subway_problem(Problem): #Sub-Class of Problem 
	def __init__(self, initial, goal, subMap): #Additional parameter refers to subway map
		Problem.__init__(self, initial, goal)
		self.subMap = subMap
	
	def successor(self, state):
		return self.subMap.adjacent_stations(state)
	
	def goal_test(self, state, alt_goal=None):
		# If an alt_goal is provided we check if the current state is the alt goal 
		# (this is used in smart search)
		if alt_goal is not None:
			return state == alt_goal 
			
		return state == self.goal

	def path_cost(self, c, state1, action, state2):
		chosenLinks = self.subMap.get_links_between(state1, state2)
		return c + next(chosenLinks).get_distance()

	def h(self, node):
		return straight_line_distance(node.state, self.goal)  # Use the standalone function
	
	def get_closest_stations(self, d): # Returns all stations within distance d from the goal
		for station in self.subMap.get_stations():
			distance_to_goal = straight_line_distance(station, self.goal)
			if distance_to_goal <= d:
				yield station

	
class puzzle_problem(Problem): #Sub-Class of Problem
	def __init__(self, initial):
		Problem.__init__(self, initial, "012345678")

	def successor(self, state):
		allOptions = []
		allStates = []
		emptySquare = state.index("0")
		if emptySquare > 2:
			allOptions.append(("Move Tile " + state[emptySquare - 3] + " down", emptySquare - 3))
		if emptySquare < 6:
			allOptions.append(("Move Tile " + state[emptySquare + 3] + " up", emptySquare + 3))
		if emptySquare % 3 < 2:
			allOptions.append(("Move Tile " + state[emptySquare + 1] + " left", emptySquare + 1))
		if emptySquare % 3 > 0:
			allOptions.append(("Move Tile " + state[emptySquare - 1] + " right", emptySquare - 1))

		for i in range(len(allOptions)):
			allStates.append((allOptions[i][0], self.movePiece(state, emptySquare, allOptions[i][1])))

		return allStates
	
	def goal_test(self, state, alt_goal=None): # Other problems require the option for an alt_goal. Allowing it here as an arugument simplifies other code
		return state == self.goal
	
	def path_cost(self, c, state1, action, state2):
		return c + 1
			
	def h(self,state):
		heuristic = 0
		for i in range(9):
			heuristic += self.manhattan(state.index(str(i)), i, 3)
		return heuristic
			
	def movePiece(self, state, emptyIndex, moveIndex):
		newString = state
		strList = list(newString)

		strList[emptyIndex] = state[moveIndex]
		strList[moveIndex] = "0"

		return "".join(strList)
	
	def manhattan(number1, number2, size): #Finds manhattan distance on a 3 by 3 grid
		vertDist = abs(math.floor(number1/size) - math.floor(number2/size))
		horDist = abs((number1 % size) - (number2 % size))
		return vertDist + horDist

#______________________________________________________________________________
'''DO NOT MODIFY THIS CLASS'''

class Node:
	"""A node in a search tree. Contains a pointer to the parent (the node
	that this is a successor of) and to the actual state for this node. Note
	that if a state is arrived at by two paths, then there are two nodes with
	the same state.  Also includes the action that got us to this state, and
	the total path_cost (also known as g) to reach the node.  Other functions
	may add an f and h value. You will not need to
	subclass this class."""

	__nextID = 1

	def __init__(self, state, parent=None, action=None, path_cost=0):
		"Create a search tree Node, derived from a parent by an action."
		self.state = state
		self.parent = parent
		self.action = action
		self.path_cost = path_cost
		self.depth = 0
		self.id = Node.__nextID
		Node.__nextID += 1
		
		if parent:
			self.depth = parent.depth + 1
			
	def __str__(self):
		return "<Node " + str(self.state) + ">"
	
	def __repr__(self):
		return "<Node " + str(self.state) + ">"	
	
	def path(self):
		"Create a list of nodes from the root to this node."
		x, result = self, [self]
		while x.parent:
			result.append(x.parent)
			x = x.parent
		return result[::-1]

	def expand(self, problem):
		"Return a list of nodes reachable from this node. [Fig. 3.8]"
		return [Node(next, self, act,
					 problem.path_cost(self.path_cost, self.state, act, next))
				for (act, next) in problem.successor(self.state)]
	
	def __eq__(self, other):
		if isinstance(other, Node):
			return self.id == other.id
		return False
	
	def __lt__(self, other):
		if isinstance(other, Node):
			return self.id < other.id
		raise TypeError("\'<\' not supported between instances of Node and "+str(type(other)))
	
	def __hash__(self):
		return hash(self.id)

#______________________________________________________________________________
## Smart search function for the subway system. returns the closest station within the given distance from the goal
def smart_search(problem, algorithm, distance):
	possible_results = []

	# Retrive all stations within the given distance from the goal
	closest_stations = problem.get_closest_stations(distance) 

	# Perform the algorithm on each of the retrieved station (passing in that station to be used for the goal test)
	for station in closest_stations:
		result = algorithm(problem, station)
		if result != None: # Check that the station is reachable from the initial station
			possible_results.append(result)
			
	# Return the station with the smallest path cost
	return min(possible_results, key=lambda x: x[0].path_cost)

#______________________________________________________________________________
## Uninformed Search algorithms

'''DO NOT MODIFY THE HEADERS OF ANY OF THESE FUNCTIONS'''
def breadth_first_search(problem, alt_goal=None):
	"""Returns a tuple with the goal Node followed by an Integer with the amount of nodes visited
	(Returns None if a solution isn't found)"""
	# Setup
	queue = deque()
	visited = set()
	nodes_visited = 0

	# Add start node
	start = Node(problem.initial)
	queue.append(start)

	# Check if it's the goal
	if problem.goal_test(start.state, alt_goal):
		return (start, 1)
	
	while queue:
		current = queue.popleft()

		if current.state not in visited:
			# Visit
			visited.add(current.state)
			nodes_visited += 1
			
			# Insert neighbors
			neighbors = current.expand(problem)
			for neighbor in neighbors:
				if neighbor.state not in visited:
					if problem.goal_test(neighbor.state, alt_goal):
						return (neighbor, nodes_visited)
					queue.append(neighbor)
	return None
		
	
def depth_first_search(problem, alt_goal=None):
	"""Returns a tuple with the goal Node followed by an Integer with the amount of nodes visited
	(Returns None if a solution isn't found)"""
	# Setup
	stack = deque()
	visited = set()
	nodes_visited = 0

	# Add start node
	start = Node(problem.initial)
	stack.append(start)

	while stack:
		current = stack.pop()

		if current.state not in visited:
			# Visit
			visited.add(current.state)
			nodes_visited += 1
			# Check if we found the solution
			if problem.goal_test(current.state, alt_goal):
				return (current, nodes_visited)
			
			# Insert neighbors
			neighbors = current.expand(problem)
			for neighbor in neighbors:
				if neighbor.state not in visited:
					stack.append(neighbor)

	return None

def uniform_cost_search(problem, alt_goal=None):
	# Setup
	pqueue = []
	visited = set()
	nodes_visited = 0
	opt_cost = {}

	# Initializing our starting node, priority queue tuple and path of best cost 
	start = Node(problem.initial)
	heapq.heappush(pqueue, (start.path_cost, start))
	opt_cost[start.state] = start.path_cost

	while pqueue: 
		# Set our current node to the node with the least cost with a pop
		_, current_node = heapq.heappop(pqueue)
		nodes_visited += 1

		# Check to see if path cost of current node is larger than our current best known cost, skips it if so
		if current_node.path_cost > opt_cost.get(current_node.state, float('inf')):
			continue

		# Goal state check
		if problem.goal_test(current_node.state, alt_goal):
			return (current_node, nodes_visited)
		
		# Add the current node to our list of visited nodes
		visited.add(current_node.state)

		# Takes a look at neighboring nodes for current node
		neighbors = current_node.expand(problem)
		for neighbor in neighbors:
			# Check to see if we haven't yet visited node
			if neighbor.state not in visited:
				# Grabs our best cost so far
				best_so_far = opt_cost.get(neighbor.state, float('inf'))
				# Grabs the cost for going to the next state from the expanded nodes
				new_cost = neighbor.path_cost
				# If the path is cheaper we add this to the priority queue for exploration
				if new_cost < best_so_far:
					opt_cost[neighbor.state] = new_cost
					heapq.heappush(pqueue, (new_cost, neighbor))

	# In case we finish checking and there is just no goal state to return
	return None
#______________________________________________________________________________
# Informed (Heuristic) Search

def astar_search(problem, alt_goal=None):
	queue = []
	visited = set()
	nodes_visited = 0
	opt_cost = {}

	start = Node(problem.initial)
	heapq.heappush(queue, (start.path_cost + problem.h(start), start))  
	opt_cost[start.state] = start.path_cost

	while queue:
		_, current_node = heapq.heappop(queue)  # Extract node with lowest f(n)
		nodes_visited += 1

		if current_node.state in visited:
			continue

		visited.add(current_node.state)

		if problem.goal_test(current_node.state, alt_goal):
			return (current_node, nodes_visited)

		for neighbor in current_node.expand(problem):
			if neighbor.state not in visited:
				old_cost = opt_cost.get(neighbor.state, float('inf'))
				if neighbor.path_cost < old_cost:
					opt_cost[neighbor.state] = neighbor.path_cost
					f_cost = neighbor.path_cost + problem.h(neighbor)  # Compute f(n)
					heapq.heappush(queue, (f_cost, neighbor))

	return None

#______________________________________________________________________________

## Output
def print_solution(solution, output_type):
	"""
	Paramaters: 
	solution. A tuple with the goal Node followed by an integer with the amount of nodes visited
	output_type. A string containing either "actions", "states" or "actions and states"
	"""
	print("Total cost: "+str(solution[0].path_cost))
	print("Number of search nodes visited: "+str(solution[1]))
	print("Final path: ")
	if output_type == "actions":
		print_actions(solution[0])
	elif output_type == "states":
		print_states(solution[0])
	elif output_type == "actions and states":
		print_actions_and_states(solution[0])
	else:
		print("Unrecognized output type")

def print_states(node):
	stack = deque()

	while node.parent:
		stack.append(node.state)
		node = node.parent
	
	stack.append(node.state)

	while stack:
		print(stack.pop())

def print_actions(node):
	stack = deque()

	while node.parent:
		stack.append(node.action)
		node = node.parent

	while stack:
		print(stack.pop())

def print_actions_and_states(node):
	stack = deque()

	while node.parent:
		stack.append(node)
		node = node.parent
	
	stack.append(node)

	while stack:
		current = stack.pop()
		print(str(current.action)+str(", ")+str(current.state))

## Main

def main():
	
	global cityMap

	# Take input
	arg1 = sys.argv[1] # Options are "eight", "boston", and "london"
	algorithm = sys.argv[2] # Options are "dfs", "bfs", ucs, and "astar"
	initial = sys.argv[3] # Either the starting subway stop, or the starting position of the number tiles

	if algorithm == "bfs":
		algorithm = breadth_first_search
	elif algorithm == "dfs":
		algorithm = depth_first_search
	elif algorithm == "ucs":
		algorithm = uniform_cost_search
	elif algorithm == "astar":
		algorithm = astar_search
	else:
		print(f"Unrecognized algorithm: {algorithm}")

	if arg1 == "eight":

		# Prepare problem
		problem = puzzle_problem(initial)
		output_type = "actions and states"
		# Compute result
		result = algorithm(problem)

	else:

		# Take more input
		goal = sys.argv[4] # the destination subway stop

		# Prepare subway search
		if arg1 == "boston":
			cityMap = subway.build_boston_map()
		elif arg1 == "london":
			cityMap = subway.build_london_map()
		
		initialCity = cityMap.get_station_by_name(initial)
		goalCity = cityMap.get_station_by_name(goal)

		# Get distance if it was provided
		if len(sys.argv) > 5:
			distance = float(sys.argv[5])
		else: 
			distance = 0

		# Prepare problem
		problem = subway_problem(initialCity, goalCity, cityMap)
		output_type = "states"

		# Compute results
		result = smart_search(problem, algorithm, distance)

	# Output results
	print_solution(result, output_type)

main()