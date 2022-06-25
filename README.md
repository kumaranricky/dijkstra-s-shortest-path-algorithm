### EXP NO: 03

### DATE: 10/05/2022


# <p align = "center"> Dijkstra's Shortest Path Algorithm </p>

## AIM

To develop a code to find the shortest route from the source to the destination point using Dijkstra's shortest path algorithm.

## THEORY
Best first search is a traversal technique that decides which node is to be visited next by checking which node is the most promising one and then check it. For this it uses an evaluation function to decide the traversal.
This best first search technique of tree traversal comes under the category of heuristic search or informed search technique.
The cost of nodes is stored in a priority queue. This makes implementation of best-first search is same as that of breadth First search. We will use the priorityqueue just like we use a queue for BFS.
## DESIGN STEPS

### STEP 1:
Identify a location in the google map:

### STEP 2:
Select a specific number of nodes with distance


### STEP 3: 
Create a dictionary with all the node pairs (keys) and their respective distances as the values

### STEP 4: 
Implement the search algorithm by passing any node and f(node) to find a Best route.

### STEP 5: 
Display the route sequence.


## ROUTE MAP
#### Include your own map
#### Example map
![map](https://user-images.githubusercontent.com/75243072/166133888-34c37b9c-e8ed-4f27-9af6-6d633ff6a3d5.png)

## PROGRAM
```
Developed By
Student name : Kumaran.B
Reg.no : 212220230026
```


```python
%matplotlib inline
import matplotlib.pyplot as plt
import random
import math
import sys
from collections import defaultdict, deque, Counter
from itertools import combinations
import heapq


class Problem(object):
    """The abstract class for a formal problem. A new domain subclasses this,
    overriding `actions` and `results`, and perhaps other methods.
    The default heuristic is 0 and the default action cost is 1 for all states.
    When yiou create an instance of a subclass, specify `initial`, and `goal` states 
    (or give an `is_goal` method) and perhaps other keyword args for the subclass."""

    def __init__(self, initial=None, goal=None, **kwds): 
        self.__dict__.update(initial=initial, goal=goal, **kwds) 
        
    def actions(self, state):        
        raise NotImplementedError
    def result(self, state, action): 
        raise NotImplementedError
    def is_goal(self, state):        
        return state == self.goal
    def action_cost(self, s, a, s1): 
        return 1
    
    def __str__(self):
        return '{0}({1}, {2})'.format(
            type(self).__name__, self.initial, self.goal)


class Node:
    "A Node in a search tree."
    def __init__(self, state, parent=None, action=None, path_cost=0):
        self.__dict__.update(state=state, parent=parent, action=action, path_cost=path_cost)

    def __str__(self): 
        return '<{0}>'.format(self.state)
    def __len__(self): 
        return 0 if self.parent is None else (1 + len(self.parent))
    def __lt__(self, other): 
        return self.path_cost < other.path_cost


failure = Node('failure', path_cost=math.inf) # Indicates an algorithm couldn't find a solution.
cutoff  = Node('cutoff',  path_cost=math.inf) # Indicates iterative deepening search was cut off.

def expand(problem, node):
    "Expand a node, generating the children nodes."
    s = node.state
    for action in problem.actions(s):
        s1 = problem.result(s, action)
        cost = node.path_cost + problem.action_cost(s, action, s1)
        yield Node(s1, node, action, cost)
        

def path_actions(node):
    "The sequence of actions to get to this node."
    if node.parent is None:
        return []  
    return path_actions(node.parent) + [node.action]


def path_states(node):
    "The sequence of states to get to this node."
    if node in (cutoff, failure, None): 
        return []
    return path_states(node.parent) + [node.state]


class PriorityQueue:
    """A queue in which the item with minimum f(item) is always popped first."""

    def __init__(self, items=(), key=lambda x: x): 
        self.key = key
        self.items = [] # a heap of (score, item) pairs
        for item in items:
            self.add(item)
         
    def add(self, item):
        """Add item to the queuez."""
        pair = (self.key(item), item)
        heapq.heappush(self.items, pair)

    def pop(self):
        """Pop and return the item with min f(item) value."""
        return heapq.heappop(self.items)[1]
    
    def top(self): return self.items[0][1]

    def __len__(self): return len(self.items)


def best_first_search(problem, f):
    "Search nodes with minimum f(node) value first."
    node = Node(problem.initial)
    frontier = PriorityQueue([node], key=f)
    reached = {problem.initial: node}
    while frontier:
        node = frontier.pop()
        if problem.is_goal(node.state):
            return node
        for child in expand(problem,node):
            s = child.state
            if s not in reached or child.path_cost < reached[s].path_cost:
                reached[s] = child
                frontier.add(child)
    return failure

def g(n): 
    return n.path_cost


class RouteProblem(Problem):
    """A problem to find a route between locations on a `Map`.
    Create a problem with RouteProblem(start, goal, map=Map(...)}).
    States are the vertexes in the Map graph; actions are destination states."""
    
    def actions(self, state): 
        """The places neighboring `state`."""
        return self.map.neighbors[state]
    
    def result(self, state, action):
        """Go to the `action` place, if the map says that is possible."""
        return action if action in self.map.neighbors[state] else state
    
    def action_cost(self, s, action, s1):
        """The distance (cost) to go from s to s1."""
        return self.map.distances[s, s1]
    
    def h(self, node):
        "Straight-line distance between state and the goal."
        locs = self.map.locations
        return straight_line_distance(locs[node.state], locs[self.goal])



class Map:
    """A map of places in a 2D world: a graph with vertexes and links between them. 
    In `Map(links, locations)`, `links` can be either [(v1, v2)...] pairs, 
    or a {(v1, v2): distance...} dict. Optional `locations` can be {v1: (x, y)} 
    If `directed=False` then for every (v1, v2) link, we add a (v2, v1) link."""

    def __init__(self, links, locations=None, directed=False):
        if not hasattr(links, 'items'): # Distances are 1 by default
            links = {link: 1 for link in links}
        if not directed:
            for (v1, v2) in list(links):
                links[v2, v1] = links[v1, v2]
        self.distances = links
        self.neighbors = multimap(links)
        self.locations = locations or defaultdict(lambda: (0, 0))

        
def multimap(pairs) -> dict:
    "Given (key, val) pairs, make a dict of {key: [val,...]}."
    result = defaultdict(list)
    for key, val in pairs:
        result[key].append(val)
    return result
    
# Create your own map and define the nodes

chennai_to_arakkonam = Map(
    {('chennai', 'annanagar'):  8, ('annanagar', 'koyambedu'): 8, ('koyambedu', 'poonamalle'): 13, ('poonamalle', 'chettipedu'): 10, ('chettipedu', 'sengadu'): 4, ('sengadu', 'perambakkam'): 13,('perambakkam', 'thakkolam'): 13, ('thakkolam', 'arakkonam'):  12,
     ('chennai', 'perambur'): 7, ('perambur', 'ambattur'): 15, ('ambattur', 'avadi'): 10, ('avadi', 'thiruvallur'): 13,('thiruvallur', 'thiruvalangadu'): 20,('thiruvalangadu', 'vyasapuaram'): 8,('vyasapuaram', 'arakkonam'): 9,
     ('poonamalle', 'thirumalizai'): 5, ('thirumalizai', 'perumalpattu'):  10, ('perumalpattu', 'manavalanagar'): 14, ('manavalanagar', 'thiruvallur'): 3,
    ('manavalanagar', 'manavur'): 42,('manavur', 'arakkonam'): 21,('manavalanagar', 'sengadu'): 26,
    ('perumalpattu', 'nemam'): 7,('nemam', 'chettipedu'): 15,})


r0 = RouteProblem('chennai', 'arakkonam', map=chennai_to_arakkonam)
goal_state_path=best_first_search(r0,g)
path_states(goal_state_path)
print("GoalStateWithPath:{0}".format(goal_state_path))
print(path_states(goal_state_path)) 
print("Total Distance={0} Kilometers".format(goal_state_path.path_cost))    

r1 = RouteProblem('chennai', 'thiruvallur', map=chennai_to_arakkonam)
goal_state_path=best_first_search(r1,g)
print("GoalStateWithPath:{0}".format(goal_state_path))
print(path_states(goal_state_path)) 
print("Total Distance={0} Kilometers".format(goal_state_path.path_cost))

r2 = RouteProblem('manavur', 'annanagar', map=chennai_to_arakkonam)
goal_state_path=best_first_search(r2,g)
print("GoalStateWithPath:{0}".format(goal_state_path))
print(path_states(goal_state_path)) 
print("Total Distance={0} Kilometers".format(goal_state_path.path_cost))

r3 = RouteProblem('arakkonam', 'ambattur', map=chennai_to_arakkonam)
goal_state_path=best_first_search(r3,g)
print("GoalStateWithPath:{0}".format(goal_state_path))
print(path_states(goal_state_path)) 
print("Total Distance={0} Kilometers".format(goal_state_path.path_cost))

r4 = RouteProblem('thiruvalangadu', 'chettipedu', map=chennai_to_arakkonam)
goal_state_path=best_first_search(r4,g)
print("GoalStateWithPath:{0}".format(goal_state_path))
print(path_states(goal_state_path)) 
print("Total Distance={0} Kilometers".format(goal_state_path.path_cost))
```

## <br><br><br>OUTPUT:
![Screenshot (155)](https://user-images.githubusercontent.com/75243072/167658293-ae138267-e6ed-4802-b67d-c764fe129834.png)
![Screenshot (156)](https://user-images.githubusercontent.com/75243072/167658311-b299a8d3-4390-4941-adf8-22fccca81fca.png)

## <br><br>SOLUTION JUSTIFICATION:
Depth first algorithm is a best algorithm of finding the lowest path cost.This algorithm will go node by node.It will select a particular
node by considering the path cost.If the path cost is low,then it will go deeper and follow the same process untill we reach the destination,
Otherwise,it will add to frontier.

## <br><br><br><br><br><br><br><br><br><br><br><br><br><br><br><br><br><br><br>RESULT:
Hence, Best-First-Search Algorithm was implemented for a route finding problem.



