#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jun  9 12:39:34 2021

@author: ashleymargetts
"""
# from itertools import combinations

from collections import defaultdict

class Graph():
    def __init__(self,vertices):
        self.graph = defaultdict(list)
        self.ungraph = defaultdict(list)
        self.V = vertices
        self.cycles = []
        self.path = []
        self.slpath = []
        self.s = [{}] * self.V
        self.l = [{}] * self.V
        self.paths = {}
        self.cycle_combos = []
        self.edge_combos = []
        self.edge_dict = {}
        self.source_dict = {}
        self.load_dict = {}
        self.power_losses = {}
 
    def addEdge(self,u,v):
        self.graph[u].append(v)
        
    def removeEdge(self, edge):
        vertices = edge.split('&')
        u = vertices[0]
        v = vertices[1]
        temp = self.graph[u].copy()
        if v in temp:
            temp.remove(v)
        self.graph[u] = temp
        temp1 = self.graph[v].copy()
        if u in temp1:
            temp1.remove(u)
        self.graph[v] = temp1
        
    def addSource(self, source, power):
        self.source_dict[source] = power
        
    def addLoad(self, load, power):
        self.load_dict[load] = power
        
    def addPowerLoss(self, node, loss):
        self.power_losses[node] = loss
        
    def newGraph(self, adj_dict):
        self.graph = adj_dict
        
    def nonDirected(self):
        for vertex in self.graph.keys():
            self.ungraph[vertex] = self.graph[vertex]
            
        for vertex in self.ungraph.keys():
            for node in self.ungraph[vertex]:
                if vertex not in self.ungraph[node]:
                    self.ungraph[node].append(vertex)
                    print(self.ungraph)

    def cycleUtil(self, v, visited, recStack, prev):
 
        # Mark current node as visited and
        # adds to recursion stack
        visited[v] = True
        recStack[v] = True
        self.path.append(v)
        
        # Recur for all neighbours
        # if any neighbour is visited and in
        # recStack then graph is cyclic
        for neighbor in self.ungraph[v]:
            if visited[neighbor] == False:
                self.cycleUtil(neighbor, visited, recStack, v)
                
            elif recStack[neighbor] == True:
                if neighbor in self.path and neighbor != prev:
                    index = self.path.index(neighbor)
                    self.path.append(neighbor)
                    new_path = self.path[index:]
                    self.cycles.append(new_path)
                    self.path.pop()
 
        # The node needs to be poped from
        # recursion stack before function ends
        recStack[v] = False
        self.path = []
 
    # Returns true if graph is cyclic else false
    def Cycles(self):
        self.nonDirected()
        visited = [False] * (self.V)
        recStack = [False] * (self.V)
        for node in range(self.V):
            if visited[node] == False:
                self.cycleUtil(node,visited,recStack, None)
                
    
    def MakeEdgeDict(self):
        "creates a dictionary where the keys are edges and the values are all the cycles each edge is in"
        adj_dict = self.graph
        cycles = self.cycles
        edge_dict = {}
        for key in adj_dict.keys():
            for vertex in adj_dict[key]:
                edge_cycles = []
                for cycle in cycles:
                    if key in cycle and vertex in cycle:
                        edge_cycles.append(cycle)
                if edge_cycles != []:
                    edge_dict[str(key)+'&' + str(vertex)] = edge_cycles
        self.edge_dict = edge_dict

    
    def breakCycleUtil(self, start, visited, recStack):
         
        # Mark current node as visited and
        # adds to recursion stack
        visited[start] = True
        edges = self.edge_dict
        start_index = self.cycle_combos.index(edges[start])
        recStack[start_index] = True
        self.path.append(start)
        cycles = self.cycles
        length = len(cycles)
        
        # Recur for all neighbours
        # if any neighbour is visited and in
        # recStack then graph is cyclic
        for edge in edges.keys():
            if len(self.path) < length:
                index = self.cycle_combos.index(edges[edge])
                if visited[edge] == False and recStack[index] == False: 
                    self.breakCycleUtil(edge, visited, recStack)
                    
            elif len(self.path) == length:
                broken = [False] * len(cycles)
                for edge in self.path:
                    for cycle in edges[edge]:
                        index = cycles.index(cycle)
                        broken[index] = True
                if broken == [True] * len(cycles):
                    path = self.path.copy()
                    path.sort()
                    if path not in self.edge_combos:
                        self.edge_combos.append(path)

 
        # The node needs to be poped from
        # recursion stack before function ends
        recStack[start_index] = False
        self.path.pop()
            
        
    def breakCycles(self):
        self.Cycles()
        self.MakeEdgeDict()
        edges = self.edge_dict
        visited_temp = {}
        for edge in edges.keys():
            visited_temp[edge] = False
            if edges[edge] not in self.cycle_combos:
                self.cycle_combos.append(edges[edge])
        recStack = [False] * len(self.cycle_combos)
        for edge in edges.keys():
            visited = visited_temp.copy()
            self.breakCycleUtil(edge,visited,recStack)
        if self.edge_combos == []:
            self.edge_combos = [[]]
        combos = []
        for combo in self.edge_combos:
            combo.sort()
            if combo not in combos:
                combos.append(combo)
        return combos
    
    
    def backtrack(self):
        oppo_dict = defaultdict(list)
        for vertex in self.graph.keys():
            for node in self.graph[vertex]:
                oppo_dict[node].append(vertex)
        return oppo_dict
    
    # def PathsUtil(self, node, visited, tracked, graph):
        
    #     if tracked == [True] * self.V:
    #         pass
    #     else:
    #         visited[node]= True
    #         self.slpath.append(node)
    #         if tracked[node] == False:
    #             self.paths[node] = self.slpath.copy()
    #             tracked[node] = True
    #         for neighbor in graph[node]:
    #             if visited[neighbor] == False:
    #                 self.PathsUtil(neighbor, visited, tracked, graph)

    #         self.slpath.pop()
    #         visited[node] = False
            
    
    # def findPaths(self):
        
    #     def_tracked = [False] * self.V
    #     for src in self.source_dict.keys():
    #         def_tracked[src] = True
    #     for load in self.load_dict.keys():
    #         def_tracked[load] = True
            
    #     for source in self.source_dict.keys():
    #         visited = [False] * self.V
    #         tracked = def_tracked.copy()
    #         self.PathsUtil(source, visited, tracked, self.graph)
    #         self.s[source] = self.paths.copy()
    #         self.paths = {}
        
    #     backtracks = self.backtrack()
        
    #     for load in self.load_dict.keys():
    #         visited = [False] *self.V
    #         tracked = def_tracked.copy()
    #         self.PathsUtil(load, visited, tracked, backtracks)
    #         self.l[load] = self.paths.copy()
    #         self.paths = {}
    
    
    def bfs(self, src, dest, pred, dist):
     
        # a queue to maintain queue of vertices whose
        # adjacency list is to be scanned as per normal
        # DFS algorithm
        queue = []
      
        # boolean array visited[] which stores the
        # information whether ith vertex is reached
        # at least once in the Breadth first search
        visited = [False] * self.V

      
        # initially all vertices are unvisited
        # so v[i] for all i is false
        # and as no path is yet constructed
        # dist[i] for all i set to infinity
        for i in range(self.V):
     
            dist[i] = 1000000
            pred[i] = -1;
         
        # now source is first to be visited and
        # distance from source to itself should be 0
        visited[src] = True;
        dist[src] = 0;
        queue.append(src);
      
        # standard BFS algorithm
        while (len(queue) != 0):
            u = queue[0];
            queue.pop(0);
            for i in range(len(self.graph[u])):
                
                val = self.graph[u][i]
                if (visited[val] == False):
                    visited[val] = True;
                    dist[val] = dist[u] + 1;
                    pred[val]= u;
                    queue.append(self.graph[u][i]);
      
                    # We stop BFS when we find
                    # destination.
                    if (self.graph[u][i] == dest):
                        return True;
      
        return False;

  
    
    def printShortestDistance(self, s, dest):
     
    # predecessor[i] array stores predecessor of
    # i and distance array stores distance of i
    # from s
        pred=[0 for i in range(self.V)]
        dist=[0 for i in range(self.V)]
  
        if (self.bfs(s, dest, pred, dist) == False):
            print("Given source and destination are not connected")
  
    # vector path stores the shortest path
        path = []
        crawl = dest
        crawl = dest
        path.append(crawl)
     
        while (pred[crawl] != -1):
            path.append(pred[crawl]);
            crawl = pred[crawl];
    
     
        for i in range(len(path)-1, -1, -1):
            print(path[i], end=' ')
            
    def findPaths(self):
        path_list = [{}]* self.V
        test_nodes = []
        for node in self.graph.keys():
            if node not in self.source_dict.keys():
                if node not in self.load_dict.keys():
                    test_nodes.append(node)
        for node in test_nodes:
            for src in self.source_dict.keys():
                path_list[src][node] = self.printShortestDistance(self.graph, src, node, self.V)
            for load in self.load_dict.keys():
                path_list[load][node] = self.printShortestDistance(self.graph, load, node, self.V)

                                   
    
    def source_power_dicts(self):
        """creates dictionary where sources are keys and values are lists of tuples, showing all other vertices and the 
        path taken to reach them
        
        then turns this into a dictionary where sources are keys and values are lists of lists, including each vertex, the power 
        at that vertex coming from a given source, and the entry vertex
        
        # Does this for each dict found in BreakCycles"""
        graph_nodes = list(self.graph.keys())
        test_nodes = graph_nodes.copy()
        for source in self.source_dict.keys():
            test_nodes.remove(source)
        for load in self.load_dict.keys():
            if load in test_nodes:
                test_nodes.remove(load)
                
        maxflow = {}
        for node in test_nodes:
            maxflow[node] = []
        
            
        for edge_combo in self.breakCycles():
            
            init_graph = self.graph.copy()
            for edge in edge_combo:
                self.removeEdge(edge)
            
            self.findPaths()
            source_map = self.s
            load_map = self.l
        
            for node in test_nodes:
                source_map_dict = defaultdict(list)
                load_map_dict = defaultdict(list)
        
                for source in self.source_dict.keys():
                    source_paths = source_map[source]
                    spower = self.source_dict[source]
                    source_path = source_paths[node]
                    for vertex in source_path[1:]:
                        spower= spower * self.power_losses[vertex]
                    source_map_dict[source_path[-2]].append(spower)
                        
                for load in self.load_dict.keys():
                    lpower = self.load_dict[load]
                    load_paths = load_map[load]
                    load_path = load_paths[node]
                    for vertex in load_path[1:]:
                        lpower = ((1-self.power_losses[vertex])*lpower) + lpower
                    load_map_dict[load_path[-2]].append(lpower)

                maxflow_dict = {}
                connections = []
                for s_connection in source_map_dict.keys():
                    connections.append(s_connection)
                        
                for l_connection in load_map_dict.keys():
                    if l_connection not in connections:
                        connections.append(l_connection)
                        
                for connection in connections:
                    load_sum = 0
                    for load_connection in load_map_dict.keys():
                        if load_connection != connection:
                            for power in load_map_dict[load_connection]:
                                load_sum += power
                                
                    source_sum = 0
                    for source_connection in source_map_dict.keys():
                        if source_connection != connection:
                            for power in source_map_dict[source_connection]:
                                source_sum += power
                                
                    source_connection_sum = 0
                    load_connection_sum  = 0
                    for source in source_map_dict[connection]:
                        source_connection_sum += source
                    for load in load_map_dict[connection]:
                        load_connection_sum += load
                        
                        
                    min1= min(source_connection_sum, load_sum )
                    min2 = min(load_connection_sum, source_sum)
                    maxflow_dict[connection] = max(min1, min2)
                    
                maxflow[node].append( max(maxflow_dict.values()))

            overall_max = {}
            for node in test_nodes:
                overall_max[node] = max(maxflow[node])
                
            self.graph = init_graph
            return overall_max

                    
    
    
f = open("AdjacencyList.txt")
# load_dict = {}
# source_dict = {}
# power_losses = {}
num_vert = 0
adjacency = True
source = False
load = False
for x in f:
    x = x.split(',')
    if x!= ['\n']:
        if 'Source List:  vertex number' in x:
            adjacency = False            
        if adjacency and 'vertex number' not in x: 
            num_vert +=1
            
g = Graph(num_vert)
adjacency = True
f = open('AdjacencyList.txt')
for x in f:
    x = x.split(',')
    if x!= ['\n']:
        if 'Source List:  vertex number' in x:
            adjacency = False
            source = True 
            
        if 'Load List:  vertex number' in x:
            source = False
            load = True  
        if adjacency and 'vertex number' not in x:
            for i in range(3, len(x)):
                if '\n' in x[i]:
                    new = x[i].translate({ord(i): None for i in '\n'})
                    g.addEdge(int(x[0]), int(new))

                else:
                    g.addEdge(int(x[0]), int(x[i]))
            
            g.addPowerLoss(int(x[0]), float(x[2]))
            # power_losses[int(x[0])] = float(x[2])
    
        if source and 'Source List:  vertex number' not in x:
            g.addSource(int(x[0]), float(x[1]))
            # source_dict[int(x[0])] = float(x[1])
    
        if load and 'Load List:  vertex number' not in x:
            g.addLoad(int(x[0]), float(x[1]))
            # load_dict[int(x[0])] = float(x[1])
            
            
            

# g = Graph(6)
# g.addEdge(1,2)
# g.addEdge(2,3)
# g.addEdge(2,5)
# g.addEdge(3,6)
# g.addEdge(4,1)
# g.addEdge(5,4)
# g.addEdge(6,5)


# g.Cycles()
# print(g.cycles)
# g.nonDirected()
# print(g.ungraph)

# print(g.breakCycles())

g.printShortestDistance(1, 2)
# print(g.source_power_dicts())

# print(g.power_losses)
# print(g.findPaths())

# 
