#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 21 12:40:42 2021

@author: ashleymargetts
"""

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
 
    # def addEdge(self,u,v):
    #     self.graph[u].append(v)
        
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
        return self.edge_combos
    
    
    def backtrack(self):
        oppo_dict = defaultdict(list)
        for vertex in self.graph.keys():
            for node in self.graph[vertex]:
                oppo_dict[node].append(vertex)
        return oppo_dict
    
    def PathsUtil(self, node, visited, tracked, graph):
        
        if tracked == [True] * self.V:
            pass
        else:
            visited[node]= True
            self.slpath.append(node)
            if tracked[node] == False:
                self.paths[node] = self.slpath.copy()
                tracked[node] = True
            for neighbor in graph[node]:
                if visited[neighbor] == False:
                    self.PathsUtil(neighbor, visited, tracked, graph)

            self.slpath.pop()
            visited[node] = False
            
    
    def findPaths(self):
        
        def_tracked = [False] * self.V
        for src in self.source_dict.keys():
            def_tracked[src] = True
        for load in self.load_dict.keys():
            def_tracked[load] = True
            
        for source in self.source_dict.keys():
            visited = [False] * self.V
            tracked = def_tracked.copy()
            self.PathsUtil(source, visited, tracked, self.graph)
            self.s[source] = self.paths.copy()
            self.paths = {}
        
        backtracks = self.backtrack()
        
        for load in self.load_dict.keys():
            visited = [False] *self.V
            tracked = def_tracked.copy()
            self.PathsUtil(load, visited, tracked, backtracks)
            self.l[load] = self.paths.copy()
            self.paths = {}
            
            
    
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