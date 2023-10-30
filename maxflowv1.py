#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jun  3 13:35:45 2021

@author: ashleymargetts
"""

""" input adjacency ldict where the keys represent each vertex in the graph and the
values are lists representing all vertices adjacent to the given vertex"""

class CycleBreak():
    def __init__(self, adjacency_dict):
        self.adjacency_dict = adjacency_dict
        self.cycles_list = []
        self.temp = []
        self.temp_prev = {}
        self.temps = []
        
    def getTemp(self):
        return self.temp
    
    def getCycles(self):
        return self.cycles_list
            
    def addCycle(self, cycle):
        self.cycles_list.append(cycle)    
    
    def addVertex(self, vertex):
        temp = self.temp
        temp.append(str(vertex))   
        
    # def sameCycle(self, cycle):
    #     for i
            
    def findCycles(self, start):
        
        self.addVertex(start)
        adjacents = self.adjacency_dict[start]
        
        for adj in adjacents: 
            if adjacents.index(adj)!= len(self.adjacency_dict[start])-1:
                self.temps.append(start)
                self.temp_prev[start]= self.getTemp().copy()
            if str(adj) in self.getTemp():
                index = self.getTemp().index(str(adj))
                self.addVertex(adj)
                temp = self.getTemp()
                if temp not in self.cycles_list:
                    self.addCycle(temp[index:])
                try:
                    # self.temp = self.temp_prev[start]
                    if adjacents.index(adj)!= len(self.adjacency_dict[start])-1:
                        self.temp = self.temp_prev[start]
                    else:
                        new_val = self.temps[self.temps.index(start)-1]   
                        self.temp = self.temp_prev[new_val] 
                except ValueError:
                    self.temp = []
                    
                
            else: 
                self.findCycles(adj)
                
    def returnCycles(self, start):
        self.findCycles(start)
        print(self.getCycles())


           
# adj_dict = {}
# adj_dict[1] = [2, 4]
# adj_dict[2] = [3]
# adj_dict[3] = [6]
# adj_dict[4] = [5, 7]
# adj_dict[5] = [2, 6]
# adj_dict[6] = [9]
# adj_dict[7] = [8]
# adj_dict[8] = [5]
# adj_dict[9] = [8]

adj_dict = {}
adj_dict[1] = [2, 4]
adj_dict[2] = [5]
adj_dict[3] = [2]
adj_dict[4] = [5]
adj_dict[5] = [6]
adj_dict[6] = [3]

# adj_dict = {}
# adj_dict[1] = [2]
# adj_dict[2] = [3,5]
# adj_dict[3] = [6]
# adj_dict[4] = [1]
# adj_dict[5] = [4]
# adj_dict[6] = [5]

# adj_dict = {}
# adj_dict[1] = [2]
# adj_dict[2] = [3, 5]
# adj_dict[3] = [6]
# adj_dict[4] = [1, 7]
# adj_dict[5] = [8,4,6]
# adj_dict[6] = []
# adj_dict[7] = [8]
# adj_dict[8] = [9]
# adj_dict[9] = [6]

      
        
CycleBreak(adj_dict).returnCycles(1)
        
        

    
    