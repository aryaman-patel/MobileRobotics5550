
from dis import dis
from math import dist
from pickle import GLOBAL
from tracemalloc import start
from scipy.spatial import distance
from PIL import Image 
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx




class PRM():
    
    def __init__(self, start, goal, V, sampling, d_max) :
        self.s = start
        self.g = goal 
        self.vertx = self.s
        self.edges = 0
        self.occu_map = V
        self.N = sampling
        self.d_max = d_max
        self.G = nx.Graph()        
        self.counter = 0
        self.neighbor_map = [(-1,-1),(0,-1),(1,-1),(-1,0),(1,0),(-1,1),(0,1),(1,1)]


    def sampling (self):
        while True:
            r = np.random.randint(0, np.shape(self.occu_map)[0])
            c = np.random.randint(0, np.shape(self.occu_map)[1])
            if self.occu_map[r][c] == 1:           # Rejection step include only those points that are 1
                self.vertx = (r,c) 
                break
          

    def dist (self,p1,p2):        
        euc_dist = distance.euclidean(p1,p2)
        return euc_dist

    def reachablity_check(self, v1, v2):

        points = self.get_line(v1[0],v1[1],v2[0],v2[1])

        for indx in points :
            for x,y in self.neighbor_map :
                search_point = (indx[0]+x,indx[1]+y)
                if self.occu_map[search_point] == 0:
                    return False
        return True


    def get_line(self, x1, y1, x2, y2):
        points = []
        issteep = abs(y2-y1) > abs(x2-x1)
        if issteep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2
        rev = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            rev = True
        deltax = x2 - x1
        deltay = abs(y2-y1)
        error = int(deltax / 2)
        y = y1
        ystep = None
        if y1 < y2:
            ystep = 1
        else:
            ystep = -1
        for x in range(x1, x2 + 1):
            if issteep:
                points.append((y, x))
            else:
                points.append((x, y))
            error -= deltay
            if error < 0:
                y += ystep
                error += deltax
        if rev:
            points.reverse()
        return points
               

    def prob_roadmap(self):
        
        for i in range(2500):
            self.sampling()
            self.add_vertx(i,self.vertx)

        return self.G

    def add_vertx(self, idx, vertx):
        
        self.counter = idx
        self.G.add_node(self.counter , pos = vertx)
        current_node = vertx
        pos = nx.get_node_attributes(self.G,'pos')
        for node in  pos :
            if pos[node] == current_node:
                continue
            if self.dist(current_node, pos[node]) <= self.d_max :
                if self.reachablity_check(current_node, pos[node]):
                    self.G.add_edge(node,self.counter, weight = self.dist(current_node, pos[node]))


if __name__ == '__main__':

    occupancy_map_img = Image.open('occupancy_map.png')
    occupancy_grid = (np.asarray(occupancy_map_img) > 0).astype(int)
    start = (635,140)
    goal = (350,400)
    sampling = 2500
    d_max = 75

    prm = PRM(start, goal, occupancy_grid, sampling, d_max)

    graph = prm.prob_roadmap()
    no_nodes = graph.number_of_nodes()
    

    prm.add_vertx(no_nodes+1,start)
    prm.add_vertx(no_nodes+2,goal)

    pos = nx.get_node_attributes(graph,'pos')
    plt.imshow(np.transpose(occupancy_grid))
    # nx.draw_networkx(prm.G,pos, with_labels= False, node_size=1, width = 0.2)
    points = nx.astar_path(prm.G, 2501, 2502)

    cost = nx.astar_path_length(prm.G, 2501, 2502)
    

    poses = []
    for i in prm.G.nodes.data('pos'):
        poses.append(i[1])
    grid = occupancy_grid
    grid[np.where(grid >0)] = 255
    for i in points:
        grid[prm.G.nodes[i]['pos'][0]][prm.G.nodes[i]['pos'][1]] = 100
    plt.imshow(np.transpose(grid) , cmap="inferno", origin = 'lower')

    for i in range(len(points)):
        if i < len(points)-1:
            nx.draw_networkx_edges(prm.G, pos=nx.get_node_attributes(prm.G, 'pos'), edgelist=[(points[i], points[i+1])], width=2, alpha=0.5, edge_color='r')

    nx.draw_networkx_nodes(prm.G, pos=nx.get_node_attributes(prm.G, 'pos'),node_size=0.2)
    # Show the graph
    print("Cost of the path - ", cost)
    plt.savefig('prm.png',dpi=500)
    plt.show()
    






