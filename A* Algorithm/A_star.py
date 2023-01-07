from scipy.spatial import distance
from PIL import Image 
import numpy as np
import heapq as hq
import matplotlib.pyplot
import matplotlib.image as plt

class A_star():

    def __init__(self, start, goal, V):
        self.vertx = [(i,j) for i in range(np.shape(V)[0]) for j in range(np.shape(V)[1])]
        self.cost_to = {}   # Assigns each vertex v the cost of the shortest known path 
        self.pred = {}
        self.est_tot_cost = {}        
        self.que = []
        self.s = start
        self.g = goal
        self.occu_grid = V

    def recover_path(self):
        
        current_node = self.g
        opt_path = []
        while goal in self.pred:
            previous = self.pred[current_node]
            opt_path.append(previous)
            current_node = previous
            if current_node == self.s:                
                break
            
        if current_node != self.s:
            print("No optimal path found")
            return 

        return opt_path

    def heu (self,p1,p2):        
        euc_dist = distance.euclidean(p1,p2)
        return euc_dist

    def neigh (self, point):        
        index = []
        for i in range(-1,2):
            for j in range(-1,2):
                if (i == 0 and j == 0):
                    continue
                if self.occu_grid[point[0]+i][point[1]+j] == 1:
                    index.append((point[0]+i, point[1]+j))
                else :
                    continue
        return index

    def update_prio(self,prev_dist, new_dist, point):
            if (prev_dist,point) in self.que :
                hq.heappush(self.que,(new_dist,point))  # Add new point 
                self.que.remove((prev_dist,point))    # Remove old point`

    def a_star_search(self):

        # Initialize the cost to infinity
        for idx in self.vertx:
            self.cost_to[idx] = np.inf 
            self.est_tot_cost[idx] = np.inf
        
        # Set the start point costs to 0
        self.cost_to[self.s] = 0
        self.est_tot_cost[self.s] = self.heu(self.s, self.g)
        self.que = [(self.heu(self.s,self.g),self.s)]
        hq.heapify(self.que)

        while self.que:
            v = hq.heappop(self.que)
            if v[1] == self.g :
                return self.recover_path()            
            
            for i in  self.neigh(v[1]) :  
            
                pvi = self.cost_to[v[1]] + self.heu(v[1], i)        # Cost to path cost till prev + new dist cost

                if i in self.est_tot_cost :
                    # Store the prev cost
                    prev_est_cost = self.est_tot_cost[i]
                
                if pvi < self.cost_to[i]:               # If new cost < cost at i point
                    self.pred[i] = v[1]
                    self.cost_to[i] = pvi
                    self.est_tot_cost[i] = pvi + self.heu(i,self.g)
                    
                    if self.que is not None:
                        if (prev_est_cost, i) in self.que:                  # Update to the new cost
                            self.update_prio(prev_est_cost, self.est_tot_cost[i], i)
                        else:
                            hq.heappush(self.que, (self.est_tot_cost[i], i))

                    else:
                        hq.heappush(self.que, (self.est_tot_cost[i], i))

            




if __name__ == '__main__':

    occupancy_map_img = Image.open('occupancy_map.png')
    occupancy_grid = (np.asarray(occupancy_map_img) > 0).astype(int)
    start = (635,140)
    goal = (350,400)

    a_str = A_star(start, goal, occupancy_grid)
    points = a_str.a_star_search()
    
    print(a_str.est_tot_cost[goal])
    new_map = occupancy_grid
    new_map[np.where(occupancy_grid > 0)] = 40
    for idx in points:
        new_map[idx[0]][idx[1]] = 100
    
    plt.imsave('A_star_optimal_path.png',new_map)
    new_image = Image.open('A_star_optimal_path.png')
    new_image.show()





                        











         

