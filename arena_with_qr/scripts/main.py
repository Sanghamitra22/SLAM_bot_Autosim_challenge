from grid import Grid
from dijkstra import dijkstra
from astar import a_star

def main(obs):
    g = Grid(9, 9, obstacles = obs )
    src = (4, 0)
    dest = (1,8)

    print("Grid visualization: ")
    print(g)

    print("\nLegend:\n\tS = Start\n\tE = End\n\tX = Obstacle\n\t_ = Empty space")

    print("\nPath by Dijkstra's algorithm:")
    print(g.plot_path(dijkstra(g, src, dest)))
    print(dijkstra(g, src, dest))

    print("\nPath by A* search algorithm:")
    print(g.plot_path(a_star(g, src, dest)))
import numpy as np

def l2g(l):
	g = []
	for j in range(1,74,9):
		for i in l:
			if  j <= i < j+9:
				g.append( ( i-j, ((j-1)/9) ) )
	return g

				
	
			


if __name__ == "__main__":
    l = [58,1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15, 16, 17, 20, 21, 22, 26, 29, 31, 33, 34, 35, 37, 38, 40, 41, 42,44,46, 48, 49, 51, 53, 55, 57, 58, 60, 62, 64, 65, 66, 68, 69, 70, 71, 72, 73, 74, 75,76, 77, 78, 79, 81,5, 81,2,74, 78]
    l = list(set(l))
    l.sort()
    #print(l)
    main_list = np.setdiff1d(np.arange(1,82),l)
    print(main_list)
    main(l2g(main_list))
  
