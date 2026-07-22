import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

# master map size
MASTER_W = 1000 
MASTER_H = 1000

RESOLUTION = 0.08 #reso in meters

LOCAL_SIZE = 70 #scanned map size

CENTER = (MASTER_W // 2, MASTER_H // 2) #center of master map

color_lookup = np.array([[255, 255, 255], [0, 0, 0], [128, 128, 128],[255,0,0]]) # color chart for displaying in map
""" 0: free, 1: occupied, 2: unknown, 3: rover """


def create_master_map():
    master_map = np.full((MASTER_W, MASTER_H), 2)
    return master_map

def get_master_map(master_map):
    return master_map.copy() 

def local_grid(map_array):
    local_grid_flat = map_array[:4900] #1D map 
    x,y,r = map_array[4900], map_array[4901], map_array[4902] #pose 
    pose = x,y,r
    local_grid = np.array(local_grid_flat, dtype=int).reshape((LOCAL_SIZE, LOCAL_SIZE)) # convert 1D array to 2D array 

    return pose, local_grid

def place_on_master(master_map, local_grid, pose):
    x, y, heading = pose

    center_x = CENTER[0] + int(round(x / RESOLUTION))
    center_y = CENTER[1] + int(round(y / RESOLUTION))
    half = LOCAL_SIZE // 2

    for ly in range(LOCAL_SIZE):
        for lx in range(LOCAL_SIZE):
            dx, dy = lx - half, ly - half
            mx, my = center_x + dx, center_y + dy

            if 0 <= mx < MASTER_W and 0 <= my < MASTER_H:
                if master_map[my][mx] == 2:  # only fill in unknown cells
                    master_map[my][mx] = local_grid[ly][lx]
    
    return center_x, center_y   # return rover grdi coord



def map_display(map_array, rover_pose):
    grid = np.array(map_array).reshape((MASTER_H, MASTER_W)).copy()  # copy so we don't mutate the real master_map

    rx, ry = rover_pose
    grid[ry][rx] = 3

    color_map = color_lookup[grid]

    plt.imshow(color_map)
    plt.show()

def RunMapUpdate(master_map, data):
    # get map and pose
    pose, local_map = local_grid(data)

    #write new map and pose into master map
    rover_grid_coord = place_on_master(master_map, local_map, pose)

    #display map
    map_display(master_map, rover_grid_coord)
    print("the rover scanned heading was ", pose[2])




    
    







