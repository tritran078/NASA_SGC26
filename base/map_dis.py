import numpy as np
import matplotlib.pyplot as plt

# master map size
MASTER_W = 1000 
MASTER_H = 1000

RESOLUTION = 0.08 #reso in meters

LOCAL_SIZE = 70 #scanned map size

CENTER = (MASTER_W // 2, MASTER_H // 2) #center of master map

color_lookup = np.array([[255, 255, 255], [0, 0, 0], [128, 128, 128],[255,0,0], [0,255,0]]) # color chart for displaying in map
""" 0: free(white), 1: occupied(black), 2: unknown(gray), 3: rover(red), 4: path(green)"""


def create_master_map():
    master_map = np.full((MASTER_W, MASTER_H), 2)
    return master_map

def get_master_map(master_map):
    return master_map.copy() 

def local_grid(grid_array):
    # grid_array is just the flat LOCAL_SIZE*LOCAL_SIZE cell list -- pose
    # travels separately over the wire, and heading isn't part of the
    # protocol at all.
    return np.array(grid_array, dtype=int).reshape((LOCAL_SIZE, LOCAL_SIZE)) # convert 1D array to 2D array

def place_on_master(master_map, local_grid, pose):
    x, y = pose

    # x, y arrive pre-converted to grid cells (see src/main.cpp), so no
    # RESOLUTION division here -- that would double-scale the position.
    center_x = CENTER[0] + x
    center_y = CENTER[1] + y
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
    grid[ry][rx] = 3 # set the rover position to 3 for display

    color_map = color_lookup[grid]

    plt.imshow(color_map)
    plt.show(block = False)
    plt.pause(0.1)

def RunMapUpdate(master_map, grid, pose):
    # grid and pose arrive separately over the wire -- see main_base.py
    local_map = local_grid(grid)

    #write new map and pose into master map
    rover_grid_coord = place_on_master(master_map, local_map, pose)

    #display map
    map_display(master_map, rover_grid_coord)

    return rover_grid_coord


def map_display_with_path(map_array, rover_pose, path=None):
    grid = np.array(map_array).reshape((MASTER_H, MASTER_W)).copy()

    if path:
        for (x, y) in path:
            if grid[y][x] == 0:
                grid[y][x] = 4  # reuse rover-color slot to show path

    rx, ry = rover_pose
    grid[ry][rx] = 3

    color_map = color_lookup[grid]
    plt.imshow(color_map)
    plt.show(block=False)
    input("Press Enter to exit...")




    
    







