import map_dis, path_planning, serial, os, time

RESOLUTION = 0.08  # meters per grid cell

ser = serial.Serial('/dev/ttyUSB0', 115200)
master_map = map_dis.create_master_map()
last_mtime = 0

try:
    while True:

        print("waiting for new map to come")
        # wait for new data (mtime changed = ESP32 base finished writing+renaming)
        while True:
            try:
                mtime = os.path.getmtime("filepath")
                if mtime != last_mtime:
                    last_mtime = mtime
                    break
            except FileNotFoundError:
                pass
            time.sleep(0.05)
        
        
        # read complete data (guaranteed whole, thanks to atomic rename)
        print("new map arrived")
        with open("scanned_map.txt", 'r') as f:
            data = [int(x) for x in f.read().split()]

        #data += [0, 0, 0] # for fake map scan

        #current_pose = (map_dis.CENTER[0], map_dis.CENTER[1])# fake pose for testing
        
        x, y, heading = data[-3:]
        grid_x = map_dis.CENTER[0] + int(round(x / RESOLUTION))
        grid_y = map_dis.CENTER[1] + int(round(y / RESOLUTION))
        current_pose = (grid_x, grid_y)
        

        
        map_dis.RunMapUpdate(master_map, data)

        new_coord = input("Please enter a new coordinate for the rover: ")
        x, y, r = map(int, new_coord.split())

        #run A star
        path = path_planning.astar(master_map, x, y, current_pose)
        if path is None:
            print("No path found to the target coordinate.")    
        
        else:
            print("path found")
            waypoints = path[::3]   # Every 3rd point

            # Make sure the final goal is included
            if waypoints[-1] != path[-1]:
                waypoints.append(path[-1])
            
            map_dis.map_display_with_path(master_map, current_pose, waypoints)

        # send path back out to ESP32 base
        print("sending the path to the rover")

        ser.write((','.join(map(str, path)) + '\n').encode())

except KeyboardInterrupt:
    print("Exiting program.")
finally:
    ser.close()

