import map_dis, path_planning, serial, struct

# Wire format, matching src/shared/two_way_protocol.h and the LoRa-ESP
# firmware's BASE role:
#   inbound  (BASE -> computer): [grid cell bytes: LOCAL_SIZE*LOCAL_SIZE][x:2][y:2], little-endian.
#   outbound (computer -> BASE): [count:1][x:2][y:2]*count, little-endian.
GRID_CELLS = map_dis.LOCAL_SIZE * map_dis.LOCAL_SIZE
MAX_WAYPOINTS = 64

ser = serial.Serial('/dev/ttyUSB0', 115200)
master_map = map_dis.create_master_map()


def read_grid_and_pose(port):
    grid_bytes = port.read(GRID_CELLS)
    pose_bytes = port.read(4)
    grid = list(grid_bytes)
    x, y = struct.unpack('<hh', pose_bytes)
    return grid, (x, y)


def encode_waypoints(waypoints):
    capped = waypoints[:MAX_WAYPOINTS]
    payload = struct.pack('<B', len(capped))
    for x, y in capped:
        payload += struct.pack('<hh', x, y)
    return payload


try:
    while True:

        print("waiting for new map to come")
        grid, pose = read_grid_and_pose(ser)
        print("new map arrived")

        current_pose = map_dis.RunMapUpdate(master_map, grid, pose)

        new_coord = input("Please enter a new coordinate for the rover: ")
        x, y, r = map(int, new_coord.split())

        #run A star
        path = path_planning.astar(master_map, x, y, current_pose)
        if path is None:
            print("No path found to the target coordinate.")
            waypoints = []

        else:
            print("path found")
            waypoints = path[::3]   # Every 3rd point

            # Make sure the final goal is included
            if waypoints[-1] != path[-1]:
                waypoints.append(path[-1])

            map_dis.map_display_with_path(master_map, current_pose, waypoints)

        # send path back out to ESP32 base
        print("sending the path to the rover")

        ser.write(encode_waypoints(waypoints))

except KeyboardInterrupt:
    print("Exiting program.")
finally:
    ser.close()
