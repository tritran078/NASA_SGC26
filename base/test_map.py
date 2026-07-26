import map_dis
import random

data = [random.randint(0, 2) for _ in range(4900)] + [random.uniform(-5,5), random.uniform(-5,5), random.uniform(0,360)]

master_map = map_dis.create_master_map()

map_dis.RunMapUpdate(master_map,data)