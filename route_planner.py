
from helpers import Map, load_map_10, load_map_40, show_map
import math

"""
%load_ext autoreload
%autoreload 2
"""

map_10 = load_map_10()
show_map(map_10)

map_10.intersections

# this shows that intersection 0 connects to intersections 7, 6, and 5
map_10.roads[0]

# This shows the full connectivity of the map
map_10.roads

# map_40 is a bigger map than map_10
map_40 = load_map_40()
show_map(map_40)


