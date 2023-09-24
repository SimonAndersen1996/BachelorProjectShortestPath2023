import geopandas as gpd
import pandas as pd

"""
u: from node
v: The node to remove
w: to node
Given a path that goes from u to w with v inbetween, remove v, the
edges (u,v) and (v,w) and replace with new edge (u,w)
That means return this new edge
"""
def NodeContraction(u,v,w):
    # Should get id of u and w
    # Also get length of (u,v) and (v,w) and sum them together
    newEdge = {'highway':None, 'oneway': None, 'id': 0, 'timestamp': 0, 'version': 0, 'osm_type': None,
               'geometry': None, 'u': 0, 'v': 0, 'length': 0}
    return newEdge