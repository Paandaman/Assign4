import numpy as np
from matplotlib import pyplot as plt
from descartes import PolygonPatch
from shapely.geometry.polygon import LinearRing, Polygon
from shapely.geometry import MultiPolygon, Point
from scipy import interpolate
from matplotlib.pyplot import quiver
import math
import random


def init_map():

	fig, ax = plt.subplots()
	size_field = 15

	lower_wallp = Polygon([(0,0), (0,3),(size_field,3),(size_field,0)])
	lower_wall = PolygonPatch(lower_wallp, facecolor='#ff3333', edgecolor='#6699cc', alpha=0.5, zorder=2)
	ax.add_patch(lower_wall)

	#left_wallp = Polygon([(-5,6), (5,6),(5,size_field+5),(-5,size_field+5)])
	#left_wall = PolygonPatch(lower_wallp, facecolor='#ff3333', edgecolor='#6699cc', alpha=0.5, zorder=2)
	#ax.add_patch(left_wall)

	ax.axis([-0, size_field, -0, size_field], 'equal')

	plt.show()


init_map()