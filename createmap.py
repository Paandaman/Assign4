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
	size_field = 40

	pos = []
	for x in range(5,40,5):
		for y in range(5,40,5):
			pos.append((x,y))
			print(x,y)

	for coord in pos:
			dd = Point(coord[0], coord[1]).buffer(1)
			rrwe = PolygonPatch(dd, facecolor='#8A2BE2', edgecolor='#8A2BE2', alpha=0.9, zorder=2)
			ax.add_patch(rrwe)


	ax.axis([0, size_field, 0, size_field], 'equal')

	hej = [1,2,3,4,5]
	jj = [6,7,"strs"]
	plut = [hej+jj]
	plut = plut[0]
	print(plut)

	plt.show()


init_map()


# def init_map(size_field, ax, radius):

#     lower_wallp = Polygon([(0,0), (0,3),(size_field,3),(size_field,0)])
#     lower_wall = PolygonPatch(lower_wallp, facecolor='#ff3333', edgecolor='#6699cc', alpha=0.5, zorder=2)
#     ax.add_patch(lower_wall)

#     left_wallp = Polygon([(0,10), (0,size_field),(15,size_field), (15,10)])
#     left_wall = PolygonPatch(left_wallp, facecolor='#ff3333', edgecolor='#6699cc', alpha=0.5, zorder=2)
#     ax.add_patch(left_wall)

#     right_wallp = Polygon([(25,10), (25,size_field),(size_field,size_field), (size_field,10)])
#     right_wall = PolygonPatch(right_wallp, facecolor='#ff3333', edgecolor='#6699cc', alpha=0.5, zorder=2)
#     ax.add_patch(right_wall)

#     ax.axis([0, size_field, 0, size_field], 'equal')


#     polygons = padd_walls(lower_wallp, left_wallp, right_wallp, radius)
#     return polygons
