from shapely.geometry import Point, LineString, Polygon, MultiLineString
import numpy as np


def draw_shape(indicator, xy_center_pos, size=6):
    if indicator == 0:  # hor_line
        p = LineString([(xy_center_pos[0] - size, xy_center_pos[1]), (xy_center_pos[0] + size, xy_center_pos[1])])

    if indicator == 1:  # ver_line
        p = LineString([(xy_center_pos[0], xy_center_pos[1] - size), (xy_center_pos[0], xy_center_pos[1] + size)])

    if indicator == 2:  # cross
        p = MultiLineString([((xy_center_pos[0] - size, xy_center_pos[1]), (xy_center_pos[0] + size, xy_center_pos[1])),
                             ((xy_center_pos[0], xy_center_pos[1] - size), (xy_center_pos[0], xy_center_pos[1] + size))])

    if indicator == 3:  # circle
        p = Point(xy_center_pos[0], xy_center_pos[1]).buffer(size)

    if indicator == 4:   # triangle
        point_1 = (xy_center_pos[0], xy_center_pos[1] + size * 2 * np.sqrt(3) / 3)
        point_2 = (xy_center_pos[0] - size, xy_center_pos[1] - size *  np.sqrt(3) / 3)
        point_3 = (xy_center_pos[0] + size, xy_center_pos[1] - size *  np.sqrt(3) / 3)
        p = Polygon([point_1, point_2, point_3])

    if indicator == 5:   # square
        point_1 = (xy_center_pos[0] + size, xy_center_pos[1] - size)
        point_2 = (xy_center_pos[0] - size, xy_center_pos[1] - size)
        point_3 = (xy_center_pos[0] - size, xy_center_pos[1] + size)
        point_4 = (xy_center_pos[0] + size, xy_center_pos[1] + size)
        p = Polygon([point_1, point_2, point_3, point_4])

    return p
