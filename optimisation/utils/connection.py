from shapely.geometry import Point, LineString, Polygon, MultiLineString
import numpy as np



# Add the offset to convert to real world coordinates (cm)
x_offset = -4
y_offset = -53
load_resistance = 45
supply_voltage = 45

bar1_x = -13
bar2_x = 13
shape_num = 5


def draw_shape(indicator, xy_center_pos, size=5):
    if indicator == 0:  # hor_line
        p = LineString([(xy_center_pos[0] - size, xy_center_pos[1]), (xy_center_pos[0] + size, xy_center_pos[1])])

    elif indicator == 1:  # ver_line
        p = LineString([(xy_center_pos[0], xy_center_pos[1] - size), (xy_center_pos[0], xy_center_pos[1] + size)])

    elif indicator == 2:  # cross
        p = MultiLineString(
            [((xy_center_pos[0] + size, xy_center_pos[1] + size), (xy_center_pos[0] - size, xy_center_pos[1] - size)),
             ((xy_center_pos[0] - size, xy_center_pos[1] + size), (xy_center_pos[0] + size, xy_center_pos[1] - size))])

    elif indicator == 3:  # circle
        p = Point(xy_center_pos[0], xy_center_pos[1]).buffer(size)

    elif indicator == 4:  # triangle
        size = 9*np.sqrt(3)/3
        point_1 = [xy_center_pos[0] - size * 2 * np.sqrt(3) / 3, xy_center_pos[1]]
        point_2 = [xy_center_pos[0] + size * np.sqrt(3) / 3, xy_center_pos[1] - size]
        point_3 = [xy_center_pos[0] + size * np.sqrt(3) / 3, xy_center_pos[1] + size]
        p = Polygon([point_1, point_2, point_3])

    elif indicator == 5:  # diamond
        size = 6
        point_1 = (xy_center_pos[0] + size, xy_center_pos[1])
        point_2 = (xy_center_pos[0], xy_center_pos[1] - size)
        point_3 = (xy_center_pos[0] - size, xy_center_pos[1])
        point_4 = (xy_center_pos[0], xy_center_pos[1] + size)      
        p = Polygon([point_1, point_2, point_3, point_4])

    return p


def check_connection(state_list, shape_num=shape_num, bar1_x=bar1_x, bar2_x=bar2_x, multi_shape=False):
    intersection_vec = []
    left_connect = False
    right_connect = False
    all_connect = False
    shape = []

    bar1 = draw_shape(1, [bar1_x, 0], size=13)
    bar2 = draw_shape(1, [bar2_x, 0], size=13)

    if multi_shape:
        shape_list = state_list[0:shape_num]
        # x_list = state_list[shape_num:2 * shape_num]
        # y_list = state_list[2 * shape_num:]
        x_list = state_list[shape_num::2]
        y_list = state_list[shape_num + 1::2]
        shape.append(bar1)
        for i in range(shape_num):
            shape.append(draw_shape(shape_list[i], [x_list[i], y_list[i]]))
        shape.append(bar2)

    else:
        x_list = state_list[0:shape_num]
        y_list = state_list[shape_num:]
        shape.append(bar1)
        for i in range(shape_num):
            shape.append(draw_shape(3, [x_list[i], y_list[i]]))
        shape.append(bar2)

    for j in range(shape_num + 2):
        intersection = []
        for k in range(shape_num + 2):
            if k != j:
                flag = shape[j].intersects(shape[k])
                if flag:
                    intersection.append(k)
        intersection_vec.append(intersection)

    if intersection_vec[0]:
        left_connect = True

    if intersection_vec[shape_num + 1]:
        right_connect = True

    def del_rep_shape(connected_list, target_list):
        for item in target_list:
            if item in connected_list:
                target_list.remove(item)

    if left_connect & right_connect:
        for item in intersection_vec[0]:
            connect_list_0 = [0, item]
            touch = intersection_vec[item].copy()
            del_rep_shape(connect_list_0, touch)
            if touch:
                if (shape_num + 1) in touch:
                    all_connect = True
                    connect_list = connect_list_0
                for item in touch:
                    connect_list_1 = connect_list_0.copy()
                    connect_list_1.append(item)
                    touch_1 = intersection_vec[item].copy()
                    del_rep_shape(connect_list_1, touch_1)
                    if touch_1:
                        if (shape_num + 1) in touch_1:
                            all_connect = True
                            connect_list = connect_list_1
                        for item in touch_1:
                            connect_list_2 = connect_list_1.copy()
                            connect_list_2.append(item)
                            touch_2 = intersection_vec[item].copy()
                            del_rep_shape(connect_list_2, touch_2)
                            if touch_2:
                                if (shape_num + 1) in touch_2:
                                    all_connect = True
                                    connect_list = connect_list_2
                                for item in touch_2:
                                    connect_list_3 = connect_list_2.copy()
                                    connect_list_3.append(item)
                                    touch_3 = intersection_vec[item].copy()
                                    del_rep_shape(connect_list_3, touch_3)
                                    if touch_3:
                                        if (shape_num + 1) in touch_3:
                                            all_connect = True
                                            connect_list = connect_list_3
                                        for item in touch_3:
                                            connect_list_4 = connect_list_3.copy()
                                            connect_list_4.append(item)
                                            touch_4 = intersection_vec[item].copy()
                                            del_rep_shape(connect_list_4, touch_4)
                                            if touch_4:
                                                if (shape_num + 1) in touch_4:
                                                    all_connect = True
                                                    connect_list = connect_list_4
                                                for item in touch_4:
                                                    connect_list_5 = connect_list_4.copy()
                                                    connect_list_5.append(item)
                                                    touch_5 = intersection_vec[item].copy()
                                                    del_rep_shape(connect_list_5, touch_5)
                                                    if touch_5:
                                                        if (shape_num + 1) in touch_5:
                                                            all_connect = True
                                                            connect_list = connect_list_5
                                                        for item in touch_5:
                                                            connect_list_6 = connect_list_5.copy()
                                                            connect_list_6.append(item)
                                                            touch_6 = intersection_vec[item].copy()
                                                            del_rep_shape(connect_list_6, touch_6)
                                                            if touch_6:
                                                                if (shape_num + 1) in touch_6:
                                                                    all_connect = True
                                                                    connect_list = connect_list_6

    # print(intersection_vec)
    # print(connect_list)
    if all_connect:
        return True

    else:
        return False
