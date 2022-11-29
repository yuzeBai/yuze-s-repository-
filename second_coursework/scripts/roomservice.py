#!/usr/bin/env python3
import rospy
from second_coursework.srv import GetRoomCoord, GetRoomCoordResponse, GetRoomCoordRequest

count = -1


def next(n):
    n = (n + 1) % 4
    return n


def room_point(req: GetRoomCoordRequest):
    res = GetRoomCoordResponse()

    room_coords_A = [(-5, 4), (-3.20, 4), (-5, 1), (-3, 1)]

    room_coords_B = [(-2, 1.3), (-2, 4.8), (1.40, 4.7), (1.27, 1.27)]

    room_coords_C = [(2.33, 4.8), (5.8, 4.53), (5.87, 1.33), (2.47, 1.20)]

    room_coords_D = [(-5.73, -0.67), (-3.73, -0.8), (-3.53, -4.60), (-6.27, -4.93)]

    room_coords_E = [(-2.20, -0.33), (0.87, -0.93), (0.93, -4.20), (-1.93, -3.87)]

    room_coords_F = [(2.53, -1.93), (5.87, -1.93), (5.87, -4.47), (2.73, -4.47)]

    global count
    count = next(count)
    if req.room_name == "A":
        res.point.x = room_coords_A[count][0]
        res.point.y = room_coords_A[count][1]
    if req.room_name == "B":
        res.point.x = room_coords_B[count][0]
        res.point.y = room_coords_B[count][1]
    if req.room_name == "C":
        res.point.x = room_coords_C[count][0]
        res.point.y = room_coords_C[count][1]
    if req.room_name == "D":
        res.point.x = room_coords_D[count][0]
        res.point.y = room_coords_D[count][1]
    if req.room_name == "E":
        res.point.x = room_coords_E[count][0]
        res.point.y = room_coords_E[count][1]
    if req.room_name == "F":
        res.point.x = room_coords_F[count][0]
        res.point.y = room_coords_F[count][1]
    return res


rospy.init_node("room_point_service")
room_srv = rospy.Service('room_get_point', GetRoomCoord, room_point)

rospy.spin()
