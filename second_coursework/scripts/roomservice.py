#!/usr/bin/env python3
import rospy
from second_coursework.srv import GetRoomCoord, GetRoomCoordResponse, GetRoomCoordRequest

count = -1


def next(n):
    n = (n + 1) % 4
    return n


def room_point(req: GetRoomCoordRequest):
    res = GetRoomCoordResponse()

    room_coords_A = [(0.78, 10), (3, 10), (2.6, 7.13), (0.927, 7)]

    room_coords_B = [(4.79, 9.72), (7.44, 9.82), (7.28, 7.38), (4.5, 7.11)]

    room_coords_C = [(9.07, 9.79), (12.2, 9.77), (12.2, 7.16), (9.08, 6.99)]

    room_coords_D = [(0.69, 5.04), (2.82, 5.12), (2.89, 1.19), (0.817, 1.17)]

    room_coords_E = [(4.52, 5.24), (7.33, 5.13), (7.44, 1.82), (4.22, 1.84)]

    room_coords_F = [(8.9, 4.3), (12.2, 4.09), (12.2, 1.27), (8.75, 1.09)]

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
    rospy.loginfo(res)
    return res


rospy.init_node("room_point_service")
room_srv = rospy.Service('room_get_point', GetRoomCoord, room_point)

rospy.spin()
