import cv2 as cv
import mediapipe as mp
import numpy as np
import socket

class Sender:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port

    def send_data(self, message):
        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp.sendto(message, (self.ip, self.port))

def most_common(lst):
    return max(set(lst), key=lst.count)


def line_equation(p1, p2, axis = "horizontal"):
    x1, y1 = p1
    x2, y2 = p2

    m = (y2 - y1) / (x2 - x1)
    n = y1 - m * x1

    def fx(x):
        return m * x + n

    def fy(y):
        return (y - n) / m

    if axis == "vertical":
        return fy

    return fx


def nine_direction(iris, key_points):
    key_point_names = ["left_bot", "left_top", "center_left_bot", "center_right_bot",
                       "center_left_top", "center_right_top", "right_bot", "right_top"]
    points = {name: point.ravel() for name, point in zip(key_point_names, key_points)}
    points["iris"] = iris.ravel()

    top_horizontal_line = line_equation(points["left_top"], points["right_top"])
    bot_horizontal_line = line_equation(points["left_bot"], points["right_bot"])
    left_vertical_line = line_equation(points["center_left_bot"], points["center_left_top"], "vertical")
    right_vertical_line = line_equation(points["center_right_bot"], points["center_right_top"], "vertical")

    row, column = "mid", "center"

    if points["iris"][1] < top_horizontal_line(points["iris"][0]):
        row = "top"
    elif points["iris"][1] > bot_horizontal_line(points["iris"][0]):
        row = "bot"

    if points["iris"][0] < left_vertical_line(points["iris"][1]):
        column = "left"
    elif points["iris"][0] > right_vertical_line(points["iris"][1]):
        column = "right"

    return row, column


mp_face_mesh = mp.solutions.face_mesh
RIGHT_IRIS = [474, 475, 476, 477]
LEFT_IRIS = [469, 470, 471, 472]
L_H_LEFT = [33]  # right eye right most landmark
L_H_RIGHT = [133]  # right eye left most landmark
R_H_LEFT = [362]  # left eye right most landmark
R_H_RIGHT = [263]  # left eye left most landmark

vertical_position_buffer = [None] * 30
horizontal_position_buffer = [None] * 30

temp_vertical_position_buffer = [None] * 5
temp_horizontal_position_buffer = [None] * 5


cap = cv.VideoCapture(0)
with mp_face_mesh.FaceMesh(
        max_num_faces=1,
        refine_landmarks=True,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
) as face_mesh:
    ip = '192.168.4.1'
    port = 4210
    sender = Sender(ip, port)
    while True:
        for i in range(5):
            ret, frame = cap.read()
            if not ret:
                break
            frame = cv.flip(frame, 1)
            rgb_frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
            img_h, img_w = frame.shape[:2]
            results = face_mesh.process(rgb_frame)
            if results.multi_face_landmarks:

                mesh_points = np.array([np.multiply([p.x, p.y], [img_w, img_h]).astype(int)
                                        for p in results.multi_face_landmarks[0].landmark])

                (l_cx, l_cy), l_radius = cv.minEnclosingCircle(mesh_points[LEFT_IRIS])
                left_center = np.array([l_cx, l_cy], dtype=np.int32)
                (r_cx, r_cy), r_radius = cv.minEnclosingCircle(mesh_points[RIGHT_IRIS])
                right_center = np.array([r_cx, r_cy], dtype=np.int32)
                cv.circle(frame, left_center, int(l_radius), (0, 255, 0), 1, cv.LINE_AA)
                cv.circle(frame, left_center, 1, (0, 255, 255), -1, cv.LINE_AA)
                cv.circle(frame, right_center, int(r_radius), (0, 255, 0), 1, cv.LINE_AA)
                cv.circle(frame, right_center, 1, (0, 255, 255), -1, cv.LINE_AA)

                position = (mesh_points[[362]][0] + mesh_points[[398]][0]) // 2
                position2 = mesh_points[[263]][0] - mesh_points[[467]][0]
                final_position = mesh_points[[263]][0] - (position2 * 0.55)

                result_row, result_col = nine_direction(right_center, [position, mesh_points[[414]][0],
                                                                       mesh_points[[380]][0], mesh_points[[374]][0],
                                                                        mesh_points[[385]][0], mesh_points[[386]][0],
                                                                        final_position, mesh_points[[466]][0]])
                temp_vertical_position_buffer[i] = result_row
                temp_horizontal_position_buffer[i] = result_col


            cv.imshow("img", frame)
            key = cv.waitKey(1)
            if key == ord('q'):
                break

        vertical_position_buffer.extend(temp_vertical_position_buffer)
        vertical_position_buffer = vertical_position_buffer[5:]

        horizontal_position_buffer.extend(temp_horizontal_position_buffer)
        horizontal_position_buffer = horizontal_position_buffer[5:]

        most_common_horizontal_position = most_common(horizontal_position_buffer)

        if most_common_horizontal_position == "center":
            most_common_vertical_position = most_common(vertical_position_buffer)
            print(most_common_vertical_position)
            byte = str(most_common_vertical_position).encode('utf-8')
            #print(byte)
            sender.send_data(byte)
        else:
            print(most_common_horizontal_position)
            byte = str(most_common_horizontal_position).encode('utf-8')
           # print(byte)
            sender.send_data(byte)

    
cap.release()
cv.destroyAllWindows()
