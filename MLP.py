import EXP_ABS as Ea
import Global_Par as Gp
import ANN
import math
import numpy as np

ann = ANN.ANN()

def def_angle(x0, y0, x1, y1, x2, y2):
    if x0 == x1 and y0 == y1:  # no move
        return 0
    cos = ((x1 - x0) * (x2 - x0) + (y1 - y0) * (y2 - y0)) / (math.sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2)) * math.sqrt(pow(x2 - x0, 2) + pow(y2 - y0, 2)))
    theta = math.acos(cos) / math.pi
    return theta

def Mlp(node_list, time, node_id_position):
    global RG
    for u in RG.nodes():
        for u, v in RG.edges(u):  # u 对v 的评价, v 相对于u的距离变化率，速度变化率，连接时长·node_id_position[self.node_id][0, 0] - self.position[0]
                # Ensure that the distance is less than the communication distance
                distance_uv = Ea.distance(node_id_position[node_list[u].node_id], node_id_position[node_list[v].node_id])
                if 0 < distance_uv < Gp.com_dis and u != v:
                    node_list[u].score_list[v] += 1
                    # Calculate the rate of change of distance
                    trans_dist = (distance_uv - RG[u][v]['lat_dist_uv']) / distance_uv
                    RG[u][v]['lat_dist_uv'] = distance_uv

                    # Whether the direction has changed
                    if node_list[u].direction == node_list[v].direction:
                        RG[u][v]['direction_is_same'] = 1  # Is the direction the same during the period
                    else:
                        RG[u][v]['direction_is_same'] = 0

                    # Gvalue change rate
                    pre_gvalue = Ea.gvlaue(v, RG)
                    Gvalue = (pre_gvalue - RG[u][v]['lat_gvalue']) / pre_gvalue
                    RG[u][v]['lat_gvalue'] = pre_gvalue

                    # Rate of change of acceleration
                    pre_accel = Ea.distance(node_list[v].velocity, node_list[u].velocity)
                    if pre_accel != 0:
                        accel_rate = abs(pre_accel - RG[u][v]['lat_accel'] / pre_accel)
                    else:
                        accel_rate = 0
                    RG[u][v]['lat_accel'] = pre_accel
                    # Angle cosine change rate

                    cos_angle_u_sds = def_angle(node_list[u].position[0], node_list[u].position[1],
                                                node_id_position[node_list[u].node_id][0, 0],
                                                node_id_position[node_list[u].node_id][0, 1], node_list[v].position[0],
                                                node_list[v].position[1])
                    cos_angle_u_sdd = def_angle(node_id_position[node_list[u].node_id][0, 0],
                                                node_id_position[node_list[u].node_id][0, 1], node_list[u].position[0],
                                                node_list[u].position[1], node_id_position[node_list[v].node_id][0, 0],
                                                node_id_position[node_list[v].node_id][0, 1])

                    cos_angle_v_sds = def_angle(node_list[v].position[0], node_list[v].position[1],
                                                node_id_position[node_list[v].node_id][0, 0],
                                                node_id_position[node_list[v].node_id][0, 1], node_list[u].position[0],
                                                node_list[u].position[1])
                    cos_angle_v_sdd = def_angle(node_id_position[node_list[v].node_id][0, 0],
                                                node_id_position[node_list[v].node_id][0, 1], node_list[v].position[0],
                                                node_list[v].position[1], node_id_position[node_list[u].node_id][0, 0],
                                                node_id_position[node_list[u].node_id][0, 1])
                    RG[u][v]['lat_angle'] = np.linalg.det(np.array([[cos_angle_u_sds, cos_angle_u_sdd], [cos_angle_v_sds, cos_angle_v_sdd]]))

                elif distance_uv > Gp.com_dis and node_list[u].score_list[v] != 0:
                    # print(RG.edges[u, v]['score'])
                    print("--准备存储--")
                    ann.DataProcess(time,
                                    node_list[u].node_id, node_list[v].node_id, node_list[u].score_list[v],
                                    trans_dist,
                                    RG[u][v]['direction_is_same'],
                                    Gvalue,
                                    accel_rate if accel_rate < pow(10, 5) else 0,
                                    RG[u][v]['lat_angle'])
                    node_list[u].score_list[v] = 0

        ANN.datasave(ann.X_data_array, ann.y_data_array, "data/x_train_12.18.txt", "data/y_train_12.18.txt")


def datasave_predict():
    X = np.loadtxt("data/x_train_12.18.txt", delimiter=',')
    y = np.loadtxt("data/y_train_12.18.txt", delimiter=',')
    X_train, X_test, y_train, y_test = ann.preprocess(X[:, 3:8], y)
    ANN.NeuralNetwork(X_train, y_train, X_test, y_test)
if __name__ == '__main__':
    datasave_predict()
