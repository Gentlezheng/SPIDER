import Get_Move as Gm
import Init
import numpy as np
import Global_Par as Gp
import time as t
import jhmmtg as jh
import junction_init as ji
import DelayCalculate as dc
import EXP_ABS as Ea
import math
import random
# import tgeaa as tg
import MLP


def Tij_calculate(controller, node_list, time):
    G = controller.junction_matrix
    for u in G.nodes():
        for u, v in G.edges(u):  # u 对v 的评价, v 相对于u的距离变化率，速度变化率，连接时长
            # 确保距离小于250

            distance_uv = Ea.distance(node_list[u].position, node_list[v].position)
            if 0 < distance_uv < Gp.com_dis and u != v:
                # 判断距离的远近
                lat_distance = controller.feature_junction_matrix[u][v]['lat_dist_uv']
                # 距离变远
                if distance_uv > lat_distance and lat_distance != 0:
                    G[u][v]['score'] = (math.sqrt(
                        math.pow(Gp.com_dis, 2) - math.pow(node_list[u].position[0] - node_list[v].position[0],
                                                           2)) - math.fabs(
                        node_list[u].position[1] - node_list[v].position[1])) / Ea.distance(node_list[u].velocity,
                                                                                            node_list[v].velocity)
                else:
                    G[u][v]['score'] = (math.sqrt(
                        math.pow(Gp.com_dis, 2) + math.pow(node_list[u].position[0] - node_list[v].position[0],
                                                           2)) - math.fabs(
                        node_list[u].position[1] - node_list[v].position[1])) / Ea.distance(node_list[u].velocity,
                                                                                            node_list[v].velocity)
    return
def start(file_name, max_delay):
    node_list = []
    # com_node_list = []

    sim_time = 309  # int(input("sim_time:"))
    # 位置文件读取
    try:
        movement_matrix, init_position_matrix = Gm.get_position(file_name)
    except Exception:
        movement_matrix, init_position_matrix = Gm.get_position_X_(file_name)
    node_num = init_position_matrix.shape[0]
    # 控制器初始化
    controller = Init.init_controller(node_num, 'DiG')
    controller.feature_junction_matrix_construction(node_num)
    # 位置数据处理
    init_position_arranged = init_position_matrix[np.lexsort(init_position_matrix[:, ::-1].T)]

    node_position = init_position_arranged[0]
    ji.inti()

    # 节点初始化
    node_list = (Init.init_node(node_position, controller))
    effi = 0
    delay = 0
    std2 = 0
    pt_sum = 0
    # com_node_list = [[435, 82], [577, 59], [21, 557], [86, 212], [117, 125], [434, 432], [509, 402], [17, 366], [347, 116], [532, 425], [399, 184], [376, 95], [469, 496], [88, 371], [288, 326], [354, 111], [264, 252], [122, 49], [548, 471], [191, 311], [350, 552], [428, 197], [575, 477], [533, 324], [388, 62], [136, 363], [442, 144], [507, 373], [255, 87], [142, 339], [389, 145], [186, 423], [372, 235], [444, 5], [571, 73], [83, 247], [273, 61], [356, 588], [456, 260], [357, 562], [280, 553], [341, 218], [597, 513], [319, 58], [185, 152], [26, 57], [556, 385], [348, 310], [13, 101], [46, 436], [380, 329], [409, 77], [6, 229], [123, 31], [508, 367], [566, 476], [461, 278], [183, 166], [466, 393], [35, 193]]
    # len_com_list = len(com_node_list)
    round = 10
    # 生成通信节点
    for i in range(round):
        com_node_list = []
        com_node_list.extend(Init.get_communication_node(node_num-1))
        start_time = t.time()

        # 以秒为间隔进行
        for time in range(200, sim_time):
            print('\nTime: %d' % time)
            # 处理位置矩阵
            current_move = movement_matrix[np.nonzero(movement_matrix[:, 0].A == time)[0], :]
            for value in current_move:
                for i in range(1, 4):
                    node_position[int(value[0, 1]), i] = value[0, i+1]
            node_id_position = node_position[:, [1, 2, 3]]
            # 所有节点更新位置，并发送hello至控制器
            for node in node_list:
                node.update_node_position(node_id_position)
                node.generate_hello(controller)
            jh.num_count()
            # 控制器更新网络全局情况
            controller.predict_position()
            controller.junction_matrix_construction(node_num)
            pt_s = t.time()
            Tij_calculate(controller, node_list, time)
            pt_e = t.time()
            pt_sum += pt_e - pt_s
            # 所有通信节点生成数据包并发送请求至控制器
            node_list[com_node_list[time % int((node_num * Gp.com_node_rate) / 2 - 1)][0]].generate_request(
                com_node_list[time % int((node_num * Gp.com_node_rate) / 2 - 1)][1], controller, 1024)
            # node_list[com_node_list[random.randint(0, len_com_list - 1)][0]].generate_request(
            #     com_node_list[random.randint(0, len_com_list - 1)][1], controller, 1024)
            # 控制器处理路由请求
            print('\nrouting request')
            controller.resolve_request(node_list, 2)

            # 所有节点处理错误路由修复请求
            print('\nerror request')
            controller.resolve_error(node_list, 2)
            print('\nforward')

            # 所有节点开始转发分组
            for node in node_list:
                node.forward_pkt_to_nbr(node_list, controller)
            jh.delete()

        end_time = t.time()
        effi += end_time-start_time
        delay += Gp.sum
        Gp.sum = 0
        pt_sum = 0
        std2 += np.std(Gp.record, ddof=1)
        Gp.record.clear()
    with open('result/data_testLinkduration-2600', 'a', encoding='utf-8') as f:
        f.write('\ncalculation time:\n')
        f.write(str(effi / round))
        f.write('\ndelay:\n')
        f.write(str(dc.delay_cal_dijk(max_delay) / (Gp.success_route + Gp.fail_route) / round))
        f.write('\ndelivery ratio:\n')
        f.write('{},{},{},{},{}'.format(Gp.fail_route, Gp.success_route, Gp.success_times, Gp.fail_times,
                                        Gp.success_times / (Gp.success_times + Gp.fail_times)))
        f.write('\njitter:\n')
        f.write(str(std2 / Gp.pps / round))
    f.close()
if __name__ == '__main__':
    # MAX_DELAY = [1.1464113310254276, 3.6542557189376863, 11.126599337919128, 39.05924780300656, 121.79891569306821,
    #              125.84178336347411, 138.5233391338528, 200, 295.3024903413865, 298.55989934801426]
    # MAX_DELAY = [56.31079316647906, 224.79528621417023, 2103.003379021351, 1382.8866850451882]
    MAX_DELAY = [61.96704364923099, 899.5857541647147, 899.5857541647147, 4082.201330988286, 4082.201330988286]
    for i in range(5):
        file_name = 'data/2600-1500/{}mobility.tcl'.format(i*100 + 200)
        with open('result/data_testLinkduration-2600', 'a', encoding='utf-8') as f:
            f.write('\n'+'-'*30+'\n')
            f.write(file_name)
        f.close()
        print(file_name)
        start(file_name, MAX_DELAY[i])