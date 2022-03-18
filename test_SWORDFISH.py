import time

import Get_Move as Gm
import Init
import numpy as np
import Global_Par as Gp
import time as t
import jhmmtg as jh
import junction_init as ji
import DelayCalculate as dc
from KINDER import SocialRelationships as Si
from KINDER import KINDER_data_processing as getdata
import os
import EXP_ABS as Ea
'''
social_relationship: obtain social relationships for information about driving trend
Time_point: get vehicle's position for every Time_point slot
Time_window: encounter history times within a Time_window
'''

def start(file_name, scheme):


    sim_time = 309  # int(input("sim_time:"))

    # 位置文件读取
    try:
        movement_matrix, init_position_matrix = Gm.get_position(file_name)
    except Exception:
        movement_matrix, init_position_matrix = Gm.get_position_X_(file_name)
    node_num = init_position_matrix.shape[0]

    # 获取社会关系
    SR = Si.socialRelationships(node_num)
    SR.social_relationship(file_name, Gp.Time_point)

    # 控制器初始化
    controller = Init.init_controller(node_num, 'DiG')
    controller.history_junction_matrix_construction(node_num)
    # 位置数据处理
    init_position_arranged = init_position_matrix[np.lexsort(init_position_matrix[:, ::-1].T)]
    node_position = init_position_arranged[0]
    ji.inti()
    # 节点初始化
    node_list = (Init.init_node(node_position, controller))
    effi = 0
    delay = 0
    std2 = 0
    round = 10
    Gp.fail_route = 0
    Gp.success_route = 0
    Gp.fail_times = 0
    Gp.success_times = 0

    for i in range(round):
        com_node_list = []
        com_node_list.extend(Init.get_communication_node(node_num - 1))

        start_time = t.time()

        # 以秒为间隔进行
        for time in range(Gp.start_time, sim_time):
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
            getdata.update_social_metrics(controller)
            # 所有通信节点生成数据包并发送请求至控制器
            node_list[com_node_list[time % int((node_num * Gp.com_node_rate) / 2 - 1)][0]].generate_request(
                com_node_list[time % int((node_num * Gp.com_node_rate) / 2 - 1)][1], controller, 1024)

            # 控制器处理路由请求
            print('\nrouting request')
            controller.resolve_request(node_list, scheme, SR, time, controller)

            # 所有节点处理错误路由修复请求
            print('\nerror request')
            controller.resolve_error(node_list, scheme, SR, time, controller)
            print('\nforward')

            # 所有节点开始转发分组
            for node in node_list:
                node.forward_pkt_to_nbr(node_list, controller)
            jh.delete()

        end_time = t.time()
        effi += end_time-start_time
        delay += Gp.sum
        Gp.sum = 0

        std2 += np.std(Gp.record, ddof=1)
        Gp.record.clear()
    Ea.save_result(result_file_root, effi, std2, controller.route_matrix.max())


if __name__ == '__main__':
    scheme = 5
    time = time.localtime(time.time())
    ROOT_1200 = '/result2/test-SWORDFISH-1200-' + str(time.tm_mday)+ str(time.tm_hour)
    ROOT_2600 = '/result2/test-SWORDFISH-2600-' + str(time.tm_mday)+ str(time.tm_hour)
    files = [ROOT_1200]
    for file in files:
        result_file_root = os.path.dirname(__file__) +file
        if file == ROOT_1200:
            data_range = 4
            data_name = '1200-1000'
        else:
            data_range = 5
            data_name = '2600-1500'
        for i in range(data_range):
            file_name = os.path.dirname(__file__) + '/data/{}/{}mobility.tcl'.format(data_name,i*100 + 200)
            with open(result_file_root, 'a', encoding='utf-8') as f:
                f.write('\n'+'-'*30+'\n')
                f.write(file_name)
            f.close()
            print(file_name)
            start(file_name, scheme)