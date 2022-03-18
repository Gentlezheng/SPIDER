import Get_Move as Gm
import ANN as ann
import Init
import numpy as np
import time as t
import jhmmtg as jh
import DelayCalculate as dc
import Global_Par as Gp
sim_time = 309
ANN = ann.ANN()
ANN.Predict_mlp("data/x_train.txt", "data/y_train.txt")
def start(file_name):
    node_list = []
    try:
        movement_matrix, init_position_matrix = Gm.get_position(file_name)
    except Exception:
        movement_matrix, init_position_matrix = Gm.get_position_X_(file_name)
    # movement_matrix, init_position_matrix = Gm.get_position(file_name)
    # Controller initialization
    node_num = init_position_matrix.shape[0]
    print('节点数：{}\n'.format(node_num))
    controller = Init.init_controller(node_num, 'G')
    controller.feature_junction_matrix_construction(node_num)
    global RG_feature
    RG_feature = controller.feature_junction_matrix
    # Location data processing
    init_position_arranged = init_position_matrix[np.lexsort(init_position_matrix[:, ::-1].T)]  # Sort by first column order
    node_position = init_position_arranged[0]

    # Node initialization The initial position and node number of each node
    node_list = (Init.init_node(node_position, controller))

    effi = 0
    delay = 0
    std2 = 0
    pt_s = 0
    pt_e = 0
    pt_sum = 0
    rec_delay = 0
    # Generate communication node
    round = 10

    for i in range(round):
        com_node_list = []
        com_node_list.extend(Init.get_communication_node(node_num - 1))
        print('通信节点列表：\n', com_node_list)
        ## Count the load of each vehicle in the current second
        # v_load = [0 for i in range(len(node_list))]
        start_time = t.time()
        # In seconds
        for time in range(200, sim_time, 1):
            print('\nTime: %d' % time)
            # Processing position matrix
            current_move = movement_matrix[np.nonzero(movement_matrix[:, 0].A == time)[0], :]

            for line in current_move:
                for i in range(1, 4):
                    node_position[int(line[0, 1]), i] = line[0, i + 1]
            node_id_position = node_position[:, [1, 2, 3]]  # 当前current_move 的后三项特征[x,y,z]
            # All nodes update their positions and send hello to the controller
            for node in node_list:
                node.update_node_position(node_id_position)
                node.generate_hello(controller)

            # The controller updates the global situation of the network
            controller.predict_position()
            controller.junction_matrix_construction(node_num)
            pt_s = t.time()
            ANN.DataGet(controller, node_list, time)
            pt_e = t.time()
            pt_sum += pt_e - pt_s
            # The communication node generates a data packet and sends a request to the controller
            node_list[com_node_list[time % int((node_num * Gp.com_node_rate) / 2 - 1)][0]].generate_request(
                com_node_list[time % int((node_num * Gp.com_node_rate) / 2 - 1)][1], controller, 1024)
            # The controller handles the routing request
            print('\nrouting request')
            controller.resolve_request(node_list, 1)
            # All nodes process error routing repair requests
            print('\nerror request')
            controller.resolve_error(node_list, 1)
            print('\nforward')
            # All nodes start to forward packets
            for node in node_list:
                node.forward_pkt_to_nbr(node_list, controller)
            jh.delete()
        end_time = t.time()
        effi += end_time - start_time - pt_sum
        pt_sum = 0
        delay += Gp.sum
        Gp.sum = 0
        std2 += np.std(Gp.record, ddof=1)
        Gp.record.clear()
    with open('result/data_testANN{}'.format('1200-1000'), 'a', encoding='utf-8') as f:
        f.write('\ncalculation time:\n')
        f.write(str(effi/round))
        f.write('\ndelay:\n')
        f.write(str(dc.delay_cal()/(Gp.success_route+Gp.fail_route)/round))
        f.write('\ndelivery ratio:\n')
        f.write('{},{},{},{},{}'.format(Gp.fail_route, Gp.success_route, Gp.success_times, Gp.fail_times, Gp.success_times/(Gp.success_times+Gp.fail_times)))
        f.write('\njitter:\n')
        f.write(str(std2/Gp.pps/round))
    f.close()

if __name__ == '__main__':
    for i in range(1, 6):
        file_name = 'data/1200-1000/{}mobility.tcl'.format(i * 100 + 100)
        print(file_name)
        with open('result/data_testANN{}'.format('1200-1000'), 'a', encoding='utf-8') as f:
            f.write('\n'+'-'*30+'\n')
            f.write(file_name)
        f.close()
        start(file_name)
