import Packet as Pkt
import Global_Par as Gp
import jhmmtg as jh
import time
import proba as pr
import math


class Node:
    def __init__(self, node_id, controller):
        self.node_id = node_id  # node id
        self.position = [0, 0, 0]  # Position three-dimensional
        self.velocity = [0, 0, 0]  # speed
        self.acceleration = []  # Acceleration
        self.routing_table = []  # Routing table
        self.geo_routing_table = []
        self.data_pkt_list = []  # Data group storage
        self.cache = 1024  # Current cache
        self.controller = controller  # Own controller
        self.pkt_seq = 0  # Current node packet sequence number
        self.junction = []
        self.grid = -1

        self.lat_position = [0, 0, 0] # Record the position of the previous moment
        self.dx = 1
        self.dy = 1
        self.dz = 1
        self.IMN = []  # Node's influence maximization node
        self.IMN_LIST = {}
        self.score = 0  # Evaluation value
        self.trans_dist = 0  # Transmission distance
        self.link_time = 0  # Link duration
        self.trans_dist_rate = self.trans_dist  # Transmission distance change rate, divided by time when in use

        # self.Gvalue = 0  # Calculate according to the degree and weight in the graph
        # self.direction_is_same = 0  # Whether the direction is the same, the same is 1, the difference is 0
        # self.accel_rate = 0
        # self.lat_gvalue = 0
        # self.lat_accel = 0
        self.features = {}
        self.score_list = {}
        for i in range(1000):
            self.score_list[i] = 0

    def angle(x0, y0, x1, y1, x2, y2):
        if x0 == x2 and y0 == y2:  # no move
            return 0
        cos = ((x1 - x0) * (x2 - x0) + (y1 - y0) * (y2 - y0)) / (math.sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2)) * math.sqrt(pow(x2 - x0, 2) + pow(y2 - y0, 2)))
        # theta = math.acos(cos) / math.pi
        return cos

    # Update your position based on the data
    def update_node_position(self, node_id_position):
        self.velocity[0] = node_id_position[self.node_id][0, 0] - self.position[0]
        self.velocity[1] = node_id_position[self.node_id][0, 1] - self.position[1]
        self.velocity[2] = node_id_position[self.node_id][0, 2] - self.position[2]

        # self.angle = Node.angle(self.position[0], self.position[1], self.position[0], self.position[1] + 1, node_id_position[self.node_id][0, 0], node_id_position[self.node_id][0, 1])
        self.lat_position = self.position
        self.position = [node_id_position[self.node_id][0, 0], node_id_position[self.node_id][0, 1], node_id_position[self.node_id][0, 2]]
        if self.velocity[0] < 0:
            self.dx = -1
        if self.velocity[1] < 0:
            self.dy = -1
        if self.velocity[2] < 0:
            self.dz = -1
        self.direction = [self.dx, self.dy, self.dz]
        self.junction = jh.junction_judge(self.position[0], self.position[1], self.node_id)
        return

    # Generate data packets
    def generate_hello(self, controller):
        # Delay processing
        controller.hello_list.append(Pkt.Hello(self.node_id,  self.position,  self.velocity,  self.acceleration,  self.cache))
        return

    # Generate a data packet and send a request to the controller, adding 1 to its serial number
    def generate_request(self,  des_id, controller, size):
        # Delay processing
        print('node %d generate packet to node %d' % (self.node_id, des_id))
        self.data_pkt_list.append(Pkt.DataPkt(self.node_id, des_id, size, 0, self.node_id, self.pkt_seq, time.time()))
        controller.flow_request_list.append(Pkt.FlowRequest(self.node_id,  des_id, self.node_id, self.pkt_seq))
        self.pkt_seq = self.pkt_seq + 1
        return

    # 产生数据包，且向控制器发送请求，自身序号加1
    # def generate_geo_request(self,  des_list, controller, size):
    #     #     # 时延处理
    #     #     print('node %d generate packet to GOI' % (self.node_id))
    #     #     print(des_list)
    #     #     self.data_pkt_list.append(Pkt.geo_DataPkt(self.node_id, des_list, size, 0, self.node_id, self.pkt_seq, time.time()))
    #     #     controller.geo_flow_request_list.append(Pkt.geo_FlowRequest(self.node_id, des_list, self.node_id, self.pkt_seq))
    #     #     self.pkt_seq = self.pkt_seq + 1
    #     #     return

    # send error request
    def generate_error(self,  source_id,  des_id, controller, seq, node_list):
        # 根据发送者节点和序号确定此错误路由是否存在，存在即错误次数加1
        for error in controller.flow_error_list:
            if error.source_id == source_id and error.source_seq == seq:
                error.time = error.time + 1
                return
        # Delay processing
        # The controller adds this error route
        controller.flow_error_list.append(Pkt.FlowError(source_id,  des_id,  self.node_id, 1, seq, seq))
        self.pkt_seq = self.pkt_seq + 1
        # controller.flow_error_list.append(Pkt.FlowError(source_id,  des_id,  self.node_id, 1, source_id, seq))
        # node_list[source_id].pkt_seq = node_list[source_id].pkt_seq + 1
        return

    # Processing routing reply (using the sender node and sequence number to determine the routing information belongs to)
    def receive_flow(self, flow_reply):
        # If the destination node has been reached, the routing table adds an entry with itself as the next hop node
        if flow_reply.des_id == self.node_id:
            self.routing_table.append(Pkt.RoutingTable(flow_reply.source_id, flow_reply.des_id, flow_reply.des_id, 0, flow_reply.node_id, flow_reply.seq))
            return

        # If this entry already exists, update the information
        for t in self.routing_table:
            if t.seq == flow_reply.seq and t.node_id == flow_reply.node_id:
                for key, node_id in enumerate(flow_reply.route):
                    if node_id == self.node_id:
                        t.next_hop_id = (flow_reply.route[key + 1])
                        return

        # Add new routing entries
        for key, node_id in enumerate(flow_reply.route):
            if node_id == self.node_id:
                self.routing_table.append(Pkt.RoutingTable(flow_reply.source_id, flow_reply.des_id, flow_reply.route[key+1], 0, flow_reply.node_id, flow_reply.seq))
                return

    # Forward self-carrying packets according to your own routing table
    def forward_pkt_to_nbr(self, node_list, controller):
        for pkt in self.data_pkt_list[::-1]:
            for table in self.routing_table[::-1]:
                # If the routing table entry matches the group
                if pkt.seq == table.seq and pkt.node_id == table.node_id:
                    # Judging whether the forwarding is successful or not
                    d = pow(node_list[table.next_hop_id].position[0] - self.position[0], 2) + pow(
                        node_list[table.next_hop_id].position[1] - self.position[1], 2)
                    # Judging whether the forwarding is successful or not
                    if d < pow(Gp.com_dis, 2):
                        # Time delay calculation
                        if pr.ratio(math.sqrt(d)) == 1:
                        # The next hop node receives the packet
                            Gp.success_times += 1
                            node_list[table.next_hop_id].receive_pkt(pkt, node_list, controller, math.sqrt(d))
                            # Change routing entry and group status to delete
                            # print('%d to %d success hop\n%d routing delete' % (self.node_id, table.next_hop_id, self.node_id))
                            self.data_pkt_list.remove(pkt)
                            if self.routing_table.count(table)!=0:
                                self.routing_table.remove(table)
                            break
                        else:
                            Gp.fail_times += 1
                            # Time delay calculation
                            # Routing error occurs, callback routing entry and packet status, send error request
                            print('node %3d to node %3d 距离超过' % (self.node_id, node_list[table.next_hop_id].node_id))
                            if self.routing_table.count(table) != 0:
                                self.routing_table.remove(table)
                            self.generate_error(pkt.source_id, pkt.des_id, self.controller, pkt.seq, node_list)
                            break
                else:
                    Gp.fail_times += 1
                    # 时延计算
                    # Routing error occurs, callback routing entry and packet status, send error request
                    print('node %3d to node %3d 距离超过' % (self.node_id, node_list[table.next_hop_id].node_id))
                    if self.routing_table.count(table) != 0:
                        self.routing_table.remove(table)
                    self.generate_error(pkt.source_id, pkt.des_id, self.controller, pkt.seq, node_list)
                    break
        return


    # Accept forwarded packets
    def receive_pkt(self, data_pkt, node_list, controller, d):
        # Reached the end and forwarded successfully
        data_pkt.delay += (224 + 0.0023629942501200486 + 29.799999999999997)/1000
        data_pkt.delay += ((d)/1000 * 5 + (d)/1000 * 5 *2*0.4169999999999999)/1000

        if data_pkt.des_id == self.node_id:
            data_pkt.e_time = time.time()
            for error in controller.flow_error_list[::-1]:
                if data_pkt.node_id == error.source_id and data_pkt.seq == error.source_seq:
                    controller.flow_error_list.remove(error)
            Gp.success_route += 1
            print('%3d to %3d successful transmission！' % (data_pkt.source_id, data_pkt.des_id))
            Gp.sum = Gp.sum + data_pkt.delay + data_pkt.e_time - data_pkt.s_time
            Gp.total_route_delay.append(data_pkt.e_time - data_pkt.s_time + data_pkt.delay)
            # Total delay calculation
            with open('test/delay.txt', 'a', encoding='utf-8') as f:
                f.write('包端到端时延：{}，包传输时延：{}，总时延：{}\n'.format(data_pkt.delay,data_pkt.e_time - data_pkt.s_time,data_pkt.e_time - data_pkt.s_time + data_pkt.delay))
            f.close()
            Gp.record.append(data_pkt.e_time - data_pkt.s_time + data_pkt.delay)

            # Packet loss rate calculation
            return
        # 自己添加改包 并继续转发

        self.data_pkt_list.append(data_pkt)
        self.forward_pkt_to_nbr(node_list, controller)
        return

