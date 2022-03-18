import Packet as Pkt
import Global_Par as Gp
import dij_test1 as dij
import networkx as nx
import junction_init as ji
import math as m
import bf_test as bf
import jhmmtg as jh
# import tgeaa as tg
# import HRLB as hr
# import HMMM as hm
import time as t
import random
# import mcds
import numpy as np



class SDVNController:
    def __init__(self, junction_matrix, node_num):
        self.hello_list = []  # hello请求列表
        self.flow_request_list = []  # 路由请求列表
        self.geo_flow_request_list = []
        self.flow_error_list = []  # 错误请求列表
        self.junction_matrix = junction_matrix  # 邻接矩阵
        self.feature_junction_matrix = nx.DiGraph()
        self.node_info_dict = {i: [[], [], [], ] for i in range(node_num)}  # 所有节点信息

    # Update the node information in the controller according to the entries in the hello list
    def predict_position(self):
        for value in self.hello_list:
            self.node_info_dict[value.node_id] = [value.position, value.velocity, value.acceleration, value.current_cache]
        self.hello_list.clear()
        return

    def junction_matrix_construction(self, node_num):
        self.junction_matrix.clear()
        for i in range(0, node_num):
            for j in range(0, i):
                a = pow(self.node_info_dict[i][0][0] - self.node_info_dict[j][0][0], 2) + pow(self.node_info_dict[i][0][1] - self.node_info_dict[j][0][1], 2)
                if a < pow(Gp.com_dis, 2):
                    self.junction_matrix.add_edge(i, j, weight=a, score=0)
                    self.junction_matrix.add_edge(j, i, weight=a, score=0)

    # Record the previous eigenvalues between global nodes
    def feature_junction_matrix_construction(self, node_num):
        self.feature_junction_matrix.clear()
        for i in range(0, node_num):
            for j in range(0, i):
                self.feature_junction_matrix.add_edge(i, j, lat_dist_uv=0, direction_is_same=0, lat_gvalue=0, lat_accel=0, lat_angle=0)
                self.feature_junction_matrix.add_edge(j, i, lat_dist_uv=0, direction_is_same=0, lat_gvalue=0, lat_accel=0, lat_angle=0)

    # Calculate routing based on node information
    def calculate_path(self, x_id, des_id, node_list,node_num, dj):
        # dijkstra
        if dj == 0:
            route = dij.Dijkstra0(self.junction_matrix, x_id, des_id)
        elif dj == 1:
            route = dij.Dijkstra1(self.junction_matrix, x_id, des_id)
        else:
            route = dij.Dijkstra2(self.junction_matrix, x_id, des_id)
        if route:
            print('路由：', route)
            return route
        else:
            print('%d to %d calculation error' % (x_id, des_id))
            return [x_id, des_id]

    # Send a route reply to each node on the route
    @staticmethod
    def send_reply(x_id, des_id, route, node_list, node_id, seq):
        flow_reply = Pkt.FlowReply(x_id, des_id, route, node_id, seq)
        for node_num in route:
            node_list[node_num].receive_flow(flow_reply)
    # Delay processing
        return

    # @staticmethod
    # def geo_send_reply(x_id, des_list, associated_node, next_hop_list, node_list, node_id, seq):
    #     for node in associated_node:
    #         flow_reply = Pkt.geo_FlowReply(x_id, des_list, next_hop_list[node], node_id, seq)
    #         node_list[node].geo_receive_flow(flow_reply)
    #     # 时延处理
    #     return

    # Process each request in the request table, calculate the route, and send a reply
    def resolve_request(self, node_list, dj):
        for request in self.flow_request_list:
            route = self.calculate_path(request.source_id, request.des_id, node_list,len(node_list), dj)
            self.send_reply(request.source_id, request.des_id, route, node_list, request.node_id, request.seq)
        self.flow_request_list.clear()
        return

    # def geo_resolve_request(self, node_list):
    #     for request in self.geo_flow_request_list:
    #         associated_node, next_hop_list = self.geo_calculate_path(request.source_id, request.des_list, node_list)
    #         self.geo_send_reply(request.source_id, request.des_list, associated_node, next_hop_list, node_list, request.node_id, request.seq)
    #     self.geo_flow_request_list.clear()
    #     return

    # Delete routing information (more than three times, you need to delete all related routing information and groups)
    def delete_routing_pkt(self, node_list, source_id, id, seq, des_id):
        # After reaching the destination node, delete the relevant information and return
        if id == des_id:
            for table in node_list[id].routing_table[::-1]:
                if table.seq == seq and table.node_id == source_id:
                    # print('node %d routing delete' % id)
                    node_list[id].routing_table.remove(table)
            for pkt in node_list[id].data_pkt_list[::-1]:
                if pkt.seq == seq and pkt.node_id == source_id:
                    # print('node %d pkt delete' % id)
                    node_list[id].data_pkt_list.remove(pkt)
            return
        # The destination node is not reached, and it is deleted recursively according to the routing table.
        for table in node_list[id].routing_table[::-1]:
            if table.seq == seq and table.node_id == source_id:
                self.delete_routing_pkt(node_list, source_id, table.next_hop_id, seq, des_id)
                # print('node %d routing delete' % id)
                node_list[id].routing_table.remove(table)
        for pkt in node_list[id].data_pkt_list[::-1]:
            if pkt.seq == seq and pkt.node_id == source_id:
                # print('node %d pkt delete' % id)
                node_list[id].data_pkt_list.remove(pkt)

    # Parse error request information
    def resolve_error(self, node_list, dj):
        # Process all nodes in the error request list
        for error in self.flow_error_list[::-1]:
            # If the number of errors in the same hop is greater than N times, this route fails
            if error.time > Gp.re_time:
                # print('%3d to %3d 路由失败 %3d %3d' % (error.error_id, error.des_id, error.source_id, error.source_seq))
                # Delete related routes
                self.delete_routing_pkt(node_list, error.source_id, error.error_id, error.source_seq, error.des_id)
                Gp.fail_route += 1
                # print('source %d seq %d des %d err %d' % (error.source_id, error.source_seq, error.des_id, error.error_id))
                # print('delete\n')
                self.flow_error_list.remove(error)
        # Calculate the route and deliver it downward
        for error1 in self.flow_error_list:
            route = self.calculate_path(error1.error_id, error1.des_id, node_list,len(node_list), dj)
            self.send_reply(error1.error_id, error1.des_id, route, node_list, error1.source_id, error1.source_seq)
        return
