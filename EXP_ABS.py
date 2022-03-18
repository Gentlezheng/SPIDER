'''
This class is used to store public functions, etc.
'''
import numpy as np
import Global_Par as glp


def distance(position1, position2):
    dist = np.sqrt(sum((np.array(position1) - np.array(position2)) ** 2))
    return dist


def direction(self, direction1, direction2):
    re = False
    if direction1 == direction2:
        re = True
    return re


def gvlaue(node1, RG):
    weight = 0
    for node1, node2 in RG.edges(node1):    # node1 的度与权重
        weight += RG[node1][node2]['weight']
    D = RG.degree(node1)
    return (D / 2) / weight


def gvlaue2(node):
    weight = 0
    # print('--------')
    for neighbor in node.IMN:    # node1 的度与权重
        # print(neighbor.node_id)
        # print(neighbor.position, node.position, node.IMN_LIST)
        weight += distance(neighbor.position, node.position)
    D = len(node.IMN)
    return (D / 2) / weight


class ExpAbs:

    def distance(self, position1, position2):
        dist = np.sqrt(sum((np.array(position1)-np.array(position2))**2))
        return dist

    def direction(self, direction1, direction2):
        re = False
        if direction1 == direction2:
            re = True
        return re

    def com_distance(self, node_list, com_node_list):
        # Calculate the list of the distance between the source node and the destination node in the communication node list
        com_distance = []
        # print(com_node_list)
        for i in range(len(com_node_list)):
            com_distance.append(
                self.distance(node_list[com_node_list[:][i][0]].position, node_list[com_node_list[:][i][1]].position))

        return com_distance

