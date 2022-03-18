# Information update package attributes: id, geographic location, speed, acceleration, current remaining cache
class Hello:
    def __init__(self, node_id, position, velocity, acceleration, current_cache):
        self.node_id = node_id
        self.position = position
        self.velocity = velocity
        self.acceleration = acceleration
        self.current_cache = current_cache


# Route request attributes: source node, destination node, sender node, sender serial number
class FlowRequest:
    def __init__(self, source_id, des_id, node_id, seq):
        self.source_id = source_id
        self.des_id = des_id
        self.seq = seq
        self.node_id = node_id


# Route reply attributes: source node, destination node, route node sequence, route issuer node, route issuer serial number
class FlowReply:
    def __init__(self, source_id, des_id, route, node_id, seq):
        self.source_id = source_id
        self.des_id = des_id
        self.route = route
        self.seq = seq
        self.node_id = node_id

# Route reply attributes: source node, destination node, route node sequence, route issuer node, route issuer serial number
class geo_FlowReply:
    def __init__(self, source_id, des_list, nexthoplist, node_id, seq):
        self.source_id = source_id
        self.des_list = des_list
        self.nexthoplist = nexthoplist
        self.seq = seq
        self.node_id = node_id

# Routing error request Attributes: source node, destination node, error node, number of errors, route issuer serial number, error issuer serial number
class FlowError:
    def __init__(self, source_id, des_id, error_id, time, source_seq, error_seq):
        self.source_id = source_id
        self.des_id = des_id
        self.error_id = error_id
        self.time = time
        self.source_seq = source_seq
        self.error_seq = error_seq


# Data packet attributes: source node, destination node, packet size, status, route sender node, route sender serial number
class DataPkt:
    def __init__(self, source_id, des_id, pkt_size, state, node_id, seq, s_time):
        self.source_id = source_id
        self.des_id = des_id
        self.pkt_size = pkt_size
        self.state = state
        self.seq = seq
        self.node_id = node_id
        self.s_time = s_time
        self.e_time = 0
        self.delay = 0

# Data packet attributes: source node, destination node, packet size, status, route sender node, route sender serial number
class geo_DataPkt:
    def __init__(self, source_id, des_list, pkt_size, state, node_id, seq, s_time):
        self.source_id = source_id
        self.des_list = des_list
        self.pkt_size = pkt_size
        self.state = state
        self.seq = seq
        self.node_id = node_id
        self.s_time = s_time
        self.e_time = 0
        self.delay = 0


class geo_FlowRequest:
    def __init__(self, source_id, des_list, node_id, seq):
        self.source_id = source_id
        self.des_list = des_list
        self.seq = seq
        self.node_id = node_id

# Routing table attributes: source node, destination node, next hop node, status, route issuer node, route issuer serial number
class RoutingTable:
    def __init__(self, source_id, des_id, next_hop_id, state, node_id, seq):
        self.source_id = source_id
        self.des_id = des_id
        self.next_hop_id = next_hop_id
        self.state = state
        self.seq = seq
        self.node_id = node_id

