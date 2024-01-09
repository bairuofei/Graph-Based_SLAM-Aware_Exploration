#!/usr/bin/python3
import rospy
from std_msgs.msg import Float64MultiArray, Int32, Int32MultiArray, Int16

from tsp_solver.greedy import solve_tsp
import networkx as nx
import math
import matplotlib.pyplot as plt
import pygraphviz as pgv
import numpy as np
from collections import defaultdict


class PoseGraph:
    """For a given pose graph, select best candidate path by simulating future graph"""
    def __init__(self, graph=None):
        if graph is None:
            self.graph = nx.Graph()
        else:
            self.graph = graph
        self.current_node = -1

    def construct_graph_from_path(self, path):
        for i in range(len(path)-1):
            self.graph.add_node(path[i])
            self.graph.add_edge(path[i], path[i+1])
        self.graph.add_node(path[-1])
        self.current_point = path[-1]

    def expected_graph_with_future_edge(self, edge):
        pass

    def expected_graph_with_future_path(self, path):
        new_graph = self.graph.copy()
        new_pose_graph = PoseGraph(new_graph)
        for i in range(len(path)-1):
            new_pose_graph.graph.add_node(path[i])
            new_pose_graph.graph.add_edge(path[i], path[i+1])
        new_pose_graph.graph.add_node(path[-1])
        return new_pose_graph


class TSP_PLANNER:
    def __init__(self):
        self.prior_graph = self.build_prior_graph()
        D = self.get_distance_matrix(self.prior_graph)
        self.path = solve_tsp(D, endpoints = (21, None))  # Given start and end points
        for i in range(len(self.path)):
            self.path[i] += 1            # For index consistence with cpp graph object

        # rospy.init_node('listener', anonymous=True)  # anonymous represents whether append random string after name or not
        rospy.init_node('tsp_planner')
        rospy.Subscriber("slam_pose_graph", Float64MultiArray, self.handle_cov)
        rospy.Subscriber('request_replan', Int16, self.handle_plan_request)
        self.pubTSP = rospy.Publisher('tsp_path', Int32MultiArray, queue_size=10)

        # Pose graph
        self.vertex_list = []
        self.edge_list = []
        self.worst_cov = np.diag([0.001, 0.001, 1e-6])

        # Index of tsp path after which to adjust
        self.adjust_index = -1

    def build_prior_graph(self, draw = True):
        graph = nx.Graph()
        n = 36
        node_attributes = []
        # Add nodes in form of (node, attribute_dict)
        start_x = -30.0
        start_y = 30.0
        k = 1
        for i in range(6):
            for j in range(6):
                x = start_x + 12 * j
                y =  start_y - 12 * i
                attr = {"pos": (x, y)}
                node_attributes.append((k, attr))
                k += 1
        graph.add_nodes_from(node_attributes)

        width = 6
        for i, _ in node_attributes:
            edge_list = []
            if i + 1 <= n and (i+1)%width != 1:
                edge_list.append((i, i+1))
            if i - 1 >= 1 and (i-1)%width != 0:
                edge_list.append((i, i-1))
            if i + width <= n:
                edge_list.append((i, i+width))
            if i - width >= 1:
                edge_list.append((i, i-width))
            for p, q in edge_list:
                if not graph.has_edge(p, q):
                    weight = math.sqrt((graph.nodes[p]["pos"][0] - graph.nodes[q]["pos"][0]) ** 2
                                    + (graph.nodes[p]["pos"][1] - graph.nodes[q]["pos"][1]) ** 2)
                    graph.add_edge(p, q, weight = weight)
        
        rospy.loginfo('Prior map build successfully!')
        
        if draw:
            dot_graph = nx.nx_agraph.to_agraph(graph)
            dot_graph.write('/home/ruofei/code/cpp/catkin_cpp_ws/src/cpp_solver/results/graph.dot')
        return graph

    def get_distance_matrix(self, graph):
        node_list = list(graph.nodes())
        node_list.sort()
        D = []
        for i in node_list:
            D.append([])
            for j in range(1, i):
                try:
                    dist = nx.shortest_path_length(graph, source=j, target=i, weight="weight", method='dijkstra')
                except nx.NodeNotFound or nx.NetworkXNoPath or nx.ValueError:
                    rospy.logwarn('Error in find Networkx path between %d and %d.', j, i)
                    continue
                D[-1].append(dist)
        return D
        
    def handle_cov(self, data):
        # The pose graph is ranked around existing prior map
        all_data = data.data
        self.vertex_list = []
        vertex_num = int(round(all_data[0]))
        idx = 1
        for i in range(vertex_num):  # Four data: id, x, y, theta
            self.vertex_list.append([])
            self.vertex_list[-1].append(round(all_data[idx + 4 * i]))
            self.vertex_list[-1] += all_data[idx+4*i+1: idx+4*(i+1)]
        idx = 4 * vertex_num + 1
        # Check data structure
        if abs(all_data[idx] + 123.45) > 0.01:
            rospy.logerr("tsp_solver: covariance data structure has error!")
            return
        
        self.edge_list = []
        edge_num = int(round(all_data[idx+1]))
        idx += 1
        for j in range(edge_num):  # Eight data: start, end, six covariance matrix members
            self.edge_list.append([])
            self.edge_list[-1].append(int(round(all_data[idx+8*j])))  # start
            self.edge_list[-1].append(int(round(all_data[idx+8*j+1]))) # end
            self.edge_list[-1] += all_data[idx+8*j+2: idx+8*(j+1)]
        self.__assign_pose_graph_to_prior_graph() # Assign current sigma to prior map

    def __assign_pose_graph_to_prior_graph(self):
        pose_to_edge_idx = defaultdict(list)
        for i, item in enumerate(self.edge_list):
            pose_to_edge_idx[item[0]].append(i)

        prior_node_to_pose = defaultdict(list)  # Group poses around prior map vertexes
        for j, item in enumerate(self.vertex_list):
            closest = -1
            best_dist = float("inf")
            for node in self.prior_graph.nodes():
                x, y = self.prior_graph.nodes[node]["pos"]
                dist = (item[1] - x) ** 2 + (item[2] - y) ** 2
                if dist < best_dist:
                    cloest = node
                    best_dist = dist
            prior_node_to_pose[node].append((j, best_dist))
        
        worst_det = -float("inf")
        for node in self.prior_graph.nodes():
            if not prior_node_to_pose[node]:
                continue
            prior_node_to_pose[node].sort(key = lambda x: x[1], reverse=True)
            vertex_id = prior_node_to_pose[node][0]
            if not pose_to_edge_idx[vertex_id]:
                continue
            edge_id = pose_to_edge_idx[vertex_id][0]
            cov = np.array([[self.edge_list[edge_id][2], self.edge_list[edge_id][3], self.edge_list[edge_id][4]],
                              [self.edge_list[edge_id][3], self.edge_list[edge_id][5], self.edge_list[edge_id][6]],
                              [self.edge_list[edge_id][4], self.edge_list[edge_id][6], self.edge_list[edge_id][7]]])
            determinant = np.linalg.det(cov)
            if abs(determinant) < 1e-10:
                rospy.logerr("Determinant of measurement covariance is 0! From " + str(self.edge_list[edge_id][0]) + " to " + str(self.edge_list[edge_id][1]))
            if determinant > worst_det:
                worst_det = determinant
                self.worst_cov = cov  # self.worst_sigma will be used in D-opt evaluation for unvisited vertex
            self.prior_graph.nodes[node]["cov"] = cov


    def handle_plan_request(self, data):
        cur_idx = data.data   # cur_idx has been visited
        if cur_idx == -1:  # Initial tsp path planning
            msg = Int32MultiArray(data=self.path)
        else: # Replan self.path
            # 1. Get current pose graph
            # 2. Allocate poses in current pose graph to prior map vertexes
            # 3. Adding observability information to prior map
            # 4. Planning over prior map
            if cur_idx == self.adjust_index:
                msg = Int32MultiArray(data=self.path)
            else:
                rospy.loginfo("Receive replanning request. Current index " + str(cur_idx))
                self.__adjust_tsp(cur_idx)
                msg = Int32MultiArray(data=self.path)
                self.adjust_index = cur_idx
        self.pubTSP.publish(msg)


    def compute_D_opt(self, graph):
        edges = list(graph.edges())
        A = nx.incidence_matrix(graph).toarray()  # row corresponds to each node
        AT = A.T

        v_num, e_num = A.shape
        state_dim = 3
        dim = v_num * state_dim
        fisher = np.zeros((dim, dim))
        for i in range(e_num):
            if "cov" in self.prior_graph.nodes[edges[i][1]]:
                cov = self.prior_graph.nodes[edges[i][1]]["cov"]
            else:
                cov = self.worst_cov
            sigma = np.linalg.inv(cov)
            fisher += np.kron(np.outer(A[:, i], AT[i, :]), sigma)

        reduced_fisher = fisher[state_dim:, state_dim:]

        eigvalue, eigenvector = np.linalg.eig(reduced_fisher)
        det_fisher = np.linalg.det(reduced_fisher)
        tradition_D_opt = np.power(det_fisher, 1/len(eigvalue))
        # Compute D-opt
        D_opt = np.exp(np.sum(np.log(eigvalue)) / len(eigvalue))
        # print(f"traditional D opt: {tradition_D_opt}")
        # print(f"Modified D opt   : {D_opt}")
        return np.real(D_opt)

    def __adjust_tsp(self, cur_idx):
        pose_graph = PoseGraph()
        pose_graph.construct_graph_from_path(self.path)  # pose_graph uses same node name as self.prior_map
        curr_d_opt = self.compute_D_opt(pose_graph.graph)
        curr_dist = compute_path_length(self.prior_graph, self.path)

        record = []
        for i in range(cur_idx+1, len(self.path)):  # Loop: i -> j
            curr = self.path[i]
            for j, closure in enumerate(self.path[:cur_idx+1]):  # Only consider closing to already visited vertex
                if (curr, closure) in pose_graph.graph.edges() or (closure, curr) in pose_graph.graph.edges() \
                        or curr == closure:
                    continue
                # Check connectivity, query priro_map
                try:
                    shortest_path = nx.shortest_path(self.prior_graph, source=closure, target=curr, weight="weight", method='dijkstra')
                except nx.NodeNotFound or nx.NetworkXNoPath or nx.ValueError:
                    print("No path between" + str(closure) + " and " +  str(curr))
                    continue
                revisit_length = 0
                for k in range(len(shortest_path) - 1):
                    revisit_length += self.prior_graph.edges[(shortest_path[k], shortest_path[k+1])]["weight"]

                # Uncertainty evaluation
                expected_pg = pose_graph.expected_graph_with_future_path([closure, curr])
                expected_d_opt = self.compute_D_opt(expected_pg.graph)
                expected_d_opt = round(expected_d_opt, 6)
                
                # print(curr, closure, expected_d_opt, revisit_length)
                # Here d_opt / distance is used to rank candidates. i and j is the index of curr and closure in path
                record.append([curr, closure, expected_d_opt, revisit_length, round(expected_d_opt / revisit_length, 6), i, j])

        record.sort(key = lambda x: -x[4])  # Sort according to D-opt / distance
        best_criteria = record[0][4]
        best_action = -1
        for i in range(len(record)):
            if record[i][4] < best_criteria:
                break
            # Will we prefer later loop, or earlier loop?
            # We prefer the earlier loop to help the afterward exploration
            if best_action == -1:
                best_action = i
            elif record[i][5] < record[best_action][5]:
                best_action = i
        
        # Modify tsp plan
        new_path = self.path[:record[best_action][5]] 
        shortest_path = nx.shortest_path(self.prior_graph, source=record[best_action][0], target=record[best_action][1], weight="weight", method='dijkstra')
        new_path += shortest_path
        # Compute new TSP path for remaining unvisited vertex
        if record[best_action][5] == len(self.path) - 1: # curr node is the end of current path
            final_tsp_path = []
            previous_tsp_path = []
        else:
            unvisited_nodes = [record[best_action][1]] + self.path[record[best_action][5]+1:]  # Here current node is included in the planning
            # Here source = 0, means we should use the first node in unvisited_nodes as start
            final_tsp_path = get_tsp_path(self.prior_graph, unvisited_nodes, source=0)  

            # Compare and decide whether to use the previous TSP path or the new one after loop closure
            # Note, multiple modification of path will also change previous active closing. So here by maintaining the following path, we can also check whether 
            # applying TSP planner once more will reduce the connectivity, in which case we can use previous_tsp_path
            connect_path = nx.shortest_path(self.prior_graph, source=record[best_action][1], target=self.path[record[best_action][5]+1], weight="weight", method='dijkstra')
            previous_tsp_path = connect_path[:-1] + self.path[record[best_action][5]+1:]
            # Compare final_tsp_path with previous_tsp_path
            cost1, cost2 = 0, 0
            for i in range(len(final_tsp_path)-1):
                cost1 += self.prior_graph.edges[(final_tsp_path[i], final_tsp_path[i+1])]["weight"]
            for i in range(len(previous_tsp_path)-1):
                cost2 += self.prior_graph.edges[(previous_tsp_path[i], previous_tsp_path[i+1])]["weight"]
            # Compare to decide whether subsequent plan change or not.
            new_path1 = new_path + final_tsp_path[1:]
            new_path2 = new_path + previous_tsp_path[1:]
            modified_pg1 = PoseGraph()
            modified_pg1.construct_graph_from_path(new_path1)
            modified_d_opt1 = self.compute_D_opt(modified_pg1.graph)
            modified_dist1 = compute_path_length(self.prior_graph, new_path1)

            modified_pg2 = PoseGraph()
            modified_pg2.construct_graph_from_path(new_path2)
            modified_d_opt2 = self.compute_D_opt(modified_pg2.graph) # The D-opt is normalized
            modified_dist2 = compute_path_length(self.prior_graph, new_path2)

            # Normalized D-opt itself cannot capture the uncertainty increase caused by longer path.
            # Why divide distance? Dividing distance measures the efforts needs to obtain such information.
            # If same amount information can be obtained by moving shorter distance, then it is better.
            if modified_d_opt1/modified_dist1 >= modified_d_opt2/modified_dist2:
                new_path = new_path1
            else:
                new_path = new_path2

        # Evaluate D-opt after modification
        modified_pg = PoseGraph()
        modified_pg.construct_graph_from_path(new_path)
        modified_d_opt = self.compute_D_opt(modified_pg.graph)
        modified_dist = compute_path_length(self.prior_graph, new_path)
        # print("Prev D-opt: " + str(curr_d_opt) + ", Prev distance: " + str(curr_dist))
        # print("Modified D-opt: " + str(modified_d_opt) + ", Modified distance: " + str(modified_dist))
        if modified_d_opt / modified_dist > curr_d_opt / curr_dist:
            rospy.loginfo("New TSP plan created. D-opt/dist: " + str(round(modified_d_opt / modified_dist, 6)) + " Vs " + str(round(curr_d_opt / curr_dist, 6)))
            self.path = new_path
        else:
            rospy.loginfo("Old TSP plan remains.")


    def main(self):
        rospy.spin()


def get_tsp_path(graph, node_list = [], source=None, end=None):
    """Here source and end are represented by indexes."""
    if not node_list:
        node_list = list(graph.nodes())
        node_list.sort()
    D = []
    for i in range(len(node_list)):
        D.append([])
        target = node_list[i]
        for j in range(i):
            source = node_list[j]
            try:
                dist = nx.shortest_path_length(graph, source=source, target=target, weight="weight", method='dijkstra')
            except nx.NodeNotFound or nx.NetworkXNoPath or nx.ValueError:
                continue
            D[-1].append(dist)

    path = solve_tsp(D, endpoints = (0, None))
    actual_path = []
    for i in range(len(path)-1):
        source = node_list[path[i]]
        target = node_list[path[i+1]]
        try:
            shortest_path = nx.shortest_path(graph, source=source, target=target, weight="weight", method='dijkstra')
        except nx.NodeNotFound or nx.NetworkXNoPath or nx.ValueError:
            continue
        actual_path += shortest_path[:-1] # Do not include the last node repetitively.

    actual_path.append(node_list[path[-1]])
    return actual_path


def compute_path_length(graph, path):
    length = 0
    for i in range(len(path) - 1):
        length += graph.edges[(path[i], path[i+1])]["weight"]
    return length



if __name__ == '__main__':
    tsp_planner = TSP_PLANNER()
    tsp_planner.main()