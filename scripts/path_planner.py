#!/usr/bin/python3

import rospy
import os
import sys
import math
import networkx as nx
import numpy as np
from functools import partial
from tsp_solver.greedy import solve_tsp
from collections import defaultdict
import time

from std_msgs.msg import Float64MultiArray, Int32, Int32MultiArray, Int16

from srv import TspPathList, TspPathListResponse, TspPathListRequest, \
    RequestGraph, RequestGraphResponse, RequestGraphRequest, ReliableLoop, ReliableLoopRequest, ReliableLoopResponse
from msg import PoseGraph, EdgeDistance


# Add python path to header file
path = os.path.abspath(".")
sys.path.insert(0, path + "/src/cpp_solver/scripts")

from utils import write_nx_graph_to_g2o, \
    add_edge_information_matrix, add_graph_weights_as_dopt, save_data, \
    get_path_length
import utils

from offline_tsp_evaluation import connect_tsp_path, offline_evaluate_tsp_path, get_distance_matrix_for_tsp,\
    get_distance_matrix_for_submap_tsp, offline_iterative_evaluate_tsp_path, concorde_tsp_solver, \
    get_distance_matrix_for_new_tsp_solver, find_last_loop_closure

from read_drawio_to_nx import build_prior_map_from_drawio



class PathPlanner:
    def __init__(self):
        """ Get intial pose of the robot w.r.t the environment center.
            Step 1: assume the robot pose is [x_r, y_r, z_r, 0] according to .world file in Stage simulator, and 
            floor plan position is [0, 0, 0, 0].
            Step 2: build a prior_graph of the environment with its origin being the center of the environment.
            Step 3: transform all vertices positions of prior_graph from (x_v, y_v, z_v) to (x_v - x_r, y_v - y_r, z_v - z_r).
            Step 4: tsp planning should start from the closest vertex to the robot.
        """
        rospy.init_node('path_planner')

        robot_position_str = rospy.get_param('/path_planner/robot_position')
        pose_list = robot_position_str.split(" ")
        self.robot_position = [float(x) for x in pose_list]
        if len(self.robot_position) < 3:
            rospy.logerr("robot initial position less than 3 parameters.")

        self.xml_path = rospy.get_param('/path_planner/xml_path', '')
        self.map_width = rospy.get_param('/path_planner/map_width', -1)
        self.use_drawio = rospy.get_param('/path_planner/use_drawio', False)
        self.only_use_tsp = rospy.get_param('/path_planner/only_use_tsp', False)
        self.tsp_solver_name = rospy.get_param('/path_planner/tsp_solver', "tsp-solver")  # Or "concorde"
        self.need_noise = rospy.get_param('/path_planner/need_noise', False)
        self.variance = rospy.get_param('/path_planner/variance', 0)

        print(self.xml_path)
        print(self.map_width)
        print(self.use_drawio)

        #******************* Prior Map Construction ******************#
        if self.use_drawio:
            self.prior_graph = self.build_prior_graph_with_xml(need_normalize=False, need_noise=self.need_noise, variance=self.variance)
        else:
            self.prior_graph = self.build_prior_graph()
        self.align_prior_map_with_robot()
        rospy.loginfo('Build prior map.')

        # Make prior map a complete graph
        # self.make_priormap_complete()

        self.update_prior_graph_attributes()

        # Change this paht to your folder
        save_data(self.prior_graph, "/home/ruofei/code/cpp/catkin_cpp_ws/src/cpp_solver/scripts/prior_map.pickle")

        self.solve_tsp_path()
        rospy.loginfo('Find initial tsp path.')

        self.modify_tsp_path()
        rospy.loginfo('Find modified curr_path and loop index')

        plan_service = rospy.Service('path_plan_service', TspPathList, self.handle_replanning)
        graph_service = rospy.Service('prior_graph_service', RequestGraph, self.handle_prior_graph)
        loop_service = rospy.Service('reliable_loop_service', ReliableLoop, self.handle_reliable_loop)
        
        self.pose_graph = nx.Graph()
        self.curr_pg_vertex_idx = 0
        self.curr_pg_edge_idx = 0
        self.pub_pose_idx = rospy.Publisher('pose_graph_idx', Int32MultiArray, queue_size=2)
        self.g2o_save_path = rospy.get_param('/path_planner/g2o_save_path')
        rospy.loginfo('g2o_save_path: {}'.format(self.g2o_save_path))

        rospy.Subscriber("slam_pose_graph", PoseGraph, self.handle_pose_graph)
        rospy.Subscriber("/edge_distance", EdgeDistance, self.handle_edge_distance)

        ## Visited vertices in prior graph
        self.visited_vertices = []
        self.vertices_to_poses = defaultdict(list)

        self.use_find_last_loop = False  # Indicator of whether find_last_loop is used
    
    def build_prior_graph_with_xml(self, need_normalize = False, need_noise=False, variance=0) -> nx.graph:
        """ Node attri:
                name: int, start from 1
                position: tuple(x, y)
            Edge attri:
                weight: float 
        """
        graph = build_prior_map_from_drawio(self.xml_path, actual_map_width=self.map_width, need_normalize=need_normalize, need_noise=need_noise, variance=variance)
        rospy.loginfo(f"Build prior map with {len(graph.nodes())} vertices.")
        return graph

    def build_prior_graph(self) -> nx.graph:
        """ Node attri:
                name: int, start from 1
                position: tuple(x, y)
            Edge attri:
                weight: float 
                information: numpy.array
                d_opt: float
        """
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
                attr = {"position": (x, y)}
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
                    weight = math.sqrt((graph.nodes[p]["position"][0] - graph.nodes[q]["position"][0]) ** 2
                                    + (graph.nodes[p]["position"][1] - graph.nodes[q]["position"][1]) ** 2)
                    graph.add_edge(p, q, weight = weight)
        return graph
    
    def align_prior_map_with_robot(self):
        for node in self.prior_graph.nodes():
            x, y = self.prior_graph.nodes()[node]["position"]
            self.prior_graph.nodes()[node]["position"] = (x - self.robot_position[0], y - self.robot_position[1])
        return
    
    def make_priormap_complete(self):
        all_length = dict(nx.all_pairs_dijkstra_path_length(self.prior_graph, cutoff=None, weight='weight'))
        for node1 in self.prior_graph.nodes():
            for node2 in self.prior_graph.nodes():
                if (node1, node2) not in self.prior_graph.edges():
                    self.prior_graph.add_edge(node1, node2, weight=all_length[node1][node2])
        print(f"8 - 6: {self.prior_graph.edges()[(8, 6)]['weight']}, 8 - 7: {self.prior_graph.edges()[(8, 7)]['weight']}")
        print(f"8 - 3: {self.prior_graph.edges()[(8, 3)]['weight']}, 8 - 9: {self.prior_graph.edges()[(8, 9)]['weight']}")
    
    def update_prior_graph_attributes(self):
        # Add attributes for path planning
        Cov = np.zeros((3, 3))
        Cov[0, 0] = 0.1
        Cov[1, 1] = 0.1
        Cov[2, 2] = 0.001
        Sigma = np.linalg.inv(Cov)
        add_edge_information_matrix(self.prior_graph, Sigma)
        add_graph_weights_as_dopt(self.prior_graph, key="d_opt")
        return
    
    def solve_tsp_path(self):
        """ Set self.tsp_path and self.full_tsp_path """
        ## Find intial TSP path
        D, tsp_node_list = get_distance_matrix_for_tsp(self.prior_graph)
        # Find the closest vertex from prior_map to robot start position
        start_idx = -1
        min_dist = float("inf")
        print(f"Robot position: ({self.robot_position[0]}, {self.robot_position[1]})")
        for i, node in enumerate(tsp_node_list):
            x, y = self.prior_graph.nodes()[node]["position"]
            dist = x ** 2 + y ** 2
            if node == 10 or node == 11:
                print(f"{node}: {dist},(x, y) = ({x}, {y})")
            if dist < min_dist:
                min_dist = dist
                start_idx = i
        rospy.logwarn(f"Robot Start At {tsp_node_list[start_idx]}")

        # Method 1: tsp-solver
        if self.tsp_solver_name == "tsp-solver":
            idx_path = solve_tsp(D, endpoints = (start_idx, None))  # Given start and end points
            print("Tsp problem solved by tsp-solver.")
            self.tsp_path = [tsp_node_list[i] for i in idx_path] 
        else: # Concorde Method
            # First construct node_list for concorde. The first node should be starring node.
            concorde_node_list = [tsp_node_list[start_idx]]   # Indicate start_idx for concorde tsp solver
            for i, node in enumerate(tsp_node_list):
                if i != start_idx:
                    concorde_node_list.append(node)
            distance_matrix = get_distance_matrix_for_new_tsp_solver(self.prior_graph,\
                                                                     node_list=concorde_node_list)
            time1 = time.time()
            new_tsp_path, new_tsp_distance = concorde_tsp_solver(distance_matrix, concorde_node_list)
            tsp_solve_time = time.time() - time1
            rospy.logwarn(f"Tsp problem solved by Concorde. {tsp_solve_time}")
            self.tsp_path = new_tsp_path

        # Here tsp_path does not include repeated visit to vertices, 
        # because the distance matrix is computed for all pairs of vertices
        self.full_tsp_path = connect_tsp_path(self.prior_graph, self.tsp_path)
        # self.full_tsp_path = self.tsp_path
        print("tsp_path VS full_tsp_path:")
        print(self.tsp_path)
        print(self.full_tsp_path)
        return

    def modify_tsp_path(self):
        """ Given initial self.tsp_path, modify it into a well connected path, and get self.is_loop """
        if self.only_use_tsp:
            self.curr_path = self.tsp_path
            self.is_loop = [False for i in range(len(self.tsp_path))]
            return

        results1 = offline_evaluate_tsp_path(self.prior_graph, self.full_tsp_path)
        is_optimized, new_path, loop_vertices, curr_obj_value, curr_d_opt, curr_path_dist, tsp_d_opt, tsp_dist = results1
        self.curr_obj_value = curr_obj_value
        self.curr_d_opt = curr_d_opt
        self.curr_path_dist = curr_path_dist
        self.tsp_d_opt = tsp_d_opt
        self.tsp_dist = tsp_dist
        # loop_vertices: set containing all indices of new_path which corresponds to a loop closing vertex
        if is_optimized:
            self.curr_path = new_path
            self.is_loop = [True if x in loop_vertices else False for x in range(len(new_path))]
        else:
            self.curr_path = self.tsp_path
            self.is_loop = [False for i in range(len(self.tsp_path))]
        return

    
    def handle_replanning(self, request: TspPathListRequest) -> TspPathListResponse:
        rospy.loginfo(f"Replanning request. curr_vertex_idx: {request.curr_vertex_idx}")
        rospy.loginfo(f"                    covered_vertices: {request.covered_vertices}")
        response = TspPathListResponse()

        if self.only_use_tsp:
            response.response = self.curr_path  
            response.isLoop = self.is_loop
            return response
        
        # Note that curr_vertex_idx has been visited, but planning start from curr_vertex_idx
        curr_robot_idx = request.curr_vertex_idx
        covered_vertices = set(request.covered_vertices)

        if not self.is_loop[curr_robot_idx]:
            # Update self.curr_path and self.is_loop inside this function
            rospy.loginfo("Handle local replanning...")
            self.handle_local_replanning(curr_robot_idx)
        elif curr_robot_idx != 0 and curr_robot_idx != len(self.curr_path) - 1:
            # FIXMED: May only have two vertices if curr_robot_idx is the last indices
            # Modify self.curr_path and self.is_loop if better path found
            pass
            # self.modify_existing_tsp_path(curr_robot_idx, covered_vertices)

        response.response = self.curr_path 
        response.isLoop = self.is_loop

        # rospy.loginfo("Test after replanning!!")
        rospy.loginfo(f"curr path length: {len(self.curr_path)}, is_loop length: {len(self.is_loop)}")

        new_path_str = ""
        for i in range(len(self.curr_path)):
            node = self.curr_path[i]
            if self.is_loop[i]:
                new_path_str += str(node) + "(L) "
            else:
                new_path_str += str(node) + " "
            if i == curr_robot_idx:
                new_path_str += " ---> "

        rospy.loginfo("Send new plan: " + new_path_str)
        return response
    
    def handle_local_replanning(self, curr_robot_idx):
        if curr_robot_idx >= len(self.curr_path):
            return
        if curr_robot_idx == len(self.curr_path) - 1:
            rospy.loginfo("Handle find last loop...")
            if self.use_find_last_loop:
                return
            self.find_last_loop(curr_robot_idx)
            self.use_find_last_loop = True
            return

        local_path = []
        start_idx = curr_robot_idx
        # Remove repeat vertices
        added_vertices = set()
        while start_idx < len(self.curr_path) and not self.is_loop[start_idx]:
            if self.curr_path[start_idx] not in added_vertices:
                local_path.append(self.curr_path[start_idx])
                added_vertices.add(self.curr_path[start_idx])
            start_idx += 1
        if start_idx < len(self.curr_path):  
            # If loop closure vertex is also in the local path, do not replan
            if self.is_loop[start_idx] and self.curr_path[start_idx] in added_vertices:
                return
            # Add target vertex
            local_path.append(self.curr_path[start_idx])
        rospy.loginfo("Local path to replan: " + str(local_path))

        if len(local_path) < 3:  # No need to replanning
            return
        
        # Construct distance matrix
        distance_matrix = get_distance_matrix_for_new_tsp_solver(self.prior_graph,\
                                                                 node_list=local_path)
        # Solve tsp problem
        specify_end = False
        if start_idx < len(self.curr_path) and self.is_loop[start_idx]:
            specify_end = True  # Only specify end if it is a loop vertex
        # print(distance_matrix)
        new_local_path, new_local_path_dist = concorde_tsp_solver(distance_matrix, \
                                                                local_path, \
                                                                specify_end = specify_end)
        # Double check the start and end of new_local_path
        if new_local_path[0] != local_path[0] or (specify_end and new_local_path[-1] != local_path[-1]):
            rospy.logerr("Local path replan get wrong start and end vertex! Specify_end: " + str(specify_end))
            rospy.logerr("new_local_path: " + str(new_local_path))
            print(distance_matrix)
            return
        if local_path == new_local_path:   # Local path does not change
            return
        # Compare the distance
        old_path_length = utils.get_path_length(self.prior_graph, local_path, weight="weight")
        new_path_length = utils.get_path_length(self.prior_graph, new_local_path, weight="weight")
        if new_path_length < old_path_length:
            first_piece = self.curr_path[:curr_robot_idx]
            last_piece = []
            if start_idx < len(self.curr_path):
                last_piece = self.curr_path[start_idx:]
            self.curr_path = first_piece + new_local_path + last_piece

            first_loop_piece = self.is_loop[:curr_robot_idx]
            new_loop_piece = [False for _ in range(len(new_local_path))]
            last_loop_piece = []
            if start_idx < len(self.is_loop):
                last_loop_piece = self.is_loop[start_idx:]
            self.is_loop = first_loop_piece + new_loop_piece + last_loop_piece
            rospy.logwarn("New local path obtained!")
        return
    

    def find_last_loop(self, curr_robot_idx):
        """ Find potential loop closure for the last vertex. 
            Update self.curr_path and self.is_loop if closure exists.
        """
        really_visit_vertices = list(self.visited_vertices)
        while(really_visit_vertices and really_visit_vertices[-1] == self.curr_path[curr_robot_idx]):
            really_visit_vertices.pop()
        # Check vertices in visited_vertices_list are fully connected over prior_graph or not
        really_visit_vertices.append(self.curr_path[curr_robot_idx])
        prefix_path = connect_tsp_path(self.prior_graph, really_visit_vertices)

        loop_edge = find_last_loop_closure(self.prior_graph, prefix_path, self.curr_obj_value) 
        # [closure, curr]
        if loop_edge[0] >= 0 and loop_edge[1] >= 0:
            self.curr_path.append(loop_edge[0])
            self.is_loop.append(True)
        return

    
    def update_curr_obj_value(self, curr_real_path):
        tsp_edges = set()
        for i in range(len(curr_real_path) - 1):
            tsp_edges.add((curr_real_path[i], curr_real_path[i+1]))
            tsp_edges.add((curr_real_path[i+1], curr_real_path[i]))
        G_tsp = self.prior_graph.edge_subgraph(tsp_edges).copy()

        # Evalute edge candidates to find maximum radius
        tsp_d_opt = utils.get_normalized_weighted_spanning_trees(G_tsp, weight_type="D-opt", weight="d_opt")
        tsp_dist = utils.get_path_length(self.prior_graph, curr_real_path, weight="weight")
        if tsp_dist < 0:
            print("Error when count TSP path distance!")
            sys.exit(1)

        rospy.loginfo(f"Update curr_obj_value: {self.curr_obj_value}  ---> {tsp_d_opt / tsp_dist} ")
        self.curr_obj_value = tsp_d_opt / tsp_dist
        self.curr_path_dist = tsp_dist
        self.curr_d_opt = tsp_d_opt
        return

    def modify_existing_tsp_path(self, curr_robot_idx: int, covered_vertices: set):
        """ Replanning after closing a loop. 
            Attention: the modified path must keep curr_robot_idx in its place.
            args: 
                covered_vertices: the vertices that have been percepted and have no frontiers
        """
        visited_nodes_on_path = set(self.curr_path[:curr_robot_idx])
        visited_nodes = visited_nodes_on_path.union(covered_vertices)
        # unvisited_nodes for sub tsp planning
        unvisited_nodes = set()
        for node in self.curr_path[curr_robot_idx:]:
            if node not in visited_nodes:
                unvisited_nodes.add(node)
        unvisited_nodes.discard(self.curr_path[curr_robot_idx])
        unvisited_node_list = list(unvisited_nodes)
        # Start from curr_robot_idx, visiting all remaining vertex
        unvisited_node_list = [self.curr_path[curr_robot_idx]] + unvisited_node_list

        # TSP planning
        D_submap = get_distance_matrix_for_submap_tsp(self.prior_graph, unvisited_node_list)
        # Set index of robot position, is always 0 because of the structure of unvisited_node_list
        start_index = unvisited_node_list.index(self.curr_path[curr_robot_idx]) 

        rospy.loginfo("Unvisited node list: " + str(unvisited_node_list))

        if self.tsp_solver_name == "tsp-solver":
            idx_path_sub = solve_tsp(D_submap, endpoints = (start_index, None))  # Given start and end points
            print("Replanning on submap solved by tsp-solver.")
            submap_tsp_path = [unvisited_node_list[i] for i in idx_path_sub]  
        else: # Concorde Method
            # First construct node_list for concorde. The first node should be starring node.
            concorde_node_list = [unvisited_node_list[start_index]]   # Indicate start_idx for concorde tsp solver
            for i, node in enumerate(unvisited_node_list):
                if i != start_index:
                    concorde_node_list.append(node)
            distance_matrix = get_distance_matrix_for_new_tsp_solver(self.prior_graph,\
                                                                     node_list=concorde_node_list)
            submap_tsp_path, new_tsp_distance = concorde_tsp_solver(distance_matrix, concorde_node_list)
            print("Replanning on submap solved by Concorde.")


        full_submap_tsp_path = connect_tsp_path(self.prior_graph, submap_tsp_path)

        new_tsp_path_str = ' '.join(str(item) for item in full_submap_tsp_path)
        prev_tsp_path_str = ' '.join(str(item) for item in self.curr_path[curr_robot_idx:])
        print("New following path (not optimized and fully connected): " + new_tsp_path_str)
        print("Prev following Path (optimized and fully connected): " + prev_tsp_path_str)

        # path_to_evaluate = self.curr_path[:curr_robot_idx] + full_submap_tsp_path
        # Change first part of the path_to_evaluate to really visited vertex.
        really_visit_vertices = list(self.visited_vertices)
        while(really_visit_vertices and really_visit_vertices[-1] == self.curr_path[curr_robot_idx]):
            really_visit_vertices.pop()
        # Check vertices in visited_vertices_list are fully connected over prior_graph or not. Should be fully connected.
        prefix_path = connect_tsp_path(self.prior_graph, really_visit_vertices)
        path_to_evaluate = prefix_path + full_submap_tsp_path  
        # path_length = get_path_length(self.prior_graph, path_to_evaluate, weight="weight")
        try:
            # curr_real_path = prefix_path + self.curr_path[curr_robot_idx:]
            # 1. Remove connect vertices in curr_path[curr_robot_idx:] according to current prior graph
            # 2. Maintain the loop vertices
            curr_path_to_follow = [self.curr_path[curr_robot_idx]]
            for kk in range(curr_robot_idx+1, len(self.curr_path)):
                if self.is_loop[kk] or self.curr_path[kk] not in self.curr_path[curr_robot_idx:]:
                    curr_path_to_follow.append(self.curr_path[kk])
                    continue
                # not loop and has been visited, then check the connectivity
                prev_idx, after_idx = kk - 1, kk + 1
                if prev_idx < 0 or after_idx >= len(self.curr_path):
                    continue
                if (self.curr_path[prev_idx], self.curr_path[after_idx]) not in self.prior_graph.edges():
                    curr_path_to_follow.append(self.curr_path[kk])
                
            curr_real_path = prefix_path + curr_path_to_follow
            self.update_curr_obj_value(curr_real_path)
        except Exception as e:
            print(e)

        # Actually self.curr_obj_value is not used in the iterations
        results2 = offline_iterative_evaluate_tsp_path(self.prior_graph, path_to_evaluate, len(prefix_path), self.curr_obj_value)
        is_optimized2, new_path2, loop_vertices2, curr_obj_value2, curr_d_opt2, curr_path_dist2, tsp_d_opt2, tsp_dist2 = results2
        
        print("            previous   vs  modifed         ")
        print(f"Obj-value: {self.curr_obj_value}  vs  {curr_obj_value2}")
        print(f"D-opt:     {self.curr_d_opt}  vs  {curr_d_opt2}")
        print(f"Distance:  {self.curr_path_dist}  vs  {curr_path_dist2}")
        
        if is_optimized2:
            modified_new_tsp_path_str = ' '.join(str(item) for item in new_path2[len(prefix_path):])
            print("Modified Following Path: " + modified_new_tsp_path_str)

        if curr_obj_value2 < self.curr_obj_value: 
            rospy.loginfo("New path gets smaller objective value, Current tsp path remains.")
            return 

        rospy.loginfo("Path planner finds better tsp path.")

        self.curr_obj_value = curr_obj_value2
        self.curr_path_dist = curr_path_dist2
        self.curr_d_opt = curr_d_opt2

        # send_path is modified to keep index curr_robot_idx align with cpp node
        send_path = self.curr_path[:curr_robot_idx] + new_path2[len(prefix_path):]
        true_loop = [True if x in loop_vertices2 else False for x in range(len(new_path2))]
        self.curr_path = send_path
        self.is_loop = self.is_loop[:curr_robot_idx] + true_loop[len(prefix_path):]
        self.is_loop[curr_robot_idx] = True
        return 

    
    def handle_prior_graph(self, request: RequestGraphRequest) -> RequestGraphResponse:
        """ Server for prior graph request. """
        response = RequestGraphResponse()
        vertices = list(self.prior_graph.nodes())
        x_coords, y_coords = [], []
        edges_start, edges_end = [], []
        for v in vertices:
            x_coords.append(self.prior_graph.nodes()[v]["position"][0])
            y_coords.append(self.prior_graph.nodes()[v]["position"][1])
        for u, v in self.prior_graph.edges():
            edges_start.append(u)
            edges_end.append(v)

        response.vertices = vertices
        response.x_coords = x_coords
        response.y_coords = y_coords
        response.edges_start = edges_start
        response.edges_end = edges_end
        return response
    
    def update_pub_pose_idx(self):
        """ Publish current received pose graph index. """
        msg = Int32MultiArray()
        msg.data = [self.curr_pg_vertex_idx, self.curr_pg_edge_idx]
        self.pub_pose_idx.publish(msg)
        # rospy.loginfo("Update pose graph publish index.")

    def write_posegraph_to_g2o(self):
        write_nx_graph_to_g2o(self.pose_graph, self.g2o_save_path)
        return

    
    def handle_pose_graph(self, msg: PoseGraph):
        """ 1. Receive and update new pose graph to pose_graph
            2. Update really visited_vertices, becuase some vertices in tsp_path will be skipped
            3. Allocate poses to each vertices in prior_map (Update self.vertices_to_poses)
        """
        needed_pose_threshold = 2

        if msg.vertex_start_idx != self.curr_pg_vertex_idx or \
                            msg.edge_start_idx != self.curr_pg_edge_idx:
            rospy.logerr("pose graph update starting index does not match.")
            return
        self.curr_pg_vertex_idx += len(msg.vertices)
        self.curr_pg_edge_idx += len(msg.edges_start)
        # Update index
        self.update_pub_pose_idx()

        newly_visited_vertices = []
        for i in range(len(msg.vertices)):
            pose = msg.vertices[i]
            x, y, theta = msg.vertex_x[i], msg.vertex_y[i], msg.vertex_theta[i]
            self.pose_graph.add_node(pose, pose=(x, y, theta))
            min_dist = float("inf")
            closest_vertex = -1
            for v in self.prior_graph.nodes():
                v_pos = self.prior_graph.nodes()[v]["position"]
                if (v_pos[0] - x)**2 + (v_pos[1] - y)**2 < min_dist:
                    closest_vertex = v
                    min_dist = (v_pos[0] - x)**2 + (v_pos[1] - y)**2
            newly_visited_vertices.append(closest_vertex)
            self.vertices_to_poses[closest_vertex].append(pose)

        count = 0
        prev_v = -1
        for k in range(len(newly_visited_vertices)):
            v = newly_visited_vertices[k]
            if prev_v == -1 or v == prev_v:
                prev_v = v
                count += 1
            else:  # v != prev_v
                if count > needed_pose_threshold:
                    if len(self.visited_vertices) == 0 or self.visited_vertices[-1] != prev_v:
                        self.visited_vertices.append(prev_v)
                prev_v = v
                count = 1
        if count > needed_pose_threshold:
            if len(self.visited_vertices) == 0 or self.visited_vertices[-1] != prev_v:
                self.visited_vertices.append(prev_v)
        visited_str = ""
        for v in self.visited_vertices:
            visited_str += str(v) + " "
        rospy.loginfo("Really visited vertices: " + visited_str)

        for i in range(len(msg.edges_start)):
            v_start, v_end = msg.edges_start[i], msg.edges_end[i]
            covariance_str = msg.covariance[i]
            cov_split = covariance_str.split(" ")
            covariance = np.array([[float(cov_split[0]), float(cov_split[1]), float(cov_split[2])], 
                                   [float(cov_split[1]), float(cov_split[3]), float(cov_split[4])], 
                                   [float(cov_split[2]), float(cov_split[4]), float(cov_split[5])]])   
            self.pose_graph.add_edge(v_start, v_end, covariance=covariance)
    
        rospy.logdebug("pose graph update: %d vertices, %d edges", self.pose_graph.number_of_nodes(), 
                                                                  self.pose_graph.number_of_edges())
        self.write_posegraph_to_g2o()
        # self.update_prior_graph()
        return
    
    def update_prior_graph(self):
        """ Add edges into prior graph according to really visited vertices """
        for i in range(len(self.visited_vertices) - 1):
            vertex1 = self.visited_vertices[i]
            vertex2 = self.visited_vertices[i+1]
            if (vertex1, vertex2) not in self.prior_graph.edges():
                weight = math.sqrt((self.prior_graph.nodes[vertex1]["position"][0] - self.prior_graph.nodes[vertex2]["position"][0])**2 \
                                   + (self.prior_graph.nodes[vertex1]["position"][1] - self.prior_graph.nodes[vertex2]["position"][1])**2)
                neighbors = [n for n in self.prior_graph.neighbors(vertex1)]
                information = self.prior_graph.edges()[(vertex1, neighbors[0])]["information"]
                d_opt = self.prior_graph.edges()[(vertex1, neighbors[0])]["d_opt"]
                self.prior_graph.add_edge(vertex1, vertex2, weight=weight, information=information, d_opt=d_opt)
                rospy.loginfo(f"New edge added into prior_map from {vertex1} to {vertex2}")
        return

    def handle_reliable_loop(self, request: ReliableLoopRequest) -> ReliableLoopResponse:
        map_vertex = request.goal_vertex
        response = ReliableLoopResponse()
        # Find a list of poses around map_vertex, and send back to the client
        if map_vertex not in self.prior_graph.nodes():
            rospy.logerr("Vertex requesting reliable loop is not in prior_graph!")
            return response
        vertex_x, vertex_y = self.prior_graph.nodes()[map_vertex]["position"]
        min_dist = float("inf")
        closest_pose_idx = -1
        if len(self.vertices_to_poses[map_vertex]) == 0:
            return response
        for i in range(len(self.vertices_to_poses[map_vertex])):
            pose = self.vertices_to_poses[map_vertex][i]
            pose_x, pose_y, _ = self.pose_graph.nodes()[pose]["pose"]
            dist = (vertex_x - pose_x)**2 + (vertex_y - pose_y)**2
            if dist < min_dist:
                closest_pose_idx = i
                min_dist = dist
        # Add 7 poses after closest_pose_idx
        for k in range(closest_pose_idx, min(closest_pose_idx+7, len(self.vertices_to_poses[map_vertex]))):
            pose = self.vertices_to_poses[map_vertex][k]
            pose_x, pose_y, _ = self.pose_graph.nodes()[pose]["pose"]
            response.loop_x_coords.append(pose_x)
            response.loop_y_coords.append(pose_y)
        return response

    def handle_edge_distance(self, msg: EdgeDistance):
        """ Update edge distance of prior map. """
        try:
            shortest_distance = nx.shortest_path_length(self.prior_graph, source=msg.vertex1, target=msg.vertex2, weight='weight')
            if msg.distance < shortest_distance:
                self.prior_graph.add_edge(msg.vertex1, msg.vertex2, weight=msg.distance)
        except nx.NetworkXNoPath:
            print("No path exists between the source and target nodes.")
        # rospy.loginfo(f"Add edge: ({msg.vertex1}, {msg.vertex2}), distance: {msg.distance}")
        return

if __name__ == "__main__":
    path_planner = PathPlanner()
    while not rospy.is_shutdown():
        rospy.spin()
    rospy.loginfo(f"Pose graph has: {path_planner.curr_pg_vertex_idx} vertices, {path_planner.curr_pg_edge_idx} edges.")

