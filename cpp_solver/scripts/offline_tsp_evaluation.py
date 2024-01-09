#!/usr/bin/python3
'''
Author: ruofei_ntu 991609404@qq.com
Date: 2023-05-31 14:13:14
LastEditors: ruofei_ntu 991609404@qq.com
LastEditTime: 2024-01-09 19:38:26
FilePath: /Graph-Based_SLAM-Aware_Exploration/scripts/offline_tsp_evaluation.py
Description: This file is used to create random prior graph for offline evaluation of active looping strategies

Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
'''
import rospy
import networkx as nx
import matplotlib.pyplot as plt
import random
import sys
import numpy as np
import math
import time

from tsp_solver.greedy import solve_tsp
from scipy.spatial import KDTree

# Concorde TSP solver
from concorde import Problem, run_concorde  

import utils

from typing import Tuple, List

## Graph manipulation
def add_weight_attr_to_graph(graph, add_random=False):
    for edge in graph.edges():
        node1, node2 = edge
        pose1, pose2 = graph.nodes()[node1]["position"], graph.nodes()[node2]["position"]
        if add_random:
            # FIXME: Only change weight and do not change position of vertices is wrong w.r.t KD-tree search
            graph.edges()[edge]["weight"] = 0.6 + random.randint(0, 6) * 0.1
        else:
            dist = math.sqrt((pose1[0] - pose2[0]) ** 2 + (pose1[1] - pose2[1]) ** 2)
            graph.edges()[edge]["weight"] = dist


def add_position_attr_to_graph(graph, scale=1):
    """ Only applicable for nodes with tuple (x, y) as their names"""
    for node in graph.nodes():
        graph.nodes()[node]["position"] = (node[0] * scale, node[1] * scale)


def random_remove_nodes(graph, num = 5):
    """ Remove num of nodes from graph while keeping graph connected. """
    node_list = list(graph.nodes())
    if len(node_list) < 2*num:
        print("Very few nodes in graph, cannot remove.")
        sys.exit(1)
    removed = set()
    while num != 0:
        idx = random.randint(0, len(node_list) - 1)
        if idx in removed:
            continue
        copy_graph = graph.copy()
        copy_graph.remove_node(node_list[idx])
        if nx.is_connected(copy_graph):
            graph.remove_node(node_list[idx])
            num -= 1
            removed.add(idx)
    return 


def random_remove_edges(graph, num=5):
    edge_list = list(graph.edges())
    if len(edge_list) < 2 * num:
        print("Very few edges in graph, cannot remove.")
        sys.exit(1)
    removed = set()
    while num != 0:
        idx = random.randint(0, len(edge_list) - 1)
        if idx in removed:
            continue
        copy_graph = graph.copy()
        copy_graph.remove_edge(edge_list[idx][0], edge_list[idx][1])
        if nx.is_connected(copy_graph):
            graph.remove_edge(edge_list[idx][0], edge_list[idx][1])
            num -= 1
            removed.add(idx)
    return 


def calculate_euclidean_edge_distance(graph, edge):
    pos1 = graph.nodes()[edge[0]]["position"]
    pos2 = graph.nodes()[edge[1]]["position"]
    return math.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2)


def calculate_edge_distance_over_priormap(graph, edge):
    closure, curr = edge
    edge_length = -1
    try:
        edge_length = nx.shortest_path_length(graph, source=closure, target=curr, weight="weight", method='dijkstra')
    except nx.NodeNotFound or nx.NetworkXNoPath or nx.ValueError:
        print(f"No path between {closure} and {curr}.")
    return edge_length


# Graph plot
def plot_nx_gridmap(graph, path = [], closures = []):
    path_edge_set = set()
    if path:
        for i in range(len(path) - 1):
            path_edge_set.add((path[i], path[i+1]))

    fig, ax = plt.subplots()
    for node in graph.nodes():
        x, y = node
        ax.plot(x, y, 'o', markersize=2, color='g', alpha = 0.7)
    for edge in graph.edges():
        node1, node2 = edge
        if (node1, node2) in path_edge_set or (node2, node1) in path_edge_set:
            ax.plot([node1[0], node2[0]], [node1[1], node2[1]], '-', color='r', alpha = 0.5, zorder=5)
        else:
            ax.plot([node1[0], node2[0]], [node1[1], node2[1]], '-', color='gray', alpha = 0.2)
    for node1, node2 in closures:
        ax.plot([node1[0], node2[0]], [node1[1], node2[1]], '-', color='b', alpha = 0.7, zorder=5)
    plt.axis('equal')
    plt.show()


def plot_modified_path(graph: nx.graph, path: list = [], closures: list = [], savefig: bool =False):
    path_edge_set = set()
    if path:
        for i in range(len(path) - 1):
            path_edge_set.add((path[i], path[i+1]))

    fig, axes = plt.subplots(1, 2)
    for node in graph.nodes():
        x, y = graph.nodes()[node]["position"]
        axes[0].plot(x, y, 'o', markersize=2, color='g', alpha = 0.7)
        axes[1].plot(x, y, 'o', markersize=2, color='g', alpha = 0.7)
    for edge in graph.edges():
        node1, node2 = edge
        x1, y1 = graph.nodes()[node1]["position"]
        x2, y2 = graph.nodes()[node2]["position"]
        if (node1, node2) in path_edge_set or (node2, node1) in path_edge_set:
            axes[0].plot([x1, x2], [y1, y2], '-', color='r', alpha = 0.5, zorder=5)
            axes[1].plot([x1, x2], [y1, y2], '-', color='r', alpha = 0.5, zorder=5)
        else:
            axes[0].plot([x1, x2], [y1, y2], '-', color='gray', alpha = 0.2)
            axes[1].plot([x1, x2], [y1, y2], '-', color='gray', alpha = 0.2)
    for node1, node2 in closures:
        x1, y1 = graph.nodes()[node1]["position"]
        x2, y2 = graph.nodes()[node2]["position"]
        axes[1].plot([x1, x2], [y1, y2], '-', color='b', alpha = 0.7, zorder=5)
    axes[0].set_aspect('equal')
    axes[1].set_aspect('equal')
    if savefig:
        prefix = time.time()
        plt.savefig("/home/ruofei/code/cpp/catkin_cpp_ws/src/cpp_solver/results/ros_tsp_path.pdf")
    # plt.axis('equal')
    else:
        plt.show()


# TSP Planner
def get_distance_matrix_for_tsp(graph: nx.graph, use_dijsktra_edge=True) -> Tuple[list, list]:
    """Given a networkx graph object, return the distance matrix for tsp-solver.
    node_list is also returned to provide index of nodes in tsp-solver.
    D = [[],
         [0-1],
         [0-2, 1-2],
         [0-3, 1-3, 2-3],
         ...]
    """
    node_list = list(graph.nodes())
    all_length = dict(nx.all_pairs_dijkstra_path_length(graph, cutoff=None, weight='weight'))
    D = []
    for i, curr in enumerate(node_list):
        D.append([])
        for j in range(i):
            prev = node_list[j]          
            if use_dijsktra_edge:  # To effectively consider the distance
                dist = all_length[prev][curr]
            else:
                if ((prev, curr) in graph.edges()) or ((curr, prev) in graph.edges()):
                    dist = graph.edges()[(prev, curr)]["weight"]
                else:
                    dist = float("inf")
            D[-1].append(dist)
    return D, node_list



def get_distance_matrix_for_submap_tsp(graph: nx.Graph, node_list: list, use_dijsktra_edge: bool=True):
    """ Given a list of nodes and a graph, calculate distance matrix between these nodes.
        The node_list should be provided beforeahead.
    """
    all_length = dict(nx.all_pairs_dijkstra_path_length(graph, cutoff=None, weight='weight'))
    D = []
    for i, curr in enumerate(node_list):
        D.append([])
        for j in range(i):
            prev = node_list[j]          
            if use_dijsktra_edge:  # To effectively consider the distance
                dist = all_length[prev][curr]
            else:
                if ((prev, curr) in graph.edges()) or ((curr, prev) in graph.edges()):
                    dist = graph.edges()[(prev, curr)]["weight"]
                else:
                    dist = float("inf")
            D[-1].append(dist)
    return D

def get_distance_matrix_for_new_tsp_solver(graph: nx.graph, node_list: list, use_dijsktra_edge=True) -> np.array:
    """ Return a symmetric distance matrix for vertex in node_list, maintaining its order.
    """
    all_length = dict(nx.all_pairs_dijkstra_path_length(graph, cutoff=None, weight='weight'))
    dist_matrix = np.zeros((len(node_list), len(node_list)))
    for i, curr in enumerate(node_list):
        for j in range(i):
            prev = node_list[j]          
            if use_dijsktra_edge:  # To effectively consider the distance
                dist = all_length[prev][curr]
            else:
                if ((prev, curr) in graph.edges()) or ((curr, prev) in graph.edges()):
                    dist = graph.edges()[(prev, curr)]["weight"]
                else:
                    dist = float("inf")
            dist_matrix[i][j] = dist
    symmetric_matrix = dist_matrix + dist_matrix.T
    return symmetric_matrix

def connect_tsp_path(graph, tsp_path):
    """ The TSP path has some vertices not connected directly. Connect these vertices over g_prior """
    path = []
    for i in range(len(tsp_path) - 1):
        curr = tsp_path[i]
        next = tsp_path[i+1]
        if (curr, next) in graph.edges() or (next, curr) in graph.edges():
            path.append(curr)
        else:
            try:
                connect_path = nx.shortest_path(graph, source=curr, target=next, weight="weight", method='dijkstra')
                # print(curr)
                # print(next)
                # print(connect_path)
                path += connect_path[:-1]
            except nx.NodeNotFound or nx.NetworkXNoPath or nx.ValueError:
                print("Cannot connect tsp_path!")
    path.append(tsp_path[-1])
    return path


# Graph connectivity evaluation
def calculate_resistance_distance(cholesky_factor, a):
    """ Given cholesky_factor of a matrix L, and a is a vector
        Return resistance distance: delta = a.T * L^-1 * a.
        Note: both inputs should refer to reduced Laplacian matrix.
    """
    y = np.linalg.solve(cholesky_factor, a)
    # Solve L * x = y
    x = np.linalg.solve(cholesky_factor.T, y)
    # Compute a^T * L^-1 * a
    return np.dot(a.T, x)[0][0]


def cholesky_rank_one_update(cholesky_factor, a, weight):
    reduced_L = np.dot(cholesky_factor, cholesky_factor.T)
    reduced_L += weight * np.dot(a, a.T)
    return np.linalg.cholesky(reduced_L)


def greedy_tsp_update(tsp_d_opt: float, tsp_dist: float, G_laplacian: np.array, laplacian_inverse: np.array, 
                      ranking_candidates: list, removed_closures: set, valid_closures: set, tsp_node_idx: dict, edge_d_opt: float):
    n = G_laplacian.shape[0]
    nn = n - 1

    # Record greedy history
    add_edge_dist_count = 0   # Bound the iterations by tsp_dist
    record_candidate_num = []  # Record fine-filtering count
    selected_edges = []       # Record all selected edges

    # Record incremental ratio
    ratio_all = []
    ratio_dopt = []
    ratio_dist = []

    curr_path_dist = tsp_dist
    curr_d_opt = tsp_d_opt
    prev_obj_value = tsp_d_opt / tsp_dist
    curr_obj_value = tsp_d_opt / tsp_dist
    
    loop_closed_vertices = set()
    iter = 0
    while True:
        ## New iteration:
        if len(valid_closures) == 0:
            print("No loop closure candidates exist. Algorithm stops.")
            break
        # Recompute for all closures?
        if iter == 0:
            new_candidates = ranking_candidates
        else:  # Update d_opt for edges in valid_closures
            new_candidates = []
            for candidate in ranking_candidates:
                closure, curr = candidate[0]
                if curr in loop_closed_vertices:
                    continue
                n1_idx, n2_idx = tsp_node_idx[closure], tsp_node_idx[curr]
                a = np.zeros((n, 1))
                a[n1_idx, 0], a[n2_idx, 0] = 1, -1
                # edge_d_opt = G_tsp.nodes()[closure]["d_opt"]
                ## Directly use inverse is quicker than cholesky update, and eigen decomposition. Hundred of times faster.
                curr_resistance = edge_d_opt * np.dot(np.dot(a[1:, :].T, laplacian_inverse), a[1:, :])[0, 0]
                ## Fine-filtering
                normalized_resistance = (1 + curr_resistance)**(1/nn)
                closure_length = candidate[2]
                if normalized_resistance / (1 + closure_length / curr_path_dist) <= 1:   ## Modified fine filtering
                    removed_closures.add((closure, curr))
                    valid_closures.discard((closure, curr))
                    valid_closures.discard((curr, closure))
                    continue
                ## Pass fine-filtering
                # Compute object_value if adding current candadate in this iteration
                object_value = curr_d_opt * normalized_resistance / (curr_path_dist + 2 * closure_length)
                new_candidates.append([(closure, curr), normalized_resistance, closure_length, object_value, curr_d_opt, curr_path_dist, iter])
            new_candidates.sort(key=lambda x: x[3])
        
        record_candidate_num.append(len(new_candidates))

        if len(new_candidates) == 0:
            print("No candidate is better than current_d_opt. Iteration stops.")
            break

        best_edge = new_candidates.pop()
        removed_closures.add(best_edge[0])
        loop_closed_vertices.add(best_edge[0][1])
        alpha = 1        # alpha is used for balancing graph enhancement and exploration efficiency
        if best_edge[3] > alpha * curr_d_opt / curr_path_dist:
            selected_edges.append(best_edge[0])
            closure, curr = best_edge[0]
            n1_idx, n2_idx = tsp_node_idx[closure], tsp_node_idx[curr]
            a = np.zeros((n, 1))
            a[n1_idx, 0], a[n2_idx, 0] = 1, -1
            # edge_d_opt = G_tsp.nodes()[closure]["d_opt"]
            G_laplacian = G_laplacian + edge_d_opt * np.dot(a, a.T)  # Forget to add edge_d_opt, the resulting graph tends to be dense
            laplacian_inverse = np.linalg.inv(G_laplacian[1:, 1:])

            # For print
            d_opt_incremental = best_edge[1]
            path_dist_incremental = 1 + 2 * best_edge[2] / curr_path_dist
            
            curr_path_dist = curr_path_dist + 2 * best_edge[2]
            curr_d_opt = curr_d_opt * best_edge[1]
            curr_obj_value = curr_d_opt / curr_path_dist
            
            ratio_all.append(d_opt_incremental / path_dist_incremental)
            ratio_dopt.append(d_opt_incremental)
            ratio_dist.append(path_dist_incremental)

            prev_obj_value = curr_obj_value
            add_edge_dist_count += 2 * best_edge[2]
            if add_edge_dist_count > tsp_dist:
                print("Exceeds the distance of initial TSP path.")
                break
            ranking_candidates = new_candidates   

            if iter % 50 == 0:
                print(f"#candidates: {len(new_candidates)}")
                print(f"current obj: {curr_d_opt / curr_path_dist}, resistance: {d_opt_incremental}, dis_incre: {path_dist_incremental} tsp_obj: {tsp_d_opt / tsp_dist}")
                print(f"Optimization ratio: {curr_obj_value / prev_obj_value}")

        else:
            print("No candidate is better than current_d_opt. Iteration stops.")
            break
        iter += 1

    return selected_edges, curr_d_opt, curr_path_dist, curr_obj_value, ratio_all, ratio_dopt, ratio_dist


def offline_evaluate_tsp_path(prior_map: nx.Graph, full_tsp_path: list):
    """ Given a full_tsp_path over prior_map, evaluate its potential in achieving better graph connectivity and disatnce.
        Input: 
            gridmap, full_tsp_path
    """
    debug = True
    ## Select best edge for one step
    tsp_edges = set()
    for i in range(len(full_tsp_path) - 1):
        tsp_edges.add((full_tsp_path[i], full_tsp_path[i+1]))
        tsp_edges.add((full_tsp_path[i+1], full_tsp_path[i]))
    G_tsp = prior_map.edge_subgraph(tsp_edges).copy()
    
    tsp_d_opt = utils.get_normalized_weighted_spanning_trees(G_tsp, weight_type="D-opt", weight="d_opt")
    tsp_dist = utils.get_path_length(prior_map, full_tsp_path, weight="weight")
    if debug: print(f"tsp_d_opt: {tsp_d_opt}, tsp_dist: {tsp_dist}")
    if tsp_dist < 0:
        print("Error when count TSP path distance!")
        sys.exit(1)

    time1 = time.time()
    
    ## Find one edge for maximal D-opt increment, compute omega_max
    G_tsp_node_list = list(G_tsp.nodes())
    G_tsp_edge_list = set(G_tsp.edges())
    tsp_node_idx = {}
    for idx, node in enumerate(G_tsp_node_list):
        tsp_node_idx[node] = idx
    n = len(G_tsp_node_list)
    nn = n - 1
    G_tsp_laplacian = nx.laplacian_matrix(G_tsp, weight="d_opt").toarray()
    G_tsp_laplacian_inverse = np.linalg.inv(G_tsp_laplacian[1:, 1:])
    a = np.zeros((n, 1))  # For edge incidence vector
    # TODO:  covariance estimation
    Cov = np.zeros((3, 3))
    Cov[0, 0] = 0.1
    Cov[1, 1] = 0.1
    Cov[2, 2] = 0.001
    Sigma = np.linalg.inv(Cov)
    edge_d_opt = utils.get_d_opt(Sigma)

    visited_edges = set()
    max_resistance = -1
    for i, curr in enumerate(full_tsp_path):
        if i % 50 == 0:
            print(f"current idx: {i} / {len(full_tsp_path) - 1}")      
        for closure in full_tsp_path[:i]:   # search in its previous vertex
            if (curr, closure) in G_tsp_edge_list or (closure, curr) in G_tsp_edge_list \
                    or curr == closure or (curr, closure) in visited_edges or (closure, curr) in visited_edges:
                continue
            visited_edges.add((closure, curr))
            n1_idx = tsp_node_idx[closure]
            n2_idx = tsp_node_idx[curr]   
            a[n1_idx, 0] = 1
            a[n2_idx, 0] = -1
            # edge_d_opt = G_tsp.nodes()[closure]["d_opt"]
            curr_resistance = edge_d_opt * np.dot(np.dot(a[1:, :].T, G_tsp_laplacian_inverse), a[1:, :])[0, 0]
            max_resistance = max(max_resistance, curr_resistance)
            a[n1_idx, 0] = 0
            a[n2_idx, 0] = 0

    max_d_opt = tsp_d_opt * (1 + max_resistance)**(1/nn)
    max_candidate_dist = (max_d_opt / tsp_d_opt - 1) * tsp_dist  # For coarse filtering

    if debug: print(f"max_candidate_dist: {max_candidate_dist}")

    time2 = time.time()

    ## Build kd-tree for neighbor finding
    points = []
    points_to_vertex = {}
    for node in G_tsp.nodes():
        points.append(G_tsp.nodes()[node]["position"])
        points_to_vertex[G_tsp.nodes()[node]["position"]] = node
        
    kdtree = KDTree(points)  # Create a KDTree
    radius = max_candidate_dist

    ## Offline greedy search
    max_edge_dist = -1
    min_edge_dist = float("inf")
    
    full_tsp_node_idx = {}
    for i, node in enumerate(full_tsp_path):
        full_tsp_node_idx[node] = i

    ## Construct candidate loop closures 
    valid_closures = set()    # Record closures pass the fine-filtering
    removed_closures = set()  # Record closures fail in fine-filtering
    ranking_candidates = []   # Maintain all valid loop closures
    for i, curr in enumerate(full_tsp_path):
        if i % 20 == 0:
            print(f"Iter {iter}: {i} / {len(full_tsp_path)}")
        ## Coarse-filtering, radius = max_candidate_dist
        neighbor_indices = kdtree.query_ball_point(G_tsp.nodes()[curr]["position"], radius)
        neighbors = [points[i] for i in neighbor_indices]
        for closure_position in neighbors:  
            closure = points_to_vertex[closure_position]
            if (curr, closure) in G_tsp.edges() or (closure, curr) in G_tsp.edges() or curr == closure:
                continue
            if (curr, closure) in removed_closures or (closure, curr) in removed_closures:
                continue
            if (curr, closure) in valid_closures or (closure, curr) in valid_closures:
                continue   
            # Fix curr-closure vs closure-curr by only considering the last same node
            if full_tsp_node_idx[curr] > full_tsp_node_idx[closure]:
                curr_loop = (closure, curr)
            else:
                # curr_loop = (curr, closure)
                continue
            valid_closures.add(curr_loop)
            # closure_length = calculate_euclidean_edge_distance(gridmap, [closure, curr])
            closure_length = calculate_edge_distance_over_priormap(prior_map, [closure, curr])
            max_edge_dist = max(max_edge_dist, closure_length)
            min_edge_dist = min(min_edge_dist, closure_length)
            # Calculate resistance distance
            n1_idx, n2_idx = tsp_node_idx[closure], tsp_node_idx[curr]
            a = np.zeros((n, 1))
            a[n1_idx, 0], a[n2_idx, 0] = 1, -1
            # edge_d_opt = G_tsp.nodes()[closure]["d_opt"]
            curr_resistance = edge_d_opt * np.dot(np.dot(a[1:, :].T, G_tsp_laplacian_inverse), a[1:, :])[0, 0]
            normalized_resistance = (1 + curr_resistance)**(1/nn)
            ## Fine-filtering
            if normalized_resistance / (1 + closure_length / tsp_dist) <= 1:
                removed_closures.add((closure, curr))
                valid_closures.discard((closure, curr))
                valid_closures.discard((curr, closure))
                continue
            ## Pass fine-filtering
            object_value = tsp_d_opt * normalized_resistance / (tsp_dist + 2 * closure_length)
            ranking_candidates.append([curr_loop, normalized_resistance, closure_length, object_value, tsp_d_opt, tsp_dist, iter])  
    ranking_candidates.sort(key=lambda x: x[3])

    # Greedy TSP path update
    results = greedy_tsp_update(tsp_d_opt, 
                      tsp_dist, 
                      G_laplacian=G_tsp_laplacian, 
                      laplacian_inverse=G_tsp_laplacian_inverse,
                      ranking_candidates=ranking_candidates,
                      removed_closures=removed_closures,
                      valid_closures=valid_closures,
                      tsp_node_idx=tsp_node_idx,
                      edge_d_opt=edge_d_opt)
    
    selected_edges = results[0]
    curr_d_opt = results[1]
    curr_path_dist = results[2]
    curr_obj_value = results[3]
    
    print(f"#Add edges: {len(selected_edges)}, {str(selected_edges)}")
    
    is_optimized = False
    if len(selected_edges) > 0:
        is_optimized = True
    if not is_optimized:
        return is_optimized, [], set(), curr_obj_value, curr_d_opt, curr_path_dist, tsp_d_opt, tsp_dist

    time3 = time.time()
    print(f"Time cost: {round(time2 - time1, 2)}s for best one; {round(time3 - time2, 2)}s for all.")
    print("  ")

    # plot_modified_path(prior_map, full_tsp_path, closures=selected_edges, savefig=False)

    ## Reconstruct the TSP path by adding selected loop closures
    selected_edges.sort(key=lambda x: full_tsp_node_idx[x[1]])
    curr_idx = 0
    new_path = []
    # Indices of loop vertices in new_path
    loop_vertices = set()
    for i, node in enumerate(full_tsp_path):
        new_path.append(node)
        # In selected_edges, each edge is represented by: closure_point -- curr_point
        if curr_idx < len(selected_edges) and node == selected_edges[curr_idx][1]: 
            new_path.append(selected_edges[curr_idx][0])  # Add closure_point into new_path
            loop_vertices.add(len(new_path) - 1)
            if i < len(full_tsp_path) - 1:
                new_path.append(node)
            curr_idx += 1

    print("optimized tsp path: "+ str(new_path))
    print("non-optimized tsp path: " + str(full_tsp_path))
    return is_optimized, new_path, loop_vertices, curr_obj_value, curr_d_opt, curr_path_dist, tsp_d_opt, tsp_dist


def offline_iterative_evaluate_tsp_path(prior_map: nx.Graph, full_tsp_path: list, robot_curr_idx: int, exist_obj_value: float):
    """ Given a tsp path over a prior graph, and an existing graph, find potential loop closures
        for the path with maximum objective value.
        
        Only difference: candidates comes only from full_tsp_path
    """
    time1 = time.time()
    
    # Build G_tsp = full_tsp_path + pose_graph
    tsp_edges = set()
    for i in range(len(full_tsp_path) - 1):
        tsp_edges.add((full_tsp_path[i], full_tsp_path[i+1]))
        tsp_edges.add((full_tsp_path[i+1], full_tsp_path[i]))
    G_tsp = prior_map.edge_subgraph(tsp_edges).copy()

    # Evalute edge candidates to find maximum radius
    tsp_d_opt = utils.get_normalized_weighted_spanning_trees(G_tsp, weight_type="D-opt", weight="d_opt")
    tsp_dist = utils.get_path_length(prior_map, full_tsp_path, weight="weight")
    if tsp_dist < 0:
        print("Error when count TSP path distance!")
        sys.exit(1)
    
    # Select best edge to set distance threshold 
    G_tsp_node_list = list(G_tsp.nodes())
    G_tsp_edge_list = set(G_tsp.edges())
    tsp_node_idx = {}
    for idx, node in enumerate(G_tsp_node_list):
        tsp_node_idx[node] = idx
    n = len(G_tsp_node_list)
    nn = n - 1
    G_tsp_laplacian = nx.laplacian_matrix(G_tsp, weight="d_opt").toarray()
    G_tsp_laplacian_inverse = np.linalg.inv(G_tsp_laplacian[1:, 1:])
    a = np.zeros((n, 1))  # For edge incidence vector
    # TODO:  covariance estimation
    Cov = np.zeros((3, 3))
    Cov[0, 0] = 0.1
    Cov[1, 1] = 0.1
    Cov[2, 2] = 0.001
    Sigma = np.linalg.inv(Cov)
    edge_d_opt = utils.get_d_opt(Sigma)

    visited_edges = set()
    max_resistance = -1
    for i, curr in enumerate(full_tsp_path[robot_curr_idx:]):
        if i % 100 == 0:
            print(f"current idx [find max threshold]: {robot_curr_idx + i} / {len(full_tsp_path) - 1}")      
        for closure in full_tsp_path[:robot_curr_idx+i]:   # search in its previous vertex
            if (curr, closure) in G_tsp_edge_list or (closure, curr) in G_tsp_edge_list \
                    or curr == closure or (curr, closure) in visited_edges or (closure, curr) in visited_edges:
                continue
            visited_edges.add((closure, curr))
            n1_idx = tsp_node_idx[closure]
            n2_idx = tsp_node_idx[curr]   
            a[n1_idx, 0] = 1
            a[n2_idx, 0] = -1
            # edge_d_opt = G_tsp.nodes()[closure]["d_opt"]
            curr_resistance = edge_d_opt * np.dot(np.dot(a[1:, :].T, G_tsp_laplacian_inverse), a[1:, :])[0, 0]
            max_resistance = max(max_resistance, curr_resistance)
            a[n1_idx, 0] = 0
            a[n2_idx, 0] = 0

    max_d_opt = tsp_d_opt * (1 + max_resistance)**(1/nn)
    max_candidate_dist = (max_d_opt / tsp_d_opt - 1) * tsp_dist  # For coarse filtering

    time2 = time.time()
    
    ## Build kd-tree for neighbor finding
    points = []
    points_to_vertex = {}
    for node in G_tsp.nodes():
        points.append(G_tsp.nodes()[node]["position"])
        points_to_vertex[G_tsp.nodes()[node]["position"]] = node
        
    kdtree = KDTree(points)  # Create a KDTree
    radius = max_candidate_dist

    ## Offline greedy search
    max_edge_dist = -1
    min_edge_dist = float("inf")
    
    full_tsp_node_idx = {}
    for i, node in enumerate(full_tsp_path):
        full_tsp_node_idx[node] = i

    ## Construct candidate loop closures 
    valid_closures = set()    # Record closures pass the fine-filtering
    removed_closures = set()  # Record closures fail in fine-filtering
    ranking_candidates = []   # Maintain all valid loop closures
    all_edge_length = dict(nx.all_pairs_dijkstra_path_length(prior_map, cutoff=None, weight='weight'))
    for i, curr in enumerate(full_tsp_path[robot_curr_idx:]):
        if i % 100 == 0:
            print(f"current idx [construct candidate loops]: {robot_curr_idx + i} / {len(full_tsp_path) - 1}")
        ## Coarse-filtering, radius = max_candidate_dist
        neighbor_indices = kdtree.query_ball_point(G_tsp.nodes()[curr]["position"], radius)
        neighbors = [points[i] for i in neighbor_indices]
        for closure_position in neighbors:
            closure = points_to_vertex[closure_position]
            if (curr, closure) in G_tsp.edges() or (closure, curr) in G_tsp.edges() or curr == closure:
                continue
            if (curr, closure) in removed_closures or (closure, curr) in removed_closures:
                continue
            if (curr, closure) in valid_closures or (closure, curr) in valid_closures:
                continue   
            # Fix curr-closure vs closure-curr by only considering the last same node
            if full_tsp_node_idx[curr] > full_tsp_node_idx[closure]:
                curr_loop = (closure, curr)
            else:
                continue
            valid_closures.add(curr_loop)
            # closure_length = calculate_euclidean_edge_distance(gridmap, [closure, curr])
            # closure_length = calculate_edge_distance_over_priormap(prior_map, [closure, curr])
            closure_length = all_edge_length[closure][curr]
            max_edge_dist = max(max_edge_dist, closure_length)
            min_edge_dist = min(min_edge_dist, closure_length)
            # Calculate resistance distance
            n1_idx, n2_idx = tsp_node_idx[closure], tsp_node_idx[curr]
            a = np.zeros((n, 1))
            a[n1_idx, 0], a[n2_idx, 0] = 1, -1
            # edge_d_opt = G_tsp.nodes()[closure]["d_opt"]
            curr_resistance = edge_d_opt * np.dot(np.dot(a[1:, :].T, G_tsp_laplacian_inverse), a[1:, :])[0, 0]
            normalized_resistance = (1 + curr_resistance)**(1/nn)
            ## Fine-filtering
            if normalized_resistance / (1 + closure_length / tsp_dist) <= 1:
                removed_closures.add((closure, curr))
                valid_closures.discard((closure, curr))
                valid_closures.discard((curr, closure))
                continue
            ## Pass fine-filtering
            object_value = tsp_d_opt * normalized_resistance / (tsp_dist + 2 * closure_length)
            ranking_candidates.append([curr_loop, normalized_resistance, closure_length, object_value, tsp_d_opt, tsp_dist, iter])  
    ranking_candidates.sort(key=lambda x: x[3])
    

    # Greedy TSP path update
    results = greedy_tsp_update(tsp_d_opt, 
                      tsp_dist, 
                      G_laplacian=G_tsp_laplacian, 
                      laplacian_inverse=G_tsp_laplacian_inverse,
                      ranking_candidates=ranking_candidates,
                      removed_closures=removed_closures,
                      valid_closures=valid_closures,
                      tsp_node_idx=tsp_node_idx,
                      edge_d_opt=edge_d_opt)
    
    selected_edges = results[0]
    curr_d_opt = results[1]
    curr_path_dist = results[2]
    curr_obj_value = results[3]
    
    print(f"#Add edges: {len(selected_edges)}, {str(selected_edges)}")
    time3 = time.time()
    print(f"Time cost: {round(time2 - time1, 2)}s for best one; {round(time3 - time2, 2)}s for all.")
    print(" ")

    is_optimized = False
    if len(selected_edges) > 0:
        is_optimized = True
    if not is_optimized:
        rospy.logerr("New tsp path cannot find good closures.")
        return is_optimized, [], set(), curr_obj_value, curr_d_opt, curr_path_dist, tsp_d_opt, tsp_dist
    
    selected_edges.sort(key=lambda x: full_tsp_node_idx[x[1]])
    insert_loop_cnt = 0
    new_path = []
    loop_vertices = set()
    for i, node in enumerate(full_tsp_path):
        new_path.append(node)
        if i < robot_curr_idx:
            continue
        if insert_loop_cnt < len(selected_edges) and node == selected_edges[insert_loop_cnt][1]: 
            new_path.append(selected_edges[insert_loop_cnt][0])
            loop_vertices.add(len(new_path) - 1)
            if i+1 < len(full_tsp_path) - 1:
                new_path.append(node)   # After loop back to original position
            insert_loop_cnt += 1
    # FIXME: Double check whether the index is correct. Should be correct because loop closure only added after robot_curr_idx
    if(new_path[robot_curr_idx] != full_tsp_path[robot_curr_idx]):
        rospy.logerr("Failed in double checking replan path.")
    print("start_index: " + str(robot_curr_idx))
    print("optimized new path: "+ str(new_path))
    print("non-optimized new path: " + str(full_tsp_path))

    return is_optimized, new_path, loop_vertices, curr_obj_value, curr_d_opt, curr_path_dist, tsp_d_opt, tsp_dist


def generate_random_gridmap():
    size_x, size_y = 6, 6
    gridmap = nx.grid_2d_graph(size_x, size_y)
    add_position_attr_to_graph(gridmap, scale = 12)
    add_weight_attr_to_graph(gridmap)
    
    # random_remove_nodes(gridmap, num=int(size_x * size_x * 0.05))
    # random_remove_edges(gridmap, num=10 * int(size_x * size_x * 0.05))


    ## Add covariance for edges. FIXME: covariance estimation based on vertex observability
    Cov = np.zeros((3, 3))
    Cov[0, 0] = 0.1
    Cov[1, 1] = 0.1
    Cov[2, 2] = 0.001
    Sigma = np.linalg.inv(Cov)
    utils.add_edge_information_matrix(gridmap, Sigma)
    utils.add_graph_weights_as_dopt(gridmap, key="d_opt")
    return gridmap


### Concorde TSP solver
def symmetricize(matrix, k=None):
    """
        Jonker-Volgenant method of transforming (n x n) asymmetric TSP, C into (2n x 2n) symmetric TSP, S.

        Let C be an asymmetric TSP matrix.
        Let k be a very large number, ie k >> C.max()
        Let U = (u_ij) with u_ii = 0, u_ij = k for i != j.

        Construct the (2n x 2n) matrix:
        
                    +-------+
                    | U |C^T|
            S =     |---+---|
                    | C | U |
                    +-------+

        S is a symmetric TSP problem on 2n nodes.
        There is a one-to-one correspondence between solutions of S and solutions of C.
    """
    # if k not provided, make it equal to 10 times the max value:
    if k is None:
        # k = round(10*matrix.max())
        k = 99
        
    matrix_bar = matrix.copy()
    np.fill_diagonal(matrix_bar, 0)
    u = np.matrix(np.ones(matrix.shape).astype(int) * k)
    np.fill_diagonal(u, 0)
    matrix_symm_top = np.concatenate((u, np.transpose(matrix_bar)), axis=1)
    matrix_symm_bottom = np.concatenate((matrix_bar, u), axis=1)
    matrix_symm = np.concatenate((matrix_symm_top, matrix_symm_bottom), axis=0)
    
    return matrix_symm

def concorde_tsp_solver(distance_matrix: np.array, node_list: list, specify_end = False):
    """ Solve open TSP problem using concorde. Better path will be selecetd from both directions."""
    distance_matrix_int = np.round(distance_matrix * 5).astype(int)
    k = 10 * np.max(distance_matrix_int)
    if specify_end:
        distance_matrix_int[1:-1, 0] = k  # Specify the starting and ending vertex
    else:
        distance_matrix_int[1:, 0] = k  # So that the staring point will be the first vertex
    
    symm_distance = symmetricize(distance_matrix_int, k = k)
    problem = Problem.from_matrix(symm_distance)
    solution = run_concorde(problem)
    path_indices_with_ghost = solution.tour
    path_indices = path_indices_with_ghost[::2]   # Extract real path index
    path_length = 0 
    for i in range(len(path_indices) - 1):
        start_idx, end_idx = path_indices[i], path_indices[i+1]
        path_length += distance_matrix[start_idx][end_idx]
    ret_indices = path_indices
    ret_length = path_length

    # May be the case that we have two paths: [a0, a1, ..., an] and [a0, an, an-1, ..., a1]
    # Check which is better, and set it as the real tsp path
    # if len(path_indices) >= 3 and not specify_end:   # If specify_end, then do not need to reverse
    if len(path_indices) >= 3:
        candidate_indices = [path_indices[0]] + path_indices[::-1]
        candidate_indices.pop()
        # print(ret_indices)
        # print(candidate_indices)
        # Compare the distance of the two candidate paths using original distance matrix
        candidate_length = 0
        for i in range(len(candidate_indices) - 1):
            start_idx, end_idx = candidate_indices[i], candidate_indices[i+1]
            candidate_length += distance_matrix[start_idx][end_idx]
        if candidate_length < path_length:
            print("Reverse tsp path is better!")
            ret_indices = candidate_indices
            ret_length = candidate_length
        else:
            print("Original tsp path is better!")

    tsp_path = []
    for idx in ret_indices:
        tsp_path.append(node_list[idx])
    return tsp_path, ret_length


def find_last_loop_closure(prior_map: nx.Graph, full_tsp_path: list, exist_obj_value: float) -> List[int]:
    """ Find loop closure starting at robot_curr_idx, which is the last index in full_tsp_path.
    Reture [-1, -1] if not closure found.
    """
    robot_curr_idx = len(full_tsp_path) - 1
    tsp_edges = set()
    for i in range(len(full_tsp_path) - 1):
        tsp_edges.add((full_tsp_path[i], full_tsp_path[i+1]))
        tsp_edges.add((full_tsp_path[i+1], full_tsp_path[i]))
    G_tsp = prior_map.edge_subgraph(tsp_edges).copy()

    # Evalute edge candidates to find maximum radius
    tsp_d_opt = utils.get_normalized_weighted_spanning_trees(G_tsp, weight_type="D-opt", weight="d_opt")
    tsp_dist = utils.get_path_length(prior_map, full_tsp_path, weight="weight")
    if abs(tsp_d_opt / tsp_dist - exist_obj_value) > 1e-6:
        print("Error when double check existing obj_value!")
    if tsp_dist < 0:
        print("Error when count TSP path distance!")
        sys.exit(1)
    
    # Select best edge to set distance threshold 
    G_tsp_node_list = list(G_tsp.nodes())
    G_tsp_edge_list = set(G_tsp.edges())
    tsp_node_idx = {}
    for idx, node in enumerate(G_tsp_node_list):
        tsp_node_idx[node] = idx
    n = len(G_tsp_node_list)
    nn = n - 1
    G_tsp_laplacian = nx.laplacian_matrix(G_tsp, weight="d_opt").toarray()
    G_tsp_laplacian_inverse = np.linalg.inv(G_tsp_laplacian[1:, 1:])
    a = np.zeros((n, 1))  # For edge incidence vector
    # TODO:  covariance estimation
    Cov = np.zeros((3, 3))
    Cov[0, 0] = 0.1
    Cov[1, 1] = 0.1
    Cov[2, 2] = 0.001
    Sigma = np.linalg.inv(Cov)
    edge_d_opt = utils.get_d_opt(Sigma)

    visited_edges = set()
    # Record best edge
    best_obj_value = exist_obj_value
    best_edge = [-1, -1]
    all_edge_length = dict(nx.all_pairs_dijkstra_path_length(prior_map, cutoff=None, weight='weight'))

    curr = full_tsp_path[robot_curr_idx]
    for closure in full_tsp_path[:robot_curr_idx]:   # search in its previous vertex
        if (curr, closure) in G_tsp_edge_list or (closure, curr) in G_tsp_edge_list \
                or curr == closure or (curr, closure) in visited_edges or (closure, curr) in visited_edges:
            continue
        visited_edges.add((closure, curr))
        n1_idx = tsp_node_idx[closure]
        n2_idx = tsp_node_idx[curr]   
        a[n1_idx, 0] = 1
        a[n2_idx, 0] = -1
        # edge_d_opt = G_tsp.nodes()[closure]["d_opt"]
        curr_resistance = edge_d_opt * np.dot(np.dot(a[1:, :].T, G_tsp_laplacian_inverse), a[1:, :])[0, 0]
        closure_length = all_edge_length[closure][curr]
        a[n1_idx, 0] = 0
        a[n2_idx, 0] = 0

        normalized_resistance = (1 + curr_resistance)**(1/nn)
        ## Pass fine-filtering
        object_value = tsp_d_opt * normalized_resistance / (tsp_dist + 2 * closure_length)
        if object_value > best_obj_value:
            best_edge[0] = closure
            best_edge[1] = curr
            best_obj_value = object_value
        
    if best_obj_value > exist_obj_value:
        print("Find last loop closure with better obj value: {exist_obj_value} -> {best_obj_value}")
    else:
        print("Find not last loop closure")
    return best_edge



if __name__ == "__main__":
    pass