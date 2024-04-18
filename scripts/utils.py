#!/usr/bin/python3
import networkx as nx
import numpy as np
import math
import scipy
import matplotlib.pyplot as plt
import pickle 
import random
import time

from typing import Tuple

EIG_TH = 1e-6


def write_nx_graph_to_g2o(graph: nx.graph, path: str):
    with open(path, 'w') as file: # w is write mode, file content will be clear. Use a if want to append
        for node in graph.nodes():
            curr_line = "VERTEX_SE2 "
            curr_line += str(node)
            for p in graph.nodes()[node]["pose"]:
                curr_line += " " + str(p)
            curr_line += "\n"
            file.write(curr_line)
        for edge in graph.edges():
            curr_line = "EDGE_SE2 "
            curr_line += str(edge[0]) + " " + str(edge[1]) + " "
            cov = graph.edges()[edge]["covariance"]
            curr_line += str(cov[0, 0]) + " " + str(cov[0, 1]) + " " + str(cov[0, 2])\
                        + " " + str(cov[1, 1]) + " " + str(cov[1, 2]) + " " \
                        + str(cov[2, 2]) + "\n"
            file.write(curr_line)
    return


## Save and read data
def save_data(data, file_path):
    file = open(file_path, 'wb')
    pickle.dump(data, file)
    file.close()
    return True

def read_data(file_path):
    with open(file_path, 'rb') as file:
        data =pickle.load(file)
        return data


## Graph construction
def construct_graph_from_path(path):
    graph = nx.Graph()
    for i in range(len(path)-1):
        graph.add_node(path[i])
        graph.add_edge(path[i], path[i+1])
    graph.add_node(path[-1])
    return graph


## Graph manipulation

def g2o_to_nx(file_path):
    graph = nx.Graph()
    with open(file_path) as g2o_graph:
        for line in g2o_graph:
            one_line = line.split(" ")
            if one_line[0] == "VERTEX_SE2":
                pose = [float(x) for x in one_line[2:5]]
                graph.add_node(one_line[1], pose = pose)
            if one_line[0] == "EDGE_SE2":
                relative_T = [float(x) for x in one_line[3:6]]
                distance = math.sqrt(relative_T[0]**2 + relative_T[1]**2)
                information = np.array([[float(one_line[6]), float(one_line[7]), float(one_line[8])],
                                        [float(one_line[7]), float(one_line[9]), float(one_line[10])],
                                        [float(one_line[8]), float(one_line[10]), float(one_line[11])]])
                graph.add_edge(one_line[1], one_line[2], relative_T=relative_T, information=information,
                               distance=distance)
    return graph


def add_graph_weights_as_dopt(graph, key="weight"):
    for edge in graph.edges():
        dopt = get_d_opt(graph.edges[edge]["information"])
        graph.edges[edge][key] = dopt # * random.randint(1, 3)
    return

def add_edge_information_matrix(graph, Sigma):
    for edge in graph.edges():
        graph.edges()[edge]["information"] = Sigma


def get_path_length(graph, path, weight="weight"):
    dist = 0
    for i in range(len(path) - 1):
        try:
            dist += nx.shortest_path_length(graph, source=path[i], target=path[i+1], weight=weight, method='dijkstra')
        except nx.NodeNotFound or nx.NetworkXNoPath or nx.ValueError:
            return -1
    return dist




## Matrix calculation

def get_determinant(A):
    # Solves a standard or generalized eigenvalue problem for a complex Hermitian or real symmetric matrix.
    eigv2 = scipy.linalg.eigvalsh(A)
    if np.iscomplex(eigv2.any()):
        print("Error: Complex Root")
    n = np.size(A, 1)
    eigv = eigv2[eigv2 > EIG_TH] # Only select eigenvalues larger than EIG_TH (1e-6)
    return np.exp(np.sum(np.log(eigv)))  # Aovid overflow


def get_d_opt(A):
    # Solves a standard or generalized eigenvalue problem for a complex Hermitian or real symmetric matrix.
    eigv2 = scipy.linalg.eigvalsh(A)
    if np.iscomplex(eigv2.any()):
        print("Error: Complex Root")
    n = np.size(A, 1)
    eigv = eigv2[eigv2 > EIG_TH] # Only select eigenvalues larger than EIG_TH (1e-6)
    return np.exp(np.sum(np.log(eigv)) / n) # Avoid overflow and do normalization


def get_weighted_spanning_trees(graph, weight_type="D-opt"):
    if weight_type != "D-opt":
        print("Error: Please indicate weight type when construct Laplacian matrix.")
        return
    laplacian = nx.laplacian_matrix(graph, weight="weight").toarray()
    reduced_laplacian = laplacian[1:, 1:]
    eigv2 = scipy.linalg.eigvalsh(reduced_laplacian)
    if np.iscomplex(eigv2.any()):
        print("Error: Complex Root")
    n = np.size(reduced_laplacian, 1)
    eigv = eigv2[eigv2 > EIG_TH] # Only select eigenvalues larger than EIG_TH (1e-6)
    return np.exp(np.sum(np.log(eigv)))  # FIXMED: Here has the overflow problem.


def get_normalized_weighted_spanning_trees(graph: nx.Graph, weight_type: str="D-opt", weight: str="weight"):
    if weight_type != "D-opt":
        print("Error: Please indicate weight type when construct Laplacian matrix.")
        return
    laplacian = nx.laplacian_matrix(graph, weight=weight).toarray()
    reduced_laplacian = laplacian[1:, 1:]
    eigv2 = scipy.linalg.eigvalsh(reduced_laplacian)
    if np.iscomplex(eigv2.any()):
        print("Error: Complex Root")
    n = np.size(reduced_laplacian, 1)
    eigv = eigv2[eigv2 > EIG_TH] # Only select eigenvalues larger than EIG_TH (1e-6)
    return np.exp(np.sum(np.log(eigv)) / n)


def compute_LB(graph, m, weight_type="D-opt"):
    if weight_type != "D-opt":
        print("Error: Please indicate weight type when construct Laplacian matrix.")
        return
    laplacian = nx.laplacian_matrix(graph, weight="weight").toarray()
    reduced_laplacian = laplacian[1:, 1:]
    eigv2 = scipy.linalg.eigvalsh(reduced_laplacian)
    if np.iscomplex(eigv2.any()):
        print("Error: Complex Root")
    n = np.size(reduced_laplacian, 1)
    eigv = eigv2[eigv2 > EIG_TH] # Only select eigenvalues larger than EIG_TH (1e-6)
    return np.exp(np.sum(np.log(eigv)) / (n+m))

## Visulation

def draw_nx_graph_at_pose(graph, save=True, save_path = ""):
    if save and not save_path:
        print("Please give save path!!")
        return
    fig, ax = plt.subplots()
    for node in graph.nodes():
        ax.plot(graph.nodes[node]["pose"][0], graph.nodes[node]["pose"][1], 'o', markersize=1, color='g', alpha = 0.7)
        # ax.text(graph.nodes[node]["pose"][0], graph.nodes[node]["pose"][1]+2, str(node), fontsize=12, ha='center', va='center')
    for start, end in graph.edges():
        x = [graph.nodes[start]["pose"][0], graph.nodes[end]["pose"][0]]
        y = [graph.nodes[start]["pose"][1], graph.nodes[end]["pose"][1]]
        ax.plot(x, y, '-', color='gray', alpha = 0.2)
    plt.axis('equal')
    if save:
        plt.savefig(save_path)
    else:
        plt.show()

def plot_path_only(graph: nx.graph, path: list = [], closures: list = [], savefig: bool =False):
    fig, ax = plt.subplots(1, 2)
    for node in path:
        pose = graph.nodes[node]["position"]
        ax[0].plot(pose[0], pose[1], 'o', markersize=2, color='g', alpha = 0.7)
    for i in range(len(path) - 1):
        edge = (path[i], path[i+1])
        pose1 = graph.nodes[path[i]]["position"]
        pose2 = graph.nodes[path[i+1]]["position"]
        ax[0].plot([pose1[0], pose2[0]], [pose1[1], pose2[1]], '-', color='gray', alpha = 0.5, zorder=5)
    for edge in closures:
        node1, node2 = edge
        pose1 = graph.nodes[node1]["position"]
        pose2 = graph.nodes[node2]["position"]
        ax[0].plot([pose1[0], pose2[0]], [pose1[1], pose2[1]], '-', color='r', alpha = 0.7, zorder=5)

    for node in path:
        pose = graph.nodes[node]["position"]
        ax[1].plot(pose[0], pose[1], 'o', markersize=2, color='g', alpha = 0.7)
    for i in range(len(path) - 1):
        edge = (path[i], path[i+1])
        pose1 = graph.nodes[path[i]]["position"]
        pose2 = graph.nodes[path[i+1]]["position"]
        ax[1].plot([pose1[0], pose2[0]], [pose1[1], pose2[1]], '-', color='gray', alpha = 0.5, zorder=5)

    ax[0].set_aspect('equal')
    ax[1].set_aspect('equal')
    plt.show()


def plot_modified_path(graph: nx.graph, path: list = [], closures: list = [], savefig: bool =False):
    """ Plot path over graph. The closure edges are highlighted as red.
    """
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
        pose1, pose2 = graph.nodes()[node1]["position"], graph.nodes()[node2]["position"]
        if (node1, node2) in path_edge_set or (node2, node1) in path_edge_set:
            axes[0].plot([pose1[0], pose2[0]], [pose1[1], pose2[1]], '-', color='r', alpha = 0.5, zorder=5)
            axes[1].plot([pose1[0], pose2[0]], [pose1[1], pose2[1]], '-', color='r', alpha = 0.5, zorder=5)
        else:
            axes[0].plot([pose1[0], pose2[0]], [pose1[1], pose2[1]], '-', color='gray', alpha = 0.2)
            axes[1].plot([pose1[0], pose2[0]], [pose1[1], pose2[1]], '-', color='gray', alpha = 0.2)
    for node1, node2 in closures:
        pose1, pose2 = graph.nodes()[node1]["position"], graph.nodes()[node2]["position"]
        axes[1].plot([pose1[0], pose2[0]], [pose1[1], pose2[1]], '-', color='b', alpha = 0.7, zorder=5)
    axes[0].set_aspect('equal')
    axes[1].set_aspect('equal')
    if savefig:
        prefix = time.time()
        plt.savefig("./results/tsp_path" + str(math.floor(prefix)) + ".pdf")
    # plt.axis('equal')
    else:
        plt.show()





if __name__ == "__main__":
    pass




