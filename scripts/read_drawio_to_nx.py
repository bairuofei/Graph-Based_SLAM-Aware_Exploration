#!/usr/bin/python3
'''
Author: ruofei_ntu 991609404@qq.com
Date: 2023-06-20 09:49:23
LastEditors: ruofei_ntu 991609404@qq.com
LastEditTime: 2024-01-09 19:40:41
FilePath: /Graph-Based_SLAM-Aware_Exploration/scripts/read_drawio_to_nx.py
Description: 

Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
'''

import networkx as nx
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import math
import numpy as np

def from_drawio_to_nx(file_name: str) -> nx.graph:
    """ Given a xml file exported from drawio, traverse it into a networkx graph object.
    """
    # Read XML file
    tree = ET.parse(file_name)
    root = tree.getroot()

    nodes_dict = {}
    edge_list = []

    # Traverse XML tree
    for element in root.iter():
        # 获取元素的标签和内容
        tag = element.tag
        if tag != "mxCell":
            continue

        # print(f"Current tag: {tag}")
        attributes = element.attrib
        if "style" not in attributes:
            continue
        if attributes["style"][:7] == "ellipse":  # node
            new_node = {}
            geometry_element = element.find("mxGeometry")
            geometry_attributes = geometry_element.attrib
            for key in ["x", "y"]:
                if key in geometry_attributes:
                    new_node[key] = float(geometry_attributes[key])
                else:
                    new_node[key] = 0
            nodes_dict[attributes["id"]] = new_node
        elif attributes["style"][:9] == "edgeStyle":   # edge
            source = attributes["source"]
            target = attributes["target"]
            edge_list.append([source, target])
            

    graph = nx.Graph()
    id_to_node = {}
    node_index = 0
    for id, node_dict in nodes_dict.items():
        x, y = node_dict["x"], node_dict["y"]
        graph.add_node(node_index, position = (x, y))
        id_to_node[id] = node_index
        node_index += 1

    for source, target in edge_list:
        source_node = id_to_node[source]
        target_node = id_to_node[target]
        graph.add_edge(source_node, target_node)

    return graph


def move_graph_to_border(graph, actual_width = -1):
    """                   mid_node
        left_node                         right_node
    """
    isolated_nodes_list = list(nx.isolates(graph))
    isolated_nodes = [graph.nodes()[node]["position"] for node in isolated_nodes_list]
    left_idx, right_index, mid_index = -1, -1, -1
    # 012, 021, 102, 120, 201, 210
    possible = [[0, 1, 2], [0, 2, 1], [1, 0, 2], [1, 2, 0], [2, 0, 1], [2, 1, 0]]
    index = -1
    if isolated_nodes[0][0] < isolated_nodes[1][0] and isolated_nodes[1][0] < isolated_nodes[2][0]:
        index = 0
    elif isolated_nodes[0][0] < isolated_nodes[2][0] and isolated_nodes[2][0] < isolated_nodes[1][0]:
        index = 1
    elif isolated_nodes[1][0] < isolated_nodes[0][0] and isolated_nodes[0][0] < isolated_nodes[2][0]:
        index = 2
    elif isolated_nodes[1][0] < isolated_nodes[2][0] and isolated_nodes[2][0] < isolated_nodes[0][0]:
        index = 3
    elif isolated_nodes[2][0] < isolated_nodes[0][0] and isolated_nodes[0][0] < isolated_nodes[1][0]:
        index = 4
    elif isolated_nodes[2][0] < isolated_nodes[1][0] and isolated_nodes[1][0] < isolated_nodes[0][0]:
        index = 5
    left_idx, mid_index, right_index = possible[index]
    left_node, mid_node, right_node = isolated_nodes[left_idx], isolated_nodes[mid_index], isolated_nodes[right_index]
    # print([left_node, mid_node, right_node])

    width_in_drawio = right_node[0] - left_node[0]
    if actual_width < 0:
        scale = 1
    else:
        scale = actual_width / width_in_drawio
    print(f"Drawio scale: {scale}")  # 0.1028 for map3
    
    center_x_drawio, center_y_drawio = 0.5 * (left_node[0] + right_node[0]), 0.5 * (left_node[1] + mid_node[1])
    # print([center_x_drawio, center_y_drawio])

    # Shift the graph so that it is centered at the mid_node, and scaled by the "scale"
    # Step 1: for all points, x = x - center_x_drawio, y = y - center_y_drawio
    # Step 2: for all points, y = -y;
    # Step 3: for all points, x = x * scale, y = y * scale
    for node in graph.nodes():
        x, y = graph.nodes()[node]["position"]
        new_x = x - center_x_drawio
        new_y = y - center_y_drawio
        new_y *= -1
        new_x *= scale
        new_y *= scale
        graph.nodes()[node]["position"] = (new_x, new_y)
        
    return

def remove_isolated_nodes(graph):
    """ Remove nodes for positioning the graph. """
    isolated_nodes_list = list(nx.isolates(graph))
    for node in isolated_nodes_list:
        graph.remove_node(node)
    return

def add_edge_distance_as_weight(graph):
    """ Add distance metric as weight in the graph. """
    for edge in graph.edges():
        source, target = edge
        pos1, pos2 = graph.nodes()[source]["position"], graph.nodes()[target]["position"]
        distance = math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
        graph.edges()[edge]["weight"] = distance
    return

def build_prior_map_from_drawio(path: str, actual_map_width: float = -1, need_normalize: bool = False, need_noise: bool = False, variance: float = 0) -> nx.graph:
    """ Given a xml file for a drawio graph, transform it into a networkx graph. 
        The node name may not start from 0, becuase isolated nodes are removed.
    """
    graph = from_drawio_to_nx(path)
    # Add noise to the position parameters
    if need_noise:
        for node in graph.nodes():
            x, y = graph.nodes()[node]["position"]
            graph.nodes()[node]["position"] = (x + np.random.normal(0, variance), y + np.random.normal(0, variance))
    # Relocate the graph
    move_graph_to_border(graph, actual_width = actual_map_width)
    remove_isolated_nodes(graph)
    add_edge_distance_as_weight(graph)

    # Test: normalize the edge distance
    if need_normalize:
        min_distance = float("inf")
        for edge in graph.edges():
            min_distance = min(min_distance, graph.edges()[edge]["weight"])
        if min_distance > 0:
            for edge in graph.edges():
                prev_distance = graph.edges()[edge]["weight"]
                divided = round(prev_distance / min_distance)
                if divided == 0:
                    print("Normalize edge distance error!")
                    continue
                graph.edges()[edge]["weight"] = divided * min_distance
    return graph


if __name__ == "__main__":

    actual_map_width = 34  # meters

    graph = from_drawio_to_nx("/home/ruofei/code/cpp/catkin_cpp_ws/src/cpp_solver/world/map7/map7.xml")
    # Relocate the graph
    move_graph_to_border(graph, actual_width = actual_map_width)
    remove_isolated_nodes(graph)
    add_edge_distance_as_weight(graph)


    fig, ax = plt.subplots()
    for node in graph.nodes():
        x, y = graph.nodes()[node]["position"]
        ax.plot(x, y, 'o', markersize=10, color='g', alpha = 0.7)
    for edge in graph.edges():
        node1, node2 = edge
        node1_pos, node2_pos = graph.nodes()[node1]["position"], graph.nodes()[node2]["position"]
        ax.plot([node1_pos[0], node2_pos[0]], [node1_pos[1], node2_pos[1]], '-', color='r', alpha = 0.5, zorder=5)
    ax.set_aspect('equal')
    plt.show()




