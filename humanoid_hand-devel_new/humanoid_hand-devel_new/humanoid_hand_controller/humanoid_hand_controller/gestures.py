import rclpy
from rclpy.node import Node
import serial
import time
from typing import List, Dict
from humanoid_hand_controller.utils import BaseHand

import networkx as nx
import numpy as np
import scipy

class HandGestures(BaseHand):
    def __init__(self, node_name, config_filepath):
        super().__init__(node_name, config_filepath)
        self.get_logger().info("Hand gestures node initialized")
        
        # Add gestures here
        self.gestures = {
            0: np.array([1000, 1000, 1000, 1000, 1000, 1000], dtype=int), # home gesture
            1: np.array([0, 0, 0, 0, 1000, 0], dtype=int), # closed gesture
            2: np.array([550, 550, 550, 550, 250, 0], dtype=int), # wrap gesture
            3: np.array([0, 0, 0, 450, 200, 0], dtype=int), # two finger gesture
            4: np.array([0, 0, 0, 1000, 200, 1000], dtype=int), # three finger gesture
            # 5: np.array([0, 0, 0 ,1000, 1000, 0])
        }

        self.speed = {
            0: np.array([1000, 1000, 1000, 1000, 1000, 1000]), # home gesture
            1: np.array([1000, 1000, 1000, 1000, 1000, 1000]), # closed gesture
            2: np.array([1000, 1000, 1000, 1000, 1000, 1000]), # wrap gesture
            3: np.array([1000, 1000, 1000, 1000, 1000, 1000]) # two finger gesture
        }

        # Create the graph first
        self.adjacency_matrix = self.get_adjacency_matrix(self.gestures)
        self.G = self.build_graph(self.adjacency_matrix)
        
        # Then calculate paths using the graph
        self.gestures_paths = {
            "path_1": self.get_shortest_path(0, 2),
            # "path_2": self.get_shortest_path(4, 3),
            # "path_3": self.get_shortest_path(4, 3),
            # "path_3": self.get_shortest_path(1, 0),
        }

        # Add untraversable edges here
        # self.set_infeasible_edges(0, 1)

    def build_graph(self, adjacency_matrix):
        G = nx.DiGraph()
        for i in range(adjacency_matrix.shape[0]):
            for j in range(adjacency_matrix.shape[1]):
                if i != j:
                    G.add_edge(i, j, weight=adjacency_matrix[i, j])
        return G
    
    def set_infeasible_edges(self, source_node: int, target_node: int):
        self.G[source_node][target_node]['weight'] = 1e10

    def get_adjacency_matrix(self, position_gestures: Dict[int, np.ndarray]) -> np.ndarray:
        """
        Stores the position of the hand gestures.
        Arguments: 
            None

        Returns:
            Adjacency matrix of the hand gestures
        """

        position_matrix = np.stack(list(position_gestures.values()))
        adjacency_matrix = scipy.spatial.distance.cdist(position_matrix, position_matrix)

        return adjacency_matrix
    
    def remove_redundant_nodes(self, nodes: List[int]):
        """Removes redundant consecutive nodes"""
        if not nodes:
            return []
            
        result = [nodes[0]]
        for i in range(1, len(nodes)):
            if nodes[i] != result[-1]:
                result.append(nodes[i])
        return result

    def get_shortest_path(self, start_node: int, end_node: int) -> List[int]:
        """
        Get the shortest path between two nodes in the graph

        Arguments:
            start_node: int
            end_node: int

        Returns:
            List of nodes in the shortest path
        """
        return nx.shortest_path(self.G, source=start_node, target=end_node, weight='weight')

