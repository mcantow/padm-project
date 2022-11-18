import numpy as np

class Graph:
    def __init__(self, start_pose, start_joints):
        self.start_pose = start_pose
        self.start_joints = start_joints
        
        self.edges = []
        self.nodes = [start_joints]
        self.joints_to_poses = {start_joints: start_pose}
        self.node_paths = {start_joints: [start_joints]}


    def add_node(self, joints, pose):
        self.nodes.append(joints)
        self.joints_to_poses[joints] = pose

    def add_edge(self, node_1, node_2):
        self.edges.append((node_1, node_2))
    
    def get_pose_distance(self, pose_1, pose_2):
        positions_1 = pose_1[0]
        positions_2 = pose_2[0]
        sum_sq = sum([(positions_1[i] - positions_2[i]) ** 2 for i in range(3)])
        return np.sqrt(sum_sq)

    

    def get_nearest_node_to_pose(self, pose):
        nearest_node = None
        nearest_node_dist = np.inf
        for joints in self.nodes:
            joint_pose = self.joints_to_poses[joints]
            distance = self.get_pose_distance(pose, joint_pose)
            if distance < nearest_node_dist:
                nearest_node_dist = distance
                nearest_node = joints
        return nearest_node
    
    def update_node_path(self, new_node, from_node, itermediate_joint_path):
        self.node_paths[new_node] = self.node_paths[from_node] + itermediate_joint_path +[new_node]
    
    def plot_graph(self, ax, radius, goal_path=None):
#         x = [node[0] for node in self.nodes]
#         y = [node[1] for node in self.nodes]

        plt.scatter(self.start_position[0], self.start_position[1], c="r", s=100)
        for edge in self.edges:
            x_vals = [edge[0][0], edge[1][0]]
            y_vals = [edge[0][1], edge[1][1]]
            ax.plot(x_vals, y_vals, c="grey")
        
        if goal_path is not None:
            # goal_path_x_vals = [node[0] for node in goal_path]
            # goal_path_y_vals = [node[1] for node in goal_path]
            for i in range(len(goal_path) - 1):
                node1 = goal_path[i]
                node2 = goal_path[i+1]
                line = LineString([node1, node2])
                expanded_line = line.buffer(radius, resolution=3)
                plot_poly(ax, expanded_line, 'green', alpha=0.2)

            # ax.plot(goal_path_x_vals, goal_path_y_vals, c="b") 
        return ax

        
def check_if_edge_allowed(node_1, node_2, environment, radius):
    line = LineString([node_1, node_2])
    expanded_line = line.buffer(radius, resolution=3)
    for i, obs in enumerate(environment.obstacles):
        if expanded_line.intersects(obs):
            return False
    return True
