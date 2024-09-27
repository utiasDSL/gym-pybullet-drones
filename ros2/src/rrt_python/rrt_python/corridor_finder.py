from scipy.spatial import KDTree as pcd_tree
from rrt_python.kdtree_c import KDNode, KDTree
import random
import numpy as np
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import plotly.graph_objs as go

class Node:
    def __init__(self):
        self.coordinates = np.zeros(3, dtype=np.float64)  # 3D coordinates
        self.radius = None # Node safe radius

        self.valid = True # Node safe or not
        self.best = False # Node in best path or not
        self.change = False

        # temporary variables to assist rewiring
        self.rel_id = 0 
        self.rel_dis = 0

        # parent and child data
        self.preNode = None
        self.childNodes = []

        self.g = 0 # cost of shortest path from node to root
        self.f = 0 # heuristic cost of the node to the target point


class RRT_start:
    def __init__(self, safety_margin, search_margin, max_radius, sample_range) -> None:
        self.safety_Margin = safety_margin
        self.search_Margin = search_margin
        self.max_Radius = max_radius
        self.sample_Range = sample_range
        
        # Tree for rrt
        self.rrtTree = KDTree(dim = 3)
        
        # point cloud search tree
        self.pcd = None

        # all nodes in current path
        self.pathList = []

        # all invalid nodes
        self.invalidSet = []

        # all nodes
        self.nodeList = []

        # all nodes that reach the end
        self.endList = []

        self.root = Node()

        self.best_endPtr = Node()

        self.informStatus = False
        self.path_exist_status = False
        self.global_navi_status = False
        
        self.x_l, self.x_h = 0, 0
        self.y_l, self.y_h = 0, 0
        self.z_l, self.z_h = 0, 0
        self.max_samples = 0
        self.inlier_ratio = 0
        self.goal_ratio = 0
        self.safety_Margin = 0
        self.rotation_inf = np.eye(3)
        self.translation_inf = np.zeros(3)
        self.eng = random.Random()

        self.startPt = None
        self.endPt = None
        self.commit_root = None

        self.min_distance = None
        self.elli_l = None
        self.elli_s = None
        self.ctheta = None
        self.stheta = None

        self.path_matrix = None
        self.radius_matrix = None
        
        self.cach_size = 100
        self.best_distance = np.inf



    def reset(self):
        self.nodeList.clear()
        self.endList.clear()
        self.invalidSet.clear()
        self.pathList.clear()

        self.best_endPtr = Node()
        self.root = Node()

        self.path_exist_status = True
        self.informStatus = False
        self.global_navi_status = False
        self.best_distance = np.inf
        
    def setInput(self, pcd):
        self.pcd = pcd_tree(pcd)

    def setPt(self, startPt, endPt, xl, xh, yl, yh, zl, zh, local_range, max_iter, sample_portion, goal_portion):
        self.startPt = startPt
        self.endPt = endPt

        self.x_l = xl
        self.y_l = yl
        self.z_l = zl

        self.x_h = xh
        self.y_h = yh
        self.z_h = zh

        self.bias_l = 0.0
        self.bias_h = 1.0
        
        # Random generators for the range
        self.rand_x = random.uniform(self.x_l, self.x_h)
        self.rand_y = random.uniform(self.y_l, self.y_h)
        self.rand_z = random.uniform(self.z_l + self.safety_Margin, self.z_h)
        self.rand_bias = random.uniform(self.bias_l, self.bias_h)

        # Local random range (based on the start point)
        self.rand_x_in = random.uniform(self.startPt[0] - local_range, self.startPt[0] + local_range)
        self.rand_y_in = random.uniform(self.startPt[1] - local_range, self.startPt[1] + local_range)
        self.rand_z_in = self.rand_z
        # Minimum distance between start and end point
        self.min_distance = np.linalg.norm(self.startPt - self.endPt)
        
        # 3D ellipsoid transformation
        self.translation_inf = (self.startPt + self.endPt) / 2.0
        
        # Define ellipsoid axes and rotation
        xtf = (self.endPt - self.translation_inf) / np.linalg.norm(self.endPt - self.translation_inf)
        downward = np.array([0, 0, -1])
        ytf = np.cross(xtf, downward)
        ytf /= np.linalg.norm(ytf)
        ztf = np.cross(xtf, ytf)

        self.rotation_inf[:, 0] = xtf
        self.rotation_inf[:, 1] = ytf
        self.rotation_inf[:, 2] = ztf

        # Set the parameters for sampling
        self.sample_range = local_range
        self.max_samples = max_iter
        self.inlier_ratio = sample_portion
        self.goal_ratio = goal_portion


    def setStartPt(self, startPt, endPt):
        self.startPt = startPt
        self.endPt = endPt
        self.rand_x_in = random.uniform(self.startPt[0] - self.sample_range, self.startPt[0] + self.sample_range)
        self.rand_y_in = random.uniform(self.startPt[1] - self.sample_range, self.startPt[1] + self.sample_range)

    def resetRoot(self, commitTarget):
        lstNode = self.pathList[0]
        if self.getDis(lstNode, commitTarget) < lstNode.radius:
            self.global_navi_status = True
            return
        
        cost_reduction = 0
        self.commit_root = commitTarget
        cutlist = []

        for node in self.nodeList:
            node.best = False
        
        del_root = False

        for node in self.pathList:
            if not del_root and self.getDis(node, commitTarget) < node.radius - 0.1:
                del_root = True
                node.best = True
                node.preNode = None
                cost_reduction = node.g
                self.root = node
                continue
            
            if del_root:
                node.best = False
                node.valid = False
                cutlist.append(node)
        
        self.solutionUpdate(cost_reduction, commitTarget)

        for node in cutlist:
            self.invalidSet.append(node)
            self.clearBranchW(node)

        self.removeInvalid()

    def solutionUpdate(self, cost_reduction, target):
        for node in self.nodeList:
            node.g -= cost_reduction
        
        self.min_distance = self.getDis(target, self.endPt)


        translation_inf = (target + self.endPt)/2
        downward = np.array([0, 0, -1])
        xtf = np.array(np.array(target) - np.array(translation_inf))
        xtf = xtf/np.linalg.norm(xtf)
        ytf = np.cross(xtf, downward)
        ytf = ytf/np.linalg.norm(ytf)
        ztf = np.cross(xtf, ytf)
        
        self.rotation_inf[:,0] = xtf
        self.rotation_inf[:,1] = ytf
        self.rotation_inf[:,2] = ztf

        self.best_distance -= cost_reduction

    def safeRegionExplansion(self, time_limit):
        time_bef_expand = time.time()
        self.commit_root = self.startPt
        self.root = Node()
        self.root.coordinates = self.startPt
        self.root.radius = self.radiusSearch(self.startPt)
        self.root.f = self.min_distance

        self.recordNode(self.root)
        self.rrtTree.insert(self.root.coordinates, self.root)
        print("root inserted:")
        for i in range(0, self.max_samples):
            time_in_expand = time.time()
            if (time_in_expand - time_bef_expand) > time_limit:
                break
            pt_sample = self.genSample()
            node_nearest_ptr = self.findNearestVertex(pt_sample)

            if (not node_nearest_ptr.valid ) or node_nearest_ptr is None:
                continue

            node_new_ptr = self.genNewNode(pt_sample, node_nearest_ptr)
            if node_new_ptr.coordinates[2] < self.z_l or node_new_ptr.radius < self.safety_Margin:
                continue

            self.treeRewire(node_new_ptr, node_nearest_ptr)
            
            if not node_new_ptr.valid:
                continue
            
            if self.checkEnd(node_new_ptr):
                if not self.informStatus:
                    self.best_endPtr = node_new_ptr

                self.endList.append(node_new_ptr)
                # print("node appended in endlist: ", node_new_ptr.coordinates, node_new_ptr.radius)
                self.updateHeuristic(node_new_ptr)
                self.informStatus = True

            pos = node_new_ptr.coordinates
            self.rrtTree.insert(pos, node_new_ptr)
            # print("coordinates: ", node_new_ptr.coordinates, "radius: ", node_new_ptr.radius)
            # print("node valid? ", node_new_ptr.valid)

            self.recordNode(node_new_ptr)
            self.treePrune(node_new_ptr)

            if len(self.invalidSet) >= self.cach_size:
                self.removeInvalid()
        
        self.removeInvalid()
        self.tracePath()

    def safeRegionRefine(self, time_limit):
        #   Every time the refine function is called, new samples are continuously generated and the tree is continuously rewired, hope to get a better solution
        #   The refine process is mainly same as the initialization of the tree

        time_bef_refine = time.time()
        pos = np.array([0, 0, 0])
        while True:
            time_in_refine = time.time()
            if time_in_refine - time_bef_refine > time_limit:
                break
            
            pt_sample = self.genSample()
            node_nearest_ptr = self.findNearestVertex(pt_sample)

            if not node_nearest_ptr.valid or node_nearest_ptr is None:
                continue

            node_new_ptr = self.genNewNode(pt_sample, node_nearest_ptr)
            if node_new_ptr.coordinates[2] > self.z_l or node_new_ptr.radius < self.safety_Margin:
                continue

            self.treeRewire(node_new_ptr, node_nearest_ptr)

            if not node_new_ptr.valid:
                continue

            if self.checkEnd(node_new_ptr):
                if not self.informStatus:
                    self.best_endPtr = node_new_ptr
                
                self.endList.append(node_new_ptr)
                self.updateHeuristic(node_new_ptr)
                self.informStatus = True
            
            pos = node_new_ptr.coordinates
            self.rrtTree.insert(pos, node_new_ptr)

            self.recordNode(node_new_ptr)
            self.treePrune(node_new_ptr)

            if len(self.invalidSet) >= self.cach_size:
                self.removeInvalid()
        
        self.removeInvalid()
        self.tracePath()

    def safeRegionEvaluate(self, time_limit):
        # The re-evaluate of the RRT*, find the best feasble path (corridor) by lazy-evaluzation
        if self.path_exist_status == False:
            print("[WARNING] no path exists")
            return
        
        time_bef_evaluation = time.time()
        fail_node_list = []
        while True:
            for i in range(0, len(self.pathList())):
                ptr = self.pathList[i]
                pre_ptr = ptr.preNode
                
                if not pre_ptr:
                    update_radius = self.checkRadius(ptr.coordinates)
                    ret = self.checkNodeUpdate(update_radius, ptr.radius) # -1: not qualified, 0: shrink, 1: no difference, continue
                    old_radius = ptr.radius
                    ptr.radius = update_radius

                    if ret == -1:
                        # The node ptr now does't have enough volume for the robot to pass, the node should be deleted from the tree;
                        # all its child nodes should be put in the rewire list
                        ptr.valid = False # Delete this node
                        self.invalidSet.append(ptr)
                        self.clearBranchS(ptr)
                        fail_node_list.append([ptr.coordinates, old_radius])

                    else:
                        # Test whether the node disconnected with its parent and all children
                        # If disconnect with parent, delete it, if discoonect with a child, delete the child
                        if self.checkNodeRelation(self.getDis(ptr, pre_ptr), ptr, pre_ptr):
                            # the child is disconnected with its parent
                            if ptr.valid:
                                ptr.valid = False
                                self.invalidSet.append(ptr)
                                self.clearBranchS(ptr)
                                fail_node_list.append([ptr.coordinates, old_radius])
                        else:
                            childList = ptr.childNodes
                            for childnode in childList:
                                # inspect each child of ptr, to see whether they are still connected
                                res = self.checkNodeRelation(self.getDis(ptr, childnode), ptr, childnode)
                                if res != -1:
                                    if childnode.valid:
                                        childnode.valid = False
                                        self.invalidSet.append(childnode)
                                        self.clearBranchS(childnode)
                                        fail_node_list.append([childnode.coordinates, childnode.radius])
        
    
            isbreak = True
            for ptr in self.pathList:
                isbreak = isbreak and ptr.valid
            
            if isbreak:
                break

            feasibleEndList = []
            for endptr in self.endList:
                if not endptr.valid or not self.checkEnd(endptr):
                    continue
                else:
                    feasibleEndList.append(endptr)

            self.endList = feasibleEndList
            time_in_evaluate = time.time()
            
            if len(feasibleEndList) == 0 or (time_in_evaluate - time_bef_evaluation) > time_limit:
                self.path_exist_status = False
                self.informStatus = False
                self.best_distance = np.inf
                break
            else:
                self.best_endPtr = feasibleEndList[0]
                self.best_cost = np.inf
                for nodeptr in feasibleEndList:
                    cost = nodeptr.g + self.getDis(nodeptr, self.endPt) + self.getDis(self.root, self.commit_root)
                    if cost < self.best_cost:
                        self.best_endPtr = nodeptr
                        self.best_cost = cost
                        self.best_distance = self.best_cost
                
                self.pathList.clear()
                ptrr = self.best_endPtr
                while ptrr is not None:
                    self.pathList.append(ptrr)
                    ptrr = ptrr.preNode

        time_aft_evaluation = time.time()
        repair_limit = time_limit - (time_aft_evaluation - time_bef_evaluation)

        self.removeInvalid()
        # self.treeRepair(repair_limit, fail_node_list) # TODO implementation
        self.tracePath()

    # def treeRepair(self, time_limit, node_list):
    #     #  This function is optional 
    #     #  Use this function to improve the success rate of the tree refinement 

    #     #  check nearby nodes in all the failure nodes' neighbour, 
    #     #  try to repair the tree, since where the nodes failed should most possibly fail again
    #     repairsize = len(node_list)
    #     time_bef_repair = time.time()
    #     for i in range(0, repairsize):
    #         time_in_repair = time.time()
    #         if time_in_repair - time_bef_repair > time_limit:
    #             break

    #         center = node_list[i][0]
    #         range = node_list[i][1]*2.0
    #         pos = center
    #         presults = self.rrtTree.nearest_range(pos, range)

    #         while presults is not None:
    #             ptr = presults.data
    #             if not ptr.valid:



    def treePrune(self, newPtr):
        ptr = newPtr
        if ptr.g + ptr.f > self.best_distance:
            ptr.valid = False
            self.invalidSet.append(ptr)
            self.clearBranchS(ptr)


    def treeRewire(self, new_ptr, nearest_ptr):
        # Step 1: Set up the range for searching nearby nodes (twice the new node's radius)
        search_radius = new_ptr.radius * 2.0
        search_point = new_ptr.coordinates

        # Step 2: Find all nodes within the search radius using the KDTree
        nearby_nodes = self.rrtTree.nearest_range(search_point, search_radius)
        
        # Temporary list to store nearby nodes and their relation
        nearPtrList = []
        is_invalid = False

        # Step 3: Check the relations between the new node and nearby nodes
        for near_ptr in nearby_nodes:
            near_ptr_node = near_ptr.data
            # Calculate the distance between the nearby node and the new node
            dis = self.getDis(near_ptr_node, new_ptr)
            
            # Determine the relation (res) between the nearby node and the new node
            res = self.checkNodeRelation(dis, near_ptr_node, new_ptr)
            near_ptr_node.rel_id = res  # Store relation temporarily
            near_ptr_node.rel_dis = dis  # Store distance temporarily

            # If the new node is contained within a nearby node, mark it as invalid
            if res == 1:
                new_ptr.valid = False
                is_invalid = True
                nearPtrList.append(near_ptr_node)
                break
            else:
                nearPtrList.append(near_ptr_node)

        # Step 4: If the new node is invalid, reset temporary variables and return
        if is_invalid:
            for nodeptr in nearPtrList:
                nodeptr.rel_id = -2
                nodeptr.rel_dis = -1.0
            return
        
        # Step 5: Otherwise, set the nearest node as the parent of the new node
        min_cost = nearest_ptr.g + self.getDis(nearest_ptr, new_ptr)
        new_ptr.preNode = nearest_ptr
        new_ptr.g = min_cost
        nearest_ptr.childNodes.append(new_ptr)
        
        lstParentPtr = nearest_ptr  # Record the last best parent of the new node

        # Step 6: Attempt to rewire the nearby nodes
        for near_ptr in nearPtrList:
            if near_ptr.rel_id == -1:
                cost = near_ptr.g + near_ptr.rel_dis
                if cost < min_cost:
                    # If a nearby node provides a shorter path, reassign the parent of the new node
                    min_cost = cost
                    new_ptr.preNode = near_ptr
                    new_ptr.g = min_cost
                    lstParentPtr.childNodes.remove(new_ptr)  # Remove the new node from the old parent's children
                    lstParentPtr = near_ptr  # Update the last parent
                    lstParentPtr.childNodes.append(new_ptr)

        # Step 7: Rewire the nearby nodes to the new node if it provides a shorter path
        for near_ptr in nearPtrList:
            if not near_ptr.valid:
                continue
            
            dis = self.getDis(near_ptr, new_ptr)
            cost = dis + new_ptr.g

            # If the new node provides a shorter path to the nearby node, rewire the nearby node
            if cost < near_ptr.g:
                if self.isSuccessor(near_ptr, new_ptr.preNode):
                    continue  # Prevent loops

                if near_ptr.preNode is None:
                    near_ptr.preNode = new_ptr
                    near_ptr.g = cost
                else:
                    # Update the parent's child list
                    lstNearParent = near_ptr.preNode
                    near_ptr.preNode = new_ptr
                    near_ptr.g = cost

                    # Mark the node for deletion from its old parent's child list
                    near_ptr.change = True
                    lstNearParent.childNodes = [child for child in lstNearParent.childNodes if not child.change]
                    near_ptr.change = False  # Reset the flag after deletion

                # Add the rewired node as a child of the new node
                new_ptr.childNodes.append(near_ptr)

    def clearBranchW(self, node_delete):
        for nodeptr in node_delete.childNodes:
            if nodeptr.best:
                continue
            else:
                if nodeptr.valid:
                    self.invalidSet.append(nodeptr)
                
                nodeptr.valid = False
                self.clearBranchW(nodeptr)

    def clearBranchS(self, node_delete):
        for nodeptr in node_delete.childNodes:
            if nodeptr.valid:
                self.invalidSet.append(nodeptr)
                nodeptr.valid = False
                self.clearBranchS(nodeptr)

    def updateHeuristic(self, update_end_node):
        #  This function update the heuristic hype-ellipsoid sampling region once and better path has been found  */
        #  Update the up-to-date traversal and conjugate diameter of the ellipsoid.
        #  If there is no improvement in the path, maintain the heuristic unchange.
        update_cost = update_end_node.g + self.getDis(update_end_node, self.endPt) + self.getDis(self.startPt, self.commit_root) 
        if update_cost < self.best_distance:
            self.best_distance = update_cost
            self.elli_l = self.best_distance
            self.elli_s = np.sqrt(self.best_distance*self.best_distance - self.min_distance*self.min_distance)

            if self.informStatus:
                for node in self.nodeList:
                    node.best = False
            
            ptr = update_end_node
            while ptr is not None:
                ptr.best = True
                ptr = ptr.preNode
            
            self.best_endPtr = update_end_node


    def recordNode(self, new_node):
        self.nodeList.append(new_node)

    def removeInvalid(self):
        UpdateNodeList = []
        UpdateEndList = []
        self.rrtTree.clear()

        for node in self.nodeList:
            if node.valid:
                self.rrtTree.insert(node.coordinates, node)
                UpdateNodeList.append(node)

                if self.checkEnd(node):
                    UpdateEndList.append(node)
        
        self.nodeList.clear()
        self.endList.clear()

        self.nodeList = UpdateNodeList
        self.endList = UpdateEndList

        for node in self.invalidSet:
            if node.preNode != None:
                node.change = True

                child_lst = node.preNode.childNodes
                node.preNode.childNodes.clear()
                for cnode in child_lst:
                    if cnode.change:
                        continue
                    else:
                        node.preNode.childNodes.append(cnode)
        
        delete_list = []
        for node in self.invalidSet:
            for cnode in node.childNodes:
                if cnode.valid:
                    cnode.preNode = None
            
            delete_list.append(node)
        
        self.invalidSet.clear()
        for delnodes in delete_list:
            del delnodes

    def treeDestruct(self):
        if self.rrtTree:
            self.rrtTree.clear()

        # Delete all nodes from the NodeList
        self.nodeList.clear()

    def tracePath(self):
        feasibleEndList = []
        for endnode in self.endList:
            if (not self.checkValidEnd(endnode)) or (not self.checkEnd(endnode)) or (not endnode.valid):
                continue
            else:
                feasibleEndList.append(endnode)

        if len(feasibleEndList) == 0:
            self.path_exist_status = False
            self.best_distance = False
            self.informStatus = False
            self.endList.clear()
            self.path_matrix = np.identity(3)
            self.radius_matrix = np.zeros(3)

            return

        self.endList = feasibleEndList

        self.best_endPtr = feasibleEndList[0]
        print("feasible list length", len(feasibleEndList))
        best_cost = np.inf
        for node in feasibleEndList:
            cost = node.g + self.getDis(node, self.endPt) + self.getDis(self.root, self.commit_root)
            if cost < best_cost:
                self.best_endPtr = node
                best_cost = cost
                self.best_distance = best_cost

        ptr = self.best_endPtr
        temp_ptr = ptr
        path_length = 0
        while temp_ptr is not None:
            path_length += 1
            temp_ptr = temp_ptr.preNode
        
        print(f"Path length: {path_length}")

        # Initialize path_matrix and radius_matrix based on the length of the path
        self.path_matrix = np.zeros((path_length, 3))
        self.radius_matrix = np.zeros(path_length)

        idx = 0
        self.pathList.clear()

        # Now traverse the path again to populate path_matrix and radius_matrix
        while ptr is not None:
            print("ptr coordinates: ", ptr.coordinates, ptr.radius)
            self.path_matrix[-(idx+1), :] = ptr.coordinates  # Fill the Path matrix in reverse order
            self.radius_matrix[-(idx+1)] = ptr.radius  # Fill the Radius vector in reverse order
            self.pathList.append(ptr)
            ptr = ptr.preNode
            idx += 1

        self.path_exist_status = True

    def getDis(self, node1, node2):
        if isinstance(node1, Node) and isinstance(node2, Node):
            return np.linalg.norm(np.array(node1.coordinates) - np.array(node2.coordinates))
        elif isinstance(node1, Node):
            return np.linalg.norm(np.array(node1.coordinates) - np.array(node2))
        elif isinstance(node2, Node):
            return np.linalg.norm(np.array(node2.coordinates) - np.array(node1))
        else:
            return np.linalg.norm(np.array(node1) - np.array(node2))


    def genSample(self):
        pt = np.zeros(3)
        
        # Generate a random bias value
        bias = random.uniform(self.bias_l, self.bias_h)
        
        if bias <= self.goal_ratio:
            # Sample the goal point directly
            pt = self.endPt
            return pt

        if not self.informStatus:
            # If not using the heuristic, sample in the local or global region
            if bias > self.goal_ratio and bias <= (self.goal_ratio + self.inlier_ratio):
                # Sample inside the local map's boundary (near the start point)
                pt[0] = random.uniform(self.startPt[0] - self.sample_range, self.startPt[0] + self.sample_range)
                pt[1] = random.uniform(self.startPt[1] - self.sample_range, self.startPt[1] + self.sample_range)
                pt[2] = random.uniform(self.startPt[2] - self.sample_range, self.startPt[2] + self.sample_range)
            else:
                # Uniformly sample in the whole region
                pt[0] = random.uniform(self.x_l, self.x_h)
                pt[1] = random.uniform(self.y_l, self.y_h)
                pt[2] = random.uniform(self.z_l, self.z_h)
        else:
            # Sample in a heuristic 3D ellipsoid
            us = random.uniform(0, 1)
            vs = random.uniform(0, 1)
            phis = random.uniform(0, 2 * np.pi)

            # Inverse CDF sampling for ellipsoid
            as_ = self.elli_l / 2.0 * np.cbrt(us)
            bs = self.elli_s / 2.0 * np.cbrt(us)
            thetas = np.arccos(1 - 2 * vs)

            # Generate points in ellipsoid
            pt[0] = as_ * np.sin(thetas) * np.cos(phis)
            pt[1] = bs * np.sin(thetas) * np.sin(phis)
            pt[2] = bs * np.cos(thetas)

            # Apply rotation and translation to move the point into the ellipsoid region
            pt = self.rotation_inf @ pt + self.translation_inf

            # Constrain the point to be within the bounds
            pt[0] = np.clip(pt[0], self.x_l, self.x_h)
            pt[1] = np.clip(pt[1], self.y_l, self.y_h)
            pt[2] = np.clip(pt[2], self.z_l, self.z_h)

        return pt


    def radiusSearch(self, node):
        if self.getDis(node, self.startPt) > self.sample_range + self.max_Radius:
            return self.max_Radius - self.search_Margin

        if isinstance(node, Node):
            radius, index = self.pcd.query(np.array(node.coordinates), 1)
            radius = np.sqrt(radius) - self.search_Margin
            return min(radius, self.max_Radius)
        
        else:
            radius, index = self.pcd.query(np.array(node), 1)
            radius = np.sqrt(radius) - self.search_Margin
            return min(radius, self.max_Radius)

    def genNewNode(self, node_vec, node_nearest):
        dis = self.getDis(node_vec, node_nearest)
        center = np.zeros(3)
        if dis > node_nearest.radius:
            steer_dis = node_nearest.radius/dis
            center = node_nearest.coordinates + (node_vec - node_nearest.coordinates)*steer_dis
        else:
            center = node_vec
        
        radius = self.radiusSearch(center)
        h_dis = self.getDis(center, self.endPt)

        node_new = Node()
        node_new.coordinates = center
        node_new.radius = radius
        # print("new node radius: ", node_new.radius, " coordinates: ", node_new.coordinates)
        node_new.g = np.inf
        node_new.f = h_dis

        return node_new

    def findNearestVertex(self, node):
        if isinstance(node, Node):
            nearest_node, nearest_dist = self.rrtTree.nearest(node.coordinates)
        else:
            nearest_node, nearest_dist = self.rrtTree.nearest(node)
        return nearest_node.data

    # def findNearest(self, node): # function initialized but not implemented

    def checkRadius(self, node):
        return self.radiusSearch(node)

    def checkValidEnd(self, endPtr):
        ptr = endPtr

        while ptr is not None:
            if not ptr.valid:
                return False
            if self.getDis(ptr, self.root) < ptr.radius:
                return True
            
            ptr = ptr.preNode

        return False

    def checkEnd(self, node):
        distance = self.getDis(node, self.endPt)
        if distance + 0.1 < node.radius:
            return True
        
        # print("node far away from endpt: ", distance, " node radius: ", node.radius)
        return False

    def checkNodeUpdate(self, new_radius, old_radius):
        if new_radius < self.safety_Margin:
            return -1   # for not qualified, this node should be deleted
        elif new_radius < old_radius:
            return 0    # for node radius shrink, this node should be inspected for connection with others
        else:
            return 1    # for no difference, nothing should be changed

    def checkNodeRelation(self, dis, node1, node2):
        if dis+node2.radius == node1.radius:
            return 1
        elif (dis+0.1) < 0.95*(node1.radius + node2.radius):
            return -1
        else:
            return 0

    def isSuccessor(self, curNode, nearNode):
        prePtr = nearNode.preNode
        while prePtr is not None:
            if prePtr == curNode:
                return True
            else:
                prePtr = prePtr.preNode
        
        return False

    def checkTrajPtCol(self, node):
        if self.radiusSearch(node) < 0.0:
            return True
        else:
            return False

    def getPath(self):
        return self.path_matrix, self.radius_matrix

    def getTree(self):
        return self.NodeList

    def getPathExistStatus(self):
        return self.path_exist_status

    def getGlobalNaviStatus(self):
        return self.global_navi_status
    
    def plotAllNodes(self):
        # Extract all node coordinates and radii
        node_coordinates = np.array([node.coordinates for node in self.nodeList])
        node_radii = np.array([node.radius for node in self.nodeList])

        # Create a trace for the start point
        start_trace = go.Scatter3d(
            x=[self.startPt[0]],
            y=[self.startPt[1]],
            z=[self.startPt[2]],
            mode='markers',
            marker=dict(size=10, color='green'),
            name='Start'
        )

        # Create a trace for the end point
        end_trace = go.Scatter3d(
            x=[self.endPt[0]],
            y=[self.endPt[1]],
            z=[self.endPt[2]],
            mode='markers',
            marker=dict(size=10, color='yellow'),
            name='End'
        )

        # Create a list to store the spheres (each node visualized as a transparent sphere)
        sphere_traces = []

        # Function to create a 3D sphere
        def create_sphere(x_center, y_center, z_center, radius):
            # Create the meshgrid for sphere
            u = np.linspace(0, 2 * np.pi, 20)
            v = np.linspace(0, np.pi, 20)
            x = x_center + radius * np.outer(np.cos(u), np.sin(v))
            y = y_center + radius * np.outer(np.sin(u), np.sin(v))
            z = z_center + radius * np.outer(np.ones(np.size(u)), np.cos(v))
            return x, y, z

        # Iterate over each node to create a transparent sphere for each one
        for i, (coord, radius) in enumerate(zip(node_coordinates, node_radii)):
            x, y, z = create_sphere(coord[0], coord[1], coord[2], radius)
            sphere_trace = go.Surface(
                x=x,
                y=y,
                z=z,
                opacity=0.5,  # Make spheres transparent
                colorscale=[[0, 'cyan'], [1, 'cyan']],  # Set the color of the spheres
                showscale=False,
                name=f'Node {i+1}'
            )
            sphere_traces.append(sphere_trace)

        # Create the layout for the plot
        layout = go.Layout(
            title="RRT Nodes and Safe Regions",
            scene=dict(
                xaxis=dict(title='X'),
                yaxis=dict(title='Y'),
                zaxis=dict(title='Z')
            ),
            showlegend=True
        )

        # Combine the start point, end point, and spheres into one figure
        fig = go.Figure(data=[start_trace, end_trace] + sphere_traces, layout=layout)

        # Display the figure
        fig.show()
    

def load_pcd_from_csv(file_path):
    """
    Reads a TXT or CSV file containing point cloud data with x, y, z coordinates.
    
    Parameters:
        file_path (str): Path to the TXT or CSV file containing point cloud data.
    
    Returns:
        numpy array
    """
    # Read the file into a Pandas DataFrame
    df = pd.read_csv(file_path, sep=",", header=None, skiprows = 1)
    print("file being read: ", file_path)
    
    # Assume the first three columns are x, y, z coordinates
    points = df.iloc[:, :3].values
    points = np.array(points)
    return points
def main():
    # Load point cloud
    file_path = "/home/astik/pcd_outputs/2024-09-17_12-03-50/pcd_0000.csv"  # Replace with your file path
    point_cloud = load_pcd_from_csv(file_path)
    
    # Initialize the RRT planner
    rrt = RRT_start(safety_margin=1.0, search_margin=0.5, max_radius=1.5, sample_range=10.0)
    
    # Set the input point cloud
    rrt.setInput(point_cloud)
    
    # Define start and end points for the path
    start_point = np.array([0, 0, 0.5])
    end_point = np.array([0, 10, 0.5])
    
    # Set other parameters (bounding box, range, etc.)
    rrt.setPt(startPt=start_point, endPt=end_point, xl=-5, xh=15, yl=-5, yh=15, zl=0.0, zh=1,
              local_range=10, max_iter=1000, sample_portion=0.1, goal_portion=0.05)

    # Plan a path within a time limit (seconds)
    rrt.safeRegionExplansion(time_limit=50)
    # Get the generated path and check if it exists
    if rrt.getPathExistStatus():
        path, radii = rrt.getPath()
        print(radii[len(radii) - 1])
        # Plot the nodes in the final path, the path itself, and the point cloud using Plotly
        plot_final_path(rrt, path, point_cloud)
    else:
        print("No valid path found.")

        # If no valid path is found, plot all the nodes for debugging
        rrt.plotAllNodes()

def plot_final_path(rrt, path, point_cloud):
    import plotly.graph_objs as go

    # Extract path coordinates
    path_coordinates = np.array(path)

    # Create a trace for the path
    path_trace = go.Scatter3d(
        x=path_coordinates[:, 0],
        y=path_coordinates[:, 1],
        z=path_coordinates[:, 2],
        mode='lines+markers',
        line=dict(color='red', width=4),
        marker=dict(size=5, color='red'),
        name='Final Path'
    )

    # Create a trace for the start point
    start_trace = go.Scatter3d(
        x=[rrt.startPt[0]],
        y=[rrt.startPt[1]],
        z=[rrt.startPt[2]],
        mode='markers',
        marker=dict(size=10, color='green'),
        name='Start'
    )

    # Create a trace for the end point
    end_trace = go.Scatter3d(
        x=[rrt.endPt[0]],
        y=[rrt.endPt[1]],
        z=[rrt.endPt[2]],
        mode='markers',
        marker=dict(size=10, color='yellow'),
        name='End'
    )

    # Create a trace for the point cloud
    pcd_trace = go.Scatter3d(
        x=point_cloud[:, 0],
        y=point_cloud[:, 1],
        z=point_cloud[:, 2],
        mode='markers',
        marker=dict(size=2, color='blue'),
        name='Point Cloud'
    )

    # Plot only the nodes in the final path as spheres
    sphere_traces = plot_spheres(rrt.pathList)

    # Combine the traces into a single plot
    fig = go.Figure(data=[pcd_trace, start_trace, end_trace, path_trace] + sphere_traces)

    # Create buttons for toggling the spheres
    buttons = [
        dict(
            args=["visible", [True, True, True, True] + [True] * len(sphere_traces)],  # Show spheres
            label="Show Spheres",
            method="restyle"
        ),
        dict(
            args=["visible", [True, True, True, True] + [False] * len(sphere_traces)],  # Hide spheres
            label="Hide Spheres",
            method="restyle"
        )
    ]

    # Get the ranges for each axis
    x_range = [min(point_cloud[:, 0]), max(point_cloud[:, 0])]
    y_range = [min(point_cloud[:, 1]) - 6 , max(point_cloud[:, 1])]
    z_range = [min(point_cloud[:, 2]), max(point_cloud[:, 2])+10]

    # Calculate the scale factor for each axis
    max_range = max(x_range[1] - x_range[0], y_range[1] - y_range[0], z_range[1] - z_range[0])

    # Set equal aspect ratio for the plot to prevent skewing
    fig.update_layout(
        title="Final Path and Nodes",
        scene=dict(
            xaxis=dict(title='X', range=x_range),
            yaxis=dict(title='Y', range=y_range),
            zaxis=dict(title='Z', range=z_range),
            aspectmode='manual',  # Manually control the aspect ratio
            aspectratio=dict(x=(x_range[1] - x_range[0]) / max_range,
                             y=(y_range[1] - y_range[0]) / max_range,
                             z=(z_range[1] - z_range[0]) / max_range)
        ),
        showlegend=True
    )

    # Show the figure
    fig.show()


def plot_spheres(nodes):
    import numpy as np
    import plotly.graph_objs as go

    # Extract all node coordinates and radii for the nodes in the final path
    node_coordinates = np.array([node.coordinates for node in nodes])
    node_radii = np.array([node.radius for node in nodes])

    # Create a list to store the spheres (each node visualized as a transparent sphere)
    sphere_traces = []

    # Function to create a 3D sphere
    def create_sphere(x_center, y_center, z_center, radius):
        # Create the meshgrid for the sphere
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        x = x_center + radius * np.outer(np.cos(u), np.sin(v))
        y = y_center + radius * np.outer(np.sin(u), np.sin(v))
        z = z_center + radius * np.outer(np.ones(np.size(u)), np.cos(v))
        return x, y, z

    # Iterate over each node in the final path to create a transparent sphere for each one
    for i, (coord, radius) in enumerate(zip(node_coordinates, node_radii)):
        x, y, z = create_sphere(coord[0], coord[1], coord[2], radius)
        sphere_trace = go.Surface(
            x=x,
            y=y,
            z=z,
            opacity=0.5,  # Make spheres transparent
            colorscale=[[0, 'cyan'], [1, 'cyan']],  # Set the color of the spheres
            showscale=False,
            name=f'Node {i+1}',
            visible=True  # Initially visible
        )
        sphere_traces.append(sphere_trace)

    return sphere_traces

if __name__ == "__main__":
    main()


