import numpy as np


class KDNode:
    def __init__(self, pos, data=None):
        self.pos = np.array(pos)
        self.data = data
        self.left = None
        self.right = None
        self.dir = 0  # dimension to split on


class KDTree:
    def __init__(self, dim):
        self.dim = dim
        self.root = None

    def insert(self, pos, data=None):
        if self.root is None:
            self.root = KDNode(pos, data)
        else:
            self._insert(self.root, pos, data, 0)

    def _insert(self, node, pos, data, depth):
        axis = depth % self.dim

        if pos[axis] < node.pos[axis]:
            if node.left is None:
                node.left = KDNode(pos, data)
            else:
                self._insert(node.left, pos, data, depth + 1)
        else:
            if node.right is None:
                node.right = KDNode(pos, data)
            else:
                self._insert(node.right, pos, data, depth + 1)

    def nearest(self, point):
        return self._nearest(self.root, point, 0, None, float('inf'))

    def _nearest(self, node, point, depth, best_node, best_dist):
        if node is None:
            return best_node, best_dist

        axis = depth % self.dim
        dist = np.linalg.norm(point - node.pos)

        if dist < best_dist:
            best_dist = dist
            best_node = node

        diff = point[axis] - node.pos[axis]

        if diff <= 0:
            best_node, best_dist = self._nearest(node.left, point, depth + 1, best_node, best_dist)
            if abs(diff) < best_dist:
                best_node, best_dist = self._nearest(node.right, point, depth + 1, best_node, best_dist)
        else:
            best_node, best_dist = self._nearest(node.right, point, depth + 1, best_node, best_dist)
            if abs(diff) < best_dist:
                best_node, best_dist = self._nearest(node.left, point, depth + 1, best_node, best_dist)

        return best_node, best_dist

    def nearest_range(self, point, range_dist):
        results = []
        self._nearest_range(self.root, point, range_dist, 0, results)
        return results

    def _nearest_range(self, node, point, range_dist, depth, results):
        if node is None:
            return

        axis = depth % self.dim
        dist = np.linalg.norm(point - node.pos)

        if dist <= range_dist:
            results.append(node)

        diff = point[axis] - node.pos[axis]

        if diff <= 0:
            self._nearest_range(node.left, point, range_dist, depth + 1, results)
            if abs(diff) < range_dist:
                self._nearest_range(node.right, point, range_dist, depth + 1, results)
        else:
            self._nearest_range(node.right, point, range_dist, depth + 1, results)
            if abs(diff) < range_dist:
                self._nearest_range(node.left, point, range_dist, depth + 1, results)
    
    def clear(self):
        """Clears the KD-tree by setting the root to None."""
        self._clear_rec(self.root)
        self.root = None

    def _clear_rec(self, node):
        """Recursive helper function to clear the nodes."""
        if node is None:
            return
        # Recursively clear left and right children
        self._clear_rec(node.left)
        self._clear_rec(node.right)
        # Now that the children are cleared, we can delete this node
        del node


# Example usage
if __name__ == "__main__":
    kdtree = KDTree(dim=3)

    # Insert some points
    kdtree.insert([2.1, 3.2, 1.1], "point_1")
    kdtree.insert([3.1, 2.9, 3.1], "point_4")
    kdtree.insert([5.4, 2.8, 1.9], "point_2")
    kdtree.insert([3.5, 7.3, 4.1], "point_3")

    # Find the nearest neighbor to the point [3, 3, 3]
    nearest_node, nearest_dist = kdtree.nearest(np.array([3, 3, 3]))
    print(f"Nearest point: {nearest_node.pos}, Distance: {nearest_dist}, Data: {nearest_node.data}")

    # Find all points within a range of 3 units from [3, 3, 3]
    nodes_in_range = kdtree.nearest_range(np.array([3, 3, 3]), 3.0)
    print("number of nodes in range: ",len(nodes_in_range))
    for node in nodes_in_range:
        print(f"Point in range: {node.pos}, Data: {node.data}")
