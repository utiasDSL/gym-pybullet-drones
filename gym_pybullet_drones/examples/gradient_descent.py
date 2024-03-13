import numpy as np


class GradientDescent:
    def __init__(self, x, USV_coord, learning_rate=0.1, num_iterations=100):
        self.x = x
        self.USV_coord = USV_coord
        self.learning_rate = learning_rate
        self.num_iterations = num_iterations

    def loss_function(self, x, USV_coord):
        return np.sum(np.min(np.linalg.norm(x[:, :, None] - USV_coord[:, None], axis=-1), axis=1) ** 2, axis=1)

    def gradient(self, x, USV_coord):
        distances = np.linalg.norm(x[:, :, None] - USV_coord[:, None], axis=-1)
        print(distances.shape)
        min_distances = np.min(distances, axis=1)
        print("min", min_distances.shape)
        diff = x[:, :, None] - USV_coord[:, None]
        print("diff", diff.shape)
        f = np.transpose(np.tile(distances[:, :, None], (3, 1)).T, (3, 2, 0, 1))
        print("f", f.shape)
        min_f = np.transpose(np.tile(min_distances[:, None, None], (3, 1)).T, (3, 2, 0, 1))
        print(min_distances[:, None, None].shape)
        grad = 2 * np.sum((diff / f) * min_f, axis=2)
        print(grad.shape)
        return grad

    def gradient_descent(self, x, USV_coord, learning_rate, num_iterations):
        for i in range(num_iterations):
            grad = self.gradient(x, USV_coord)
            x -= learning_rate * grad
        return x
    def gradient(x, usv_coord):
        dist_x = x[::-1] - x
        dist_norm_x = np.linalg.norm(dist_x, axis=-1)
        dist_sum  = np.sum(dist_norm_x**2)
        distances = np.linalg.norm(x[:, None] - usv_coord[None], axis=-1)
        min_distances = np.min(distances, axis=1)
        diff = x[:, None] - usv_coord[None]
        f = np.tile(distances[:, :, None], (1, 3))
        min_f = np.tile(min_distances[:, None, None], (4, 3))
        grad = 2 * np.sum((diff / f) * min_f, axis=1) #+ np.sum(diff_uav/new_dist, axis=1))

        return grad

    def gradient(x, USV_coord):
        distances = np.linalg.norm(x[:, :, None] - USV_coord[:, None], axis=-1)
        min_distances = np.min(distances, axis=1)
        diff = x[:, :, None] - USV_coord[:, None]
        f = np.transpose(np.tile(distances[:, :, None], (3, 1)).T, (3, 2, 0, 1))
        min_f = np.transpose(np.tile(min_distances[:, None, None], (3, 1)).T, (3, 2, 0, 1))
        grad = 2 * np.sum((diff / f) * min_f, axis=2)
        return grad


