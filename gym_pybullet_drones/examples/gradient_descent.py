import numpy as np


class LossFunction:
    def __init__(self, x, usv_coord):
        self.x = x
        self.USV_coord = usv_coord

    @staticmethod
    def communication_quality_function(x, usv_coord):
        norm = np.linalg.norm(x[:, :, None] - x[:, None], axis=-1) ** 2
        uav_sum_dist = np.sum(norm.reshape(norm.shape[0], -1), axis=1) / 2
        uav_usv_sum_dist = np.sum(np.min(np.linalg.norm(x[:, :, None] - usv_coord[:, None], axis=-1), axis=1) ** 2,
                                  axis=1)
        return uav_usv_sum_dist + 0.5 * uav_sum_dist
