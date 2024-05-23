import os
import cv2
import numpy as np
from utils import plot_one_box, cal_iou, xyxy_to_xywh, xywh_to_xyxy, updata_trace_list, draw_trace
import datetime


class KalmanFilter(object):
    def __init__(self):
        # 状态转移矩阵，上一时刻的状态转移到当前时刻
        self.A = np.array([[1, 0, 0, 0, 1, 0],
                      [0, 1, 0, 0, 0, 1],
                      [0, 0, 1, 0, 0, 0],
                      [0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1]])

        # 状态观测矩阵
        self.H = np.eye(6)

        # 过程噪声协方差矩阵Q，p(w)~N(0,Q)，噪声来自真实世界中的不确定性,
        # 在跟踪任务当中，过程噪声来自于目标移动的不确定性（突然加速、减速、转弯等）
        self.Q = np.eye(6) * 0.1

        # 观测噪声协方差矩阵R，p(v)~N(0,R)
        # 观测噪声来自于检测框丢失、重叠等
        self.R = np.eye(6) * 1

        # 控制输入矩阵B
        self.B = None
        # 状态估计协方差矩阵P初始化
        self.P = np.eye(6)

    def predict(self, X, cov):
        """
             预测步骤：根据先验状态和协方差矩阵进行状态预测。

             :param X: 先验状态
             :param cov: 先验协方差矩阵
             :return: 预测后的状态和协方差矩阵
        """
        X_predict = np.dot(self.A, X)
        cov1 = np.dot(self.A, cov)
        cov_predict = np.dot(cov1, self.A.T) + self.Q
        return X_predict, cov_predict
    def update(self, X_predict, cov_predict, Z):
        """
        更新步骤：根据观测值进行状态更新。

        :param X_predict: 先验状态
        :param cov_predict: 先验协方差矩阵
        :param Z: 当前观测到的状态
        :return: 后验状态和后验协方差矩阵
        """
        # ------计算卡尔曼增益---------------------
        # Z是当前观测到的状态
        k1 = np.dot(cov_predict, self.H.T)
        k2 = np.dot(np.dot(self.H, cov_predict), self.H.T) + self.R
        K = np.dot(k1, np.linalg.inv(k2))
        # --------------后验估计------------
        X_posterior_1 = Z - np.dot(self.H, X_predict)
        X_posterior = X_predict + np.dot(K, X_posterior_1)
        # box_posterior = xywh_to_xyxy(X_posterior[0:4])
        # plot_one_box(box_posterior, frame, color=(255, 255, 255), target=False)
        # ---------更新状态估计协方差矩阵P-----
        P_posterior_1 = np.eye(6) - np.dot(K, self.H)
        P_posterior = np.dot(P_posterior_1, cov_predict)
        return X_posterior, P_posterior


if __name__ == "__main__":
    pass