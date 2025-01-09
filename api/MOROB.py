import numpy as np

class LBRiiwa7R800:

    def __init__(self, joint_angles):
        self.joint_angles = joint_angles
        # 定义D-H参数
        self.dh_params = [
            (joint_angles[0], 0.34, 0, 0),  # Joint 1
            (joint_angles[1], 0, 0, -np.pi / 2),  # Joint 2
            (joint_angles[2], 0.4, 0, np.pi / 2),  # Joint 3
            (joint_angles[3], 0., 0, np.pi / 2),  # Joint 4
            (joint_angles[4], 0.4, 0, -np.pi / 2),  # Joint 5
            (joint_angles[5], 0, 0, -np.pi / 2),  # Joint 6
            (joint_angles[6], 0.126, 0, np.pi / 2)  # Joint 7
        ]

    def bouger(self, q):
        self.joint_angles = q
        self.dh_params = [
            (self.joint_angles[0], 0.34, 0, 0),  # Joint 1
            (self.joint_angles[1], 0, 0, -np.pi / 2),  # Joint 2
            (self.joint_angles[2], 0.4, 0, np.pi / 2),  # Joint 3
            (self.joint_angles[3], 0., 0, np.pi / 2),  # Joint 4
            (self.joint_angles[4], 0.4, 0, -np.pi / 2),  # Joint 5
            (self.joint_angles[5], 0, 0, -np.pi / 2),  # Joint 6
            (self.joint_angles[6], 0.126, 0, np.pi / 2)  # Joint 7
        ]

    def dh_transform(self, a, alpha, d, theta):
        """基于D-H参数计算齐次变换矩阵"""
        return np.array([
            [np.cos(theta), -np.sin(theta), 0, a],
            [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha), -d * np.sin(alpha)],
            [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha), d * np.cos(alpha)],
            [0, 0, 0, 1]
        ])

    def forward_kinematics(self):
        """正向运动学，返回末端的齐次变换矩阵和所有关节的变换"""
        # LBR iiwa 7 R800的D-H参数表
        dh_params = self.dh_params

        T = np.eye(4)  # 初始化为单位矩阵
        transforms = []  # 存储每个关节的齐次变换
        for i in range(7):
            theta, d, a, alpha = dh_params[i]
            T_next = self.dh_transform(a, alpha, d, theta)
            T = T @ T_next  # 累乘
            transforms.append(T)
        return transforms

    def compute_jacobian(self):
        """计算雅可比矩阵"""
        transforms = self.forward_kinematics()
        T_end = transforms[6]  # 末端变换矩阵
        p_end = T_end[:3, 3]  # 末端位置向量

        J = np.zeros((6, 7))  # 初始化雅可比矩阵
        for i in range(7):
            T_i = transforms[i]
            z_i = T_i[:3, 2]  # 当前关节的z轴
            p_i = T_i[:3, 3]  # 当前关节的位置

            J_v = np.cross(z_i, p_end - p_i)  # 线速度部分
            J_w = z_i  # 角速度部分

            J[:3, i] = J_v  # 填充线速度
            J[3:, i] = J_w  # 填充角速度

        return J















