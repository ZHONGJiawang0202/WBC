import numpy as np

class LBRiiwa7R800:

    def __init__(self, joint_angles):
        self.joint_angles = joint_angles
        # D-H table
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
        # mettre a jour la configuration de robot
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
        """transform matrix"""
        return np.array([
            [np.cos(theta), -np.sin(theta), 0, a],
            [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha), -d * np.sin(alpha)],
            [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha), d * np.cos(alpha)],
            [0, 0, 0, 1]
        ])

    def forward_kinematics(self):
        # LBR iiwa 7 R800 D-H table
        dh_params = self.dh_params

        T = np.eye(4)
        transforms = []  # transform matrix for every coude
        for i in range(7):
            theta, d, a, alpha = dh_params[i]
            T_next = self.dh_transform(a, alpha, d, theta)
            T = T @ T_next  # 累乘
            transforms.append(T)
        return transforms

    def compute_jacobian(self):
        """Jacobien matrix"""
        transforms = self.forward_kinematics()
        T_end = transforms[6]  # total transform matrix
        p_end = T_end[:3, 3]  # position vector of end effecteur z

        J = np.zeros((6, 7))
        for i in range(7):
            T_i = transforms[i]
            z_i = T_i[:3, 2]  # transform matrix for current coude
            p_i = T_i[:3, 3]  # position vector of current coude z

            J_v = np.cross(z_i, p_end - p_i)
            J_w = z_i

            J[:3, i] = J_v
            J[3:, i] = J_w

        return J















