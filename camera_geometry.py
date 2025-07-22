import numpy as np
from scipy.interpolate import LinearNDInterpolator
from typing import List, Tuple
import time
import json


def get_intrinsic_matrix(field_of_view_deg, image_width, image_height):
    # For our Carla camera alpha_u = alpha_v = alpha
    # alpha can be computed given the cameras field of view via
    field_of_view_rad = field_of_view_deg * np.pi / 180
    alpha = (image_width / 2.0) / np.tan(field_of_view_rad / 2.0)
    Cu = image_width / 2.0
    Cv = image_height / 2.0
    return np.array([[alpha, 0, Cu], [0, alpha, Cv], [0, 0, 1.0]])


def project_polyline(polyline_world, trafo_world_to_cam, K):
    x, y, z = polyline_world[:, 0], polyline_world[:, 1], polyline_world[:, 2]
    homvec = np.stack((x, y, z, np.ones_like(x)))
    proj_mat = K @ trafo_world_to_cam[:3, :]
    pl_uv_cam = (proj_mat @ homvec).T
    u = pl_uv_cam[:, 0] / pl_uv_cam[:, 2]
    v = pl_uv_cam[:, 1] / pl_uv_cam[:, 2]
    return np.stack((u, v)).T


class CameraGeometry(object):
    def __init__(
        self,
        height=1.3,
        yaw_deg=0,
        pitch_deg=-5,
        roll_deg=0,
        image_width=1024,
        image_height=512,
        field_of_view_deg=45,
    ):
        self.debug = True
        # scalar constants
        self.image_width = image_width
        self.image_height = image_height
        self.field_of_view_deg = field_of_view_deg
        # calculate the default camera intriniscs and extrinsics
        self.intrinsic_matrix = get_intrinsic_matrix(
            field_of_view_deg, image_width, image_height
        )
        self.inverse_intrinsic_matrix = np.linalg.inv(self.intrinsic_matrix)
        self.set_pose(height, yaw_deg, pitch_deg, roll_deg)
        self.cut_v = 0
        self.cut_x = 999.0
        self.forward_map_x = None
        self.forward_map_y = None
        self.inverse_map_u = None
        self.inverse_map_v = None
        self.inverse_interpolator_u = None
        self.inverse_interpolator_v = None
        # 需要定义道路坐标的离散化范围
        self.road_x_min, self.road_x_max = -10, 100  # 根据实际情况调整
        self.road_y_min, self.road_y_max = -10, 10
        self.road_resolution = 0.1  # 米/像素
        self.vp_u = image_width / 2.0  # 消失点默认在图像中的水平坐标
        self.vp_v = image_height / 2.0  # 消失点默认在图像中的垂直坐标
        self.cut_v = self.vp_v + 1

    def set_intrinsic_matrix(self, intrinsic_matrix: np.ndarray):
        """
        Set the intrinsic matrix of the camera
        This function should be called once at the beginning of the calibration process
        """
        self.intrinsic_matrix = intrinsic_matrix
        self.inverse_intrinsic_matrix = np.linalg.inv(self.intrinsic_matrix)

    def set_pose(self, height=1.3, yaw_deg=0, pitch_deg=-5, roll_deg=0, vp_u=0, vp_v=0):
        """
        Set the pose of the camera
        This function should be called when the extrinsic parameters are estimated
        """
        self.height = height
        self.pitch_deg = pitch_deg
        self.roll_deg = roll_deg
        self.yaw_deg = yaw_deg
        self.vp_u = vp_u
        self.vp_v = vp_v
        self.cut_v = self.vp_v + 1
        ## Note that "rotation_cam_to_road" has the math symbol R_{rc} in the book
        yaw = np.deg2rad(yaw_deg)
        pitch = np.deg2rad(pitch_deg)
        roll = np.deg2rad(roll_deg)
        cy, sy = np.cos(yaw), np.sin(yaw)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cr, sr = np.cos(roll), np.sin(roll)
        rotation_road_to_cam = np.array(
            [
                [cr * cy + sp * sr * sy, cr * sp * sy - cy * sr, -cp * sy],
                [cp * sr, cp * cr, sp],
                [cr * sy - cy * sp * sr, -cr * cy * sp - sr * sy, cp * cy],
            ]
        )
        self.rotation_cam_to_road = (
            rotation_road_to_cam.T
        )  # for rotation matrices, taking the transpose is the same as inversion
        self.translation_cam_to_road = np.array([0, -self.height, 0])
        self.trafo_cam_to_road = np.eye(4)
        self.trafo_cam_to_road[0:3, 0:3] = self.rotation_cam_to_road
        self.trafo_cam_to_road[0:3, 3] = self.translation_cam_to_road
        self.trafo_road_to_cam = np.linalg.inv(self.trafo_cam_to_road)
        # compute vector nc. Note that R_{rc}^T = R_{cr}
        self.road_normal_camframe = self.rotation_cam_to_road.T @ np.array([0, 1, 0])

    def camframe_to_roadframe(self, vec_in_cam_frame):
        return (
            self.rotation_cam_to_road @ vec_in_cam_frame + self.translation_cam_to_road
        )

    def roadframe_to_camframe(self, vec_in_road_frame):
        return self.rotation_cam_to_road.T @ (
            vec_in_road_frame - self.translation_cam_to_road
        )

    def uv_to_roadXYZ_camframe(self, u, v):
        # NOTE: The results depend very much on the pitch angle (0.5 degree error yields bad result)
        # Here is a paper on vehicle pitch estimation:
        # https://refubium.fu-berlin.de/handle/fub188/26792
        uv_hom = np.array([u, v, 1])
        Kinv_uv_hom = self.inverse_intrinsic_matrix @ uv_hom
        denominator = self.road_normal_camframe.dot(Kinv_uv_hom)
        return self.height * Kinv_uv_hom / denominator

    def uv_to_roadXYZ_roadframe(self, u, v):
        r_camframe = self.uv_to_roadXYZ_camframe(u, v)
        return self.camframe_to_roadframe(r_camframe)

    def uv_to_roadXYZ_roadframe_iso8855(self, u, v):
        if v < self.cut_v:
            return np.nan, np.nan, np.nan
        X, Y, Z = self.uv_to_roadXYZ_roadframe(u, v)
        return np.array(
            [Z, -X, -Y]
        )  # read book section on coordinate systems to understand this

    def roadXYZ_roadframe_iso8855_to_uv(self, X, Y, Z):
        # print(f"roadXYZ_roadframe_iso8855_to_uv: {X}, {Y}, {Z}")
        roadframe_xyz = np.array([-Y, -Z, X])  # iso8855 frame to road frame
        polyline_world = np.array([roadframe_xyz])
        uv = project_polyline(
            polyline_world, self.trafo_road_to_cam, self.intrinsic_matrix
        )
        # print(f"uv: {uv}")
        u, v = uv[0][0], uv[0][1]
        return u, v

    def roadXYZ_roadframe_iso8855_to_uv_polyline(
        self, road_x: List[float], road_y: List[float], road_z: List[float]
    ):
        roadframe_x = -np.array(road_y)
        roadframe_y = -np.array(road_z)
        roadframe_z = np.array(road_x)
        polyline_world = np.zeros((len(roadframe_x), 3))
        for i in range(len(roadframe_x)):
            polyline_world[i] = np.array(
                [roadframe_x[i], roadframe_y[i], roadframe_z[i]]
            )
        uv = project_polyline(
            polyline_world, self.trafo_road_to_cam, self.intrinsic_matrix
        )
        return uv

    def get_vp_from_pitch_yaw(self, camera_intrinsic_matrix: np.ndarray):
        """
        从俯仰角和偏航角计算消失点

        参数:
        - camera_intrinsic_matrix: 相机内参矩阵
        - pitch: 俯仰角（弧度）
        - yaw: 偏航角（弧度）

        返回:
        - u_i, v_i: 消失点在图像中的像素坐标
        """
        pitch = np.deg2rad(self.pitch_deg)
        yaw = np.deg2rad(self.yaw_deg)
        # 从俯仰角和偏航角计算方向向量 r3
        # r3 是相机坐标系中的方向向量
        r3 = np.array(
            [
                -np.sin(yaw) * np.cos(pitch),  # x
                np.sin(pitch),  # y
                np.cos(yaw) * np.cos(pitch),  # z
            ]
        )

        # 使用相机内参矩阵将方向向量投影到图像平面
        p_infinity = camera_intrinsic_matrix @ r3

        # 进行透视除法，得到实际的像素坐标
        u_i = p_infinity[0] / p_infinity[2]
        v_i = p_infinity[1] / p_infinity[2]

        return int(round(u_i)), int(round(v_i))

    # Fast converter
    def precompute_grid(self, dist=60):
        cut_v = int(self.compute_minimum_v(dist=dist) + 1)
        xy = []
        for v in range(cut_v, self.image_height):
            for u in range(self.image_width):
                X, Y, Z = self.uv_to_roadXYZ_roadframe_iso8855(u, v)
                xy.append(np.array([X, Y]))
        xy = np.array(xy)
        return cut_v, xy

    def compute_minimum_v(self, dist):
        """
        Find cut_v such that pixels with v<cut_v are irrelevant for polynomial fitting.
        Everything that is further than `dist` along the road is considered irrelevant.
        """
        trafo_road_to_cam = np.linalg.inv(self.trafo_cam_to_road)
        point_far_away_on_road = trafo_road_to_cam @ np.array([0, 0, dist, 1])
        uv_vec = self.intrinsic_matrix @ point_far_away_on_road[:3]
        uv_vec /= uv_vec[2]
        cut_v = uv_vec[1]
        return cut_v

    def dump_forward_map(self, file_path):
        with open(file_path, "w") as f:
            for v in range(self.cut_v, self.image_height):
                for u in range(self.image_width):
                    x, y, z = self.uv_to_roadXYZ_roadframe_iso8855(u, v)
                    f.write(f"{v} {u} {x} {y} {z}\n")

    def precompute_bidirectional_mapping(self, dist=100):
        """预计算双向映射表"""
        print("Precomputing bidirectional mapping")
        # 正向映射：uv -> road (ISO 8855)
        # 初始值为nan
        self.forward_map_x = np.full(
            (self.image_height, self.image_width), dtype=np.float32, fill_value=np.nan
        )
        self.forward_map_y = np.full(
            (self.image_height, self.image_width), dtype=np.float32, fill_value=np.nan
        )
        # 填充映射表
        cut_v = int(self.compute_minimum_v(dist=dist) + 1)
        print(f"precompute_bidirectional_mapping: cut_v={cut_v}, cut_dist={dist}")
        self.cut_v = max(self.cut_v, cut_v)

        # 正向映射填充
        road_points = []
        for v in range(self.cut_v, self.image_height):
            for u in range(self.image_width):
                # 正向映射：uv -> road
                x, y, z = self.uv_to_roadXYZ_roadframe_iso8855(u, v)
                self.forward_map_x[v, u] = x
                self.forward_map_y[v, u] = y
                # Fix me: we discard the Z coordinate here

                # 收集逆向映射数据
                road_points.append((x, y, u, v))
        # self._precompute_inverse_mapping(road_points)
        print("Precomputing bidirectional mapping done")

    def _precompute_inverse_mapping(self, road_points):
        # TODO: This function is not used now.
        # road_points: list of (X, Y, u, v)
        # 逆向映射：road (ISO 8855) -> uv
        # 创建逆向映射网格
        road_grid_w = int((self.road_x_max - self.road_x_min) / self.road_resolution)
        road_grid_h = int((self.road_y_max - self.road_y_min) / self.road_resolution)
        self.inverse_map_u = np.zeros((road_grid_h, road_grid_w), dtype=np.float32)
        self.inverse_map_v = np.zeros((road_grid_h, road_grid_w), dtype=np.float32)

        # 构建逆向查找表（使用网格插值）
        road_coords = np.array([(p[0], p[1]) for p in road_points])  # X, Y
        uv_values = np.array([(p[2], p[3]) for p in road_points])  # u, v

        # 创建逆向插值器
        self.inverse_interpolator_u = LinearNDInterpolator(road_coords, uv_values[:, 0])
        self.inverse_interpolator_v = LinearNDInterpolator(road_coords, uv_values[:, 1])

    def save_ipm_json_params(self, file_path):
        """保存IPM参数"""
        timestamp = int(time.time())
        u = self.image_width / 2.0
        v = self.cut_v
        x, y, z = self.uv_to_roadXYZ_roadframe_iso8855(u, v)
        self.cut_x = x
        with open(file_path, "w") as f:
            json.dump(
                {
                    "cut_v": float(self.cut_v),
                    "cut_x": float(self.cut_x),
                    "image_width": int(self.image_width),
                    "image_height": int(self.image_height),
                    "camera_height": float(self.height),
                    "camera_pitch": float(self.pitch_deg),
                    "camera_yaw": float(self.yaw_deg),
                    "camera_roll": float(self.roll_deg),
                    "vp_u": float(self.vp_u),
                    "vp_v": float(self.vp_v),
                    "timestamp": timestamp,
                },
                f,
            )

    def load_ipm_json_params(self, file_path):
        """加载IPM参数"""
        with open(file_path, "r") as f:
            params = json.load(f)
        # TODO: cut_v的保存和加载逻辑有点乱，需要梳理简化
        self.cut_v = params["cut_v"]
        self.cut_x = params["cut_x"]
        self.image_width = params["image_width"]
        self.image_height = params["image_height"]
        self.height = params["camera_height"]
        self.pitch_deg = params["camera_pitch"]
        self.yaw_deg = params["camera_yaw"]
        self.roll_deg = params["camera_roll"]
        self.vp_u = params["vp_u"]
        self.vp_v = params["vp_v"]
        self.timestamp = params["timestamp"]
        self.set_pose(
            self.height,
            self.yaw_deg,
            self.pitch_deg,
            self.roll_deg,
            self.vp_u,
            self.vp_v,
        )
        print(
            f"Loaded IPM params from {file_path}: cut_v={self.cut_v}, cut_x={self.cut_x}, image_width={self.image_width}, image_height={self.image_height}, camera_height={self.height}, camera_pitch={self.pitch_deg}deg, camera_yaw={self.yaw_deg}deg, camera_roll={self.roll_deg}deg, timestamp={self.timestamp}"
        )

    def uv_to_roadxy_iso8855_fast(self, u, v):
        """使用预计算的映射表实现uv到road坐标的映射"""
        if self.forward_map_x is None or self.forward_map_y is None:
            raise ValueError("Forward map is not precomputed")
        return self.forward_map_x[v, u], self.forward_map_y[v, u]

    def roadxy_iso8855_to_uv_fast(self, x, y):
        """使用插值器实现road到uv的映射"""
        if self.inverse_interpolator_u is None or self.inverse_interpolator_v is None:
            raise ValueError("Inverse map is not precomputed")
        road_coords = np.array(np.array([x, y]))
        uv_coords = self.road_coords_iso8855_to_uv_coords_fast(road_coords)
        return uv_coords[0][0], uv_coords[0][1]

    def uv_coords_to_roadxy_iso8855_fast(self, uv_coords):
        """
        使用预计算的映射表实现uv到road坐标的映射
        input: uv_coords: Nx2 数组，包含[u, v]坐标
        output: road_coords: Nx2 数组，包含[X, Y]坐标
        """
        if self.forward_map_x is None or self.forward_map_y is None:
            raise ValueError("Forward map is not precomputed")
        uv_coords = np.array(uv_coords)
        u, v = uv_coords[:, 0], uv_coords[:, 1]
        # truncate u, v to be in the range of the image
        u = np.clip(u, 0, self.image_width - 1)
        v = np.clip(v, 0, self.image_height - 1)
        road_coords = np.stack(
            (self.forward_map_x[v, u], self.forward_map_y[v, u]), axis=1
        )
        return road_coords

    def road_coords_iso8855_to_uv_coords_fast(self, road_coords):
        """
        使用插值器实现road到uv的映射
        input: road_coords: Nx2 数组，包含[X, Y]坐标
        output: uv_coords: Nx2 数组，包含[u, v]坐标
        """

        if self.inverse_interpolator_u is None or self.inverse_interpolator_v is None:
            raise ValueError("Inverse map is not precomputed")
        u = self.inverse_interpolator_u(road_coords)
        v = self.inverse_interpolator_v(road_coords)
        return np.stack((u, v), axis=1).astype(np.int32)
