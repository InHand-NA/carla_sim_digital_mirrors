import carla
from carla import Location, Rotation, Transform
import numpy as np
import matplotlib.pyplot as plt
import cv2

"""
坐标系描述：
Carla坐标系： 
- world坐标系： 左手坐标系，map坐标系
- vehicle坐标系： 左手坐标系，原点在自车中心的地面投影，vehicle坐标系与world坐标系的关系由ego_transform矩阵描述
- camera坐标系： 右手坐标系，camera坐标系与world坐标系的关系由camera.transform矩阵描述


InADAS项目坐标系：
- ISO8855坐标系： 右手坐标系，原点在自车相机安装位置的地面投影，x轴指向自车前进方向，y轴指向自车左侧，z轴指向上方
"""


def coor_world_to_vehicle(ego_transform, world_location):
    # 将世界坐标转换为右手车辆坐标
    # ego_transform 是自车的变换矩阵, 可以把自车坐标变换为世界坐标
    # 因此, ego_transform 的逆矩阵可以把世界坐标变换为自车坐标
    # 注意： Carla采用左手坐标系；
    # Get the inverse matrix of ego_transform
    Ri = ego_transform.get_inverse_matrix()
    # Transform the world_location to vehicle coordinate
    ext_world_location = np.array(
        [world_location.x, world_location.y, world_location.z, 1.0]
    )
    vehicle_coord = np.dot(Ri, ext_world_location)
    vehicle_coord = vehicle_coord[:3]
    return vehicle_coord


def build_projection_matrix(w, h, fov):
    focal = w / (2.0 * np.tan(fov * np.pi / 360.0))
    K = np.identity(3)
    K[0, 0] = K[1, 1] = focal
    K[0, 2] = w / 2.0
    K[1, 2] = h / 2.0
    return K


def get_image_point(world_location, world_2_camera, K):
    # Calculate 2D projection of bone coordinate

    # get the world location of the bone root
    ext_world_location = np.array(
        [world_location.x, world_location.y, world_location.z, 1.0]
    )
    # transform to camera coordinates
    point_camera = np.dot(world_2_camera, ext_world_location)

    # New we must change from UE4's coordinate system to an "standard"
    # (x, y ,z) -> (y, -z, x)
    # and we remove the fourth component also
    point_camera = [point_camera[1], -point_camera[2], point_camera[0]]
    # point_camera = point_camera[:3]
    if point_camera[2] <= 0:
        return None

    # now project 3D->2D using the camera matrix
    point_img = np.dot(K, point_camera)
    # normalize
    point_img[0] /= point_img[2]
    point_img[1] /= point_img[2]

    # print(f"world_location: {world_location} => image point: {point_img}")
    return int(round(point_img[0])), int(round(point_img[1]))


def get_forward_lane_2d(waypoint, world_2_camera, K, img_w, img_h, distance=100):
    """
    Get the forward lane of the ego vehicle
    """
    step = 1.0
    count = 10
    left_lane_line_2d = []
    right_lane_line_2d = []
    if count == 0:
        current = waypoint
    else:
        current = waypoint.next(count)[0]  # debug:
    while current and count < distance:
        lane_width = current.lane_width  # 单位为 meter
        lane_type = current.lane_type

        lm_width = 0 #current.left_lane_marking.width
        rm_width = 0 #current.right_lane_marking.width
        #if lane_type == carla.LaneType.Shoulder:
        #    lm_width = current.left_lane_marking.width
        #    rm_width = current.right_lane_marking.width
        #    print(f"Shoulder: lm_width: {lm_width}, rm_width: {rm_width}")

        # 获取左侧和右侧车道线中心的位置（以当前 waypoint 为基准）
        left_marking_location = (
            current.transform.location
            + current.transform.get_right_vector() * (-lane_width / 2.0 + lm_width / 2.0)
        )
        right_marking_location = (
            current.transform.location
            + current.transform.get_right_vector() * (lane_width / 2.0 - rm_width / 2.0)
        )

        left_point = get_image_point(left_marking_location, world_2_camera, K)
        right_point = get_image_point(right_marking_location, world_2_camera, K)
        # left_point = world_to_pixel(left_marking_location, camera_transform, K)
        # right_point = world_to_pixel(right_marking_location, camera_transform, K)
        if (
            left_point is not None
            and left_point[0] > 0
            and left_point[0] < img_w
            and left_point[1] > 0
            and left_point[1] < img_h
        ):
            left_lane_line_2d.append(left_point)
        if (
            right_point is not None
            and right_point[0] > 0
            and right_point[0] < img_w
            and right_point[1] > 0
            and right_point[1] < img_h
        ):
            right_lane_line_2d.append(right_point)

        next_list = current.next(step)
        if not next_list:
            break
        current = next_list[0]
        count += step
    return left_lane_line_2d, right_lane_line_2d


def get_forward_lane(waypoint, ego_transform, cam_locx=3.8, distance=100):
    """
    Get the forward lane of the ego vehicle
    """
    step = 1.0
    count = 0
    # segment = []
    left_lane_line = []
    right_lane_line = []
    current = waypoint
    # while current and not current.is_junction and count < distance:
    while current and count < distance:
        x, y, z = (
            current.transform.location.x,
            current.transform.location.y,
            current.transform.location.z,
        )
        vehicle_coord = coor_world_to_vehicle(ego_transform, Location(x=x, y=y, z=z))
        lane_width = current.lane_width

        # segment.append([vehicle_coord[0], vehicle_coord[1], vehicle_coord[2], lane_width])
        # ISO8855坐标系
        left_lane_line.append(
            [
                float(vehicle_coord[0] - cam_locx),
                float(-vehicle_coord[1] + lane_width / 2),
                float(vehicle_coord[2]),
                float(lane_width),
            ]
        )
        right_lane_line.append(
            [
                float(vehicle_coord[0] - cam_locx),
                float(-vehicle_coord[1] - lane_width / 2),
                float(vehicle_coord[2]),
                float(lane_width),
            ]
        )
        next_list = current.next(step)
        if not next_list:
            break
        # print("next_list len:", len(next_list))
        current = next_list[0]
        count += step
    return left_lane_line, right_lane_line


def print_lane_data(lane_data):
    """
    Print the lane data
    """
    for data in lane_data:
        print(data)


def print_lanes_data(lanes_data):
    """
    Print the lanes data
    """
    for i, lane_data in enumerate(lanes_data):
        line_name = ["left_left", "left", "right", "right_right"]
        print(f"{line_name[i]} lane:")
        print_lane_data(lane_data)


def figure_to_cv2_image(fig, size=(250, 250)):
    """
    将matplotlib figure转换为OpenCV图像格式（BGR）
    :param fig: matplotlib figure对象
    :return: OpenCV图像（numpy数组）
    """
    # 将figure渲染到canvas
    fig.canvas.draw()
    # 获取RGB像素数据
    img_rgb = np.array(fig.canvas.renderer.buffer_rgba())[..., :3]  # 去除alpha通道
    # 转换RGB到BGR
    img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
    # 关闭figure释放内存
    plt.close(fig)
    # 调整图像大小
    img_bgr = cv2.resize(img_bgr, size)
    return img_bgr


def draw_lane_data(lane_data):
    # Draw the lane line with plt
    fig = plt.figure(figsize=(250, 250), dpi=100)
    x = [data[0] for data in lane_data]
    y = [data[1] for data in lane_data]

    plt.plot(y, x)
    plt.xlim(-50, 50)
    plt.ylim(-10, 100)
    plt.show()
    img = figure_to_cv2_image(fig)
    return img


def gather_lane_data(world, ego_vehicle, cam_locx=3.8, camera=None, log=False, driving_lanes_only=True):
    """
    Gather lane data for the ego vehicle
    """

    veh_location = ego_vehicle.get_location()
    map = world.get_map()
    waypoint = map.get_waypoint(
        veh_location, project_to_road=True, lane_type=carla.LaneType.Driving
    )

    # present lane data
    lane_id = waypoint.lane_id
    lane_type = waypoint.lane_type
    lane_width = waypoint.lane_width
    is_junction = waypoint.is_junction

    ego_transform = ego_vehicle.get_transform()
    left_lane_wp = waypoint.get_left_lane()
    right_lane_wp = waypoint.get_right_lane()

    # x, y, z = waypoint.transform.location.x, waypoint.transform.location.y, waypoint.transform.location.z
    # print([x, y, z, lane_width])
    left_lane_line, right_lane_line = get_forward_lane(
        waypoint, ego_transform, cam_locx=cam_locx, distance=100.0
    )

    if left_lane_wp is not None and (not driving_lanes_only or left_lane_wp.lane_type == carla.LaneType.Driving):
        left_left_lane_line, left_right_lane_line = get_forward_lane(
            left_lane_wp, ego_transform, cam_locx=cam_locx, distance=100.0
        )
    else:
        left_left_lane_line = []
        left_right_lane_line = []
    if right_lane_wp is not None and (not driving_lanes_only or right_lane_wp.lane_type == carla.LaneType.Driving):
        right_left_lane_line, right_right_lane_line = get_forward_lane(
            right_lane_wp, ego_transform, cam_locx=cam_locx, distance=100.0
        )
    else:
        right_left_lane_line = []
        right_right_lane_line = []
    # img = draw_lane_data(forward_lane)
    # img = None
    lanes_data = [
        left_left_lane_line,
        left_lane_line,
        right_lane_line,
        right_right_lane_line,
    ]
    if log:
        print_lanes_data(lanes_data)
    return lanes_data


def gather_lane_data_2d(ego_vehicle, world, camera, img_h, img_w, fov, log=False, driving_lanes_only=True):
    """
    Gather lane data for the ego vehicle
    """
    K = build_projection_matrix(img_w, img_h, fov)
    world_2_camera = np.array(camera.get_transform().get_inverse_matrix())
    veh_location = ego_vehicle.get_location()
    map = world.get_map()
    waypoint = map.get_waypoint(
        veh_location, project_to_road=True, lane_type=carla.LaneType.Driving
    )
    left_lane_line_2d, right_lane_line_2d = get_forward_lane_2d(
        waypoint, world_2_camera, K, img_w, img_h, distance=100.0
    )

    left_lane_wp = waypoint.get_left_lane()
    right_lane_wp = waypoint.get_right_lane()
    # Fixme: what about the other lane types?
    if left_lane_wp is not None and (not driving_lanes_only or left_lane_wp.lane_type == carla.LaneType.Driving):
        left_left_lane_line_2d, left_right_lane_line_2d = get_forward_lane_2d(
            left_lane_wp, world_2_camera, K, img_w, img_h, distance=100.0
        )
    else:
        left_left_lane_line_2d = []
        left_right_lane_line_2d = []
    if right_lane_wp is not None and (not driving_lanes_only or right_lane_wp.lane_type == carla.LaneType.Driving):
        right_left_lane_line_2d, right_right_lane_line_2d = get_forward_lane_2d(
            right_lane_wp, world_2_camera, K, img_w, img_h, distance=100.0
        )
    else:
        right_left_lane_line_2d = []
        right_right_lane_line_2d = []

    lanes_data_2d = [
        left_left_lane_line_2d,
        left_lane_line_2d,
        right_lane_line_2d,
        right_right_lane_line_2d,
    ]
    return lanes_data_2d
