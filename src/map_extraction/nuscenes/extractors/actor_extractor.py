"""
Actor Extractor: CARLAマップからアクター情報を抽出する
"""
import logging
import math
from collections import defaultdict
from dataclasses import dataclass, field
from typing import List, Dict, Tuple

logger = logging.getLogger(__name__)


@dataclass
class TrafficLightHeadInfo:
    """灯器ヘッド（筐体）1個の情報"""
    x: float  # ヘッド中心の位置（CARLA座標系）
    y: float
    z: float
    extent_x: float  # バウンディングボックスの半寸法（メートル）
    extent_y: float
    extent_z: float


@dataclass
class TrafficLightActorInfo:
    """信号機アクターの情報（nuScenes map expansionのtraffic_light/stop_line構築用）"""
    x: float  # ポール（アクター）の位置（CARLA座標系）
    y: float
    z: float
    yaw: float  # 灯器正面（ドライバーから見える面）の向き（CARLA座標系、度）
    stop_line_points: List[Tuple[float, float]]  # 制御対象レーンを横断する停止線の点列（CARLA座標系）
    orientation: str  # 'Positive', 'Negative', 'Both'
    light_heads: List[TrafficLightHeadInfo] = field(default_factory=list)  # 灯器ヘッドのリスト


# get_light_boxes()にはヘッド以外の微小な箱（歩行者ボタン等）が含まれるため、高さで除外する
_MIN_HEAD_HALF_HEIGHT = 0.3

def extract_traffic_light_actors(carla_world) -> List[TrafficLightActorInfo]:
    """
    CARLAマップから信号機情報を抽出する。
    :param carla_world: carla.World
    :return: TrafficLightActorInfo のリスト
    """
    carla_map = carla_world.get_map()
    traffic_lights = carla_world.get_actors().filter('traffic.traffic_light*')

    tl_infos = []
    for tl in traffic_lights:
        # 位置を取得
        transform = tl.get_transform()
        # 停止線の座標をリストとして取得（制御対象レーンの停止waypointを両端のレーン幅分だけ延長）
        waypoints = tl.get_stop_waypoints()
        if waypoints:
            start_loc = waypoints[0].transform.location - waypoints[0].transform.rotation.get_right_vector() * waypoints[0].lane_width * 0.5
            centor_locs = [wp.transform.location for wp in waypoints]
            end_loc = waypoints[-1].transform.location + waypoints[-1].transform.rotation.get_right_vector() * waypoints[-1].lane_width * 0.5
            stop_line_points = [(start_loc.x, start_loc.y), *[(loc.x, loc.y) for loc in centor_locs], (end_loc.x, end_loc.y)]
        else:
            stop_line_points = []
        # 灯器正面（ドライバーから見える面）の向き = 制御レーン進行方向 + 180°
        # stop waypointが無い場合はアクターyaw + 90°にフォールバック
        # （CARLAのアクターyawはポールのアーム方向を向いており、制御レーン進行方向 + 90°に等しい）
        if waypoints:
            facing_yaw = waypoints[0].transform.rotation.yaw + 180.0
        else:
            facing_yaw = transform.rotation.yaw + 90.0
        facing_yaw = (facing_yaw + 180.0) % 360.0 - 180.0  # [-180, 180)に正規化
        # 灯器ヘッド（筐体）のバウンディングボックスを取得（微小な箱は歩行者ボタン等のため除外）
        light_heads = [
            TrafficLightHeadInfo(
                x=box.location.x,
                y=box.location.y,
                z=box.location.z,
                extent_x=box.extent.x,
                extent_y=box.extent.y,
                extent_z=box.extent.z,
            )
            for box in tl.get_light_boxes()
            if box.extent.z >= _MIN_HEAD_HALF_HEIGHT
        ]
        # orientationはアクターではなく対応するOpenDRIVEランドマークが持つ
        landmarks = carla_map.get_all_landmarks_from_id(tl.get_opendrive_id())
        orientation = str(landmarks[0].orientation).split(".")[-1] if landmarks else "Both"
        tl_info = TrafficLightActorInfo(
            x=transform.location.x,
            y=transform.location.y,
            z=transform.location.z,
            yaw=facing_yaw,
            stop_line_points=stop_line_points,
            orientation=orientation,
            light_heads=light_heads,
        )
        tl_infos.append(tl_info)
        logger.debug(f"Extracted traffic light actor at ({tl_info.x}, {tl_info.y}, {tl_info.z}) with orientation {tl_info.orientation}")

    logger.info(f"Extracted {len(tl_infos)} traffic lights")
    return tl_infos