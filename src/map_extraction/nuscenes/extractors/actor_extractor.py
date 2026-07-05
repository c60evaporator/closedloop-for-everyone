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
class TrafficLightActorInfo:
    """信号機アクターの情報"""
    x: float
    y: float
    z: float
    yaw: float
    stop_line_points: List[Tuple[float, float]]
    orientation: str  # 'Positive', 'Negative', 'Both'

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
        # 停止線の座標をリストとして取得
        waypoints = tl.get_stop_waypoints()
        start_loc = waypoints[0].transform.location - waypoints[0].transform.rotation.get_right_vector() * waypoints[0].lane_width * 0.5
        centor_locs = [wp.transform.location for wp in waypoints]
        end_loc = waypoints[-1].transform.location + waypoints[-1].transform.rotation.get_right_vector() * waypoints[-1].lane_width * 0.5
        stop_line_points = [(start_loc.x, start_loc.y), *[(loc.x, loc.y) for loc in centor_locs], (end_loc.x, end_loc.y)]
        # orientationはアクターではなく対応するOpenDRIVEランドマークが持つ
        landmarks = carla_map.get_all_landmarks_from_id(tl.get_opendrive_id())
        orientation = str(landmarks[0].orientation).split(".")[-1] if landmarks else "Both"
        tl_info = TrafficLightActorInfo(
            x=transform.location.x,
            y=transform.location.y,
            z=transform.location.z,
            yaw=transform.rotation.yaw,
            stop_line_points=stop_line_points,
            orientation=orientation,
        )
        tl_infos.append(tl_info)
        logger.debug(f"Extracted traffic light actor at ({tl_info.x}, {tl_info.y}, {tl_info.z}) with orientation {tl_info.orientation}")

    logger.info(f"Extracted {len(tl_infos)} traffic lights")
    return tl_infos