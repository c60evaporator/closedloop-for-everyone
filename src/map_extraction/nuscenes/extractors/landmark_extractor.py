"""
Landmark Extractor: CARLAマップから横断歩道・信号機・標識等を抽出する
"""
import logging
from dataclasses import dataclass, field
from typing import List, Tuple

logger = logging.getLogger(__name__)


@dataclass
class CrosswalkPolygon:
    """横断歩道ポリゴン"""
    vertices: List[Tuple[float, float]] = field(default_factory=list)


@dataclass
class TrafficLightInfo:
    """信号機の情報"""
    landmark_id: str
    road_id: int
    x: float
    y: float
    z: float
    yaw: float
    pitch: float
    roll: float
    orientation: str  # 'Positive', 'Negative', 'Both'


@dataclass
class StopSignInfo:
    """停止標識の情報"""
    landmark_id: str
    road_id: int
    x: float
    y: float
    z: float


def extract_crosswalks(carla_map) -> List[CrosswalkPolygon]:
    """
    CARLAマップから横断歩道ポリゴンを抽出する。
    get_crosswalks() は carla.Location のフラットなリストを返し、
    最初の頂点が最後に繰り返されることでポリゴンの区切りを示す。
    :param carla_map: carla.Map
    :return: CrosswalkPolygon のリスト
    """
    raw_locations = carla_map.get_crosswalks()
    logger.info(f"Raw crosswalk locations: {len(raw_locations)} points")

    crosswalks = []
    current_polygon: List[Tuple[float, float]] = []

    for loc in raw_locations:
        point = (loc.x, loc.y)

        if len(current_polygon) > 2 and _points_close(point, current_polygon[0]):
            # ポリゴンが閉じた
            crosswalks.append(CrosswalkPolygon(vertices=current_polygon))
            current_polygon = []
        else:
            current_polygon.append(point)

    # 最後のポリゴンが閉じていない場合
    if len(current_polygon) > 2:
        crosswalks.append(CrosswalkPolygon(vertices=current_polygon))

    logger.info(f"Extracted {len(crosswalks)} crosswalk polygons")
    return crosswalks


def _points_close(p1: Tuple[float, float], p2: Tuple[float, float], threshold: float = 0.1) -> bool:
    """2点が近いかどうか判定"""
    return abs(p1[0] - p2[0]) < threshold and abs(p1[1] - p2[1]) < threshold


def extract_traffic_lights(carla_map) -> List[TrafficLightInfo]:
    """
    CARLAマップから信号機情報を抽出する。
    :param carla_map: carla.Map
    :return: TrafficLightInfo のリスト
    """
    landmarks = carla_map.get_all_landmarks()
    traffic_lights = []

    for lm in landmarks:
        # 信号機は type が特定の値を持つ（国コードによる。ドイツ2017標準ではtype=1000001等）
        # is_dynamic=True のランドマークの多くは信号機
        if not lm.is_dynamic:
            continue
        loc = lm.transform.location
        tl = TrafficLightInfo(
            landmark_id=lm.id,
            road_id=lm.road_id,
            x=loc.x,
            y=loc.y,
            z=loc.z,
            yaw=lm.transform.rotation.yaw,
            pitch=lm.transform.rotation.pitch,
            roll=lm.transform.rotation.roll,
            orientation=str(lm.orientation).split(".")[-1],
        )
        traffic_lights.append(tl)

    logger.info(f"Extracted {len(traffic_lights)} traffic lights")
    return traffic_lights


def extract_stop_signs(carla_map) -> List[StopSignInfo]:
    """
    CARLAマップから停止標識を抽出する。
    :param carla_map: carla.Map
    :return: StopSignInfo のリスト
    """
    landmarks = carla_map.get_all_landmarks()
    stop_signs = []

    for lm in landmarks:
        # OpenDRIVEのドイツ2017標準: type=206 がStopSign
        if lm.type == "206":
            loc = lm.transform.location
            stop_signs.append(StopSignInfo(
                landmark_id=lm.id,
                road_id=lm.road_id,
                x=loc.x,
                y=loc.y,
                z=loc.z,
            ))

    logger.info(f"Extracted {len(stop_signs)} stop signs")
    return stop_signs
