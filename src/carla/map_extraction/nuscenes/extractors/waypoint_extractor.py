"""
Waypoint Extractor: CARLAマップからウェイポイントを抽出し、レーン境界を計算する
"""
import logging
import math
from collections import defaultdict
from dataclasses import dataclass, field
from typing import List, Dict, Tuple
import xml.etree.ElementTree as ET

import carla

logger = logging.getLogger(__name__)


@dataclass
class LaneBoundaryPoint:
    """レーン境界点（左右のオフセット点）"""
    left_x: float
    left_y: float
    right_x: float
    right_y: float


@dataclass
class LaneInfo:
    """単一レーンの情報 (ParkingやSidewalkも含む)"""
    road_id: int
    section_id: int
    lane_id: int
    lane_type: str  # 'Driving', 'Parking', 'Sidewalk', 'Shoulder', etc.
    is_junction: bool
    junction_id: int = -1
    # 左右通行どちらか
    traffic_direction: str = 'righthand'  # 'righthand' or 'lefthand'
    # ウェイポイントの中心線座標リスト
    centerline: List[Tuple[float, float]] = field(default_factory=list)
    # レーン境界点のリスト
    boundary_points: List[LaneBoundaryPoint] = field(default_factory=list)
    lane_width: float = 0.0
    # レーンマーキング情報
    left_marking_type: str = ""
    right_marking_type: str = ""


def _compute_boundary_points(wp) -> LaneBoundaryPoint:
    """
    ウェイポイントの位置とlane_widthから左右の境界点を計算する。
    CARLAの座標系: X=前方, Y=右, Z=上
    ウェイポイントのtransformのforward vectorに対して垂直方向にオフセット。
    """
    loc = wp.transform.location
    rot = wp.transform.rotation
    yaw = math.radians(rot.yaw)
    half_width = wp.lane_width / 2.0

    # 右方向ベクトル (yawに対して+90度)
    right_x = math.cos(yaw + math.pi / 2)
    right_y = math.sin(yaw + math.pi / 2)

    return LaneBoundaryPoint(
        left_x=loc.x - right_x * half_width,
        left_y=loc.y - right_y * half_width,
        right_x=loc.x + right_x * half_width,
        right_y=loc.y + right_y * half_width,
    )

class TrafficRuleResolver:
    def __init__(self, carla_map: carla.Map):
        """
        CARLA 0.9.15向け。
        指定した road_id が RHT / LHT のどちらかをOpenDRIVEから判定する。
        """
        self._rule_by_road_id = self._build_cache(carla_map)

    @staticmethod
    def _build_cache(carla_map: carla.Map) -> Dict[int, str]:
        xodr = carla_map.to_opendrive()
        root = ET.fromstring(xodr)
        rule_by_road_id = {}
        for road in root.findall("road"):
            road_id = int(road.get("id"))
            # OpenDRIVEでは rule が無い場合は RHT 扱い
            rule = road.get("rule", "RHT").upper()
            if rule not in {"RHT", "LHT"}:
                rule = "RHT"
            rule_by_road_id[road_id] = rule
        return rule_by_road_id

    def get_rule(self, road_id: int) -> str:
        return self._rule_by_road_id.get(road_id, "RHT")

    def is_rht(self, road_id: int) -> bool:
        return self.get_rule(road_id) == "RHT"

    def is_lht(self, road_id: int) -> bool:
        return self.get_rule(road_id) == "LHT"

class WaypointExtractor:
    def __init__(self, carla_map, sampling_resolution: float = 1.0):
        """
        CARLAマップからウェイポイントを抽出し、Lane, Parking, Sidewalkなどのレーンセグメントを構築するクラス

        :param carla_map: carla.Map
        :param sampling_resolution: サンプリング間隔
        """
        # CARLAマップからウェイポイントを抽出
        logger.info(f"Generating waypoints with resolution={sampling_resolution}m ...")
        self.waypoints = carla_map.generate_waypoints(sampling_resolution)
        logger.info(f"Generated {len(self.waypoints)} waypoints")
        # CARLA0.9.15以前ではroad_idごとに通行方向を判定しておく
        if self.waypoints and not hasattr(self.waypoints[0], 'is_rht'):
            road_ids = set(wp.road_id for wp in self.waypoints)
            direction_resolver = TrafficRuleResolver(carla_map)
            self.road_id_to_traffic_rule = {rid: "righthand" if direction_resolver.is_rht(rid) else "lefthand" for rid in road_ids}
            logger.info(f"This CARLA version doesn't have 'is_rht' attribute. Determined traffic rules for {len(self.road_id_to_traffic_rule)} roads")
        # 抽出したLane, Parking, Sidewalkセグメントを格納するリスト
        self.lane: List[dict] = []
        self.parking: List[dict] = []
        self.sidewalk: List[dict] = []

    def _get_traffic_direction(self, wp) -> str:
        if hasattr(wp, 'is_rht'):
            return 'righthand' if wp.is_rht else 'lefthand'
        else:
            return self.road_id_to_traffic_rule.get(wp.road_id, 'righthand')  # デフォルトはrighthand

    def extract_lanes(self) -> Dict[Tuple[int, int, int], LaneInfo]:
        """
        レーンを抽出
        :return: (road_id, section_id, lane_id) -> LaneInfo のDict

        > アルゴリズム:
        > 1. carla_map.generate_waypoints() でsampling_resolution間隔で全ウェイポイントを取得
        > 2. 各ウェイポイントについて、(road_id, section_id, lane_id) でグルーピングしてLaneとする
        > 3. ウェイポイントの位置 (transform.location) をLaneの中心線とする
        > 4. 中心線から左右にlane_width/2オフセットしてレーン境界点を計算 (_compute_boundary_points関数)
        > ※ポリゴンではなく中心線と左右境界線の点列で表現するので注意
        """
        lanes: Dict[Tuple[int, int, int], LaneInfo] = {}

        for wp in self.waypoints:
            key = (wp.road_id, wp.section_id, wp.lane_id)

            if key not in lanes:
                lane_type_name = str(wp.lane_type).split(".")[-1]
                lanes[key] = LaneInfo(
                    road_id=wp.road_id,
                    section_id=wp.section_id,
                    lane_id=wp.lane_id,
                    lane_type=lane_type_name,
                    is_junction=wp.is_junction,
                    junction_id=wp.get_junction().id if wp.is_junction and wp.get_junction() else -1,
                    lane_width=wp.lane_width,
                )
                # レーンマーキング情報
                if wp.left_lane_marking:
                    lanes[key].left_marking_type = str(wp.left_lane_marking.type).split(".")[-1]
                if wp.right_lane_marking:
                    lanes[key].right_marking_type = str(wp.right_lane_marking.type).split(".")[-1]

            loc = wp.transform.location
            lanes[key].centerline.append((loc.x, loc.y))
            lanes[key].boundary_points.append(_compute_boundary_points(wp))
            lanes[key].traffic_direction = self._get_traffic_direction(wp)

        logger.info(f"Extracted {len(lanes)} unique lanes")

        # レーンタイプ別の統計
        type_counts = defaultdict(int)
        for lane_info in lanes.values():
            type_counts[lane_info.lane_type] += 1
        for lt, count in sorted(type_counts.items()):
            logger.info(f"  {lt}: {count} lanes")

        return lanes

    def extract_parkings_sidewalks_from_opendrive(self) -> List[LaneInfo]:
        """
        CARLAのウェイポイントAPIを使って駐車スペースと歩道セグメントを抽出する。
        generate_waypointsはDrivingレーンのみ返すため、
        各Drivingレーンの隣接レーンを辿ってParking/Sidewalkタイプを探す。

        :return: ParkingSegment, SidewalkSegment のリスト
        """
        # Parking/Sidewalkウェイポイントを収集（隣接レーンを辿る）
        parkings: Dict[Tuple[int, int, int], LaneInfo] = {}
        sidewalks: Dict[Tuple[int, int, int], LaneInfo] = {}

        for wp in self.waypoints:
            # 左右のレーンを辿ってSidewalkを探す
            for direction in ['left', 'right']:
                current = wp
                for _ in range(5):  # 最大5レーン分辿る
                    adj = current.get_left_lane() if direction == 'left' else current.get_right_lane()
                    if adj is None:
                        break
                    key = (adj.road_id, adj.section_id, adj.lane_id)
                    loc = adj.transform.location
                    lane_type_name = str(wp.lane_type).split(".")[-1]
                    # Parking
                    if adj.lane_type == carla.LaneType.Parking:
                        if key not in parkings:
                            parkings[key] = LaneInfo(
                                road_id=adj.road_id,
                                section_id=adj.section_id,
                                lane_id=adj.lane_id,
                                lane_type=lane_type_name,
                                is_junction=adj.is_junction,
                                junction_id=adj.get_junction().id if adj.is_junction and adj.get_junction() else -1,
                                lane_width=adj.lane_width
                            )
                            if adj.left_lane_marking:
                                parkings[key].left_marking_type = str(adj.left_lane_marking.type).split(".")[-1]
                            if adj.right_lane_marking:
                                parkings[key].right_marking_type = str(adj.right_lane_marking.type).split(".")[-1]
                        parkings[key].centerline.append((loc.x, loc.y))
                        parkings[key].boundary_points.append(_compute_boundary_points(adj))
                        parkings[key].traffic_direction = self._get_traffic_direction(adj)
                    # Sidewalk
                    if adj.lane_type == carla.LaneType.Sidewalk:
                        if key not in sidewalks:
                            sidewalks[key] = LaneInfo(
                                road_id=adj.road_id,
                                section_id=adj.section_id,
                                lane_id=adj.lane_id,
                                lane_type=lane_type_name,
                                is_junction=adj.is_junction,
                                junction_id=adj.get_junction().id if adj.is_junction and adj.get_junction() else -1,
                                lane_width=adj.lane_width
                            )
                            if adj.left_lane_marking:
                                sidewalks[key].left_marking_type = str(adj.left_lane_marking.type).split(".")[-1]
                            if adj.right_lane_marking:
                                sidewalks[key].right_marking_type = str(adj.right_lane_marking.type).split(".")[-1]
                        sidewalks[key].centerline.append((loc.x, loc.y))
                        sidewalks[key].boundary_points.append(_compute_boundary_points(adj))
                        sidewalks[key].traffic_direction = self._get_traffic_direction(adj)
                    current = adj

        logger.info(f"Extracted {len(parkings)} parking segments and {len(sidewalks)} sidewalk segments")

        return parkings, sidewalks