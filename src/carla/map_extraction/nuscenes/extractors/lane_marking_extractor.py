"""
Lane Marking Extractor: レーンマーキング情報からroad_dividerとlane_dividerを抽出する
"""
import logging
from dataclasses import dataclass, field
from typing import List, Dict, Tuple
from collections import defaultdict
from src.carla.map.nuscenes.extractors.waypoint_extractor import LaneInfo

logger = logging.getLogger(__name__)


@dataclass
class DividerLine:
    """分離線の情報"""
    road_id: int
    section_id: int
    is_junction: bool
    divider_type: str  # 'road_divider' or 'lane_divider'
    marking_type: str  # 'Solid', 'Broken', 'SolidSolid', etc.
    points: List[Tuple[float, float]] = field(default_factory=list)
    # どのレーン間の分離線か
    lane_key_left: Tuple[int, int, int] = (0, 0, 0)
    lane_key_right: Tuple[int, int, int] = (0, 0, 0)

def _classify_divider_type(marking_type: str) -> str:
    """
    マーキングタイプからdivider種別を判定する。
    SolidSolid（二重実線）やCurb（縁石）はroad_divider（道路分離帯）、
    それ以外はlane_divider（レーン間分離線）として分類。
    """
    road_divider_markings = {'SolidSolid', 'Curb', 'NONE'}
    if marking_type in road_divider_markings:
        return 'road_divider'
    return 'lane_divider'

def _get_divider_line(lane_info, direction: str = 'bigger',
                      center: bool = False, traffic_direction='righthand') -> DividerLine:
    """
    lane_infoと方向からDividerLineを生成する。
    :param lane_info: LaneInfo オブジェクト
    :param direction: 'smaller' or 'bigger' - lane_idが小さい側が'smaller'、大きい側が'bigger'
    :param center: 隣接レーンとの符号が異なる場合、中心線としてroad_dividerに分類するフラグ
    :param traffic_direction: 'righthand' or 'lefthand' - 交通方向（右側通行か左側通行か）を指定。右側
    """
    # 右側通行の場合
    if traffic_direction == 'righthand':
        if lane_info.lane_id < 0:
            boundary_direction = 'right' if direction == 'smaller' else 'left'
        else:
            boundary_direction = 'left' if direction == 'smaller' else 'right'
    # 左側通行の場合
    else:
        if lane_info.lane_id < 0:
            boundary_direction = 'left' if direction == 'smaller' else 'right'
        else:
            boundary_direction = 'right' if direction == 'smaller' else 'left'

    if boundary_direction == 'left':
        marking = lane_info.left_marking_type
        boundary_points = [(bp.left_x, bp.left_y) for bp in lane_info.boundary_points]
    else:
        marking = lane_info.right_marking_type
        boundary_points = [(bp.right_x, bp.right_y) for bp in lane_info.boundary_points]

    divider_type = _classify_divider_type(marking) if not center else 'road_divider'
    return DividerLine(
        road_id=lane_info.road_id,
        section_id=lane_info.section_id,
        is_junction=lane_info.is_junction,
        divider_type=divider_type,
        marking_type=marking,
        points=boundary_points,
        lane_key_left=(lane_info.road_id, lane_info.section_id, lane_info.lane_id),
        lane_key_right=None,  # 後で隣接レーンとペアリングして設定
    )

def extract_dividers(
    lanes: Dict[Tuple[int, int, int], LaneInfo],
    traffic_direction='righthand',
) -> List[DividerLine]:
    """
    レーン情報からlane_divider（同一道路内のレーン間分離線）を抽出する。
    隣接レーン間の境界をライン化。
    :param lanes: waypoint_extractor.extract_waypoints() の結果
    :return: DividerLine のリスト
    """
    dividers = []

    # road_idでグルーピング
    groups: dict[int, list[LaneInfo]] = defaultdict(list)    
    for k, lane_info in lanes.items():
        groups[lane_info.road_id].append(lane_info)

    # グループごとにlaneを処理
    for group_key, lane_list in groups.items():
        # lane_idが小さい順に走査（OpenDRIVE規約で左側が正、右側が負なので右側から処理）
        lane_list.sort(key=lambda x: x.lane_id)
        for i, lane_info in enumerate(lane_list):
            # 最初のlaneのみ、lane_idが小さい側のboundaryをlane_dividerとして追加
            if i == 0:
                dividers.append(_get_divider_line(lane_info, direction='smaller'))
            # 次のlaneとの符号が異なる場合、centerフラグを立てる
            center = (i < len(lane_list) - 1 and lane_info.lane_id * lane_list[i+1].lane_id < 0)
            # lane_idが大きい側のboundaryを、centerフラグが立っている場合はroad_dividerとして追加、それ以外は通常のlane_dividerとして追加
            if center:
                dividers.append(_get_divider_line(lane_info, direction='bigger', center=True,
                                                  traffic_direction=traffic_direction))
            else:
                dividers.append(_get_divider_line(lane_info, direction='bigger',
                                                  traffic_direction=traffic_direction))

    # road_divider と lane_divider を分離
    road_dividers = [d for d in dividers if d.divider_type == 'road_divider']
    lane_dividers = [d for d in dividers if d.divider_type == 'lane_divider']

    logger.info(f"Extracted {len(road_dividers)} road dividers, {len(lane_dividers)} lane dividers")
    return road_dividers, lane_dividers
