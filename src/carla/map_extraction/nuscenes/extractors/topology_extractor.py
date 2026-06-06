"""
Topology Extractor: CARLAマップからトポロジー（道路ネットワーク）とジャンクション情報を抽出する
"""
import logging
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Set

import carla

logger = logging.getLogger(__name__)


@dataclass
class TopologyEdge:
    """トポロジーグラフの辺（レーン接続）"""
    start_road_id: int
    start_section_id: int
    start_lane_id: int
    start_x: float
    start_y: float
    end_road_id: int
    end_section_id: int
    end_lane_id: int
    end_x: float
    end_y: float


@dataclass
class JunctionInfo:
    """ジャンクション（交差点）の情報"""
    junction_id: int
    bounding_box_center: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    bounding_box_extent: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    # ジャンクション内のレーンペア (入口wp, 出口wp) のroad/lane情報
    lane_pairs: List[Tuple[Tuple[int, int, int], Tuple[int, int, int]]] = field(default_factory=list)


def extract_topology(carla_map) -> List[TopologyEdge]:
    """
    CARLAマップからトポロジー（レーン接続グラフ）を抽出する。
    :param carla_map: carla.Map
    :return: TopologyEdge のリスト
    """
    raw_topology = carla_map.get_topology()
    logger.info(f"Topology has {len(raw_topology)} edges")

    edges = []
    for start_wp, end_wp in raw_topology:
        edge = TopologyEdge(
            start_road_id=start_wp.road_id,
            start_section_id=start_wp.section_id,
            start_lane_id=start_wp.lane_id,
            start_x=start_wp.transform.location.x,
            start_y=start_wp.transform.location.y,
            end_road_id=end_wp.road_id,
            end_section_id=end_wp.section_id,
            end_lane_id=end_wp.lane_id,
            end_x=end_wp.transform.location.x,
            end_y=end_wp.transform.location.y,
        )
        edges.append(edge)

    return edges


def extract_junctions(carla_map, sampling_resolution) -> Dict[int, JunctionInfo]:
    """
    CARLAマップからジャンクション情報を抽出する。
    :param carla_map: carla.Map
    :param sampling_resolution: ウェイポイントのサンプリング解像度
    :return: junction_id -> JunctionInfo のDict

    > アルゴリズム:
    > 1. トポロジーからジャンクションIDを一括取得
    > 2. 各ジャンクションのIDと位置情報 (バウンディングボックス)を取得
    > 3. 各ジャンクションのレーンペア((road_id1, section_id1, lane_id1), (road_id2, section_id2, lane_id2))のリストを取得
    """

    topology = carla_map.get_topology()
    junction_ids: Set[int] = set()

    # トポロジーからジャンクションIDを収集
    for start_wp, end_wp in topology:
        if start_wp.is_junction and start_wp.get_junction():
            junction_ids.add(start_wp.get_junction().id)
        if end_wp.is_junction and end_wp.get_junction():
            junction_ids.add(end_wp.get_junction().id)

    logger.info(f"Found {len(junction_ids)} junctions")

    junctions: Dict[int, JunctionInfo] = {}

    # 各ジャンクションのIDと位置情報（バウンディングボックス）を取得
    # generate_waypointsからジャンクション情報を再収集
    waypoints = carla_map.generate_waypoints(sampling_resolution)
    for wp in waypoints:
        if not wp.is_junction:
            continue
        junc = wp.get_junction()
        if junc is None:
            continue
        jid = junc.id
        if jid not in junctions:
            bb = junc.bounding_box
            junctions[jid] = JunctionInfo(
                junction_id=jid,
                bounding_box_center=(bb.location.x, bb.location.y, bb.location.z),
                bounding_box_extent=(bb.extent.x, bb.extent.y, bb.extent.z),
            )

    # ジャンクション内のレーンペア((road_id1, section_id1, lane_id1), (road_id2, section_id2, lane_id2))のリストを取得
    for jid, jinfo in junctions.items():
        # get_junction().get_waypoints() でジャンクション内のレーンを取得
        for wp in waypoints:
            if wp.is_junction and wp.get_junction() and wp.get_junction().id == jid:
                junc = wp.get_junction()
                lane_wps = junc.get_waypoints(carla.LaneType.Driving)
                for start_wp, end_wp in lane_wps:
                    pair = (
                        (start_wp.road_id, start_wp.section_id, start_wp.lane_id),
                        (end_wp.road_id, end_wp.section_id, end_wp.lane_id),
                    )
                    if pair not in jinfo.lane_pairs:
                        jinfo.lane_pairs.append(pair)
                break  # 1つのウェイポイントから取得すれば十分

    for jid, jinfo in junctions.items():
        logger.info(f"  Junction {jid}: {len(jinfo.lane_pairs)} lane pairs")

    return junctions


def build_lane_connectivity(topology_edges: List[TopologyEdge]) -> Dict[Tuple[int, int, int], Dict]:
    """
    トポロジーからレーンのpredecessor/successor関係を構築する。
    :param topology_edges: extract_topology()で収集したトポロジー情報
    :return: (road_id, section_id, lane_id) -> {'predecessors': [...], 'successors': [...]}

    > returnする辞書のキーがレーン、値がそのレーンと接続している他のレーンのリストを表す
    """
    connectivity: Dict[Tuple[int, int, int], Dict] = {}

    for edge in topology_edges:
        start_key = (edge.start_road_id, edge.start_section_id, edge.start_lane_id)
        end_key = (edge.end_road_id, edge.end_section_id, edge.end_lane_id)

        if start_key not in connectivity:
            connectivity[start_key] = {'predecessors': [], 'successors': []}
        if end_key not in connectivity:
            connectivity[end_key] = {'predecessors': [], 'successors': []}

        if end_key not in connectivity[start_key]['successors']:
            connectivity[start_key]['successors'].append(end_key)
        if start_key not in connectivity[end_key]['predecessors']:
            connectivity[end_key]['predecessors'].append(start_key)

    return connectivity
