"""
Connectivity Builder: レーン接続性（lane_connector）とarcline_path_3を生成する
"""
import logging
import math
from typing import List, Dict, Tuple

from src.map_extraction.nuscenes.converters.geometry_builder import GeometryBuilder, generate_token
from src.map_extraction.nuscenes.extractors.topology_extractor import JunctionInfo
from src.map_extraction.nuscenes.extractors.waypoint_extractor import LaneInfo

logger = logging.getLogger(__name__)


def build_lane_connectors(
    junctions: Dict[int, JunctionInfo],
    lanes: Dict[Tuple[int, int, int], LaneInfo],
    geometry_builder: GeometryBuilder,
    lane_key_to_token: Dict[Tuple[int, int, int], str],
) -> Tuple[List[dict], Dict[str, dict], Dict[str, List[Tuple[float, float]]]]:
    """
    ジャンクション内のレーン接続（lane_connector）を構築する。
    :return: (connector_records, connector_connectivity, connector_centerlines)
        - connector_records: spec準拠のレコード (token, polygon_token のみ)
        - connector_connectivity: token -> {incoming, outgoing}
        - connector_centerlines: token -> CARLA座標系のcenterline (arcline生成用)
    """
    connectors = []
    connector_connectivity: Dict[str, dict] = {}
    connector_centerlines: Dict[str, List[Tuple[float, float]]] = {}

    for jid, jinfo in junctions.items():
        for start_key, end_key in jinfo.lane_pairs:
            for key, lane_info in lanes.items():
                if lane_info.is_junction and lane_info.junction_id == jid:
                    if not lane_info.boundary_points:
                        continue
                    polygon_token = geometry_builder.build_lane_polygon(lane_info.boundary_points)
                    if not polygon_token:
                        continue

                    connector_token = generate_token()
                    connectors.append({
                        'token': connector_token,
                        'polygon_token': polygon_token,
                    })

                    incoming = []
                    outgoing = []
                    if start_key in lane_key_to_token:
                        incoming.append(lane_key_to_token[start_key])
                    if end_key in lane_key_to_token:
                        outgoing.append(lane_key_to_token[end_key])

                    connector_connectivity[connector_token] = {
                        'incoming': incoming,
                        'outgoing': outgoing,
                    }

                    if lane_info.centerline:
                        connector_centerlines[connector_token] = lane_info.centerline

                    break

    logger.info(f"Built {len(connectors)} lane_connectors")
    return connectors, connector_connectivity, connector_centerlines


def _build_arcline_segments(
    centerline: List[Tuple[float, float]],
    geometry_builder: GeometryBuilder,
) -> List[dict]:
    """中心線のポイントリストからarcline_path_3セグメントリストを生成する"""
    segments = []
    for i in range(len(centerline) - 1):
        x0, y0 = centerline[i]
        x1, y1 = centerline[i + 1]

        ns_x0, ns_y0 = geometry_builder.transformer.transform(x0, y0)
        ns_x1, ns_y1 = geometry_builder.transformer.transform(x1, y1)

        dx = ns_x1 - ns_x0
        dy = ns_y1 - ns_y0
        length = math.sqrt(dx * dx + dy * dy)

        if length < 1e-6:
            continue

        heading = math.atan2(dy, dx)

        segment = {
            'start_pose': [ns_x0, ns_y0, heading],
            'end_pose': [ns_x1, ns_y1, heading],
            'shape': 'LSL',
            'radius': 999999.0,
            'segment_length': [0.0, length, 0.0],
        }
        segments.append(segment)

    return segments


def build_arcline_paths(
    lanes: Dict[Tuple[int, int, int], LaneInfo],
    lane_key_to_token: Dict[Tuple[int, int, int], str],
    geometry_builder: GeometryBuilder,
    connector_centerlines: Dict[str, List[Tuple[float, float]]] = None,
) -> Dict[str, List[dict]]:
    """
    レーン中心線からarcline_path_3を生成する。
    :param connector_centerlines: connector_token -> centerline の辞書（コネクタ用）
    :return: lane_token -> [arcline_path_segment, ...] のDict
    """
    arcline_paths: Dict[str, List[dict]] = {}

    # レーンのarcline paths
    for key, lane_info in lanes.items():
        if key not in lane_key_to_token:
            continue
        if lane_info.lane_type != 'Driving':
            continue
        if len(lane_info.centerline) < 2:
            continue

        lane_token = lane_key_to_token[key]
        segments = _build_arcline_segments(lane_info.centerline, geometry_builder)
        if segments:
            arcline_paths[lane_token] = segments

    # コネクタのarcline paths
    if connector_centerlines:
        for connector_token, centerline in connector_centerlines.items():
            if len(centerline) < 2:
                continue
            segments = _build_arcline_segments(centerline, geometry_builder)
            if segments:
                arcline_paths[connector_token] = segments

    logger.info(f"Built arcline_paths for {len(arcline_paths)} lanes/connectors")
    return arcline_paths


def build_connectivity_dict(
    lane_connectivity: Dict[str, dict],
    connector_connectivity: Dict[str, dict] = None,
) -> Dict[str, dict]:
    """
    レーンとlane_connectorのconnectivityデータを統合する。
    :param lane_connectivity: lane_token -> {incoming: [...], outgoing: [...]}
    :param connector_connectivity: connector_token -> {incoming: [...], outgoing: [...]}
    :return: token -> {incoming: [...], outgoing: [...]} のDict
    """
    connectivity: Dict[str, dict] = {}

    for token, conn in lane_connectivity.items():
        connectivity[token] = {
            'incoming': conn['incoming'],
            'outgoing': conn['outgoing'],
        }

    if connector_connectivity:
        for token, conn in connector_connectivity.items():
            connectivity[token] = {
                'incoming': conn['incoming'],
                'outgoing': conn['outgoing'],
            }

    logger.info(f"Built connectivity for {len(connectivity)} entries")
    return connectivity
