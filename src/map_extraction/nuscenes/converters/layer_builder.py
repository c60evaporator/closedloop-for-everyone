"""
Layer Builder: CARLAから抽出したデータをnuScenes Map Expansionのレイヤーに変換する
"""
import logging
import math
from typing import List, Dict, Tuple
from collections import defaultdict

from shapely.geometry import Polygon as ShapelyPolygon
from shapely.geometry import LineString as ShapelyLineString
from shapely.ops import unary_union

from src.map_extraction.nuscenes.converters.geometry_builder import GeometryBuilder, generate_token
from src.map_extraction.nuscenes.extractors.waypoint_extractor import LaneInfo
from src.map_extraction.nuscenes.extractors.landmark_extractor import (
    CrosswalkPolygon, StopSignInfo,
)
from src.map_extraction.nuscenes.extractors.actor_extractor import TrafficLightActorInfo
from src.map_extraction.nuscenes.extractors.lane_marking_extractor import DividerLine
from src.map_extraction.utils.geom import get_lane_polygon_points, split_polygon_by_line, merge_linestrings_greedy

logger = logging.getLogger(__name__)

# CARLAのマーキングタイプからnuScenesのsegment_typeへの変換
_MARKING_TO_SEGMENT_TYPE = {
    'Solid': 'SOLID_WHITE',
    'Broken': 'DASHED_WHITE',
    'SolidSolid': 'DOUBLE_SOLID_WHITE',
    'BrokenBroken': 'DOUBLE_DASHED_WHITE',
    'BrokenSolid': 'DASHED_SOLID_WHITE',
    'SolidBroken': 'SOLID_DASHED_WHITE',
    'Other': 'NIL',
    'NONE': 'NIL',
    'Curb': 'NIL',
    '': 'NIL',
}


class NuScenesLayerBuilder:
    """nuScenes Map Expansion のレイヤーを構築する"""

    def __init__(self, geometry_builder: GeometryBuilder):
        self.gb = geometry_builder
        # nuScenes レイヤーデータ
        self.drivable_area: List[dict] = []
        self.road_segment: List[dict] = []
        self.road_block: List[dict] = []
        self.lane: List[dict] = []
        self.lane_connector: List[dict] = []
        self.ped_crossing: List[dict] = []
        self.walkway: List[dict] = []
        self.stop_line: List[dict] = []
        self.carpark_area: List[dict] = []
        self.road_divider: List[dict] = []
        self.lane_divider: List[dict] = []
        self.traffic_light: List[dict] = []

        # 内部マッピング: (road_id, section_id, lane_id) -> lane token
        self._lane_key_to_token: Dict[Tuple[int, int, int], str] = {}
        # build_traffic_lightsで生成したtoken（入力アクターと同順に、アクターごとの全ヘッドtoken。stop_line紐付け用）
        self._traffic_light_tokens: List[List[str]] = []

    def _calc_from_to_edge_lines(self, lane_info: LaneInfo) -> Tuple[List[Tuple[float, float]], List[Tuple[float, float]]]:
        # 走行方向の判定: boundary_pointsの並び方向に対してleft境界が左側にあるか（外積で判定）
        bp_first = lane_info.boundary_points[0]
        bp_last = lane_info.boundary_points[-1]

        # 中心線の方向ベクトル (first -> last)
        cx_first = (bp_first.left_x + bp_first.right_x) / 2
        cy_first = (bp_first.left_y + bp_first.right_y) / 2
        cx_last = (bp_last.left_x + bp_last.right_x) / 2
        cy_last = (bp_last.left_y + bp_last.right_y) / 2
        dir_x = cx_last - cx_first
        dir_y = cy_last - cy_first

        # first地点での中心→左境界ベクトル
        to_left_x = bp_first.left_x - cx_first
        to_left_y = bp_first.left_y - cy_first

        # 外積: dir × to_left > 0 なら左境界が進行方向の左側（正しい）
        cross = dir_x * to_left_y - dir_y * to_left_x
        forward = cross > 0  # Trueならboundary_pointsの並びが走行方向と一致

        if forward:
            from_bp = bp_first
            to_bp = bp_last
        else:
            from_bp = bp_last
            to_bp = bp_first

        to_edge_line = [
            (from_bp.left_x, from_bp.left_y),
            (from_bp.right_x, from_bp.right_y),
        ]
        from_edge_line = [
            (to_bp.left_x, to_bp.left_y),
            (to_bp.right_x, to_bp.right_y),
        ]
        return from_edge_line, to_edge_line

    def build_lanes(
        self,
        lanes: Dict[Tuple[int, int, int], LaneInfo],
        connectivity: Dict[Tuple[int, int, int], Dict],
    ) -> None:
        """
        レーンレイヤーを構築する。Drivingタイプのレーンのみ。
        """
        driving_lanes = {k: v for k, v in lanes.items() if v.lane_type == 'Driving' and not v.is_junction}
        logger.info(f"Building {len(driving_lanes)} driving lanes (excluding junction lanes)")

        for key, lane_info in driving_lanes.items():
            if not lane_info.boundary_points:
                continue

            polygon_token = self.gb.build_lane_polygon(lane_info.boundary_points)
            if not polygon_token:
                continue

            lane_token = generate_token()
            self._lane_key_to_token[key] = lane_token

            # predecessor/successor トークン（後で解決）
            conn = connectivity.get(key, {'predecessors': [], 'successors': []})

            # from/to edge lines from boundary points
            from_edge_line, to_edge_line = self._calc_from_to_edge_lines(lane_info)
            to_edge_line_token = self.gb.add_line(to_edge_line)
            from_edge_line_token = self.gb.add_line(from_edge_line)

            # left/right lane divider segments from boundary points
            left_segment_type = _MARKING_TO_SEGMENT_TYPE.get(lane_info.left_marking_type, 'NIL')
            right_segment_type = _MARKING_TO_SEGMENT_TYPE.get(lane_info.right_marking_type, 'NIL')

            left_lane_divider_segments = []
            for bp in lane_info.boundary_points:
                node_token = self.gb.add_node(bp.left_x, bp.left_y)
                left_lane_divider_segments.append({
                    'node_token': node_token,
                    'segment_type': left_segment_type,
                })

            right_lane_divider_segments = []
            for bp in lane_info.boundary_points:
                node_token = self.gb.add_node(bp.right_x, bp.right_y)
                right_lane_divider_segments.append({
                    'node_token': node_token,
                    'segment_type': right_segment_type,
                })

            record = {
                'token': lane_token,
                'polygon_token': polygon_token,
                'lane_type': 'CAR',
                'from_edge_line_token': from_edge_line_token,
                'to_edge_line_token': to_edge_line_token,
                'left_lane_divider_segments': left_lane_divider_segments,
                'right_lane_divider_segments': right_lane_divider_segments,
                '_carla_key': key,  # 内部参照用（最終出力からは除外）
                '_predecessors': conn['predecessors'],
                '_successors': conn['successors'],
            }
            self.lane.append(record)

    def build_lane_connectors(
        self,
        lanes: Dict[Tuple[int, int, int], LaneInfo],
        connectivity: Dict[Tuple[int, int, int], Dict],
    ) -> None:
        """
        ジャンクションレーンをlane_connectorとして構築する。
        """
        junction_lanes = {k: v for k, v in lanes.items() if v.lane_type == 'Driving' and v.is_junction}
        logger.info(f"Building {len(junction_lanes)} lane connectors from junction lanes")

        for key, lane_info in junction_lanes.items():
            if not lane_info.boundary_points:
                continue

            polygon_token = self.gb.build_lane_polygon(lane_info.boundary_points)
            if not polygon_token:
                continue

            connector_token = generate_token()
            self._lane_key_to_token[key] = connector_token

            conn = connectivity.get(key, {'predecessors': [], 'successors': []})

            record = {
                'token': connector_token,
                'polygon_token': polygon_token,
                '_carla_key': key,
                '_predecessors': conn['predecessors'],
                '_successors': conn['successors'],
            }
            self.lane_connector.append(record)

    def resolve_lane_connectivity(self) -> None:
        """レーンとlane_connectorのpredecessor/successorトークンを解決し、内部connectivity dictに格納する"""
        self._lane_connectivity: Dict[str, dict] = {}

        for record in self.lane + self.lane_connector:
            incoming = []
            outgoing = []

            for pred_key in record.get('_predecessors', []):
                if pred_key in self._lane_key_to_token:
                    incoming.append(self._lane_key_to_token[pred_key])

            for succ_key in record.get('_successors', []):
                if succ_key in self._lane_key_to_token:
                    outgoing.append(self._lane_key_to_token[succ_key])

            self._lane_connectivity[record['token']] = {
                'incoming': incoming,
                'outgoing': outgoing,
            }

            # 内部参照キーを削除
            record.pop('_carla_key', None)
            record.pop('_predecessors', None)
            record.pop('_successors', None)

    def build_road_segments(
        self,
        lanes: Dict[Tuple[int, int, int], LaneInfo],
        parkings: Dict[Tuple[int, int, int], LaneInfo],
        merge_buffer: float = 0.2,
    ) -> None:
        """
        road_segment および carpark_areaレイヤーを構築する。
        非ジャンクションレーンをroad_id毎に統合、ジャンクションレーンをjunction_id毎に統合。
        """
        def _lanes_to_shapely(lane_infos: List[LaneInfo]) -> List:
            polys = []
            for lane_info in lane_infos:
                if not lane_info.boundary_points:
                    continue
                polygon_points = get_lane_polygon_points(lane_info.boundary_points)
                if len(polygon_points) >= 3:
                    try:
                        poly = ShapelyPolygon(polygon_points)
                        if poly.is_valid:
                            polys.append(poly)
                        else:
                            polys.append(poly.buffer(0))
                    except Exception:
                        continue
            return polys

        def _merge_and_add(shapely_polygons, is_intersection: bool, merge_buffer: float = 0.2,
                           road_id: int = None) -> None:
            if not shapely_polygons:
                return
            try:
                # 微小バッファで隣接レーン間のギャップを埋めてからマージ（Segmentのポリゴンが想定外に分割されるのを防止）
                buffered = [p.buffer(merge_buffer) for p in shapely_polygons]
                merged = unary_union(buffered).buffer(-merge_buffer)
            except Exception:
                return

            if merged.geom_type == 'Polygon':
                polygons_to_add = [merged]
            elif merged.geom_type == 'MultiPolygon':
                polygons_to_add = list(merged.geoms)
            else:
                return

            for poly in polygons_to_add:
                # Shapely includes closing vertex; strip it
                exterior_coords = list(poly.exterior.coords)
                if len(exterior_coords) > 1 and exterior_coords[0] == exterior_coords[-1]:
                    exterior_coords = exterior_coords[:-1]
                # Extract holes from Shapely interiors
                holes = None
                if poly.interiors:
                    holes = []
                    for interior in poly.interiors:
                        hole_coords = list(interior.coords)
                        if len(hole_coords) > 1 and hole_coords[0] == hole_coords[-1]:
                            hole_coords = hole_coords[:-1]
                        holes.append(hole_coords)
                polygon_token = self.gb.add_polygon(exterior_coords, holes=holes)
                road_segment_record = {
                    'token': generate_token(),
                    'polygon_token': polygon_token,
                    'is_intersection': is_intersection,
                    'drivable_area_token': '',
                }
                # road_divider, road_block, walkway用にcarla座標系のshapelyポリゴンを追加（最終出力からは除外）
                road_segment_record['_shapely_polygon'] = poly
                # road_divider, road_block用にroad_idを追加（最終出力からは除外）
                if road_id is not None:
                    road_segment_record['_road_id'] = road_id
                # road_segmentを追加
                self.road_segment.append(road_segment_record)

        # 非ジャンクション: road_id毎にレーンを集約
        road_lanes: Dict[int, List[LaneInfo]] = defaultdict(list)
        # ジャンクション: junction_id毎にレーンを集約
        junction_lanes: Dict[int, List[LaneInfo]] = defaultdict(list)

        for key, lane_info in lanes.items():
            if lane_info.lane_type != 'Driving':
                continue
            if lane_info.is_junction:
                junction_lanes[lane_info.junction_id].append(lane_info)
            else:
                road_lanes[lane_info.road_id].append(lane_info)

        logger.info(f"Building road segments for {len(road_lanes)} roads + {len(junction_lanes)} junctions")

        for road_id, road_lane_infos in road_lanes.items():
            parking_in_segment = [p for k, p in parkings.items() if k[0] == road_id]
            parking_polys = _lanes_to_shapely(parking_in_segment)
            lane_polys = _lanes_to_shapely(road_lane_infos)
            _merge_and_add(parking_polys + lane_polys, is_intersection=False, merge_buffer=merge_buffer,
                           road_id=road_id)

        for jid, junc_lane_infos in junction_lanes.items():
            polys = _lanes_to_shapely(junc_lane_infos)
            _merge_and_add(polys, is_intersection=True, merge_buffer=merge_buffer)

    def build_road_blocks(
        self,
        lanes: Dict[Tuple[int, int, int], LaneInfo],
        parkings: Dict[Tuple[int, int, int], LaneInfo],
    ) -> None:
        """
        road_block レイヤーを構築する。
        非ジャンクションのroad_segmentを、lane_idの符号が反転するlane境界線で分割してroad_blockとする。
        """
        def _get_from_to_edge_lines(lane_infos):
            """複数laneのfrom/to edge lineを計算して結合して返す"""
            from_edge_lines, to_edge_lines = [], []
            for lane_info in lane_infos:
                if not lane_info.boundary_points:
                    continue
                from_edge_line, to_edge_line = self._calc_from_to_edge_lines(lane_info)
                from_edge_lines.append(ShapelyLineString(from_edge_line))
                to_edge_lines.append(ShapelyLineString(to_edge_line))
            # 複数laneのedge lineを端点の近さで順に連結して1本のLineStringにする
            from_edge_merged = merge_linestrings_greedy(from_edge_lines)
            to_edge_merged = merge_linestrings_greedy(to_edge_lines)
            return from_edge_merged, to_edge_merged
        
        for rs in self.road_segment:
            # 非ジャンクションかつroad_idが存在するroad_segmentのみ処理
            road_id = rs.get('_road_id')            
            if road_id is None or rs.get('is_intersection'):
                continue
            # 対応するparkingとlaneを抽出して結合し、ソート
            parking_infos = [p for k, p in parkings.items() if p.road_id == road_id]
            lane_infos = [li for k, li in lanes.items() if li.road_id == road_id and not li.is_junction and li.lane_type == 'Driving']
            merged_lane_infos = parking_infos + lane_infos
            if not merged_lane_infos:
                continue
            merged_lane_infos.sort(key=lambda x: x.lane_id)
            # road_segmentと重なりがあるlaneのみを抽出（同じroad_idでもポリゴンがつながっていない場合別のroad_segmentに分割済のため、これに対応）
            merged_lane_infos = [li for li in merged_lane_infos if rs['_shapely_polygon'].intersects(ShapelyPolygon(get_lane_polygon_points(li.boundary_points)))]
            # lane_idの符号が反転するlaneを特定（次のlaneと比較）。反転がない場合road_segmentをroad_blockとして追加して終了。
            reversed_idx = next((i for i, li in enumerate(merged_lane_infos[:-1]) if li.lane_id * merged_lane_infos[i + 1].lane_id < 0), None)
            if reversed_idx is None:
                # from/to edge lineを計算してroad_blockを追加
                from_edge_line, to_edge_line = _get_from_to_edge_lines(merged_lane_infos)
                from_edge_line_token = self.gb.add_line([(x, y) for x, y in from_edge_line.coords])
                to_edge_line_token = self.gb.add_line([(x, y) for x, y in to_edge_line.coords])
                self.road_block.append({
                    'token': generate_token(),
                    'polygon_token': rs['polygon_token'],
                    'from_edge_line_token': from_edge_line_token,
                    'to_edge_line_token': to_edge_line_token,
                    'road_segment_token': rs['token'],
                    '_shapely_polygon': rs['_shapely_polygon'],  # carpark_area紐付け用（最終出力からは除外）
                })
                continue

            # 符号が反転する車線のboundaryを取得してShapelyのLineStringに変換 (road_dividerに相当)
            reversed_lane = lane_infos[reversed_idx]
            boundary_direction = 'left' if reversed_lane.traffic_direction == 'righthand' else 'right'
            if boundary_direction == 'left':
                boundary_points = [(bp.left_x, bp.left_y) for bp in reversed_lane.boundary_points]
            else:
                boundary_points = [(bp.right_x, bp.right_y) for bp in reversed_lane.boundary_points]
            splitter_line = ShapelyLineString(boundary_points)
            # splitter_lineでroad_segmentを分割してroad_blockを生成
            edge_boundary = rs['_shapely_polygon'].bounds  # (min_x, min_y, max_x, max_y)
            split_polygons, _ = split_polygon_by_line(rs['_shapely_polygon'], splitter_line, edge_boundary, extend_line_to_edge=True)
            if split_polygons is None:
                continue
            # 分割されたポリゴンをlane_idが小さい側と大きい側に分類（reversed_laneと重なり面積が大きい方を小さい側とする）
            # blocks[0] = lane_id小さい側, blocks[1] = lane_id大きい側
            blocks = list(split_polygons.geoms)
            reversed_lane_polygon = ShapelyPolygon(get_lane_polygon_points(reversed_lane.boundary_points))
            if len(blocks) == 2:
                if blocks[0].intersection(reversed_lane_polygon).area <= blocks[1].intersection(reversed_lane_polygon).area:
                    blocks[0], blocks[1] = blocks[1], blocks[0]
            else:
                raise ValueError('Expected 2 blocks after splitting road segment, got {}'.format(len(blocks)))
            # 分割されたブロックをroad_blockとして追加（各ブロックに対応するlaneからedge lineを計算）
            negative_lanes = [li for li in lane_infos if li.lane_id < 0]
            positive_lanes = [li for li in lane_infos if li.lane_id > 0]
            for idx, block in enumerate(blocks):
                exterior_coords = list(block.exterior.coords)
                polygon_token = self.gb.add_polygon(exterior_coords)
                # edge lineを計算
                block_lanes = negative_lanes if idx == 0 else positive_lanes
                from_edge_line, to_edge_line = _get_from_to_edge_lines(block_lanes)
                from_edge_line_token = self.gb.add_line([(x, y) for x, y in from_edge_line.coords])
                to_edge_line_token = self.gb.add_line([(x, y) for x, y in to_edge_line.coords])
                self.road_block.append({
                    'token': generate_token(),
                    'polygon_token': polygon_token,
                    'from_edge_line_token': from_edge_line_token,
                    'to_edge_line_token': to_edge_line_token,
                    'road_segment_token': rs['token'],
                    '_shapely_polygon': block,  # carpark_area紐付け用（最終出力からは除外）
                })

    def build_drivable_area(self) -> None:
        """
        drivable_area レイヤーを構築する。
        全road_segmentのポリゴンを参照し、各road_segmentにdrivable_area_tokenを設定する。
        """
        if not self.road_segment:
            return

        polygon_tokens = [rs['polygon_token'] for rs in self.road_segment]
        da_token = generate_token()
        self.drivable_area.append({
            'token': da_token,
            'polygon_tokens': polygon_tokens,
        })

        # 全road_segmentにdrivable_area_tokenを設定
        for rs in self.road_segment:
            rs['drivable_area_token'] = da_token

        logger.info(f"Built drivable_area with {len(polygon_tokens)} polygons")

    def build_ped_crossings(self, crosswalks: List[CrosswalkPolygon]) -> None:
        """横断歩道レイヤーを構築する"""
        road_segment_polygons = [rs['_shapely_polygon'] for rs in self.road_segment]
        # 各crosswalkを走査
        for cw in crosswalks:
            if len(cw.vertices) < 3:
                continue
            polygon_token = self.gb.add_polygon(cw.vertices)
            # crosswalkのポリゴンと交差が最大のroad_segmentを特定してtokenを紐づける
            cw_polygon = ShapelyPolygon(cw.vertices)
            max_intersection, max_index = max(
                (cw_polygon.intersection(rspoly).area, idx)
                for idx, rspoly in enumerate(road_segment_polygons)
            )
            if max_intersection > 0:
                road_segment_token = self.road_segment[max_index]['token']
            else:
                road_segment_token = ''
            self.ped_crossing.append({
                'token': generate_token(),
                'polygon_token': polygon_token,
                'road_segment_token': road_segment_token,
            })
        logger.info(f"Built {len(self.ped_crossing)} ped_crossings")

    def build_carpark_areas(self, parkings: List[LaneInfo]) -> None:
        """駐車スペースレイヤーを構築する"""
        road_block_polygons = [rb['_shapely_polygon'] for rb in self.road_block]
        # 各parkingを走査
        for key, carpark in parkings.items():
            polygon_token = self.gb.build_lane_polygon(carpark.boundary_points)
            # carpark_areaのポリゴンと交差が最大のroad_blockを特定してtokenを紐づける
            carpark_polygon = ShapelyPolygon(get_lane_polygon_points(carpark.boundary_points))
            max_intersection, max_index = max(
                (carpark_polygon.intersection(rbpoly).area, idx)
                for idx, rbpoly in enumerate(road_block_polygons)
            )
            if max_intersection > 0:
                road_block_token = self.road_block[max_index]['token']
            else:
                road_block_token = ''
            # Orientationをcenterlineの方向から計算
            centerline_dir = math.atan2(carpark.centerline[-1][1]-carpark.centerline[0][1], 
                                        carpark.centerline[-1][0]-carpark.centerline[0][0])
            self.carpark_area.append({
                'token': generate_token(),
                'polygon_token': polygon_token,
                'orientation': centerline_dir,
                'road_block_token': road_block_token,
            })

        logger.info(f"Built {len(self.carpark_area)} carpark_areas")

    def build_walkways(self, sidewalks: List[LaneInfo]) -> None:
        """歩道レイヤーを構築する"""
        for key, sw in sidewalks.items():
            polygon_token = self.gb.build_lane_polygon(sw.boundary_points)
            self.walkway.append({
                'token': generate_token(),
                'polygon_token': polygon_token,
            })

        logger.info(f"Built {len(self.walkway)} walkways")

    def build_dividers(self, divider_lines: List[DividerLine]) -> None:
        """road_divider, lane_divider, road_block レイヤーを構築する"""
        for dl in divider_lines:
            # ジャンクションに属するラインは無視
            if dl.is_junction:
                continue
            # ポイント数が2未満のラインは無視（nuScenesのラインは少なくとも2点必要）
            if len(dl.points) < 2:
                continue
            line_token = self.gb.add_line(dl.points)

            record = {
                'token': generate_token(),
                'line_token': line_token,
            }

            if dl.divider_type == 'road_divider':
                # 紐づくroad_segmentを検出（road_idが同じで、かつdivider_lineとroad_segmentのポリゴンが交差するもの）
                record['road_segment_token'] = ''
                for rs in self.road_segment:
                    if rs.get('_road_id') == dl.road_id and rs.get('_shapely_polygon') and ShapelyLineString(dl.points).intersects(rs['_shapely_polygon']):
                        record['road_segment_token'] = rs['token']
                self.road_divider.append(record)
            else:
                # Build lane_divider_segments from line's node_tokens
                line_record = self.gb.lines[-1]
                segment_type = _MARKING_TO_SEGMENT_TYPE.get(dl.marking_type, 'NIL')
                record['lane_divider_segments'] = [
                    {'node_token': nt, 'segment_type': segment_type}
                    for nt in line_record['node_tokens']
                ]
                self.lane_divider.append(record)

        logger.info(f"Built {len(self.road_divider)} road_dividers, {len(self.lane_divider)} lane_dividers")

    def _find_road_block_token(self, geometry) -> str:
        """ジオメトリとの交差面積が最大のroad_blockのtokenを返す（交差するものがなければ空文字）"""
        best_token = ''
        best_area = 0.0
        for rb in self.road_block:
            rb_polygon = rb.get('_shapely_polygon')
            if rb_polygon is None:
                continue
            area = rb_polygon.intersection(geometry).area
            if area > best_area:
                best_area = area
                best_token = rb['token']
        return best_token

    def _make_stop_line_geometry(self, stop_line_points: List[Tuple[float, float]]):
        """停止線の点列から0.6m幅の帯状ポリゴン（Shapely、CARLA座標系）を生成する"""
        return ShapelyLineString(stop_line_points).buffer(0.3, cap_style=2)

    def build_stop_lines(
        self,
        crosswalks: List[CrosswalkPolygon],
        stop_signs: List[StopSignInfo],
        traffic_light_actors: List[TrafficLightActorInfo],
    ) -> None:
        """stop_line レイヤーを構築する。信号機由来のstop_lineを含むため、build_traffic_lightsの後に呼ぶこと"""
        # 横断歩道に付随する停止線
        for i, cw in enumerate(crosswalks):
            if len(cw.vertices) < 2:
                continue

            # 横断歩道の一辺から薄いポリゴンを生成
            p1 = cw.vertices[0]
            p2 = cw.vertices[1]
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            length = math.sqrt(dx * dx + dy * dy)
            if length < 1e-6:
                continue
            # 0.3m幅の薄い矩形ポリゴン
            nx = -dy / length * 0.3
            ny = dx / length * 0.3
            stop_polygon = [
                (p1[0] - nx, p1[1] - ny),
                (p2[0] - nx, p2[1] - ny),
                (p2[0] + nx, p2[1] + ny),
                (p1[0] + nx, p1[1] + ny),
            ]
            polygon_token = self.gb.add_polygon(stop_polygon)

            ped_crossing_token = self.ped_crossing[i]['token'] if i < len(self.ped_crossing) else ''

            self.stop_line.append({
                'token': generate_token(),
                'polygon_token': polygon_token,
                'stop_line_type': 'PED_CROSSING',
                'ped_crossing_tokens': [ped_crossing_token] if ped_crossing_token else [],
                'traffic_light_tokens': [],
                'road_block_token': '',
            })

        # 信号機に付随する停止線（アクターの停止waypointから生成し、同一ポールの全ヘッドのtokenと紐づける）
        if traffic_light_actors and len(self._traffic_light_tokens) != len(traffic_light_actors):
            logger.warning("build_traffic_lights must be called before build_stop_lines to link traffic_light_tokens")
        for tl, head_tokens in zip(traffic_light_actors, self._traffic_light_tokens):
            if len(tl.stop_line_points) < 2:
                continue
            stop_line_geom = self._make_stop_line_geometry(tl.stop_line_points)
            polygon_token = self.gb.add_polygon(list(stop_line_geom.exterior.coords))
            self.stop_line.append({
                'token': generate_token(),
                'polygon_token': polygon_token,
                'stop_line_type': 'TRAFFIC_LIGHT',
                'ped_crossing_tokens': [],
                'traffic_light_tokens': head_tokens,
                'road_block_token': self._find_road_block_token(stop_line_geom),
            })

        logger.info(f"Built {len(self.stop_line)} stop_lines")

    def build_traffic_lights(self, traffic_light_actors: List[TrafficLightActorInfo]) -> None:
        """
        traffic_light レイヤーを構築する。build_road_blocksの後に呼ぶこと。
        本家nuScenesに合わせて1レコード=1灯器ヘッドとし、同一ポールのヘッド群は
        from_road_block_token（および_traffic_light_tokens経由でstop_line）を共有する。
        """
        self._traffic_light_tokens = []

        def _append_record(x: float, y: float, z: float, yaw_rad: float,
                           traffic_light_type: str, items: List[dict],
                           from_road_block_token: str) -> str:
            token = generate_token()
            # 灯器正面（yaw）方向へ1mのラインとして登録（poseと同情報を表す）
            line_token = self.gb.add_line([
                (x, y),
                (x + math.cos(yaw_rad), y + math.sin(yaw_rad)),
            ])
            ns_x, ns_y = self.gb.transformer.transform(x, y)
            self.traffic_light.append({
                'token': token,
                'line_token': line_token,
                'traffic_light_type': traffic_light_type,
                'from_road_block_token': from_road_block_token,
                'items': items,
                'pose': {
                    'tx': ns_x,
                    'ty': ns_y,
                    'tz': z,
                    'rx': 0.0,
                    'ry': 0.0,
                    # CARLA（左手系、Y南向き）→nuScenes（右手系）でY軸反転するためyawは符号反転
                    'rz': -yaw_rad,
                },
            })
            return token

        for tl in traffic_light_actors:
            yaw_rad = math.radians(tl.yaw)
            # 停止線（制御対象レーン上）との交差面積が最大のroad_blockを進入元として紐づける
            from_road_block_token = ''
            if len(tl.stop_line_points) >= 2:
                from_road_block_token = self._find_road_block_token(
                    self._make_stop_line_geometry(tl.stop_line_points))

            head_tokens = []
            for head in tl.light_heads:
                # CARLAでは個々のバルブ位置が取れないため、筐体の高さを3等分してRED/YELLOW/GREENを合成する
                bulb_spacing = 2 * head.extent_z / 3
                items = [
                    {'color': color, 'shape': 'CIRCLE',
                     'rel_pos': {'tx': 0.0, 'ty': 0.0, 'tz': tz, 'rx': 0.0, 'ry': 0.0, 'rz': 0.0},
                     'to_road_block_tokens': []}
                    for color, tz in [('RED', bulb_spacing), ('YELLOW', 0.0), ('GREEN', -bulb_spacing)]
                ]
                traffic_light_type = 'VERTICAL' if head.extent_z >= max(head.extent_x, head.extent_y) else 'HORIZONTAL'
                head_tokens.append(_append_record(
                    head.x, head.y, head.z, yaw_rad, traffic_light_type, items, from_road_block_token))

            # ヘッドが取得できないアクターはポール位置で1レコードを出してstop_lineとのリンクを維持
            if not head_tokens:
                head_tokens.append(_append_record(
                    tl.x, tl.y, tl.z, yaw_rad, 'VERTICAL', [], from_road_block_token))

            self._traffic_light_tokens.append(head_tokens)

        logger.info(f"Built {len(self.traffic_light)} traffic_lights from {len(traffic_light_actors)} actors")

    def remove_internal_keys(self) -> None:
        """最終出力前に、内部参照用のキーを全てのレコードから削除する"""
        for record in self.lane + self.lane_connector + self.road_segment + self.road_block:
            record.pop('_carla_key', None)
            record.pop('_predecessors', None)
            record.pop('_successors', None)
            record.pop('_shapely_polygon', None)
            record.pop('_road_id', None)
