import logging

from src.carla.map.nuscenes.extractors.waypoint_extractor import WaypointExtractor
from src.carla.map.nuscenes.extractors.topology_extractor import (
    extract_topology, extract_junctions, build_lane_connectivity,
)
from src.carla.map.nuscenes.extractors.landmark_extractor import (
    extract_crosswalks, extract_traffic_lights, extract_stop_signs,
)
from src.carla.map.nuscenes.extractors.lane_marking_extractor import extract_dividers
from src.carla.map.nuscenes.extractors.actor_extractor import extract_traffic_light_actors
from src.carla.map.nuscenes.converters.geometry_builder import GeometryBuilder, CoordinateTransformer
from src.carla.map.nuscenes.converters.layer_builder import NuScenesLayerBuilder
from src.carla.map.nuscenes.converters.connectivity_builder import (
    build_arcline_paths, build_connectivity_dict,
)
from src.carla.map.nuscenes.output.basemap_generator import generate_basemap

logger = logging.getLogger(__name__)

def extract_carla_map_data(carla_world, sampling_resolution):
    """
    CARLAマップから必要なデータを抽出する。
    :param carla_world: CARLAワールドオブジェクト
    :param sampling_resolution: ウェイポイントのサンプリング間隔
    :return: 抽出されたデータの辞書
    """
    carla_map = carla_world.get_map()

    # waypointsからレーン情報を抽出する
    waypoint_extractor = WaypointExtractor(carla_map, sampling_resolution)
    lanes = waypoint_extractor.extract_lanes()
    parkings, sidewalks = waypoint_extractor.extract_parkings_sidewalks_from_opendrive()

    # トポロジー抽出
    topology_edges = extract_topology(carla_map)
    junctions = extract_junctions(carla_map, sampling_resolution=sampling_resolution)
    lane_connectivity = build_lane_connectivity(topology_edges)

    # ランドマーク抽出
    crosswalks = extract_crosswalks(carla_map)
    traffic_lights = extract_traffic_lights(carla_map)
    stop_signs = extract_stop_signs(carla_map)

    # アクター抽出
    extract_traffic_light_actors(carla_world)

    # レーンマーキング抽出
    road_dividers, lane_dividers = extract_dividers(lanes)
    
    # 統計情報ログ
    logger.info(f"Extracted data from CARLA map:")
    logger.info(f"  Lanes: {len(lanes)}")
    logger.info(f"  Topology edges: {len(topology_edges)}")
    logger.info(f"  Junctions: {len(junctions)}")
    logger.info(f"  Lane connectivity pairs: {len(lane_connectivity)}")
    logger.info(f"  Crosswalks: {len(crosswalks)}")
    logger.info(f"  Traffic lights: {len(traffic_lights)}")
    logger.info(f"  Stop signs: {len(stop_signs)}")
    logger.info(f"  Sidewalks: {len(sidewalks)}")
    logger.info(f"  Lane dividers: {len(lane_dividers)}")
    logger.info(f"  Road dividers: {len(road_dividers)}")

    return {
        'lanes': lanes,
        'topology_edges': topology_edges,
        'junctions': junctions,
        'lane_connectivity': lane_connectivity,
        'crosswalks': crosswalks,
        'traffic_lights': traffic_lights,
        'stop_signs': stop_signs,
        'parkings': parkings,
        'sidewalks': sidewalks,
        'lane_dividers': lane_dividers,
        'road_dividers': road_dividers,
    }

def convert_carla_map_to_nuscenes(carla_map_data, closing_kernel_size=9, merge_buffer=0.2, map_expansion_version="1.3"):
    """
    抽出されたCARLAマップデータをnuScenes形式に変換する。
    :param carla_map_data: extract_carla_map_dataで抽出されたデータの辞書
    :param closing_kernel_size: ベースマップ生成時のモルフォロジー閉処理のカーネルサイズ
    :param merge_buffer: ポリゴンマージ時のバッファサイズ
    :param map_expansion_version: nuScenesマップ拡張バージョン
    :return: nuScenes形式のマップデータ構造
    """
    # 座標変換の準備
    transformer = CoordinateTransformer()
    all_points = []
    for lane_info in carla_map_data['lanes'].values():
        all_points.extend(lane_info.centerline)
    for cw in carla_map_data['crosswalks']:
        all_points.extend(cw.vertices)
    transformer.compute_offsets(all_points)

    # 幾何プリミティブ構築
    gb = GeometryBuilder(transformer)

    # レイヤー構築
    lb = NuScenesLayerBuilder(gb)
    lb.build_lanes(carla_map_data['lanes'], carla_map_data['lane_connectivity'])
    lb.build_lane_connectors(carla_map_data['lanes'], carla_map_data['lane_connectivity'])
    lb.build_road_segments(carla_map_data['lanes'], parkings=carla_map_data['parkings'],
                           merge_buffer=merge_buffer)
    lb.build_road_blocks(carla_map_data['lanes'], parkings=carla_map_data['parkings'])
    lb.build_drivable_area()
    lb.build_ped_crossings(carla_map_data['crosswalks'])
    lb.build_carpark_areas(carla_map_data['parkings'])
    lb.build_walkways(carla_map_data['sidewalks'])
    lb.build_dividers(carla_map_data['lane_dividers'] + carla_map_data['road_dividers'])
    lb.build_stop_lines(carla_map_data['crosswalks'], carla_map_data['stop_signs'], carla_map_data['traffic_lights'])
    lb.build_traffic_lights(carla_map_data['traffic_lights'])
    lb.resolve_lane_connectivity()
    # 不要な内部キーを削除
    lb.remove_internal_keys()

    # 接続性構築
    arcline_paths = build_arcline_paths(
        carla_map_data['lanes'], lb._lane_key_to_token, gb,
    )
    connectivity = build_connectivity_dict(lb._lane_connectivity)

    # basemap PNG
    basemap_image, canvas_edge = generate_basemap(gb, lb, output_path=None, closing_kernel_size=closing_kernel_size)

    # 出力JSONデータ構造
    map_data = {
        'version': map_expansion_version,
        'canvas_edge': list(canvas_edge),
        # 幾何プリミティブ
        'node': gb.nodes,
        'line': gb.lines,
        'polygon': gb.polygons,
        # ポリゴンレイヤー
        'drivable_area': lb.drivable_area,
        'road_segment': lb.road_segment,
        'road_block': lb.road_block,
        'lane': lb.lane,
        'ped_crossing': lb.ped_crossing,
        'walkway': lb.walkway,
        'stop_line': lb.stop_line,
        'carpark_area': lb.carpark_area,
        # ラインレイヤー
        'road_divider': lb.road_divider,
        'lane_divider': lb.lane_divider,
        'traffic_light': lb.traffic_light,
        # 接続性
        'lane_connector': lb.lane_connector,
        'arcline_path_3': arcline_paths,
        'connectivity': connectivity,
    }

    # 統計情報
    total_records = sum(
        len(v) if isinstance(v, list) else len(v) if isinstance(v, dict) else 0
        for k, v in map_data.items()
        if k not in ('version', 'canvas_edge')
    )
    logger.info(f"Nuscenes map data generated")
    logger.info(f"  Total records: {total_records}")
    logger.info(f"  Nodes: {len(gb.nodes)}")
    logger.info(f"  Lines: {len(gb.lines)}")
    logger.info(f"  Polygons: {len(gb.polygons)}")
    logger.info(f"  Lanes: {len(lb.lane)}")
    logger.info(f"  Road segments: {len(lb.road_segment)}")
    logger.info(f"  Ped crossings: {len(lb.ped_crossing)}")
    logger.info(f"  Walkways: {len(lb.walkway)}")

    return map_data, basemap_image
