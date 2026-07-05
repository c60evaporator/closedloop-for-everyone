"""
CARLA → nuScenes Map Expansion 変換ツール

CARLAシミュレーターに接続し、マップデータを抽出して
nuScenes Map Expansion形式（PNG basemap + JSON）として出力する。
"""
import argparse
import logging
import os
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path.cwd()
sys.path.insert(0, str(PROJECT_ROOT))

from dotenv import load_dotenv
import cv2

from src.carla_utils.carla_client import get_carla_client, set_world
from src.map_extraction.nuscenes.create_nusc_map import extract_carla_map_data, convert_carla_map_to_nuscenes

# ログ設定
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(name)s] %(levelname)s: %(message)s',
    datefmt='%H:%M:%S',
)
logger = logging.getLogger(__name__)


def carla_map_to_nuscenes(host, port, map_name, output_dir, project_name, sampling_resolution, closing_kernel_size):
    """
    CARLAマップをnuScenes Map Expansion形式に変換するメイン関数。

    Args:
        host (str): CARLAサーバのホスト名/IPアドレス（例: localhost）
        port (int): CARLAサーバのポート番号（例: 2000）
        map_name (str): 読み込むマップ名（例: Town01）
        output_dir (str): 出力ディレクトリ
        project_name (str): プロジェクト名（出力フォルダ構造用）
        sampling_resolution (float): ウェイポイントのサンプリング解像度（メートル単位）
        closing_kernel_size (int): ベースマップ生成時の形態学的クロージングのカーネルサイズ
    """
    logger.info("=" * 60)
    logger.info(f"Starting to extract {map_name} and convert to nuScenes Map Expansion format")
    logger.info("=" * 60)

    # ==================================================
    # Phase 1: CARLA接続
    # ==================================================
    logger.info("\n--- Phase 1: Connecting to CARLA ---")
    client = get_carla_client(host=host, port=port)
    world = set_world(client, map_name=map_name)
    map_name = world.get_map().name.split('/')[-1]

    # ==================================================
    # Phase 2: データ抽出
    # ==================================================
    logger.info("\n--- Phase 2: Extracting data from CARLA ---")
    carla_map_data = extract_carla_map_data(world, sampling_resolution=sampling_resolution)

    # ==================================================
    # Phase 3: nuScenes形式への変換
    # ==================================================
    logger.info("\n--- Phase 3: Converting to nuScenes format ---")
    map_data, basemap_image, coords_offset = convert_carla_map_to_nuscenes(carla_map_data, closing_kernel_size=closing_kernel_size)

    # ==================================================
    # Phase 4: 出力
    # ==================================================
    logger.info("\n--- Phase 4: Generating output ---")

    # 出力パス構築
    nuscenes_dir = os.path.join(output_dir, project_name, 'map_annotation/nuscenes')
    maps_dir = os.path.join(nuscenes_dir, 'maps')
    expansion_dir = os.path.join(maps_dir, 'expansion')
    os.makedirs(expansion_dir, exist_ok=True)

    # basemap PNG出力
    basemap_path = os.path.join(maps_dir, f'basemap_{map_name}.png')
    cv2.imwrite(basemap_path, basemap_image)

    # Map Expansion JSON出力
    json_path = os.path.join(expansion_dir, f'{map_name}.json')
    os.makedirs(os.path.dirname(json_path), exist_ok=True)
    with open(json_path, 'w') as f:
        json.dump(map_data, f, indent=2)

    # CARLA原点とnuScenesマップ原点（basemap左下端）のオフセット出力
    # （CARLA原点基準のグローバル座標を使う外部プログラムとの座標合わせ用）
    offset_path = os.path.join(maps_dir, f'{map_name}_coords_offset.json')
    with open(offset_path, 'w') as f:
        json.dump(coords_offset, f, indent=2)

    logger.info("\n" + "=" * 60)
    logger.info("Conversion complete!")
    logger.info(f"  Basemap: {basemap_path}")
    logger.info(f"  Map JSON: {json_path}")
    logger.info(f"  Coords offset: {offset_path}")
    logger.info("=" * 60)

def parse_args():
    parser = argparse.ArgumentParser(
        description='CARLA to nuScenes Map Expansion converter'
    )
    parser.add_argument('--host', type=str, default=None,
                        help='CARLA server host (default: env CARLA_HOST or localhost)')
    parser.add_argument('--port', type=int, default=None,
                        help='CARLA server port (default: env CARLA_PORT or 2000)')
    parser.add_argument('--map-name', type=str, default=None,
                        help='Map name to load (e.g., Town01). If not specified, uses current map')
    parser.add_argument('--output-dir', type=str, default=None,
                        help='Output directory (default: env DATASET_ROOT or ./data)')
    parser.add_argument('--project-name', type=str, default='debug',
                        help='Project name for output folder structure')
    parser.add_argument('--sampling-resolution', type=float, default=1.0,
                        help='Waypoint sampling resolution in meters (default: 1.0)')
    parser.add_argument('--closing-kernel-size', type=int, default=9,
                        help='Kernel size for morphological closing in basemap generation (default: 9)')
    return parser.parse_args()


def main():
    load_dotenv()
    args = parse_args()

    logger.info("=" * 60)
    logger.info("CARLA → nuScenes Map Expansion Converter")
    logger.info("=" * 60)

    output_dir = args.output_dir or os.environ.get('DATASET_ROOT', './data')
    carla_map_to_nuscenes(
        host=args.host or os.environ.get('CARLA_HOST', 'localhost'),
        port=args.port or int(os.environ.get('CARLA_PORT', 2000)),
        map_name=args.map_name,
        output_dir=output_dir,
        project_name=args.project_name,
        sampling_resolution=args.sampling_resolution,
        closing_kernel_size=args.closing_kernel_size
    )

    logger.info("=" * 60)
    logger.info("All done!")
    logger.info("=" * 60)

if __name__ == '__main__':
    main()
