"""
Basemap Generator: nuScenes形式のベースマップPNGを生成する

nuScenes basemap仕様:
  - グレースケール (mode 'L'), 2値: 0=黒(背景), 255=白(走行可能領域)
  - 解像度: 10 px/m (= 0.1 m/px)
  - 画像サイズ: canvas_edge[0] * 10 × canvas_edge[1] * 10 pixels
  - 座標系:
    - グローバルメートル座標: 左下端が原点, 右=X正, 上=Y正
    - 画像ピクセル座標: 左上端が原点, 右=X正, 下=Y正
    - 変換: px = x / resolution, py = -y / resolution + canvasH_px
"""
import logging
from typing import Tuple

import cv2
import numpy as np

from src.map_extraction.nuscenes.converters.geometry_builder import GeometryBuilder

logger = logging.getLogger(__name__)

# nuScenes basemapの解像度
RESOLUTION = 0.1  # m/px (= 10 px/m)


def generate_basemap(
    geometry_builder: GeometryBuilder,
    layer_builder,
    output_path: str,
    closing_kernel_size: int = 9,
) -> None:
    """
    nuScenes形式のベースマップPNG（白黒）を生成する。
    白(255) = 走行可能領域 (drivable_area, road_segment, lane, ped_crossing, walkway等)
    黒(0) = 背景
    """
    canvas_edge = geometry_builder.get_canvas_edge()
    width_m, height_m = canvas_edge

    width_px = int(round(width_m / RESOLUTION))
    height_px = int(round(height_m / RESOLUTION))

    logger.info(f"Generating basemap: {width_px}x{height_px} pixels "
                f"({width_m:.1f}x{height_m:.1f}m, resolution={RESOLUTION} m/px)")

    # 黒背景の画像を作成（グレースケール）
    image = np.zeros((height_px, width_px), dtype=np.uint8)

    # ノードのトークン→座標マッピング
    node_map = {n['token']: (n['x'], n['y']) for n in geometry_builder.nodes}

    def to_pixel(x_m: float, y_m: float) -> Tuple[int, int]:
        """
        nuScenesグローバルメートル座標 → 画像ピクセル座標
        devkitの MapMask.to_pixel_coords() と同一の変換式:
          px = x / resolution
          py = -y / resolution + canvasH_px
        """
        px = int(round(x_m / RESOLUTION))
        py = int(round(-y_m / RESOLUTION)) + height_px
        # クリッピング
        px = max(0, min(width_px - 1, px))
        py = max(0, min(height_px - 1, py))
        return (px, py)

    def get_polygon_pixels(polygon_token: str) -> np.ndarray:
        """ポリゴントークンからピクセル座標配列を取得"""
        for p in geometry_builder.polygons:
            if p['token'] == polygon_token:
                points = []
                for nt in p['exterior_node_tokens']:
                    if nt in node_map:
                        x, y = node_map[nt]
                        points.append(to_pixel(x, y))
                return np.array(points, dtype=np.int32) if points else np.array([])
        return np.array([])

    # 白(255)で塗りつぶすポリゴンレイヤー
    polygon_layers = [
        ('drivable_area', layer_builder.drivable_area),
        ('road_segment', layer_builder.road_segment),
        ('walkway', layer_builder.walkway),
        ('ped_crossing', layer_builder.ped_crossing),
        ('carpark_area', layer_builder.carpark_area),
    ]

    for layer_name, records in polygon_layers:
        for record in records:
            if 'polygon_token' in record:
                pts = get_polygon_pixels(record['polygon_token'])
                if len(pts) > 2:
                    cv2.fillPoly(image, [pts], 255)
            elif 'polygon_tokens' in record:
                for pt in record['polygon_tokens']:
                    pts = get_polygon_pixels(pt)
                    if len(pts) > 2:
                        cv2.fillPoly(image, [pts], 255)

    nonzero = np.count_nonzero(image)
    total = image.shape[0] * image.shape[1]
    logger.info(f"Basemap saved to {output_path}")
    logger.info(f"  White pixels: {nonzero} ({100*nonzero/total:.1f}% of total)")

    # クロージング処理で小さな穴を埋める
    kernel = np.ones((closing_kernel_size, closing_kernel_size), dtype=np.uint8)
    closed_image = cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel)

    return closed_image, canvas_edge
