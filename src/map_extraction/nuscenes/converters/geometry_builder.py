"""
Geometry Builder: CARLAから抽出したデータをnuScenes形式の幾何プリミティブ（node, line, polygon）に変換する
"""
import logging
import uuid
from typing import List, Tuple, Dict, Optional

from shapely.geometry import Polygon

from src.map_extraction.utils.geom import get_lane_polygon_points

logger = logging.getLogger(__name__)


def generate_token() -> str:
    """nuScenes形式のUUIDトークンを生成"""
    return str(uuid.uuid4())


class CoordinateTransformer:
    """
    CARLAの座標系からnuScenes Map座標系への変換。

    CARLA (Unreal Engine, left-handed):
      - X: Forward → 鳥瞰図では右方向 (East)
      - Y: Right   → 鳥瞰図では下方向 (South)
      - Z: Up
      - 単位: メートル
      - yaw=0°で+X方向、yaw=90°で+Y方向

    nuScenes Map (right-handed):
      - X: 右方向 (East)
      - Y: 上方向 (North)
      - 単位: メートル
      - 原点: マップ左下端（全座標が正になるようにオフセット）

    変換: nuScenes_x = CARLA_x + offset_x
          nuScenes_y = -CARLA_y + offset_y
    """

    def __init__(self):
        self.x_offset = 0.0
        self.y_offset = 0.0

    def transform(self, carla_x: float, carla_y: float) -> Tuple[float, float]:
        """CARLA座標をnuScenes座標に変換"""
        ns_x = carla_x + self.x_offset
        ns_y = -carla_y + self.y_offset
        return (ns_x, ns_y)

    def compute_offsets(self, all_carla_points: List[Tuple[float, float]], margin: float = 50.0):
        """
        全座標点から、全てが正の値になるようにオフセットを計算する。
        :param all_carla_points: CARLA座標系の全点
        :param margin: マップ端のマージン（メートル）
        """
        if not all_carla_points:
            return

        xs = [p[0] for p in all_carla_points]
        ys = [-p[1] for p in all_carla_points]  # Y反転後

        self.x_offset = -min(xs) + margin
        self.y_offset = -min(ys) + margin

        logger.info(f"Coordinate offsets: x_offset={self.x_offset:.1f}, y_offset={self.y_offset:.1f}")


class GeometryBuilder:
    """nuScenes形式の幾何プリミティブを構築するビルダー"""

    def __init__(self, transformer: CoordinateTransformer):
        self.transformer = transformer
        self.nodes: List[dict] = []
        self.lines: List[dict] = []
        self.polygons: List[dict] = []
        # 座標からnodeトークンへの逆引き（重複ノード防止）
        self._coord_to_token: Dict[Tuple[float, float], str] = {}
        # ノードtoken→座標の逆引き
        self._token_to_coord: Dict[str, Tuple[float, float]] = {}
        # polygon_tokenからexterior_node_tokens, holesへの逆引き
        self._polygon_token_to_nodes: Dict[str, dict] = {}

    def add_node(self, carla_x: float, carla_y: float) -> str:
        """
        ノードを追加し、トークンを返す。同じ座標のノードが既にあれば既存トークンを返す。
        :return: ノードのトークン
        """
        ns_x, ns_y = self.transformer.transform(carla_x, carla_y)
        # 座標を丸めて重複判定（0.01m精度）
        rounded = (round(ns_x, 2), round(ns_y, 2))

        if rounded in self._coord_to_token:
            return self._coord_to_token[rounded]

        token = generate_token()
        self.nodes.append({
            'token': token,
            'x': ns_x,
            'y': ns_y,
        })
        self._coord_to_token[rounded] = token
        self._token_to_coord[token] = (ns_x, ns_y)
        return token

    def add_line(self, carla_points: List[Tuple[float, float]]) -> str:
        """
        ライン（ポリライン）を追加し、トークンを返す。
        :param carla_points: CARLA座標系の点列
        :return: ラインのトークン
        """
        node_tokens = [self.add_node(x, y) for x, y in carla_points]
        token = generate_token()
        self.lines.append({
            'token': token,
            'node_tokens': node_tokens,
        })
        return token

    def add_polygon(self, carla_points: List[Tuple[float, float]],
                    holes: Optional[List[List[Tuple[float, float]]]] = None) -> str:
        """
        ポリゴンを追加し、トークンを返す。
        :param carla_points: CARLA座標系の外周点列
        :param holes: 穴のリスト（各穴は点列）
        :return: ポリゴンのトークン
        """
        # TODO: Douglas-Peucker法などで点数削減しても良いかも（現状は全点をノード化している）
        exterior_node_tokens = [self.add_node(x, y) for x, y in carla_points]

        hole_tokens_list = []
        if holes:
            for hole in holes:
                hole_tokens = [self.add_node(x, y) for x, y in hole]
                hole_tokens_list.append({'node_tokens': hole_tokens})

        token = generate_token()
        self.polygons.append({
            'token': token,
            'exterior_node_tokens': exterior_node_tokens,
            'holes': hole_tokens_list,
        })
        self._polygon_token_to_nodes[token] = {
            'exterior': exterior_node_tokens,
            'holes': hole_tokens_list,
        }
        return token

    def build_lane_polygon(self, boundary_points) -> str:
        """
        レーン境界点からポリゴンを構築する。
        左側境界点列を順方向、右側境界点列を逆方向に並べて閉じたポリゴンにする。
        :param boundary_points: LaneBoundaryPoint のリスト
        :return: ポリゴンのトークン
        """
        if not boundary_points:
            return ""

        polygon_points = get_lane_polygon_points(boundary_points)

        return self.add_polygon(polygon_points)

    def get_canvas_edge(self) -> Tuple[float, float]:
        """
        全ノードの範囲からcanvas_edge [width_m, height_m] を計算する。
        nuScenes形式: canvas_edge = [X方向の幅(m), Y方向の高さ(m)]
        """
        if not self.nodes:
            return (0, 0)

        xs = [n['x'] for n in self.nodes]
        ys = [n['y'] for n in self.nodes]
        # 座標の最大値がcanvas_edgeとなる（原点=0,0から最大値まで）
        width = max(xs)
        height = max(ys)
        return (width, height)

    def get_polygon_from_token(self, polygon_token: str) -> Optional[List[Tuple[float, float]]]:
        """
        ポリゴントークンからShapely形式のPolygonを返す。
        :param polygon_token: ポリゴンのトークン
        :return: ShapelyのPolygonオブジェクト（取得できない場合はNone）
        """
        if polygon_token not in self._polygon_token_to_nodes:
            return None

        node_info = self._polygon_token_to_nodes[polygon_token]
        exterior_coords = [self._token_to_coord[token] for token in node_info['exterior']]
        holes_coords = []
        for hole in node_info['holes']:
            hole_coords = [self._token_to_coord[token] for token in hole['node_tokens']]
            holes_coords.append(hole_coords)

        return Polygon(exterior_coords, holes=holes_coords)
