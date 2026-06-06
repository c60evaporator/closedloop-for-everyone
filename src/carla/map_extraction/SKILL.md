---
name: drive-nuscenes-from-carla-map
description: "CARLAシミュレーターからnuScenes Map Expansion形式のHDマップを生成するフローの説明。Use when user modifies or extends the CARLA to nuScenes map conversion pipeline."
allowed-tools: ["Read", "Write", "Edit", "Bash", "Grep", "Glob"]
argument-hint: "<entity-name>"
user-invocable: false
---

## 概要

CARLAシミュレーターに接続し、マップデータを抽出してnuScenes Map Expansion形式（basemap PNG + expansion JSON）として出力する。処理は4フェーズで構成される。

## 全体フロー

```
Phase 1: CARLA接続
  carla_client.py
    └─ carla.Client → carla.Map 取得

Phase 2: データ抽出 (extractors/)
  waypoint_extractor.py   ← generate_waypoints()
    └─ LaneInfo (centerline, boundary_points, marking_type)
  topology_extractor.py   ← get_topology(), get_junction()
    └─ TopologyEdge, JunctionInfo, lane_connectivity
  landmark_extractor.py   ← get_crosswalks(), get_all_landmarks(), 隣接レーン探索
    └─ CrosswalkPolygon, TrafficLightInfo, StopSignInfo, SidewalkSegment
  lane_marking_extractor.py  ← waypointの境界点から算出
    └─ DividerLine (road_divider / lane_divider)

Phase 3: nuScenes形式への変換 (converters/)
  CoordinateTransformer   CARLA座標 → nuScenes座標のオフセット算出
  GeometryBuilder         node / line / polygon プリミティブ生成
  NuScenesLayerBuilder    各レイヤーの構築
  ConnectivityBuilder     lane_connector / arcline_path_3 / connectivity 生成

Phase 4: 出力 (output/)
  basemap_generator.py    basemap PNG（白黒、0.1 m/px）
  json_writer.py          expansion JSON（全レイヤー統合）
```

## Phase 1: CARLA接続

`src/carla_client.py`

```python
client = carla.Client(host, port)    # CARLAサーバーに接続
world = client.get_world()           # ワールド取得（--map-name指定時はload_world()後）
carla_map = world.get_map()          # carla.Map オブジェクト取得
```

`carla.Map` が後続の全Extractorへの入力となる。

## Phase 2: データ抽出

### waypoint_extractor — レーン情報の抽出

`src/extractors/waypoint_extractor.py`

**CARLA API**: `carla_map.generate_waypoints(sampling_resolution)`

全Drivingレーンのウェイポイントを等間隔でサンプリングし、`(road_id, section_id, lane_id)` をキーとした `LaneInfo` の辞書を返す。

```
LaneInfo
├─ road_id, section_id, lane_id     レーン識別キー
├─ lane_type                        "Driving" / "Sidewalk" / ...
├─ is_junction, junction_id         ジャンクション所属情報
├─ centerline: [(x, y), ...]        中心線のCARLA座標列
├─ boundary_points: [LaneBoundaryPoint, ...]
│   └─ left_x, left_y, right_x, right_y   ウェイポイントのyawと幅から算出
├─ lane_width                       レーン幅 (m)
└─ left_marking_type, right_marking_type   "Solid", "Broken", "SolidSolid" 等
```

**boundary_pointsの算出方法**: ウェイポイントのtransform.rotation.yawから進行方向の右方向ベクトルを算出し、lane_width/2でオフセットして左右の境界点を計算。

### topology_extractor — トポロジーとジャンクション

`src/extractors/topology_extractor.py`

**CARLA API**: `carla_map.get_topology()`, `carla_map.generate_waypoints()`, `wp.get_junction()`

1. **extract_topology()**: `get_topology()` から全レーン接続エッジ (`TopologyEdge`) を抽出
2. **extract_junctions()**: ジャンクション情報 (`JunctionInfo`) を抽出。`junction.get_waypoints(Driving)` で各ジャンクション内のレーンペア（入口→出口）を取得
3. **build_lane_connectivity()**: トポロジーエッジから各レーンの predecessor / successor 関係を構築

```
TopologyEdge: (start_road_id, start_section_id, start_lane_id) → (end_...)
JunctionInfo: junction_id, bounding_box, lane_pairs: [(入口key, 出口key), ...]
lane_connectivity: {lane_key: {predecessors: [...], successors: [...]}}
```

### landmark_extractor — ランドマーク抽出

`src/extractors/landmark_extractor.py`

| 関数 | CARLA API | 出力 |
|---|---|---|
| `extract_crosswalks()` | `carla_map.get_crosswalks()` | `CrosswalkPolygon` (頂点リスト) |
| `extract_traffic_lights()` | `carla_map.get_all_landmarks()` (is_dynamic) | `TrafficLightInfo` (位置, road_id, orientation) |
| `extract_stop_signs()` | `carla_map.get_all_landmarks()` (type=206) | `StopSignInfo` (位置, road_id) |
| `extract_sidewalks_from_opendrive()` | `generate_waypoints()` → 隣接レーン探索 | `SidewalkSegment` (中心線, 幅) |

**横断歩道の抽出**: `get_crosswalks()` は `carla.Location` のフラットリストを返す。最初の頂点が最後に繰り返されることでポリゴンの区切りを示す。

**歩道の抽出**: CARLA APIはSidewalk用の直接的なAPIを持たないため、各Drivingレーンの隣接レーンを左右に最大5レーン辿り、`lane_type == Sidewalk` のウェイポイントを収集する。

### lane_marking_extractor — レーンマーキング

`src/extractors/lane_marking_extractor.py`

waypoint_extractorの結果から、隣接レーン間の境界線を抽出。

1. **extract_lane_dividers()**: 同一road内で `lane_id` が ±1 の隣接レーン間の境界。マーキングタイプ（Solid, Broken等）で `road_divider` / `lane_divider` に分類
2. **extract_road_center_dividers()**: `lane_id=1` と `lane_id=-1` の間の中央線（対向車線境界）

```
DividerLine
├─ divider_type    "road_divider" / "lane_divider"
├─ marking_type    "Solid", "Broken", "SolidSolid" 等
├─ points          CARLA座標の境界点列
└─ lane_key_left, lane_key_right   境界の両側のレーンキー
```

## Phase 3: nuScenes形式への変換

### 座標変換

`src/converters/geometry_builder.py` — `CoordinateTransformer`

```
CARLA座標系 (Unreal Engine, 左手系)     nuScenes Map座標系 (右手系)
  X: 前方 → 鳥瞰図では右 (East)         X: 右方向 (East)
  Y: 右  → 鳥瞰図では下 (South)         Y: 上方向 (North)
  Z: 上                                  原点: マップ左下端（全座標が正）

変換式:
  nuScenes_x =  CARLA_x + offset_x
  nuScenes_y = -CARLA_y + offset_y
```

`compute_offsets()` で全点の最小値からオフセットを算出し、全座標が正になるようにする（margin=50m）。

### 幾何プリミティブ構築

`src/converters/geometry_builder.py` — `GeometryBuilder`

| メソッド | 生成するプリミティブ | 用途 |
|---|---|---|
| `add_node(x, y)` | node (token, x, y) | 座標点。0.01m精度で重複排除 |
| `add_line(points)` | line (token, node_tokens) | 折れ線。road_divider, lane_divider, edge_line |
| `add_polygon(points, holes)` | polygon (token, exterior_node_tokens, holes) | 閉多角形。lane, road_segment等 |
| `build_lane_polygon(boundary_points)` | polygon | 左境界(順)+右境界(逆)で閉じたポリゴン |

`polygon.holes` は `[{"node_tokens": [token, ...]}, ...]` 形式（Map Expansion仕様準拠）。

### レイヤー構築

`src/converters/layer_builder.py` — `NuScenesLayerBuilder`

構築順序と、各メソッドが生成するnuScenesレイヤーの対応:

```
1. build_lanes()           → lane レコード
   - Drivingレーンのみ対象
   - boundary_pointsからpolygon生成
   - 先頭/末尾のboundary_pointからfrom/to_edge_line生成
   - predecessor/successorは内部データとして保持（_predecessors, _successors）

2. build_road_segments()   → road_segment レコード
   - 非ジャンクション: road_id毎にレーンをShapelyで統合 (unary_union)
   - ジャンクション: junction_id毎に統合、is_intersection=True
   - Shapelyのexterior.coordsから閉頂点を除去、interiorsをholes化

3. build_drivable_area()   → drivable_area レコード
   - 全road_segmentのpolygon_tokensを集約
   - 生成したdrivable_area_tokenを全road_segmentに逆設定

4. build_ped_crossings()   → ped_crossing レコード
5. build_walkways()        → walkway レコード（中心線+幅から矩形ポリゴン生成）
6. build_dividers()        → road_divider / lane_divider レコード
   - road_divider: road_segment_token (空文字)
   - lane_divider: lane_divider_segments (marking_typeから変換)

7. build_stop_lines()      → stop_line レコード
   - 横断歩道の一辺から0.3m幅の薄い矩形ポリゴンを生成

8. build_traffic_lights()  → traffic_light レコード
   - traffic_light_type: "VERTICAL"
   - pose: CARLA座標から変換した {tx, ty, tz, rx, ry, rz}

9. resolve_lane_connectivity()
   - predecessor/successorをトークンに解決
   - _lane_connectivity辞書に格納（connectivity構築用）
   - laneレコードから内部フィールドを除去
```

**マーキングタイプの変換** (CARLA → nuScenes):

| CARLA | nuScenes segment_type |
|---|---|
| Solid | SOLID_WHITE |
| Broken | DASHED_WHITE |
| SolidSolid | DOUBLE_SOLID_WHITE |
| BrokenBroken | DOUBLE_DASHED_WHITE |
| BrokenSolid | DASHED_SOLID_WHITE |
| SolidBroken | SOLID_DASHED_WHITE |
| Other, NONE, Curb | NIL |

### 接続性構築

`src/converters/connectivity_builder.py`

#### build_lane_connectors()

ジャンクション内のレーンペアごとに `lane_connector` を生成。3つの値を返す:
- **connector_records**: spec準拠のレコード (`token`, `polygon_token` のみ)
- **connector_connectivity**: `{token: {incoming: [...], outgoing: [...]}}` — connectivity構築用
- **connector_centerlines**: `{token: [(x,y), ...]}` — arcline_path_3構築用

#### build_arcline_paths()

レーンとコネクタの中心線からDubins path形式の `arcline_path_3` を生成。現在は直線近似:

```python
segment = {
    'start_pose': [x0, y0, heading],
    'end_pose':   [x1, y1, heading],
    'shape': 'LSL',
    'radius': 999999.0,              # 直線 ≒ 無限大半径
    'segment_length': [0.0, L, 0.0], # [左旋回=0, 直進=L, 左旋回=0]
}
```

#### build_connectivity_dict()

`_lane_connectivity` と `connector_connectivity` を統合して `connectivity` 辞書を生成:
```python
{token: {"incoming": [token, ...], "outgoing": [token, ...]}}
```

## Phase 4: 出力

### basemap PNG

`src/output/basemap_generator.py`

- **画像仕様**: グレースケール (mode 'L')、2値: 0=黒(背景), 255=白(走行可能領域)
- **解像度**: 0.1 m/px (= 10 px/m)
- **座標変換**: `px = x / 0.1`, `py = -y / 0.1 + height_px`
- **白で塗りつぶすレイヤー**: drivable_area, road_segment, walkway, ped_crossing, carpark_area

### expansion JSON

`src/output/json_writer.py`

全レイヤーを統合してMap Expansion形式のJSONとして出力。出力フォーマットの詳細は `drive-nuscenes-mapexpansion` スキルを参照。

## ファイル構成

```
src/
├─ main.py                          エントリポイント (4フェーズの制御)
├─ carla_client.py                  CARLA接続
├─ extractors/
│   ├─ waypoint_extractor.py        レーン情報抽出
│   ├─ topology_extractor.py        トポロジー・ジャンクション抽出
│   ├─ landmark_extractor.py        横断歩道・信号機・歩道抽出
│   └─ lane_marking_extractor.py    レーンマーキング抽出
├─ converters/
│   ├─ geometry_builder.py          座標変換・node/line/polygon生成
│   ├─ layer_builder.py             nuScenesレイヤー構築
│   └─ connectivity_builder.py      lane_connector・arcline・connectivity
└─ output/
    ├─ basemap_generator.py         basemap PNG生成
    └─ json_writer.py               expansion JSON出力
```

## 出力先フォルダ構成

```
{output_dir}/{dataset_name}/nuscenes/
└─ maps/
    ├─ basemap_{map_name}.png       ベースマップ画像
    └─ expansion/
        └─ {map_name}.json          Map Expansionメタデータ
```

## 現在の制限事項

- **road_block**: 未実装（空リスト）。road_segmentの上下車線分離が必要
- **carpark_area**: 未実装（CARLAにパーキングエリアの概念がない）
- **lane edge lines**: boundary_pointsの先頭/末尾から生成。road_blockのedge lineとは別物
- **arcline_path_3**: 中心線の直線近似のみ。実際の曲率は反映されていない
- **空間的な紐付け**: ped_crossing.road_segment_token, road_divider.road_segment_token 等は空文字（空間マッチング未実装）
- **stop_line**: 横断歩道に基づく停止線のみ生成。信号機・停止標識に基づく停止線は未実装
- **traffic_light.items**: 空リスト（赤青黄の個別アイテム情報未実装）
