# データ収集用ベースクラス

高性能エージェントPDM-Liteで自動運転しつつ**オリジナルのセンサ構成でデータ収集**するために、以下のデータ収集エージェント作成用ベースクラスを準備しています。

- [`GeneralizedDataAgent`]()：収集したセンサデータを**ファイル保存**するエージェントのベースクラス
- [`GeneralizedROS2DataAgent`]()：収集したセンサデータを**ROS2 Topicで出力**するエージェントのベースクラス

このクラスを継承した自作クラスを作成し、`_sensors`メソッド内にオリジナルのセンサ構成を記述することで、自動運転のプランニングにPDM-Liteを使用したオリジナルのデータ収集エージェントを作成できます。

## GeneralizedDataAgent

収集したセンサデータを**ファイル保存**するエージェントのベースクラスです

### 自作エージェント作成方法

以下の手順で自作エージェントを作成します

- 自作クラスファイルの作成：`GeneralizedDataAgent`を継承した自作クラスを`team_code/data_agents`フォルダ内に作成する
- クラス定数の記述：自作クラスの冒頭にクラス定数を適切に記述する
- センサ構成の記述：自作クラスの`_sensors`メソッド内にセンサ構成を記述する

#### 自作クラスファイルの作成

`GeneralizedDataAgent`を継承した自作クラスを`team_code/data_agents`フォルダ内に作成し、`COORDINATE_SYSTEM`等の各種クラス定数（[詳細は後述]()）と`_sensors`メソッド内のセンサ構成（[こちらも記載方法は後述]()）を記述します。

例えばnuScenesのセンサ構成でデータを出力するエージェント`data_agent_nuscenes.py`は以下のように作成できます。

```python
from generalized_data_agent import GeneralizedDataAgent

REAR_AXLE_TO_CENTER = 1.42  # Lincoln MKZ wheelbase (2.85 m) / 2

def get_entry_point():
    return 'DataAgentNuScenes'

class DataAgentNuScenes(GeneralizedDataAgent):
    """
    Child of GeneralizedDataAgent with a nuScenes-style 6 camera + LiDAR rig.
    """
    COORDINATE_SYSTEM = 'nuscenes'
    LIDAR_FORMAT = 'pcd_bin'

    def _sensors(self):
        return [{
            'type': 'sensor.camera.rgb',
            'x': 0.28, 'y': 0.0, 'z': 1.51,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            'width': 1600, 'height': 900, 'fov': 70,
            'id': 'CAM_FRONT'
        }, {
            'type': 'sensor.camera.rgb',
            'x': 0.15, 'y': -0.50, 'z': 1.52,
            'roll': 0.0, 'pitch': 0.0, 'yaw': -55.0,
            'width': 1600, 'height': 900, 'fov': 70,
            'id': 'CAM_FRONT_LEFT'
        }, {
            'type': 'sensor.camera.rgb',
            'x': 0.16, 'y': 0.50, 'z': 1.52,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 55.0,
            'width': 1600, 'height': 900, 'fov': 70,
            'id': 'CAM_FRONT_RIGHT'
        }, {
            'type': 'sensor.camera.rgb',
            'x': -1.37, 'y': 0.0, 'z': 1.57,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 180.0,
            'width': 1600, 'height': 900, 'fov': 110,
            'id': 'CAM_BACK'
        }, {
            'type': 'sensor.camera.rgb',
            'x': -0.38, 'y': -0.48, 'z': 1.56,
            'roll': 0.0, 'pitch': 0.0, 'yaw': -110.0,
            'width': 1600, 'height': 900, 'fov': 70,
            'id': 'CAM_BACK_LEFT'
        }, {
            'type': 'sensor.camera.rgb',
            'x': -0.36, 'y': 0.47, 'z': 1.61,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 110.0,
            'width': 1600, 'height': 900, 'fov': 70,
            'id': 'CAM_BACK_RIGHT'
        }, {
            # nuScenes LIDAR_TOP mounting position (translation [0.94, 0.0, 1.84] in the nuScenes ego frame).
            'type': 'sensor.lidar.ray_cast',
            'x': 0.94 - REAR_AXLE_TO_CENTER, 'y': 0.0, 'z': 1.84,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            'rotation_frequency': 20,
            'points_per_second': 695000,
            'channels': 32,
            'range': 70,
            'upper_fov': 10.67,
            'lower_fov': -30.67,
            'id': 'LIDAR_TOP'
        }]
```

実行時は、Dockerコンテナ内から以下コマンドを打つことで作成したエージェントを用いたデータ収集が始まります。

```bash
TEAM_AGENT=team_code/data_agents/<作成したAgentのファイル名> \
bash tools/collect_dataset_multi.sh ${CARLA_GARAGE_ROOT}/data
```

例えば上のDataAgentNuScenesを使用する場合、以下を実行します。

```bash
TEAM_AGENT=team_code/data_agents/data_agent_nuscenes.py \
bash tools/collect_dataset_multi.sh ${CARLA_GARAGE_ROOT}/data
```

#### クラス定数の記述

自作クラス内で以下のクラス定数を定義することで、保存データの種類やフォーマットを変更することができます（重要な定数は太字で記載）。

|定数名|必須|型|内容|
|---|---|---|---|
|**`REAR_AXLE_TO_CENTER`**|☑️|float|車両中心から後車軸までの距離（単位メートル）。Leaderboardのデフォルト車両Lincoln MKZを使用する場合は`1.42`を指定|
|**`SAVE_BEV_SEMANTICS`**||bool|自車中心BEV形式のセマンティックマップを保存するか選択（TransFuser++等で使用）。これを保存しない場合、UniAD等のマップ情報を使用するモデルではnuScenes map expansion形式のようなセマンティックマップを別途準備する必要がある|
|`BEV_RESOLUTION_WIDTH`||int|`SAVE_BEV_SEMANTICS`有効時のBEVラスタ幅 (pixels)。学習したいモデルと解像度を合わせる必要あり|
|`BEV_RESOLUTION_HEIGHT`||int|`SAVE_BEV_SEMANTICS`有効時のBEVラスタ高さ (pixels)。学習したいモデルと解像度を合わせる必要あり|
|**`LIDAR_FORMAT`**||str ("laz" | "pcd_bin")|LiDARデータの保存形式。`.laz `（CARLA Garage形式）と`.pcd.bin`（nuScenes形式）が選択可能|
|`LAZ_POINT_FORMAT`||int|`LIDAR_FORMAT='laz'`のときに使用するパラメータ|
|`LAZ_POINT_PRECISION`||float|`LIDAR_FORMAT='laz'`のときに使用するパラメータ|
|**`COORDINATE_SYSTEM`**||str ("carla" | "nuscenes")|保存するデータの座標系をCARLA形式かnuScenes形式かを選択する|


特に重要なクラス変数に`COORDINATE_SYSTEM`ついて以下で詳細を解説します。

##### `COORDINATE_SYSTEM`変数（保存データの座標系の変更）

クラス属性`COORDINATE_SYSTEM`の値を変えることで、以下のように出力される座標系を変更することができます。

- `COORDINATE_SYSTEM="carla"`: CALRAの座標系を使用
- `COORDINATE_SYSTEM="nuscenes"`: nuScenesの座標系を使用

※注意：後述する`measurements`**フォルダ内のメタデータは**`COORDINATE_SYSTEM="nuscenes"`**であってもCARLA座標系で保存されます**

両者は以下のような違いがあります。

||CARLA|nuScenes|
|---|---|---|
|グローバル座標の原点|マップ定義による|basemapの左下端|
|グローバル座標の方向|x:前方, y:右方, z:上方（左手系）|x:前方（basemapの右方）, y:左方（basemapの上方）, z:上方（右手系、一般的な数学でのxyz座標に近い）|
|ローカル座標の原点|後輪の車軸中心|後輪の車軸中心|
|ローカル座標の方向|x:前方, y:右方, z:上方（左手系）|x:前方, y:右方, z:上方（右手系）|

なお、`COORDINATE_SYSTEM="nuscenes"`を指定した場合、グローバル座標の原点はCARLAの原点をそのまま使用します（basemapの作り方によりnuScenesの原点が変わるため）。実際のnuScenes形式データを作成する場合は、**以下のグローバル座標**については**nuScenesの原点とCARLAの原点の差分をオフセットとして差し引く**必要があります。

|保存されるグローバル座標|保存されるファイル|nuScenesでの対応ファイル|
|---|---|---|
|自車位置|`measurements/****.json.gz`の"pos_global"|`ego_pose.json`|
|アノテーション位置|"matrix"||

#### センサ構成の記述

センサ構成は、`_sensor()`メソッドにdict形式のリストとして記述していきます。例えば以下記述ではカメラとLiDARが1個ずつ設置されます。

```python
    def _sensors(self):
        return [{
            'type': 'sensor.camera.rgb',
            'x': 0.28, 'y': 0.0, 'z': 1.51,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            'width': 1600, 'height': 900, 'fov': 70,
            'id': 'CAM_FRONT'
        }, {
            'type': 'sensor.lidar.ray_cast',
            'x': 0.94 - REAR_AXLE_TO_CENTER, 'y': 0.0, 'z': 1.84,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            'rotation_frequency': 20,
            'points_per_second': 695000,
            'channels': 32,
            'range': 70,
            'upper_fov': 10.67,
            'lower_fov': -30.67,
            'id': 'LIDAR_TOP'
        }]
```

センサの種類は以下から選ぶことができます。

|センサの種類|`_sensor()`メソッドに記載する`type`|
|---|---|
|RGBカメラ|`sensor.camera.rgb`|
|LiDAR|`sensor.lidar.ray_cast`|
|RADAR|`sensor.other.radar`|
|Depthカメラ|`sensor.camera.depth`|
|GNSS|`sensor.other.gnss`|

全センサ共通で記述すべき必須パラメータは以下となります。

|キー|意味|
|---|---|
|type|センサ種別|
|id|センサID。データ保存フォルダ名(例: LIDAR_TOP/)やsensor_calibration.json のキーになる|
|x, y, z|車両基準の取付位置 [m](CARLA座標系: x前方、y右、z上、原点は車両バウンディングボックス中心)|
|roll, pitch, yaw|取付姿勢 [deg]|

各センサごとに指定できるパラメータを以下に記載します

##### RGBカメラ (sensor.camera.rgb)

以下のキーはすべて必須です（省略すると`agent_wrapper_local.py`がKeyErrorを送出します）。

|キー|意味|
|---|---|
|width|画像幅 [pixels]|
|height|画像高さ [pixels]|
|fov|水平視野角 [deg]|

この3キーからカメラ内部パラメータ行列が計算され、`sensor_calibration.json`に`intrinsic`として記録されます。それ以外のCARLAカメラ属性（gamma、モーションブラー等）は指定できません。

##### LiDAR (sensor.lidar.ray_cast)

以下のキーはすべて省略可能で、省略時は下表のデフォルト値が使用されます。

|キー|省略時のデフォルト|意味|
|---|---|---|
|rotation_frequency|10|回転周波数 [Hz]。シミュレーションが20Hzであることから、合成の都合上**20の約数を指定する必要**あり|
|points_per_second|600000|秒間点数|
|channels|64|ビーム(レーザー)本数|
|range|85|最大測定距離 [m]|
|upper_fov|10|上方視野角 [deg]|
|lower_fov|-30|下方視野角 [deg]|
|atmosphere_attenuation_rate|0.004|大気減衰係数(強度計算用)|
|dropoff_general_rate|0.45|ランダムに落とす点の割合|
|dropoff_intensity_limit|0.8|この強度以上の点はドロップ対象外|
|dropoff_zero_intensity|0.4|強度ゼロの点をドロップする確率|

なお、`rotation_frequency=20`の場合は取得した点群がそのまま保存されますが、それ以外の場合は以前に取得した点群と合成して360度点群が作成されます。例えば`rotation_frequency=5`の場合、1tick（1/20秒）では90度分の点群しか取得できないので、過去3tick分の点群も合成することで360度分の点群を作成し、これを`<id>/<frame>.laz`（または`<id>/<frame>.pcd.bin`）として保存します。

補足:

- `channels`や`rotation_frequency`を変更する場合、点群密度が実機相当になるよう`points_per_second`も合わせてスケールしてください（`points_per_second ≒ channels × 水平解像度 × rotation_frequency`）
- 指定したビームパラメータは取付位置とともに`sensor_calibration.json`のlidarエントリに記録されます
- `rotation_frequency`と`points_per_second`以外のキーの上書きは`tools/leaderboard_local/agent_wrapper_patches.py`のランタイムパッチで実現されており、`tools/collect_dataset_multi.sh`経由の実行では自動適用されます（CARLA Garage純正のevaluatorを直接実行した場合は無視されデフォルト値になるので注意）

##### RADAR (sensor.other.radar)

以下のキーはすべて必須です（省略すると`agent_wrapper_local.py`がKeyErrorを送出します）。

|キー|意味|
|---|---|
|horizontal_fov|水平視野角 [deg]|
|vertical_fov|垂直視野角 [deg]|

以下のパラメータは`agent_wrapper_local.py`にハードコードされており、現状specから変更できません（LiDARと異なりランタイムパッチ未対応）。

|パラメータ|固定値|意味|
|---|---|---|
|points_per_second|1500|秒間検出点数|
|range|100|最大測定距離 [m]|

##### Depthカメラ (sensor.camera.depth)

RGBカメラと同じく`width`・`height`・`fov`の3キーが必須です（wrapperは`sensor.camera.*`を共通処理するため）。深度は8bit正規化値のPNGとして保存されます。

##### GNSS (sensor.other.gnss)

追加で指定できるキーはありません。共通の必須パラメータは以下に注意してください。

- 取付姿勢（`roll`, `pitch`, `yaw`）はwrapperにより無視され、常にゼロとして扱われます（位置`x`, `y`, `z`のみ有効）
- 観測ノイズパラメータは`agent_wrapper_local.py`にハードコードされており変更できません（緯度/経度/高度の標準偏差 0.000005、バイアス 0.0）

### 保存データの形式

#### センサデータ

各センサデータは以下の形式で保存されます。

|センサの種類|`_sensor()`メソッドに記載する`type`|保存形式|
|---|---|---|
|RGBカメラ|`sensor.camera.rgb`|`<id>/<frame>.jpg`|
|LiDAR|`sensor.lidar.ray_cast`|`<id>/<frame>.laz`または`<id>/<frame>.pcd.bin` (クラス定数`LIDAR_FORMAT`で選択可)|
|RADAR|`sensor.other.radar`|`<id>/<frame>.npy`|
|Depthカメラ|`sensor.camera.depth`|`<id>/<frame>.png` (8bit normalized depth)|
|GNSS|`sensor.other.gnss`|`<id>/<frame>.json`|

#### その他の保存情報

`_sensors()`メソッドで指定したセンサデータ以外にも、以下の情報が記録されます

|センサの種類|保存フォルダと形式|内容|
|---|---|---|
|バウンディングボックス|`boxes/<frame>.json.gz`|周囲のアクターのアノテーション情報|
|メタデータ|`measurements/<frame>.json.gz`|自車の状態、ローカル座標系の経路、制御入力、ハザード判定|
|BEVセマンティックマップ|`boxes/<frame>.png`|自車中心BEV形式のセマンティックマップ（クラスIDをそのままモノクロ階調値で保存）。`SAVE_BEV_SEMANTICS=True`のときのみ保存される|

バウンディングボックスとメタデータについて詳細を記述します。

##### バウンディングボックス

1フレーム分のバウンディングボックス（アノテーション）情報が`boxes/<frame>.json.gz`に保存されます。
具体的には以下のフィールドが含まれています（これ以外にもclassに応じたフィールドが入ります）。

| キー | 型 | 意味 |
|---|---:|---|
| `class` | `string` | 物体種別。例: `ego_car`, `car`, `walker`, `static`, `traffic_light`, `stop_sign` |
| `extent` | `float[3]` | bounding box の半径サイズ `[x, y, z]`。CARLA の `bounding_box.extent` |
| `position` | `float[3]` | ego座標系での相対位置 `[x, y, z]` |
| `yaw` | `float` | ego基準の相対yaw角。ラジアン |
| `num_points` | `int` | LiDAR点群がそのbbox内に何点入っているか。egoは`-1` |
| `distance` | `number` | egoからの距離。ego自身は `-1` |
| `speed` | `number` | forward speed |
| `id` | `number` | CARLA actor ID |
| `matrix` | `number[4][4]` | actorのグローバル座標系での4x4 transform matrix |

##### メタデータ

1フレーム分のメタデータが`measurements/<frame>.json.gz`に保存され、自車の状態、ローカル座標系の経路、制御入力、ハザード判定が入っています。
具体的には以下のフィールドが含まれています。

| キー | 型 | このファイルの値/形 | 意味 |
|---|---:|---|---|
| `pos_global` | `number[2]` | `[6435.357..., 4245.204...]` | 自車のグローバル位置 `[x, y]` |
| `theta` | `number` | `-3.04887...` | 自車方位/compass |
| `speed` | `number` | `0.0910...` | 現在速度 |
| `target_speed` | `number` | `10` | 目標速度 |
| `speed_limit` | `number` | `13.888...` | 速度制限 |
| `target_point` | `number[2]` | `[183.704..., 74.183...]` | 自車ローカル座標系の目標点 |
| `target_point_next` | `number[2]` | `[187.413..., 75.897...]` | 次の目標点 |
| `command` | `number` | `4` | RoadOption系のコマンド。`4` は `LANEFOLLOW` |
| `next_command` | `number` | `4` | 次コマンド |
| `aim_wp` | `number[2]` | `[7.713..., 0.128...]` | expert/autopilotが狙う waypoint |
| `route` | `number[20][2]` | 20点 | 自車ローカル座標系の残り経路 |
| `route_original` | `number[20][2]` | 20点 | 元の経路。今回は `route` と同じ |
| `changed_route` | `boolean` | `false` | 経路が変更されたか |
| `steer` | `number` | `0.04` | ステアリング制御 |
| `throttle` | `number` | `1` | スロットル制御 |
| `brake` | `boolean` | `false` | ブレーキ判定 |
| `control_brake` | `boolean` | `false` | 制御上のブレーキ |
| `junction` | `boolean` | `false` | 交差点内/近傍判定 |
| `vehicle_hazard` | `boolean` | `false` | 車両ハザード |
| `light_hazard` | `boolean` | `false` | 赤信号などの信号ハザード |
| `walker_hazard` | `boolean` | `false` | 歩行者ハザード |
| `stop_sign_hazard` | `boolean` | `false` | 一時停止標識ハザード |
| `stop_sign_close` | `boolean` | `false` | 近くに一時停止標識があるか |
| `walker_close` | `boolean` | `false` | 近くに歩行者がいるか |
| `*_id` 系 | `number` or `null` | 今回は `null` | 影響している actor ID |
| `speed_reduced_by_obj_*` | `string/number` or `null` | 今回は全部 `null` | 速度低下要因オブジェクト |
| `angle` | `number` | `0.0106...` | aim point などから来る角度系の値 |
| `augmentation_translation` | `number` | `0` | データ拡張の平行移動量 |
| `augmentation_rotation` | `number` | `0` | データ拡張の回転量 |
| `ego_matrix` | `number[4][4]` | 4x4行列 | CARLAの自車transform行列 |

なお、**このメタデータは**`COORDINATE_SYSTEM="nuscenes"`**であってもCARLA座標系で保存されます**（PDM-Lite自身が走行制御に使う内部表現との整合性を取るため）

よって出力したデータをnuScese形式のjsonに変換したい場合、基本的には`measurements`ではなく`boxes`から情報を取ることが推奨されます（例：`ego_poses.json`は、`measurements`の`ego_matrix`ではなく`boxes`の1レコード目に記録された`ego_car`の`matrix`から変換する）

## GeneralizedROS2DataAgent

収集したセンサデータを**ROS2 Topicで出力**するエージェントのベースクラスです

### 自作エージェント作成方法

以下の手順で自作エージェントを作成します

- 自作クラスファイルの作成：`GeneralizedDataAgent`を継承した自作クラスを`team_code/data_agents`フォルダ内に作成する
- クラス定数の記述：自作クラスの冒頭にクラス定数を適切に記述する
- センサ構成の記述：自作クラスの`_sensors`メソッド内にセンサ構成を記述する

#### 自作クラスファイルの作成

`GeneralizedROS2DataAgent`を継承した自作クラスを`team_code/data_agents`フォルダ内に作成し、`TOPIC_NAMESPACE`等の各種クラス定数（[詳細は後述]()）と、`_sensors`メソッド内のセンサ構成（[GeneralizedDataAgentの記載方法]()と同様）を記述します。なお、出力されるTopicのフォーマットはROS2の定義に従うため、フォーマットを指定するためのクラス定数の記述は不要です。

例えばnuScenes形式でデータをTopic出力するエージェント`ros2_data_agent_nuscenes.py`は以下のように作成できます。

```python
from generalized_ros2_data_agent import GeneralizedROS2DataAgent

REAR_AXLE_TO_CENTER = 1.42  # Lincoln MKZ wheelbase (2.85 m) / 2

def get_entry_point():
    return 'ROS2DataAgentNuScenes'

class ROS2DataAgentNuScenes(GeneralizedROS2DataAgent):
    """
    Child of GeneralizedROS2DataAgent with a nuScenes-style 6 camera + LiDAR + GNSS rig.
    """
    TOPIC_NAMESPACE = '/nuscenes'

    def _sensors(self):
        return [{
            'type': 'sensor.camera.rgb',
            'x': 0.28, 'y': 0.0, 'z': 1.51,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            'width': 1600, 'height': 900, 'fov': 70,
            'id': 'cam_front'
        }, {
            'type': 'sensor.camera.rgb',
            'x': 0.15, 'y': -0.50, 'z': 1.52,
            'roll': 0.0, 'pitch': 0.0, 'yaw': -55.0,
            'width': 1600, 'height': 900, 'fov': 70,
            'id': 'cam_front_left'
        }, {
            'type': 'sensor.camera.rgb',
            'x': 0.16, 'y': 0.50, 'z': 1.52,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 55.0,
            'width': 1600, 'height': 900, 'fov': 70,
            'id': 'cam_front_right'
        }, {
            'type': 'sensor.camera.rgb',
            'x': -1.37, 'y': 0.0, 'z': 1.57,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 180.0,
            'width': 1600, 'height': 900, 'fov': 110,
            'id': 'cam_back'
        }, {
            'type': 'sensor.camera.rgb',
            'x': -0.38, 'y': -0.48, 'z': 1.56,
            'roll': 0.0, 'pitch': 0.0, 'yaw': -110.0,
            'width': 1600, 'height': 900, 'fov': 70,
            'id': 'cam_back_left'
        }, {
            'type': 'sensor.camera.rgb',
            'x': -0.36, 'y': 0.47, 'z': 1.61,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 110.0,
            'width': 1600, 'height': 900, 'fov': 70,
            'id': 'cam_back_right'
        }, {
            'type': 'sensor.lidar.ray_cast',
            'x': 0.94 - REAR_AXLE_TO_CENTER, 'y': 0.0, 'z': 1.84,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            'rotation_frequency': 20,
            'points_per_second': 695000,
            'channels': 32,
            'range': 70,
            'upper_fov': 10.67,
            'lower_fov': -30.67,
            'id': 'lidar_top'
        }, {
            # Global Planが必要な実車に合わせるためGNSSデータも出力
            'type': 'sensor.other.gnss',
            'x': 0.0, 'y': 0.0, 'z': 0.0,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            'id': 'gnss'
        }]
```

実行時は、Dockerコンテナ内から以下コマンドを打つことで作成したエージェントを用いたデータ収集が始まります。

```bash
TEAM_AGENT=team_code/data_agents/<作成したAgentのファイル名> \
bash tools/collect_dataset_multi.sh ${CARLA_GARAGE_ROOT}/data
```

例えば上のROS2DataAgentNuScenesを使用する場合、以下を実行します。

```bash
TEAM_AGENT=team_code/data_agents/ros2_data_agent_nuscenes.py \
bash tools/collect_dataset_multi.sh ${CARLA_GARAGE_ROOT}/data
```

#### クラス定数の記述

自作クラス内で以下のクラス定数を定義する必要があります。

|定数名|必須|型|内容|
|---|---|---|---|
|**`REAR_AXLE_TO_CENTER`**|☑️|float|車両中心から後車軸までの距離（単位メートル）。Leaderboardのデフォルト車両Lincoln MKZを使用する場合は`1.42`を指定|
|**`TOPIC_NAMESPACE`**|☑️|str|出力されるセンサデータ等のトピックの名前空間。例えば`TOPIC_NAMESPACE="/nuscenes"`とした場合、idが"lidar_top"のRGBカメラのトピック名は`/nuscenes/`|

#### センサ構成の記述

センサ構成は、[GeneralizedDataAgentと同様のフォーマット]()で、`_sensor()`メソッドにdict形式のリストとして記述します。

### 出力されるTopicの形式

#### センサデータ

各センサデータは以下の形式でTopic出力されます

|センサの種類|`_sensor()`メソッドに記載する`type`|Topic名|Topic型|frame_id|座標系|
|---|---|---|---|---|---|
|RGBカメラ（画像本体）|`sensor.camera.rgb`|`<namsepace>/<id>/image_raw/compressed`|sensor_msgs/CompressedImage|<id>|-|
|RGBカメラ（内部パラメータ）|`sensor.camera.rgb`|`<namsepace>/<id>/camera_info`|sensor_msgs/CameraInfo|<id>|-|
|LiDAR|`sensor.lidar.ray_cast`|`<namsepace>/<id>/points`|sensor_msgs/PointCloud2|<id>|センサ位置を原点とした相対座標|
|RADAR|`sensor.other.radar`|-（未対応）|-|-|-|
|Depthカメラ|`sensor.camera.depth`|-（未対応）|-|-|-|
|GNSS|`sensor.other.gnss`|`<namsepace>/<id>/fix`|sensor_msgs/NavSatFix|gnss|緯度経度|

※RADAR・Depthカメラ・セマンティックセグメンテーションカメラは`GeneralizedROS2DataAgent`では**現状未対応**です。`_sensors()`メソッドにこれらのセンサを含めると、`setup()`が`NotImplementedError`を送出します（ファイル保存の`GeneralizedDataAgent`では使用可能）。対応する場合の実装候補は、RADARが`sensor_msgs/PointCloud2`（xyz+速度チャンネル）または`radar_msgs/RadarScan`、Depthカメラが`sensor_msgs/Image`（32FC1、depth_image_procの規約）です。

LiDAR点群の出力内容について以下補足します

##### LiDAR点群の出力内容について

- LiDARは1スイープ=1メッセージとする。例えば`rotation_frequency=20`のときは毎tickトピックを出力し、`rotation_frequency=5`のときは4tick分の点群をバッファに蓄積して作成した360度点群を、4tickに1回トピック出力する
- ringはCARLAに直接ないので垂直角から算出して付与

#### センサデータ以外のTopic

センサデータ以外にも、Ground Truth等の情報がTopic出力されます。なお多くのTopicの座標系は、以下のフレームのどちらかが基準（frame_id）となっています。

- `map`: グローバル座標の原点。CALRAの原点と一致するが、右手系に変換済
- `base_link`: 自車位置の基準となるフレーム。後車軸の真下の地面と接する位置を原点とし、前方がx、左方がy、上方がz（右手系）

具体的には、以下のTopicが出力されます。

|Topic名|Topic型|frame_id|座標系|内容|
|---|---|---|---|---|
|`<namespace>/imu/data`|sensor_msgs/Imu|imu|IMUセンサ座標系（x:前方、y:左方、z:上方）|IMUの出力|
|`<namespace>/vehicle/drive_state`|ackermann_msgs/AckermannDriveStamped|base_link|-|speed（m/s、後退負）+ steering_angle（rad）|
|`<namespace>/vehicle/pedals`|sensor_msgs/JointState|""|-|ペダルストローク（正規化[0,1]）|
|`<namespace>/vehicle/reverse`|std_msgs/Bool|-|-|ギア後退|
|`<namespace>/vehicle/handbrake`|std_msgs/Bool|-|-|パーキングブレーキ|
|`<namespace>/gt/ego_odom`|nav_msgs/Odometry|map（base_linkがchild）|グローバル座標（ROS右手系）|自車位置（`/tf`をnav_msgs/msg/Odometry型で表したもの）|
|`<namespace>/gt/objects`|vision_msgs/Detection3DArray|map|グローバル座標（ROS右手系）|全アクターの3D BBox座標（アノテーション情報）|
|`<namespace>/agent/plan`|nav_msgs/Path|map|グローバル座標（ROS右手系）|PDM-Liteの計画軌跡|
|`/clock`|rosgraph_msgs/Clock|-|-|シミュレーション時刻|
|`/tf_static`|tf2_msgs/TFMessage|base_link（各センサがchild）|自車位置を原点とした相対座標|自車基準位置（base_link=後車軸の真下の地面）から各センサ位置までの相対座標|
|`/tf`|tf2_msgs/TFMessage|map（base_linkがchild）|グローバル座標（ROS右手系）|自車基準位置（base_link）のグローバル座標|
