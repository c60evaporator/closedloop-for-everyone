
## インストール

### CARLA Garageリポジトリのクローン

まずcarla_garageリポジトリを以下コマンドでCloneします

```bash
git clone https://github.com/autonomousvision/carla_garage.git
```

## デバッグ動作確認

Webアプリを立ち上げずに、各種スクリプトが動作することを確認する方法を解説します。

### uv環境の構築

以下コマンドでuv環境を初期化します。

```bash
uv init -p 3.10.15
```

以下コマンドでCarla Garageの必要ライブラリをインストールします

```bash
uv add -r carla_garage/team_code/requirements.txt
```

以下コマンドで本リポジトリの必要ラリブラリをインストールします

```bash
uv add -r src/requirements.txt
```

以下コマンドでCARLA、leaderboard, scenario_runnerをパッケージとして使用できるようパスを通します

```bash
echo "<carlaルートフォルダの絶対パス>/PythonAPI/carla/" >> .venv/lib/python3.10/site-packages/carla.pth
echo "<プロジェクトのルートフォルダの絶対パス>/carla_garage/leaderboard/" >> .venv/lib/python3.10/site-packages/leaderboard.pth
echo "<プロジェクトのルートフォルダの絶対パス>/carla_garage/scenario_runner/" >> .venv/lib/python3.10/site-packages/leaderboard.pth
```

以下のようにCARLAのルートフォルダを`CARLA_ROOT`という環境変数に登録しておくと便利です

```bash
echo "export CARLA_ROOT=<carlaルートフォルダの絶対パス>" >> ~/.bashrc
```

### 基本操作

マップ一覧表示

```bash
uv run python -c "import carla; c=carla.Client('localhost', 2000); c.set_timeout(10.0); print('\n'.join(c.get_available_maps()))"
```

マップ変更

```bash
uv run $CARLA_ROOT/PythonAPI/util/config.py --map <マップ名>
```

### マップアノテーション抽出

以下コマンドでCARLAマップからnuScenes map expansion形式のアノテーションファイルを作成

```bash
uv run tools/carla_map_to_nuscenes.py --map-name <マップ名> 
```

`notebooks/viz_map_expansion.ipynb`で抽出したアノテーションファイルを可視化可能。
また`notebooks/viz_nusc_map_converter.ipynb`や`notebooks/viz_nusc_map_extractor.ipynb`でCARLAから動的にアノテーションを抽出（carla_map_to_nuscenes.pyと同じメソッドを使用）して可視化することもできる。

### データ収集

データ収集は、以下の2通りの方法があります。

1. ファイル保存：取得したセンサデータやアノテーション情報をファイルとして保存する（気軽にモデル学習を行いたいケースでおすめ）
2. ROS2 Topic出力：取得したセンサデータやアノテーション情報をROS2 Topicとして出力し、別途準備した受信システムで保存する（実写でROS2を使用している場合にエコシステムを統一したいケースで有効。[shasou-recorder]()との併用がおすすめ）

#### 1. ファイル保存する場合

##### CARLAの起動

まず以下コマンドでCALRAを起動します（マルチGPU実行したい場合、適宜`.env`の`EVAL_GPUS`を指定してください）

```bash
bash tools/launch_carla_servers.sh
```

##### データ収集開始

**別ターミナル**で以下コマンドでDockerコンテナに入り

```bash
docker exec -it carlagarage_dev bash
```

以下コマンドでデータ収集を実行できます

```bash
bash tools/collect_dataset_multi.sh <ルート定義ファイル格納フォルダ> <エージェント名>
```

例えばCARLA Garage公式の`carla_garage/data`フォルダを指定してPDM-Liteでnuscenes形式のデータ収集する場合、以下を実行します

```bash
bash tools/collect_dataset_multi.sh ${CARLA_GARAGE_ROOT}/data pdmlite_nuscenes
```

評価を途中から再開したい場合以下のように--resumeオプションを付けます

```bash
bash tools/collect_dataset_multi.sh ${CARLA_GARAGE_ROOT}/data pdmlite_nuscenes --resume
```

使用できるエージェント名は以下となります

|エージェント名|内容|
|---|---|
|pdmlite|CARLA Garage公式のPDM-Liteデータ収集エージェント|
|pdmlite_nuscenes|nuScenesのセンサ構成・座標系でデータ出力するPDM-Liteデータ収集エージェント|

#### 2. ROS2 Topic出力する場合

[shasou-recorder]()ライブラリを使ってROS2でデータ収集する方法を解説します。

##### ROS2とRviz2のインストール

まず必要パッケージをインストールします。

[こちら](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)を参考に、ROS2 Humble（`Humble`の部分はUbuntuのバージョンに合わせて適宜変えてください）のDesktop Installを実施します。

インストールが終わったら、以下コマンドでros2コマンドをスタートアップで有効にすると良いでしょう（`humble`の部分は適宜変えてください）

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

以下コマンドで今回のTopicのrviz表示に必要な拡張プラグインをインストールし、`Detection3DArray`を表示できるようにします

```bash
sudo apt install ros-humble-image-transport-plugins ros-humble-vision-msgs-rviz-plugins
```

以下でRvizが起動するか確認します

```bash
ros2 run rviz rviz
```

##### データ収集エージェントのrvizによる動作確認

以下でCARLAを起動し、CARLAのGUIが表示されることを確認します。

```bash
cd <CARLAのルートフォルダ>
bash ./CarlaUE4.sh
```

**別ターミナル**で以下コマンドを実行し、表示に必要な設定を適用したrvizを起動します（この時点では何もセンサデータが表示されていない状態でOKです）

```
ros2 launch tools/nuscenes/rviz_nuscenes.launch.py
```

さらに**別ターミナル**で以下コマンドでDockerコンテナに入り

```bash
docker exec -it carlagarage_dev bash
```

以下コマンドでデータ収集を実行します

```bash
bash tools/collect_dataset_multi.sh ${CARLA_GARAGE_ROOT}/data pdmlite_nuscenes_ros2
```

rviz2画面に各種センサデータが表示され、CARLAのGUIと表示されるデータの位置関係が合っていればOKです。具体的には以下項目をチェックすると良いでしょう

|rviz2のDisplaysでの項目|Topic型|Topic名|確認ポイント|
|---|---|---|---|
|TF|TF|`/tf_static` + `/tf` (自動)|`base_link`が地面に接しているか。`cam_front`(前方+x, 高さ1.51m)、`lidar_top`(高さ1.84m)等が正しい相対位置にあるか。gt_objectsの表示をオフにすると見やすい|
|lidar_top|PointCloud2|`/shasou/lidar_top/points`|地面の点がz≈0(map系)に来るか、建物・車両の点がgt_objectsのboxと重なるか|
|gt_ego_odom|Odometry|`/shasou/gt/ego_odom`|赤い矢印が自車位置で進行方向を向くか。Covariance表示はオフ推奨|
|agent_plan|Path|`/shasou/agent/plan`|自車前方に経路（緑色の線）が伸びるか|
|gt_objects|Detection3DArray|`/shasou/gt/objects`|周囲車両のboxが点群の車両クラスタと一致するか|

##### shasou-recorderのインストール

##### shasou-recorderと組み合わせたデータ収集

まず以下コマンドでCALRAを起動します。マルチGPU実行できない（トピックが混信する）ので、`.env`の`EVAL_GPUS`を1個のみ指定してください

```bash
bash tools/launch_carla_servers.sh
```

と起動
