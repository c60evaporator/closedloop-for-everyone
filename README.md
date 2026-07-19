
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

まず以下コマンドで

```bash
bash tools/launch_carla_servers.sh
```

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
|pdmlite_nuscenes_ros2|nuScenesのセンサ構成・座標系でros2 Topicを出力するPDM-Liteデータ収集エージェント（**シングルGPU**での実行のみ対応）|


#### ROS2によるデータ収集

[shasou-recorder]()ライブラリを使ってROS2でデータ収集する方法を解説します。

##### ROS2とRviz2のインストール

[こちら](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)を参考に、ROS2 Humble（`Humble`の部分はUbuntuのバージョンに合わせて適宜変えてください）のDesktop Installを実施します。

インストールが終わったら、以下コマンドでros2コマンドをスタートアップで有効にすると良いでしょう（`humble`の部分は適宜変えてください）

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

以下コマンドでRvizで`Detection3DArray`を表示できるようにします

```bash
sudo apt install ros-humble-vision-msgs-rviz-plugins
```

##### CARLAの起動

```
cd <CARLAのインストールフォルダ>

```
