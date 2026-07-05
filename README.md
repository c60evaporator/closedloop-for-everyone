
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
