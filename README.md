
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
