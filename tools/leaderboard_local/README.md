# leaderboard_localフォルダ

データ収集（`collect_dataset_multi.sh`で実行）とCARLA Leaderboard評価（`evaluate_leaderboard_multi.sh`で実行）で使用するスクリプト・パッチ（ユーザーの直接実行対象ではなく、他のスクリプトから呼び出されるコード類）を格納。

|ファイル名|処理内容|
|---|---|
|`collect_dataset.sh`|`collect_dataset_multi.sh`から呼び出され、各GPUにおけるデータ収集を実行するスクリプト|
|`evaluate_leaderboard.sh`|`evaluate_leaderboard_multi.sh`から呼び出され、各GPUにおけるCARLA Leaderboard評価を実行するスクリプト|
|`leaderboard_evaluator_local_ext.py`|`carla_garage/leaderboard_autopilot/leaderboard/leaderboard_evaluator_local.py`（データ収集）または`carla_garage/leaderboard/leaderboard/leaderboard_evaluator_local.py`（Leaderboard評価）の`LeaderboardEvaluator`クラスを継承して修正を加えたパッチ。`collect_dataset.sh`および`evaluate_leaderboard.sh`から呼び出され、データ収集やCARLA Leaderboard評価のワークフロー管理を実施|
|`agent_wrapper_patches.py`|`carla_garage/leaderboard_autopilot/leaderboard/autoagents/agent_wrapper_local.py`の`AgentWrapper`クラスを継承して修正を加えたパッチ。データ収集用エージェントの共通処理を記述|

`leaderboard_evaluator_local_ext.sh`および`agent_wrapper_patches.py`の修正内容を後述します。

### leaderboard_evaluator_local_ext.py

元々のCARLA Garageではデータ収集やCARLA Leaderboard評価のワークフロー管理に使用していた`carla_garage/leaderboard/leaderboard/leaderboard_evaluator_local.py`の利便性を向上させるため、この内部の`LeaderboardEvaluator`クラスを継承して以下の修正を加えたもの

- Traffic Managerのポート被りエラーが発生しやすいので、GPUごとにポート範囲を分離する処理を追加
- CARLAの同期設定（leaderboard_evaluator_local.pyでは起動時に1回のみ適用される）が大型マップロード時にリセットされることがあるので、マップロード時に毎回適用されるよう修正

### agent_wrapper_patches.py

元々のCARLA Garageではデータ収集用エージェントの共通処理を記述していた`carla_garage/leaderboard_autopilot/leaderboard/autoagents/agent_wrapper_local.py`の利便性を向上させるため、この内部の`AgentWrapper`クラスを継承して以下の修正を加えたもの

- LiDARのパラメータが`channels=64`、`range=85`などがハードコードされていたが、エージェントの`_sensors()`メソッドでこれらのLiDARスペックを指定できるように修正
