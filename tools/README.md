
```
tools
├── b2d/                          <- Bench2Drive評価で使用するスクリプト類（直接実行はしない）
├── b2d_leaderboard_common/       <- Bench2Drive評価＆Leaderboard評価で使用するスクリプト類（直接実行はしない）
├── carla_launch/                 <- CARLA一斉起動で使用するスクリプト類（直接実行はしない）
├── leaderboard_local/            <- データ収集＆Leaderboard評価で使用するスクリプト類（直接実行はしない）
├── tfpp/                         <- TransFuser++の動作に必要なスクリプト類
├── carla_map_to_nuscenes.py      <- CARLAに接続してnuScenes Map expansion形式のマップアノテーションを作成するスクリプト
├── launch_carla_servers.sh       <- CARLAをマルチGPU一斉実行してwatchdogも起動するスクリプト（データ収集、Laderboard/B2D評価で使用）
├── collect_dataset_multi.sh      <- PDM Liteによるデータ収集を実行するスクリプト
├── evaluate_leaderboard_multi.sh <- CARA Leaderboard評価を実行するスクリプト
├── evaluate_b2d_multi.sh         <- Bench2Drive評価を実行するスクリプト
├── convert_to_nuscenes.py        <- PDM Liteで収集したデータをnuScenesフォーマットに変換するスクリプト
└── README.md
```
