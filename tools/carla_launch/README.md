# carla_launchフォルダ

CARLAのマルチGPU一斉起動（`launch_carla_servers.sh`で実行）で使用するスクリプト（ユーザーの直接実行対象ではなく、他のスクリプトから呼び出されるスクリプト類）を格納。

|ファイル名|処理内容|
|---|---|
|`_carla_lib.sh`|`launch_carla_servers.sh`と`_carla_watchdog.sh`からsourceされる共通ライブラリ（直接実行不可）。`.env`の読み込み、設定デフォルト値（`CARLA_BASE_PORT=2000`、`CARLA_PORT_STEP=150`、各種タイムアウト等）の定義、およびヘルパー関数を提供。中核は`launch_one`関数で、GPUごとにポート`CARLA_BASE_PORT + i×CARLA_PORT_STEP`でCARLAをオフスクリーン起動し、RPCポートの接続可否で起動完了を確認（失敗時は`CARLA_MAX_RETRIES`回リトライ）、PIDを`.pids/`に記録する。ほかにポートからのプロセス検索（`find_pid`）・強制killコマンド（`kill_by_port`）・ポート接続待ち（`wait_port`）を含む|
|`_carla_watchdog.sh`|`launch_carla_servers.sh`から起動される常駐監視プロセス。各CARLAサーバの生存を`WATCHDOG_INTERVAL`（デフォルト60秒）ごとにプロセス検索で確認し、死んでいれば`launch_one`で再起動する（CARLAはよくクラッシュで落ちるので、この機能がないと評価が途中で止まる原因となる）。加えて、後述のsentinelファイル`.restart_request_<port>`を5秒ごとにポーリングし、要求があれば生きているCARLAも強制再起動して要求ファイルを削除する|
|`_restart_request.sh`|コンテナ側の実行スクリプト（`collect_dataset_multi.sh`、`evaluate_leaderboard_multi.sh`、`evaluate_b2d_multi.sh`）がsourceして使う`request_carla_restart`関数を提供（直接実行不可）。評価がexit code 0以外で失敗した後のリトライ前に呼ばれ、sentinelファイル`.restart_request_<port>`を作成してホスト側watchdogにCARLA再起動を要求し、ファイルの消滅（=再起動完了）を待つ。クラッシュした評価の残骸（ego車両・センサ・Traffic Manager）が残ったCARLAをそのまま再利用すると以降の実行が汚染されるため、リトライ前に必ずまっさらなインスタンスに入れ替えるのが目的。タイムアウト時（watchdog未起動等）は要求を取り下げて既存インスタンスを再利用するフォールバック動作となる|
