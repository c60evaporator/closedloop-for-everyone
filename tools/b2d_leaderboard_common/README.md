# b2d_leaderboard_commonフォルダ

CARLA Leaderboard評価（`evaluate_leaderboard_multi.sh`で実行）とBench2Drive評価（`evaluate_b2d_multi.sh`で実行）共通で使用するスクリプト（ユーザーの直接実行対象ではなく、他のスクリプトから呼び出されるコード類）を格納。

|ファイル名|処理内容|
|---|---|
|`skip_route.py`|CARLAのハードクラッシュ（例：`spawn_parked_vehicles`起因のC++側クラッシュ）が同じルートで繰り返され評価が先に進まなくなったとき、そのルートを強制スキップするスクリプト。チェックポイントJSONの`_checkpoint.progress[0]`からスタック中のルートを特定し、ルートXMLからメタデータ（route_id・town・シナリオ名・天候ID）を取得して`"Failed - Simulation crashed"`のダミー評価レコード（スコア0）を挿入し、`progress[0]`を+1して次のルートから評価が再開されるようにする。`evaluate_b2d_multi.sh`のスタック検知（同一ルートで`MAX_STUCK`回連続の非前進クラッシュ）から自動で呼び出される。※チェックポイント形式がBench2Drive用のため、CARLA Leaderboard評価（`evaluate_leaderboard_multi.sh`）では使用されない|
|`split_route_xml.py`|ルートXMLファイルを指定した個数に分割するスクリプト（`python split_route_xml.py <base_route> <task_num>`で`<base_route>.xml`を`<base_route>_0.xml`〜`<base_route>_<task_num-1>.xml`に均等分割）。マルチGPU並列評価で各GPUプロセスに別々のルートファイルを割り当てるために使用|
