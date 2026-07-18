# b2dフォルダ

Bench2Drive評価（`evaluate_b2d_multi.sh`で実行）で使用するパッチ（ユーザーの直接実行対象ではなく、他のスクリプトから呼び出されるコード類）を格納。

|ファイル名|処理内容|
|---|---|
|`leaderboard_evaluator_b2d_ext.py`|`carla_garage/Bench2Drive/leaderboard/leaderboard/leaderboard_evaluator.py`の`LeaderboardEvaluator`クラスを継承して修正を加えたパッチ。`evaluate_b2d_multi.sh`から呼び出され、Bench2Drive評価のワークフロー管理を実施|
|`pdm_lite_patches.py`|Bench2DriveでPDM-Liteを動作させるためのパッチ|

`leaderboard_evaluator_b2d_ext.py`および`pdm_lite_patches.py`の修正内容を後述します。

### leaderboard_evaluator_b2d_ext.py

元々のCARLA GarageではBench2Driveのワークフロー管理に使用していた`carla_garage/Bench2Drive/leaderboard/leaderboard/leaderboard_evaluator.py`の利便性を向上させるため、この内部の`LeaderboardEvaluator`クラスを継承して以下の修正を加えたもの

- デフォルトのBench2DriveはCARLAをleaderboard_evaluator.pyから起動しているが、これだとDockerコンテナ内でCARLAが起動されてvulkan周りの問題が発生しやすいため、ホスト側で起動済のCARLAに接続するよう修正
- データ記録用に車両から離れた位置にBEVカメラを設置すると距離制限に引っかかるため、`leaderboard.autoagents.agent_wrapper.MAX_ALLOWED_RADIUS_SENSOR`に100のような大きな値を指定
- `SAVE_PATH=1`を指定しても`if self.save_path is not None and route_index is not None`という条件文のせいで常に`self.save_path = None`となり、想定通りデータが保存されないため、`self.save_path`にパスを入れてデータ保存が適用されるように修正
- `leaderboard_evaluator_local_ext`と同様、Traffic Managerのポート被りエラー対策を追加


### pdm_lite_patches.py

Bench2Drive付属のPDM-Liteエージェント（`carla_garage/Bench2Drive/team_code`の`autopilot.py`とその依存モジュール`kinematic_bicycle_model.py`）はNumPy 1.24で削除された旧APIに依存しており、新しいNumPy環境ではそのまま動作しません。このファイルは、carla_garage側のコードを変更せずにランタイムで以下2つの互換性修正を適用するパッチです。

- **NumPy型エイリアスの復元**（本モジュールのimport時に自動適用）: NumPy 1.24で削除された非推奨エイリアス`np.float`, `np.int`, `np.bool`, `np.complex`, `np.object`, `np.str`を組み込み型（`float`, `int`, ...）に割り当て直し、これらを使用するBench2Drive `team_code`のimportが`AttributeError`で失敗しないようにする
- **`KinematicBicycleModel.forecast_ego_vehicle`メソッドの差し替え**（`apply_pdm_lite_patches()`の呼び出し時に適用）: 元の実装は形状`(1,)`の配列とスカラーが混在した`np.array([...])`を構築しており、NumPy 1.24以降では非均質配列としてエラーになる。パッチ版は全入力をPythonのfloatにsqueezeしてから速度予測の多項式特徴ベクトルを均質なスカラーのみで再構築する実装に置換する（自転車モデルの計算式自体は変更なし）。呼び出し元が期待する出力形状（location: `(3,)`, heading: `(1,)`, speed: `(1,)`）は維持される

使用方法は`leaderboard_evaluator_b2d_ext.py`に実装されています。型エイリアスの復元は`from pdm_lite_patches import apply_pdm_lite_patches`のimport文の時点で（`team_code`のimportより前に）適用され、`apply_pdm_lite_patches()`自体はエージェントモジュールのロード後（`kinematic_bicycle_model`が`sys.path`から解決可能になった後）に呼び出す必要があります。複数回呼び出しても安全（冪等）で、`team_code`が未ロードの場合は何もせずスキップします。
