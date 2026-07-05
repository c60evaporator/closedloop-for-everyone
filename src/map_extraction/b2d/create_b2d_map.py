import logging

import numpy as np

from src.map_extraction.b2d.extractors.b2d_lanemark_extractor import get_lanemarkings
from src.map_extraction.b2d.extractors.b2d_trigger_volume_extractor import (
    get_stop_sign_trigger_volume, get_traffic_light_trigger_volume,
)

logger = logging.getLogger(__name__)

def extract_carla_map_data(carla_world, sampling_resolution):
    """
    CARLAマップから必要なデータを抽出する。
    :param carla_world: CARLAワールドオブジェクト
    :param sampling_resolution: ウェイポイントのサンプリング間隔
    :return: 抽出されたデータの辞書
    """
    carla_map = carla_world.get_map()

    # 道路トポロジーからレーン情報を抽出する
    lane_marking_dict = get_lanemarkings(carla_map, precision=sampling_resolution)

    # 停止線、信号機のアクターを取得
    all_actors = carla_world.get_actors()
    all_stop_sign_actors = []
    all_traffic_light_actors = []
    for actor in all_actors:
        if 'traffic.stop' in actor.type_id:
            all_stop_sign_actors.append(actor)
        if 'traffic_light' in actor.type_id:
            all_traffic_light_actors.append(actor)

    print("Getting all trigger volumes ...")
    # 停止線のトリガーボリュームを抽出する
    get_stop_sign_trigger_volume(all_stop_sign_actors, lane_marking_dict, carla_map)
    # 信号機のトリガーボリュームを抽出する
    get_traffic_light_trigger_volume(all_traffic_light_actors, lane_marking_dict, carla_map)
    print("******* Have get all trigger volumes ! *********")

    # Numpy配列に変換
    arr = np.array(list(lane_marking_dict.items()), dtype=object)

    return arr
