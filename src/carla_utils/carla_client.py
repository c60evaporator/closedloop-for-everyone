"""
CARLA Client: CARLAシミュレーターへの接続とMapオブジェクトの取得
"""
import os
import sys
import logging

import carla

logger = logging.getLogger(__name__)


def get_carla_client(host: str = None, port: int = None, timeout: float = 10.0):
    """
    CARLAサーバーに接続してClientオブジェクトを返す。
    :param host: CARLAサーバーのホスト (デフォルト: env CARLA_HOST or localhost)
    :param port: CARLAサーバーのポート (デフォルト: env CARLA_PORT or 2000)
    :param timeout: 接続タイムアウト秒数
    :return: carla.Client
    """

    host = host or os.environ.get("CARLA_HOST", "localhost")
    port = port or int(os.environ.get("CARLA_PORT", "2000"))

    logger.info(f"Connecting to CARLA server at {host}:{port} ...")
    client = carla.Client(host, port)
    client.set_timeout(timeout)

    server_version = client.get_server_version()
    client_version = client.get_client_version()
    logger.info(f"Connected. Server: {server_version}, Client: {client_version}")

    return client

def set_world(client, map_name=None):
    """
    CARLAワールドを取得する。必要に応じてマップをロードする。
    :param client: carla.Client
    :param map_name: ロードするマップ名 (例: 'Town01'). Noneの場合は現在のマップを使用
    :return: carla.World
    """
    if map_name:
        logger.info(f"Loading map: {map_name}")
        client.load_world(map_name)
    world = client.get_world()
    logger.info(f"Current map: {world.get_map().name.split('/')[-1]}")
    return world
