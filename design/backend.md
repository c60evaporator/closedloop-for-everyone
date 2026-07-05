```
data/   # 出力先フォルダのルート（docker_composeでマウントされる）
├── {project_name1}/    # Project1の出力フォルダ
│   ├── {project_name1}.sqlite   # メタデータ管理用のDBファイル
|   ├── map_annotation
|   │   ├── nuscenes/   # nuScenes Map expansion形式の出力フォルダ
|   │   │   ├── basemap_{map_name1}.png  # CARLAから抽出したMap1のベースマップ（png形式）
|   │   │   ├── basemap_{map_name2}.png  # CARLAから抽出したMap2のベースマップ（png形式）
|   │   │   :
|   │   │   └── expansion <- Map Expansionのメタデータ
|   │   └── speed_limits/  # Carla GARAGE用のspeed_limitsファイル
|   │       ├── {map_name1}_speed_limits.npy  # CARLAから抽出したMap1のspeed_limitsファイル（npy形式）
|   │       :
|   ├── data_collection
|   │   ├── garage/           # CARLA Garage形式データセットの出力フォルダ
|   │   ├── garage_nuscenes/  # CARLA Garage nuScenes用データセットの出力フォルダ
|   │   ├── nuscenes/         # nuScenes形式データセットの出力フォルダ
|   │   :   └── {agent_name}_{datagen_route}_{timestamp}/
|   │           ├── results/  # データ収集時のresult.json
|   │           |   ├── {scenario_type}_{route_name}_result.json
|   │           |   :
|   │           ├── v1.0-trainval/
|   │           ├── sweeps/
|   │           └── samples/
│   ├── agents/                     # Leaderboard形式のエージェントコード置き場（デフォルトはleaderboardのものを使用）
│   │   ├── {agent_name1}_{version} # あるエージェントのあるバージョンのファイル保持用フォルダ
│   │   :   ├── Dockerfile_sim       # シミュレーション実行用Dockerfile
│   │       ├── Dockerfile_submit    # Leaderboard提出用Dockerfile
│   │       └── team_code            # このフォルダがコンテナにマウントされる
│   │           ├── {agent_name1}.py # エージェントファイル本体
│   │           :                    # その他の依存ファイル（planner等）
│   ├── parked_vehicles/  # Leaderboard形式の駐車車両定義ファイル置き場（デフォルトはleaderboardのものを使用）
│   │   ├── {parked_vehicle_name1}.py
│   │   :
│   └── scenarios/  # シナリオ定義ファイル置き場（デフォルトはscenario_runnerのものを使用）
│       ├── {scenario_name1}.py
│       :
├── {project_name2}/    # Dataset2の出力フォルダ
```

### メタデータ管理用DBの仕様

```mermaid
erDiagram
    projects{
        int id
        UUID public_id
        string name
        datetime created_at
        datetime updated_at
    }

    weather_groups{
        int id
        UUID public_id
        string name
        int project_id
        int project_index
        datetime created_at
        datetime updated_at
    }

    weathers{
        int id
        UUID public_id
        string name
        int project_id
        int project_index
        float cloudiness
        datetime created_at
        datetime updated_at
    }

    leaderboards{
        int id
        UUID public_id
        string name
        int project_id
        int project_index
        bool is_datagen
        int datagen_routes_id
        string parked_vehicle_file
        datetime created_at
        datetime updated_at
    }

    weather_connections{
        int id
        int weather_group_id
        int weather_id
    }

    routes{
        int id
        UUID public_id
        string name
        int road_id
        string town_name
        JSON waypoints
        int leaderboard_id
        int weather_group_id
        int version
        datetime created_at
        datetime updated_at
    }

    scenarios{
        int id
        UUID public_id
        string name
        int project_id
        int project_index
        string filename
        int version
        datetime created_at
        datetime updated_at
    }

    scenario_connections{
        int id
        int route_id
        int scenario_id
        float trigger_point_x
        float trigger_point_y
        float trigger_point_z
        float trigger_point_yaw
        JSON scenario_params
    }

    datagen_routes{
        int id
        UUID public_id
        string name
        int project_id
        int project_index
        datetime created_at
        datetime updated_at
    }

    parked_groups{
        int id
        UUID public_id
        string name
        int project_id
        int project_index
        datetime created_at
        datetime updated_at
    }

    parked_vehicles{
        int id
        UUID public_id
        int parked_group_id
        string map
        int tilex
        int tiley
        float location_x
        float location_y
        float location_z
        float rotation_pitch
        float rotation_yaw
        float rotation_pitch
        string mesh
        datetime created_at
        datetime updated_at
    }

    agents{
        int id
        UUID public_id
        string name
        int project_id
        int project_index
        string agent_file
        int version
        datetime created_at
        datetime updated_at
    }

    ego_vehicles{
        int id
        UUID public_id
        string name
        int project_id
        int project_index
        string blueprint
        datetime created_at
        datetime updated_at
    }

    sensors{
        int id
        UUID public_id
        string name
        int project_id
        int project_index
        string type
        JSON sensor_attributes
        datetime created_at
        datetime updated_at
    }

    sensor_connections{
        int id
        int ego_vehicle_id
        int sensor_id
        string alias
        float x
        float y
        float z
        float roll
        float pitch
        float yaw
    }

    nuscenes_maps{
        int id
        UUID public_id
        string name
        int project_id
        int project_index
        datetime created_at
        datetime updated_at
    }

    infraction_penalties{
        int id
        UUID public_id
        string name
        int project_id
        int project_index
        string score_version
        float collisions_pedestrian
        float collisions_vehicle
        float collisions_layout
        float red_light
        float yield_emergency_vehicle_infractions
        float stop_infraction
        float scenario_timeouts
        float min_speed_infractions
        datetime created_at
        datetime updated_at
    }

    evaluations{
        int id
        UUID public_id
        string name
        int project_id
        int project_index
        int leaderboard_id
        int parked_group_id
        int agent_id
        int infraction_penalty_id
        datetime created_at
        datetime updated_at
    }

    nuscenes_data{
        int id
        UUID public_id
        string name
        int project_id
        int project_index
        int leaderboard_id
        string collect_agent
        datetime created_at
        datetime updated_at
    }

    nuscenes_map_connections{
        int id
        int nuscenes_data_id
        int nuscenes_map_id
    }

    projects ||--|{ weather_groups: ""
    projects ||--|{ weathers: ""
    projects ||--|{ leaderboards: ""
    projects ||--|{ scenarios: ""
    projects ||--|{ datagen_routes: ""
    projects ||--|{ parked_groups: ""
    projects ||--|{ agents: ""
    projects ||--|{ ego_vehicles: ""
    projects ||--|{ sensors: ""
    projects ||--|{ infraction_penalties: ""
    projects ||--|{ nuscenes_maps: ""
    weather_groups ||--o{ weather_connections: ""
    weathers ||--o{ weather_connections: ""
    leaderboards ||--|{ routes: ""
    weather_groups ||--|{ routes: ""
    routes ||--o{ scenario_connections: ""
    scenarios ||--o{ scenario_connections: ""
    datagen_routes ||--|{ leaderboards: ""
    parked_groups ||--|{ parked_vehicles: ""
    ego_vehicles ||--o{ sensor_connections: ""
    sensors ||--o{ sensor_connections: ""
    leaderboards ||--o{ evaluations: ""
    parked_vehicles ||--o{ evaluations: ""
    agents ||--o{ evaluations: ""
    infraction_penalties ||--o{ evaluations: ""
    leaderboards ||--o{ nuscenes_data: ""
    parked_vehicles ||--o{ nuscenes_data: ""
    nuscenes_data ||--o{ nuscenes_map_connections: ""
    nuscenes_maps ||--o{ nuscenes_map_connections: ""
```


### テンプレートファイル

#### データ収集用エージェント

- PDM-Lite: carla_garage/collect_dataset_slurm.pyをベースに

#### planner

- 


## 注意点

- ego_vehicleの種類は`leaderboard/scenarios/route_scenario.py`の`_spawn_ego_vehicle`で"vehicle.lincoln.mkz_2020"にハードコーディングされているため、置換処理が必要（シミュレーション実行用コンテナ内の`route_scenario.py`をreplaceするのが良いか）
