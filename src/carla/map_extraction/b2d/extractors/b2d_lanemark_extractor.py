"""
LaneMarking (左右の車線境界線の3D座標列 + Type (Broken/Solid/SolidSolid等) + Color)を抽出する

 (参考: https://github.com/Thinklab-SJTU/Bench2Drive/blob/main/tools/gen_hdmap.py#L144)
"""
import carla

def _check_waypoints_status(waypoints_list):
    first_wp = waypoints_list[0]
    init_status = first_wp.is_junction
    current_status = first_wp.is_junction
    change_status_time = 0
    for wp in waypoints_list[1:]:
        if wp.is_junction != current_status:
            current_status = wp.is_junction
            change_status_time += 1
        pass
    if change_status_time == 0:
        return 'Junction' if init_status else 'Normal'
    elif change_status_time == 1:
        return 'EnterNormal' if init_status else 'EnterJunction'
    elif change_status_time == 2:
        return 'PassNormal' if init_status else 'PassJunction'
    else:
        return 'StartJunctionMultiChange' if init_status else 'StartNormalMultiChange'

def _get_connected_road_id(waypoint):
    next_waypoint = waypoint.next(0.05)
    if next_waypoint is None:
        return [None]
    else:
        return [(w.road_id, w.lane_id) for w in next_waypoint if w.lane_type == carla.LaneType.Driving]

def _get_lateral_shifted_transform(transform, shift):
    right_vector = transform.get_right_vector()
    x_offset = right_vector.x * shift
    y_offset = right_vector.y * shift
    z_offset = right_vector.z * shift
    x = transform.location.x + x_offset
    y = transform.location.y + y_offset
    z = transform.location.z + z_offset
    roll = transform.rotation.roll
    pitch = transform.rotation.pitch
    yaw = transform.rotation.yaw
    return ((x, y, z), (roll, pitch, yaw))

def _get_lane_markings_two_side(waypoints, lane_marking_dict):
    left_lane_marking_list = []
    right_lane_marking_list = []
    
    center_lane_list = []
    center_lane_wps = []
    
    left_previous_lane_marking_type = 1
    left_previous_lane_marking_color = 1
    right_previous_lane_marking_type = 1
    right_previous_lane_marking_color = 1
    
    center_previous_lane_id = waypoints[0].lane_id
    
    for waypoint in waypoints:
        flag = False
        if waypoint.lane_id != center_previous_lane_id:
            if len(center_lane_list) > 1:
                if waypoint.road_id not in lane_marking_dict:
                    lane_marking_dict[waypoint.road_id] = {}
                    status = _check_waypoints_status(center_lane_wps)
                    lane_marking_dict[waypoint.road_id][center_previous_lane_id] = []
                    lane_marking_dict[waypoint.road_id][center_previous_lane_id].append({'Points': center_lane_list[:], 'Type': 'Center', 'Color': 'White', 'Topology': _get_connected_road_id(waypoint)[:], 'TopologyType': status, 'Left':(center_lane_wps[-1].get_left_lane().road_id if center_lane_wps[-1].get_left_lane() else None, center_lane_wps[-1].get_left_lane().lane_id if center_lane_wps[-1].get_left_lane() else None), 'Right':(center_lane_wps[-1].get_right_lane().road_id if center_lane_wps[-1].get_right_lane() else None, center_lane_wps[-1].get_right_lane().lane_id if center_lane_wps[-1].get_right_lane() else None)})
                elif center_previous_lane_id not in lane_marking_dict[waypoint.road_id]:
                    status = _check_waypoints_status(center_lane_wps)
                    lane_marking_dict[waypoint.road_id][center_previous_lane_id] = []
                    lane_marking_dict[waypoint.road_id][center_previous_lane_id].append({'Points': center_lane_list[:], 'Type': 'Center', 'Color': 'White', 'Topology': _get_connected_road_id(waypoint)[:], 'TopologyType': status, 'Left':(center_lane_wps[-1].get_left_lane().road_id if center_lane_wps[-1].get_left_lane() else None, center_lane_wps[-1].get_left_lane().lane_id if center_lane_wps[-1].get_left_lane() else None), 'Right':(center_lane_wps[-1].get_right_lane().road_id if center_lane_wps[-1].get_right_lane() else None, center_lane_wps[-1].get_right_lane().lane_id if center_lane_wps[-1].get_right_lane() else None)})
                else:
                    status = _check_waypoints_status(center_lane_wps)
                    lane_marking_dict[waypoint.road_id][center_previous_lane_id].append({'Points': center_lane_list[:], 'Type': 'Center', 'Color': 'White', 'Topology': _get_connected_road_id(waypoint)[:], 'TopologyType': status, 'Left':(center_lane_wps[-1].get_left_lane().road_id if center_lane_wps[-1].get_left_lane() else None, center_lane_wps[-1].get_left_lane().lane_id if center_lane_wps[-1].get_left_lane() else None), 'Right':(center_lane_wps[-1].get_right_lane().road_id if center_lane_wps[-1].get_right_lane() else None, center_lane_wps[-1].get_right_lane().lane_id if center_lane_wps[-1].get_right_lane() else None)})
            flag = True
            center_lane_list = []
            center_lane_wps = []
        left_lane_marking = waypoint.left_lane_marking
        if left_lane_marking.type != left_previous_lane_marking_type or\
            left_lane_marking.color != left_previous_lane_marking_color or flag:
                if len(left_lane_marking_list) > 1:
                    connect_to = _get_connected_road_id(waypoint)
                    candidate_dict = {'Points': left_lane_marking_list[:], 'Type': str(left_previous_lane_marking_type), 'Color': str(left_previous_lane_marking_color), 'Topology': connect_to[:]}
                    if waypoint.road_id not in lane_marking_dict:
                        lane_marking_dict[waypoint.road_id] = {}
                        lane_marking_dict[waypoint.road_id][center_previous_lane_id] = [candidate_dict]
                    elif center_previous_lane_id not in lane_marking_dict[waypoint.road_id]:
                        lane_marking_dict[waypoint.road_id][center_previous_lane_id] = [candidate_dict]
                    else:
                        lane_marking_dict[waypoint.road_id][center_previous_lane_id].append(candidate_dict)
                    left_lane_marking_list = []
        right_lane_marking = waypoint.right_lane_marking
        if right_lane_marking.type != right_previous_lane_marking_type or\
            right_lane_marking.color != right_previous_lane_marking_color or flag:
                if len(right_lane_marking_list) > 1:
                    connect_to = _get_connected_road_id(waypoint)
                    candidate_dict = {'Points': right_lane_marking_list[:], 'Type': str(right_previous_lane_marking_type), 'Color': str(right_previous_lane_marking_color), 'Topology': connect_to[:]}
                    if waypoint.road_id not in lane_marking_dict:
                        lane_marking_dict[waypoint.road_id] = {}
                        lane_marking_dict[waypoint.road_id][center_previous_lane_id] = [candidate_dict]
                    elif center_previous_lane_id not in lane_marking_dict[waypoint.road_id]:
                        lane_marking_dict[waypoint.road_id][center_previous_lane_id] = [candidate_dict]
                    else:
                        lane_marking_dict[waypoint.road_id][center_previous_lane_id].append(candidate_dict)
                    right_lane_marking_list = []
                    
        center_lane_list.append((*_get_lateral_shifted_transform(waypoint.transform, 0), waypoint.is_junction))
        center_lane_wps.append(waypoint)
        
        left_lane_marking_list.append(_get_lateral_shifted_transform(waypoint.transform, -0.5*waypoint.lane_width))
        
        right_lane_marking_list.append(_get_lateral_shifted_transform(waypoint.transform, 0.5*waypoint.lane_width))
        
        left_previous_lane_marking_type = left_lane_marking.type
        left_previous_lane_marking_color = left_lane_marking.color
        right_previous_lane_marking_type = right_lane_marking.type
        right_previous_lane_marking_color = right_lane_marking.color
        center_previous_lane_id = waypoint.lane_id
    
    if len(left_lane_marking_list) > 1:
        connect_to = _get_connected_road_id(waypoint)
        candidate_dict = {'Points': left_lane_marking_list[:], 'Type': str(left_lane_marking.type), 'Color': str(left_previous_lane_marking_color), 'Topology': connect_to[:]}
        if waypoint.road_id not in lane_marking_dict:
            lane_marking_dict[waypoint.road_id] = {}
            lane_marking_dict[waypoint.road_id][center_previous_lane_id] = [candidate_dict]
        elif center_previous_lane_id not in lane_marking_dict[waypoint.road_id]:
            lane_marking_dict[waypoint.road_id][center_previous_lane_id] = [candidate_dict]
        else:
            lane_marking_dict[waypoint.road_id][center_previous_lane_id].append(candidate_dict)
        left_lane_marking_list = []
    if len(right_lane_marking_list) > 1:
        connect_to = _get_connected_road_id(waypoint)
        candidate_dict = {'Points': right_lane_marking_list[:], 'Type': str(right_lane_marking.type), 'Color': str(right_previous_lane_marking_color), 'Topology': connect_to[:]}
        if waypoint.road_id not in lane_marking_dict:
            lane_marking_dict[waypoint.road_id] = {}
            lane_marking_dict[waypoint.road_id][center_previous_lane_id] = [candidate_dict]
        elif center_previous_lane_id not in lane_marking_dict[waypoint.road_id]:
            lane_marking_dict[waypoint.road_id][center_previous_lane_id] = [candidate_dict]
        else:
            lane_marking_dict[waypoint.road_id][center_previous_lane_id].append(candidate_dict)
        right_lane_marking_list = []
    if len(center_lane_list) > 0:
        if waypoint.road_id not in lane_marking_dict:
            lane_marking_dict[waypoint.road_id] = {}
            status = _check_waypoints_status(center_lane_wps)
            lane_marking_dict[waypoint.road_id][center_previous_lane_id] = []
            lane_marking_dict[waypoint.road_id][center_previous_lane_id].append({'Points': center_lane_list[:], 'Type': 'Center', 'Color': 'White', 'Topology': _get_connected_road_id(waypoint)[:], 'TopologyType': status, 'Left':(center_lane_wps[-1].get_left_lane().road_id if center_lane_wps[-1].get_left_lane() else None, center_lane_wps[-1].get_left_lane().lane_id if center_lane_wps[-1].get_left_lane() else None), 'Right':(center_lane_wps[-1].get_right_lane().road_id if center_lane_wps[-1].get_right_lane() else None, center_lane_wps[-1].get_right_lane().lane_id if center_lane_wps[-1].get_right_lane() else None)})
        elif center_previous_lane_id not in lane_marking_dict[waypoint.road_id]:
            status = _check_waypoints_status(center_lane_wps)
            lane_marking_dict[waypoint.road_id][center_previous_lane_id] = []
            lane_marking_dict[waypoint.road_id][center_previous_lane_id].append({'Points': center_lane_list[:], 'Type': 'Center', 'Color': 'White', 'Topology': _get_connected_road_id(waypoint)[:], 'TopologyType': status, 'Left':(center_lane_wps[-1].get_left_lane().road_id if center_lane_wps[-1].get_left_lane() else None, center_lane_wps[-1].get_left_lane().lane_id if center_lane_wps[-1].get_left_lane() else None), 'Right':(center_lane_wps[-1].get_right_lane().road_id if center_lane_wps[-1].get_right_lane() else None, center_lane_wps[-1].get_right_lane().lane_id if center_lane_wps[-1].get_right_lane() else None)})
        else:
            status = _check_waypoints_status(center_lane_wps)
            lane_marking_dict[waypoint.road_id][center_previous_lane_id].append({'Points': center_lane_list[:], 'Type': 'Center', 'Color': 'White', 'Topology': _get_connected_road_id(waypoint)[:], 'TopologyType': status, 'Left':(center_lane_wps[-1].get_left_lane().road_id if center_lane_wps[-1].get_left_lane() else None, center_lane_wps[-1].get_left_lane().lane_id if center_lane_wps[-1].get_left_lane() else None), 'Right':(center_lane_wps[-1].get_right_lane().road_id if center_lane_wps[-1].get_right_lane() else None, center_lane_wps[-1].get_right_lane().lane_id if center_lane_wps[-1].get_right_lane() else None)})

def get_lanemarkings(carla_map, precision=0.05):
    """
    Extract lane markings and build a connectivity graph based on the topology.
    :param carla_map: carla.Map
    :param precision: distance in meters to consider two waypoints connected (default: 0.05m)
    :return: List of TopologyEdge representing the lane connectivity graph
    """
    topology = [x[0] for x in carla_map.get_topology()]
    topology = sorted(topology, key=lambda w: w.road_id)
    print(f"Topology has {len(topology)} edges")

    lane_marking_dict={}

    for waypoint in topology:
        waypoints = [waypoint]
        # Generate waypoints of a road id. Stop when road id differs
        nxt = waypoint.next(precision)
        if len(nxt) > 0:
            nxt = nxt[0]
            temp_wp = nxt
            while nxt.road_id == waypoint.road_id:
                waypoints.append(nxt)
                nxt = nxt.next(precision)
                if len(nxt) > 0:
                    nxt = nxt[0]
                else:
                    break

        print("current road id: ", waypoint.road_id)
        print("lane id:", waypoint.lane_id)
        _get_lane_markings_two_side(waypoints, lane_marking_dict)

    return lane_marking_dict
