import numpy as np
import carla

def _get_corners_from_actor_list(actor_list):
    for actor_transform, bb_loc, bb_ext in actor_list:

        corners = [carla.Location(x=-bb_ext.x, y=-bb_ext.y),
                    carla.Location(x=bb_ext.x, y=-bb_ext.y),
                    carla.Location(x=bb_ext.x, y=0),
                    carla.Location(x=bb_ext.x, y=bb_ext.y),
                    carla.Location(x=-bb_ext.x, y=bb_ext.y)]
        corners = [bb_loc + corner for corner in corners]

        corners = [actor_transform.transform(corner) for corner in corners]
        corners = [[corner.x, corner.y, corner.z] for corner in corners]
    return corners

def _insert_point_into_dict(lane_marking_dict, corners, road_id, parent_actor_location, Volume_Type=None):
    if road_id not in lane_marking_dict.keys():
        print("Cannot find road:", road_id)
        raise
    if Volume_Type is None:
        print("Missing 'Volume Type' ")
        raise
    if 'Trigger_Volumes' not in lane_marking_dict[road_id]:
        lane_marking_dict[road_id]['Trigger_Volumes'] = [{'Points': corners[:], 'Type': Volume_Type, 'ParentActor_Location': parent_actor_location[:]}]
    else:
        lane_marking_dict[road_id]['Trigger_Volumes'].append({'Points': corners[:], 'Type': Volume_Type, 'ParentActor_Location': parent_actor_location[:]})

def get_stop_sign_trigger_volume(all_stop_sign_actors, lane_marking_dict, carla_map):
    for actor in all_stop_sign_actors:
        bb_loc = carla.Location(actor.trigger_volume.location)
        bb_ext = carla.Vector3D(actor.trigger_volume.extent)
        bb_ext.x = max(bb_ext.x, bb_ext.y)
        bb_ext.y = max(bb_ext.x, bb_ext.y)
        base_transform = actor.get_transform()
        stop_info_list = [(carla.Transform(base_transform.location, base_transform.rotation), bb_loc, bb_ext)]
        corners = _get_corners_from_actor_list(stop_info_list)
        
        trigger_volume_wp = carla_map.get_waypoint(base_transform.transform(bb_loc))
        actor_loc = actor.get_location()
        actor_loc_points = [actor_loc.x, actor_loc.y, actor_loc.z]
        _insert_point_into_dict(lane_marking_dict, corners, trigger_volume_wp.road_id, actor_loc_points, Volume_Type='StopSign')
        
    pass


def get_traffic_light_trigger_volume(all_trafficlight_actors, lane_marking_dict, carla_map):
    for actor in all_trafficlight_actors:
        base_transform = actor.get_transform()
        tv_loc = actor.trigger_volume.location
        tv_ext = actor.trigger_volume.extent
        x_values = np.arange(-0.9 * tv_ext.x, 0.9 * tv_ext.x, 1.0)
        area = []
        for x in x_values:
            point_location = base_transform.transform(tv_loc + carla.Location(x=x)) 
            area.append(point_location)
        ini_wps = []
        for pt in area:
            wpx = carla_map.get_waypoint(pt)
            # As x_values are arranged in order, only the last one has to be checked
            if not ini_wps or ini_wps[-1].road_id != wpx.road_id or ini_wps[-1].lane_id != wpx.lane_id:
                ini_wps.append(wpx)
        
        close2junction_points = []
        littlefar2junction_points = []
        for wpx in ini_wps:
            while not wpx.is_intersection:
                next_wp = wpx.next(0.5)
                if not next_wp:
                    break
                next_wp = next_wp[0]
                if next_wp and not next_wp.is_intersection:
                    wpx = next_wp
                else:
                    break
            vec_forward = wpx.transform.get_forward_vector()
            vec_right = carla.Vector3D(x=-vec_forward.y, y=vec_forward.x, z=0) # 2D

            loc_left = wpx.transform.location - 0.4 * wpx.lane_width * vec_right
            loc_right = wpx.transform.location + 0.4 * wpx.lane_width * vec_right
            close2junction_points.append([loc_left.x, loc_left.y, loc_left.z])
            close2junction_points.append([loc_right.x, loc_right.y, loc_right.z])
            
            try:
                loc_far_left = wpx.previous(0.5)[0].transform.location - 0.4 * wpx.lane_width * vec_right
                loc_far_right = wpx.previous(0.5)[0].transform.location + 0.4 * wpx.lane_width * vec_right
            except Exception:
                continue
            
            littlefar2junction_points.append([loc_far_left.x, loc_far_left.y, loc_far_left.z])
            littlefar2junction_points.append([loc_far_right.x, loc_far_right.y, loc_far_right.z])
            
        traffic_light_points = close2junction_points + littlefar2junction_points[::-1]
        trigger_volume_wp = carla_map.get_waypoint(base_transform.transform(tv_loc))
        actor_loc = actor.get_location()
        actor_loc_points = [actor_loc.x, actor_loc.y, actor_loc.z]
        _insert_point_into_dict(lane_marking_dict, traffic_light_points, trigger_volume_wp.road_id, actor_loc_points, Volume_Type='TrafficLight')
    pass