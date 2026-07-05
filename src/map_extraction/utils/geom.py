import time

import numpy as np
import cv2
from shapely.geometry import Polygon, LineString
from shapely.ops import split, linemerge

def merge_linestrings_greedy(linestrings):
    """
    Greedily merge a list of LineStrings into a single LineString by connecting the closest endpoints.
    """
    if not linestrings:
        return LineString()
    if len(linestrings) == 1:
        return linestrings[0]
    remaining = list(linestrings)
    chain = list(remaining.pop(0).coords)
    while remaining:
        best_idx, best_dist, best_reverse = 0, float('inf'), False
        tail = chain[-1]
        for i, seg in enumerate(remaining):
            c = list(seg.coords)
            d_start = (tail[0] - c[0][0]) ** 2 + (tail[1] - c[0][1]) ** 2
            d_end = (tail[0] - c[-1][0]) ** 2 + (tail[1] - c[-1][1]) ** 2
            if d_start < best_dist:
                best_idx, best_dist, best_reverse = i, d_start, False
            if d_end < best_dist:
                best_idx, best_dist, best_reverse = i, d_end, True
        seg = remaining.pop(best_idx)
        c = list(seg.coords)
        if best_reverse:
            c = c[::-1]
        chain.extend(c[1:])  # skip duplicate start point
    return LineString(chain)

def get_lane_polygon_points(boundary_points):
    """
    Docstring for get_lane_polygon
    
    :param boundary_points: List of boundary points, each with attributes left_x, left_y, right_x, right_y
    """
    left = [(bp.left_x, bp.left_y) for bp in boundary_points]
    right = [(bp.right_x, bp.right_y) for bp in reversed(boundary_points)]
    polygon_coords = left + right
    return polygon_coords

def _extend_line_to_edge_boundary(p1, p2, edge_boundary):
    """
    Extend a line defined by two points to the edges of a boundary. The direction is from p1 to p2.
    Args:
        p1: Tuple of (x1, y1) coordinates of the first point.
        p2: Tuple of (x2, y2) coordinates of the second point.
        edge_boundary: Tuple of (min_x, min_y, max_x, max_y) representing the boundary.
    Returns:
        A list [x_edge, y_edge] representing the coordinates where the line intersects the boundary edge.
    """
    (x1, y1), (x2, y2) = p1, p2
    min_x, min_y, max_x, max_y = edge_boundary
    if x2 - x1 == 0:  # Vertical line
        return [x1, min_y] if y1 > y2 else [x1, max_y]
    slope = (y2 - y1) / (x2 - x1)
    intercept = y1 - slope * x1
    # Check intersection with left edge (x=min_x)
    if x1 > x2:
        y_at_left = slope * min_x + intercept
        if min_y <= y_at_left <= max_y:
            return [min_x, y_at_left]
    # Check intersection with right edge (x=max_x)
    elif x2 > x1:
        y_at_right = slope * max_x + intercept
        if min_y <= y_at_right <= max_y:
            return [max_x, y_at_right]
    # Check intersection with top edge (y=min_y)
    if y1 > y2:
        x_at_top = (min_y - intercept) / slope
        if min_x <= x_at_top <= max_x:
            return [x_at_top, min_y]
    # Check intersection with bottom edge (y=max_y)
    elif y2 > y1:
        x_at_bottom = (max_y - intercept) / slope
        if min_x <= x_at_bottom <= max_x:
            return [x_at_bottom, max_y]
    raise ValueError('Line does not intersect the boundary edges properly.')

def split_polygon_by_line(polygon, splitter_line, edge_boundary, 
                          extend_line_to_edge=False, save_image_debug=False):
    """Split a polygon into two polygons using a linestring as divider.
    Args:
        polygon (shapely.geometry.Polygon): The polygon to be split.
        splitter_line (shapely.geometry.LineString): The linestring used to split the polygon.
        edge_boundary (tuple): The boundary of the edge (min_x, min_y, max_x, max_y) for extending lines to edges.
        extend_line_to_edge (bool): Whether to extend the divider line to the edge boundary if it does not intersect the polygon.
        save_image_debug (bool): Whether to save debug image of the polygon and divider line.
    
    Returns:
        split_polygons (shapely.geometry.MultiPolygon): The two or more resulting polygons after the split.
        divider_linestring (shapely.geometry.LineString): The linestring used as divider inside the polygon.
    """
    if extend_line_to_edge:
        splitter_line_used = list(splitter_line.coords)
        # Extend the divider line from the first end to the mask edge
        first_end = splitter_line.coords[0]
        if first_end[0] > edge_boundary[0] and first_end[0] < edge_boundary[2] and first_end[1] > edge_boundary[1] and first_end[1] < edge_boundary[3]:
            extended_point1 = _extend_line_to_edge_boundary(splitter_line.coords[1], first_end, edge_boundary)
            splitter_line_used = [extended_point1] + splitter_line_used
            #print(f"Extending point from {splitter_line.coords[1]} to {first_end} is {extended_point1}")
        # Extend the divider line from the second end to the mask edge        second_end = splitter_line.coords[-1]
        second_end = splitter_line.coords[-1]
        if second_end[0] > edge_boundary[0] and second_end[0] < edge_boundary[2] and second_end[1] > edge_boundary[1] and second_end[1] < edge_boundary[3]:
            extended_point2 = _extend_line_to_edge_boundary(splitter_line.coords[-2], second_end, edge_boundary)
            splitter_line_used = splitter_line_used + [extended_point2]
            #print(f"Extending point from {splitter_line.coords[-2]} to {second_end} is {extended_point2}")
        splitter_line_used = LineString(splitter_line_used)
    else:
        splitter_line_used = splitter_line
    # Check intersections of the linestring and polygon boundary
    intersections = splitter_line_used.intersection(polygon.boundary)
    if intersections.geom_type != 'MultiPoint':
        print("Warning: failed to find intersection points of polygon_split_line and polygon boundary.")
        return None, None
    if len(intersections.geoms) > 2:
        print("Warning: more than two intersection points found between polygon_split_line and polygon boundary.")
    # Save the polygon and line for debugging
    if save_image_debug:
        mask_debug_rgb = np.zeros((int(edge_boundary[3] - edge_boundary[1]) + 1, int(edge_boundary[2] - edge_boundary[0]) + 1, 3), dtype=np.uint8)
        exterior_coords_crop = np.array(polygon.exterior.coords) - np.array([edge_boundary[0], edge_boundary[1]])
        cv2.polylines(mask_debug_rgb, [exterior_coords_crop.astype(np.int32).reshape((-1, 1, 2))], isClosed=True, color=(0, 255, 0), thickness=1)
        line_coords_crop = np.array(splitter_line_used.coords) - np.array([edge_boundary[0], edge_boundary[1]])
        cv2.polylines(mask_debug_rgb, [line_coords_crop.astype(np.int32).reshape((-1, 1, 2))], isClosed=False, color=(255, 0, 0), thickness=1)
        cv2.imwrite(f'debug_polygon_split_{int(time.time())}.png', mask_debug_rgb)
    # Split the polygon using the splitter line
    split_polygons = split(polygon, splitter_line_used)
    if len(split_polygons.geoms) != 2:
        print("Warning: polygon splitting did not result in two polygons. The split component count is:", len(split_polygons.geoms))
        return None, None
    
    # Calculate split line inside polygon for using it as divider line
    split_lines_inside = split(splitter_line_used, polygon)
    if len(split_lines_inside.geoms) == 0:
        raise ValueError("No split line found inside polygon after splitting.")
    elif len(split_lines_inside.geoms) == 1:
        divider_linestring = split_lines_inside.geoms[0]
    else:
        # First, try to choose the lines inside the polygon
        divider_lines = [line for line in split_lines_inside.geoms if polygon.contains(line)]
        if len(divider_lines) == 0:
            # If no lines inside the polygon found and there are three lines, choose the middle one
            if len(split_lines_inside.geoms) == 3:
                divider_lines = [split_lines_inside.geoms[1]]
                divider_linestring = divider_lines[0]
            else:
                raise ValueError("No valid divider line found inside polygon after splitting.")
        if len(divider_lines) == 1:
            divider_linestring = divider_lines[0]
        # If multiple lines exist, merge them
        else:
            divider_linestring = linemerge(divider_lines)
    
    return split_polygons, divider_linestring
