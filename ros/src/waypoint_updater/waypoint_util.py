import math

def distance(x1, y1, x2, y2):
    return math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))

def closest_waypoint(x, y, waypoints, start_wp_idx=0, lookup_range=100):

    closestLen = 100000; #large number
    closestWaypoint = 0;

    # -1 or 0 for full scan
    if start_wp_idx <= 0:
        start_wp_idx = 0
        end_wp_idx = len(waypoints) 
    else:
        end_wp_idx = start_wp_idx + lookup_range
        end_wp_idx = min(end_wp_idx, len(waypoints))

    i = start_wp_idx
    # for performance optimization
    for waypoint in waypoints[start_wp_idx:end_wp_idx]:
        map_x = waypoint.pose.pose.position.x
        map_y = waypoint.pose.pose.position.y
        dist = distance(x, y, map_x, map_y)
        if dist < closestLen:
            closestLen = dist
            closestWaypoint = i
        i += 1

    return closestWaypoint


def next_waypoint(x, y, theta, waypoints, start_wp_idx):

    closestWaypoint = closest_waypoint(x, y, waypoints, start_wp_idx)

    map_x = waypoints[closestWaypoint].pose.pose.position.x
    map_y = waypoints[closestWaypoint].pose.pose.position.y

    heading = math.atan2((map_y-y), (map_x-x))

    angle = math.fabs(theta-heading)
    angle = min(2*math.pi - angle, angle)

    if angle > math.pi/4:
        closestWaypoint += 1
        if closestWaypoint == len(waypoints):
            closestWaypoint = 0

    return closestWaypoint
