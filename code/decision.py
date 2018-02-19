import numpy as np
import math
from pathfinding import astar
from scipy.ndimage.filters import gaussian_filter
from d_star import DStarNavigator

navigator = DStarNavigator()

def steering_angle_between_points(start, end, current_yaw):
    # Returns the steering angle required to for the rover to face the given point
    # angle is in degrees, and between -180 and 180
    desired_yaw = math.atan2(end[1] - start[1], end[0] - start[0])
    steer_angle = math.degrees(desired_yaw) - current_yaw

    # keep angle between -180 and 180
    while steer_angle < -180:
        steer_angle += 360
    while steer_angle > 180:
        steer_angle -= 360

    return steer_angle


def get_destination(rover):
    """
    Returns the (x, y) coordinates of the closest unexplored point.
    """

    # todo handle when all points are explored

    def manhattan_distance(point):
        # manhattan distance from rover
        return abs(point[0] - rover.pos[0]) + abs(point[1] - rover.pos[1])

    def score(point, rover):
        # returns a score that captures how difficult navigating to the selected point will be
        # takes into account straight line distance, steering angle, and proximity to walls
        distance = math.sqrt((point[0] - rover.pos[0]) ** 2 + (point[1] - rover.pos[1]) ** 2)
        angle = steering_angle_between_points(rover.pos, point, rover.yaw)
        return distance + abs(angle) / 5 + min(rover.worldmap[point[1], point[0], 0], 30)

    y_unexplored, x_unexplored = rover.unexplored.nonzero()
    unexplored_points = [(x, y) for x, y in zip(x_unexplored, y_unexplored)]
    unexplored_points.sort(key=manhattan_distance)  # sort by distance from rover

    # get fifty closest points
    closest_points = unexplored_points[:min(50, len(unexplored_points))]

    # sort again, this time prioritizing points in front of the rover
    closest_points.sort(key=lambda point: score(point, rover))

    # return best point
    return closest_points[0]


def get_steer_angle(rover):
    """
    Returns the angle to the next point on the path to the destination
    :param rover:
    :return:
    """
    destination = get_destination(rover)
    costs = gaussian_filter(rover.worldmap[:,:,0], 0.8)

    path = navigator.find_path((int(rover.pos[1]), int(rover.pos[0])), (destination[1], destination[0]), costs)

    # rover.worldmap[:, :, 2] = np.zeros_like(rover.worldmap[:, :, 2])
    # for point in path:
    #     rover.worldmap[point[0]][point[1]][1] = 255
    #     rover.worldmap[point[0]][point[1]][2] = 255

    index = min(3, len(path) - 1)  # start a few points away from the current position
    waypoint = (path[index][1] + 0.5, path[index][0] + 0.5)  # switch xy and try to move to center of square

    return steering_angle_between_points(rover.pos, waypoint, rover.yaw)


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                # Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                steering_angle = get_steer_angle(Rover)
                if abs(steering_angle) > 30:
                    if Rover.vel > 0.1:
                        # slow down if steering angle is very large
                        Rover.throttle = 0
                        # Set brake to stored brake value
                        Rover.brake = Rover.brake_set
                    else:
                        Rover.throttle = 0
                        Rover.brake = 0 # release brake to allow turning
                    Rover.steer = steering_angle
                else:

                    Rover.steer = steering_angle
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    # update explored area
    dist = 8
    xpos = int(Rover.pos[0])
    ypos = int(Rover.pos[1])
    Rover.unexplored[ypos-dist:ypos + dist, xpos-dist:xpos + dist] = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover

