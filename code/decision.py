import numpy as np
import math
from pathfinding import astar

def nearest_unexplored(rover):
    """
    Returns the (x, y) coordinates of the closest unexplored point.
    """

    xpos = rover.pos[0]
    ypos = rover.pos[1]

    y_unexplored, x_unexplored = rover.unexplored.nonzero()
    return min(zip(x_unexplored, y_unexplored), key=lambda point: abs(point[0] - xpos) + abs(point[1] - ypos))

def nearest_unexplored_path(rover):
    """
    Returns the angle to the next point on the path to then nearest unexplored area
    :param rover:
    :return:
    """
    unexplored = nearest_unexplored(rover)
    path = astar(rover.search_grid, (int(rover.pos[1]), int(rover.pos[0])), (unexplored[1], unexplored[0]))
    rover.worldmap[:,:,2] = np.zeros_like(rover.worldmap[:,:,2])
    for point in path:
        rover.worldmap[point[0]][point[1]][0] = 255
        rover.worldmap[point[0]][point[1]][1] = 255
        rover.worldmap[point[0]][point[1]][2] = 255
    index = min(8, len(path) - 1) # start a few points away from the current position
    waypoint = (path[index][1], path[index][0]) # skip start point and switch xy
    desired_yaw = math.atan2(waypoint[1] - rover.pos[1], waypoint[0] - rover.pos[0])
    yaw_difference = math.degrees(desired_yaw) - rover.yaw
    while yaw_difference < -180:
        yaw_difference += 360
    while yaw_difference > 180:
        yaw_difference -= 360
    print("\nWaypoint {}, Steering angle {}".format(waypoint,yaw_difference))
    return yaw_difference

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
                steering_angle = nearest_unexplored_path(Rover)
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
    dist = 5
    xpos = int(Rover.pos[0])
    ypos = int(Rover.pos[1])
    Rover.unexplored[ypos-dist:ypos + dist, xpos-dist:xpos + dist] = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover

