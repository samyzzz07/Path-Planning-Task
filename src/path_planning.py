from __future__ import annotations

from typing import List

from src.models import CarPose, Cone, Path2D
import math

WIDTH = 3.0  
class PathPlanning:
    """Student-implemented path planner.

    You are given the car pose and an array of detected cones, each cone with (x, y, color)
    where color is 0 for yellow (right side) and 1 for blue (left side). The goal is to
    generate a sequence of path points that the car should follow.

    Implement ONLY the generatePath function.
    """

    def __init__(self, car_pose: CarPose, cones: List[Cone]):
        self.car_pose = car_pose
        self.cones = cones

        
    # Used to calculate distance between car and cone to help sort cones
    def generatekey(self,cone):
        return math.hypot(cone[0] - self.car_pose.x, cone[1] - self.car_pose.y)

    
    # A function I implemented to get track direction so the car doesn't divert after not seeing cones
    #  either if cones are on one side or there is one gate only
    def trackDirection(self, blue, yellow):
        # If I have more than 1 cone pair
        for cones in (blue, yellow):
            if len(cones) >= 2:
                direction_x = cones[-1][0] - cones[0][0]
                direction_y = cones[-1][1] - cones[0][1]
                norm = math.hypot(direction_x, direction_y)
                return direction_x / norm, direction_y / norm
        # If I have one cone pair only
        if blue and yellow:
            gatex = blue[0][0] - yellow[0][0]
            gatey = blue[0][1] - yellow[0][1]
            norm = math.hypot(gatex, gatey)
            return gatey / norm, -gatex / norm
        # If no cone pairs
        return math.cos(self.car_pose.yaw), math.sin(self.car_pose.yaw)

    # This function is meant to get the center point between each 2 cones (yellow and blue), called waypoints
    def buildWaypoints(self, blue, yellow):
        yaw = self.car_pose.yaw
        tangentx, tangenty = self.trackDirection(blue, yellow)
        normal_x, normal_y = tangenty, -tangentx     
        n = min(len(blue), len(yellow))
        if n > 0:
            w = sum(abs((blue[i][0] - yellow[i][0]) * normal_x +(blue[i][1] - yellow[i][1]) * normal_y) for i in range(n)) / n
        else:
            w = WIDTH

        waypoints = [(self.car_pose.x, self.car_pose.y)]
        for i in range(max(len(blue), len(yellow))):
            if i < len(blue) and i < len(yellow):
                b = blue[i]
                y = yellow[i]
            elif i < len(blue):                       
                b = blue[i]
                y = (b[0] + w * normal_x, b[1] + w * normal_y)
            else:                                     
                y = yellow[i]
                b = (y[0] - w * normal_x, y[1] - w * normal_y)
            waypoints.append(((b[0] + y[0]) / 2, (b[1] + y[1]) / 2))



        return waypoints

    def generatePath(self) -> Path2D:
        """Return a list of path points (x, y) in world frame.

        Requirements and notes:
        - Cones: color==0 (yellow) are on the RIGHT of the track; color==1 (blue) are on the LEFT.
        - You may be given 2, 1, or 0 cones on each side.
        - Use the car pose (x, y, yaw) to seed your path direction if needed.
        - Return a drivable path that stays between left (blue) and right (yellow) cones.
        - The returned path will be visualized by PathTester.

        The path can contain as many points as you like, but it should be between 5-10 meters,
        with a step size <= 0.5. Units are meters.

        Replace the placeholder implementation below with your algorithm.
        """

        # Default: produce a short straight-ahead path from the current pose.
        # delete/replace this with your own algorithm.
        num_points = 20
        step = 0.5
        cx = self.car_pose.x
        cy = self.car_pose.y


        path: Path2D = []
        yellow_cones = []
        blue_cones = []

        for i in self.cones:
            if i.color == 0:
                yellow_cones.append((i.x, i.y))
            elif i.color == 1:
                blue_cones.append((i.x, i.y))
        blue_cones.sort(key=self.generatekey)
        yellow_cones.sort(key=self.generatekey)

        if not blue_cones and not yellow_cones:
            # Straight line if the car can't see any cones
            for i in range(1, num_points + 1):
                distance_x = math.cos(self.car_pose.yaw) * step * i
                distance_y = math.sin(self.car_pose.yaw) * step * i
                path.append((cx + distance_x, cy + distance_y))
        else:
            waypoints = self.buildWaypoints(blue_cones, yellow_cones)
            for i in range(len(waypoints) - 1):
                x1, y1 = waypoints[i]
                x2, y2 = waypoints[i + 1]
                distance = math.hypot(x2 - x1, y2 - y1)
                n = max(1, math.ceil(distance / step))
                for j in range(n):
                    path.append((x1 + (x2 - x1) * (j / n), y1 + (y2 - y1) * (j / n)))
            path.append(waypoints[-1])
    
            length = 0.0
            for i in range(len(path) - 1):
                length += math.hypot(path[i+1][0] - path[i][0], path[i+1][1] - path[i][1])
            # Used to extend path in case it is less than 5 m
            if length < 5.0:
                unit_x, unit_y = self.trackDirection(blue_cones, yellow_cones)
                if len(waypoints) >= 3:
                    distance_x = waypoints[-1][0] - waypoints[-2][0]
                    distance_y = waypoints[-1][1] - waypoints[-2][1]
                    norm = math.hypot(distance_x, distance_y)
                    unit_x, unit_y = distance_x / norm, distance_y / norm

                while length < 5.0:
                    x, y = path[-1]
                    path.append((x + unit_x * step, y + unit_y * step))
                    length += step
            # Used to shorten path in case it is more than 10 m
            while length > 10.0:
                last = path.pop()
                length -= math.hypot(last[0] - path[-1][0], last[1] - path[-1][1])

        return path
