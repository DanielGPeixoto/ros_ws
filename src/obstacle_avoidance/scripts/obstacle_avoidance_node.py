
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

# Configuration parameters
OBSTACLE_DIST = 0.5      # Threshold for detecting obstacles
REGIONAL_ANGLE = 30      # Angle for each region (degrees)
NORMAL_LIN_VEL = 0.4     # Linear speed when clear
TRANS_LIN_VEL = -0.09    # Linear speed when turning/backing up
TRANS_ANG_VEL = 1.75     # Angular velocity when turning
CYLINDER_RADIUS = 0.12   # Adjusted expected radius of cylindrical obstacles
RADIUS_TOLERANCE = 0.03  # Try increasing to 0.04 or 0.05

# Define min/max radius based on tolerance
MIN_RADIUS = CYLINDER_RADIUS - RADIUS_TOLERANCE
MAX_RADIUS = CYLINDER_RADIUS + RADIUS_TOLERANCE

# Define the regions and their respective weights for deviation calculation
Regions_Report = {
    "front_C": [], "front_L": [], "left_R": [],
    "left_C": [], "left_L": [], "back_R": [],
    "back_C": [], "back_L": [], "right_R": [],
    "right_C": [], "right_L": [], "front_R": [],
}

Regions_Distances = {
    "front_C":  0, "front_L":  1, "left_R":  2,
    "left_C":   3, "left_L":   4, "back_R":  5,
    "back_C":   6, "back_L":  -5, "right_R": -4,
    "right_C": -3, "right_L": -2, "front_R": -1,
}

def convert_to_cartesian(scan_data):
    points = []
    for angle, distance in enumerate(scan_data.ranges):
        if distance < scan_data.range_max and distance > scan_data.range_min:
            angle_rad = np.deg2rad(angle)
            x = distance * np.cos(angle_rad)
            y = distance * np.sin(angle_rad)
            points.append((x, y))
    return points

def fit_circle(points):
    x = np.array([p[0] for p in points])
    y = np.array([p[1] for p in points])

    A = np.array([x, y, np.ones(len(x))]).T
    b = x**2 + y**2

    # Solve for circle parameters (center and radius)
    c, d, e = np.linalg.lstsq(A, b, rcond=None)[0]
    center_x = c / 2
    center_y = d / 2
    radius = np.sqrt(center_x**2 + center_y**2 + e)

    return (center_x, center_y), radius

def detect_cylinder(scan_data, expected_radius, radius_tolerance):
    points = convert_to_cartesian(scan_data)

    # Filter out points farther than slightly larger radius
    points = [p for p in points if np.linalg.norm(p) < 0.3]  # Increase slightly from 0.25 to 0.3

    if not points or len(points) < 10:
        return []

    clusters = [points]  # Assuming one cluster for simplicity
    detected_cylinders = []

    for cluster in clusters:
        center, radius = fit_circle(cluster)

        # Check if the fitted radius is within the allowed range
        if MIN_RADIUS <= radius <= MAX_RADIUS:
            detected_cylinders.append({
                "center": center,
                "radius": radius,
                "points": cluster
            })

    if detected_cylinders:
        rospy.loginfo("Cylinder detected within specified tolerance!")

    return detected_cylinders

def identify_regions(scan):
    """
    Populates the Regions_Report with obstacles (distances) detected in each region.
    """
    REGIONS = [
        "front_C", "front_L", "left_R",
        "left_C", "left_L", "back_R",
        "back_C", "back_L", "right_R",
        "right_C", "right_L", "front_R",
    ]
    
    # Get front center range (from both sides of the front sector)
    intermediary = scan.ranges[:int(REGIONAL_ANGLE/2)] + scan.ranges[-int(REGIONAL_ANGLE/2):]
    Regions_Report["front_C"] = [x for x in intermediary if x <= OBSTACLE_DIST and x != float('inf')]
    
    # Populate the rest of the regions
    for i, region in enumerate(REGIONS[1:], 1):
        start = REGIONAL_ANGLE * i
        end = REGIONAL_ANGLE * (i + 1)
        Regions_Report[region] = [x for x in scan.ranges[start:end] if x <= OBSTACLE_DIST and x != float('inf')]

def clearance_test():
    """
    Determines if the robot should avoid an obstacle and calculates the turning direction.
    """
    goal = "front_C"
    closest = float('inf')
    maxima = {"destination": "back_C", "distance": 0}
    
    for region, distances in Regions_Report.items():
        regional_dist = abs(Regions_Distances[region] - Regions_Distances[goal])
        
        if not distances:  # No obstacles in region
            if regional_dist < closest:
                closest = regional_dist
                maxima["destination"] = region
                maxima["distance"] = OBSTACLE_DIST
        elif max(distances) > maxima["distance"]:
            maxima["distance"] = max(distances)
            maxima["destination"] = region
    
    # Calculate the cost to the chosen orientation
    deviation_cost = Regions_Distances[maxima["destination"]] - Regions_Distances[goal]
    return closest != 0, (deviation_cost / (abs(deviation_cost) if deviation_cost != 0 else 1)) * TRANS_ANG_VEL

def steer(cmd, avoid=False, ang_vel=0):
    """
    Sets linear and angular velocities based on the avoidance decision.
    """
    if avoid:
        cmd.linear.x = TRANS_LIN_VEL
    else:
        cmd.linear.x = NORMAL_LIN_VEL
    cmd.angular.z = ang_vel
    return cmd

def scan_callback(scan_data):
    """
    Main callback function for processing scan data and publishing velocity commands.
    """
    identify_regions(scan_data)
    avoid, ang_vel = clearance_test()

    # Detect any cylindrical objects in the scan data
    cylinders = detect_cylinder(scan_data, CYLINDER_RADIUS, RADIUS_TOLERANCE)
    if cylinders:
        rospy.loginfo("Cylinder detected!")

    cmd = Twist()
    vel_pub.publish(steer(cmd, avoid, ang_vel))

# ROS node setup
rospy.init_node('obstacle_avoider')
vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.Subscriber('/scan', LaserScan, scan_callback)

rospy.spin()

