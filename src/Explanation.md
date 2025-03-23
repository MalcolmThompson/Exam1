# Exam 1
This document is a technical summary of how I approached and solved the challenges using ROS and the turtlesim package. Each section briefly outlines the problem, solution strategy, and the tools used to build the final programs.

## Problem 1: Creating and Deleting Turtles
Goal:
- Remove the default turtle (turtle1).
- Spawn multiple turtles at random positions.

### Initialize ROS Node:

rospy.init_node("turtle_manager")

### Delete turtle1:

rospy.wait_for_service("/kill")
kill = rospy.ServiceProxy("/kill", Kill)
kill("turtle1")

### Spawn Random Turtles:
for i in range(5):
    x = random.uniform(1, 10)
    y = random.uniform(1, 10)
    spawn("turtle" + str(i), x, y, theta=0)

## Problem 2: Turtle Motion Control
Goal:

Make the turtles move around within the simulation area using custom movement logic.
- Subscribe to /pose: To track each turtle’s current location.
- Publish to /cmd_vel: To send movement commands.

### Key Snippet:
def move_turtle(pose):
    vel_msg = Twist()
    vel_msg.linear.x = 2.0
    vel_msg.angular.z = 1.5
    pub.publish(vel_msg)

This exam was a great opportunity to apply core ROS concepts, such as:

- Using services (/spawn, /kill) for dynamic entity management

- Handling topics for subscribing to real-time data and publishing commands