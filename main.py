import pybullet as p
import pybullet_data
import time
import heapq
import math


# =========================================================
# START PYBULLET
# =========================================================

p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 240)

p.resetDebugVisualizerCamera(
    cameraDistance=12,
    cameraYaw=45,
    cameraPitch=-65,
    cameraTargetPosition=[0, 0, 0]
)

p.loadURDF("plane.urdf")


# =========================================================
# CREATE WALLS / OBSTACLES
# =========================================================

def create_box(position, size, color):

    collision_shape = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[
            size[0] / 2,
            size[1] / 2,
            size[2] / 2
        ]
    )

    visual_shape = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[
            size[0] / 2,
            size[1] / 2,
            size[2] / 2
        ],
        rgbaColor=color
    )

    return p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=position
    )


# -------------------------
# OUTER WALLS
# -------------------------

create_box(
    [0, 5, 0.5],
    [10, 0.2, 1],
    [0.6, 0.6, 0.6, 1]
)

create_box(
    [0, -5, 0.5],
    [10, 0.2, 1],
    [0.6, 0.6, 0.6, 1]
)

create_box(
    [-5, 0, 0.5],
    [0.2, 10, 1],
    [0.6, 0.6, 0.6, 1]
)

create_box(
    [5, 0, 0.5],
    [0.2, 10, 1],
    [0.6, 0.6, 0.6, 1]
)


# -------------------------
# RED OBSTACLES
# -------------------------

create_box(
    [-1.5, -1.0, 0.5],
    [1.0, 4.0, 1.0],
    [0.8, 0.3, 0.3, 1]
)

create_box(
    [1.5, 1.5, 0.5],
    [1.0, 3.0, 1.0],
    [0.8, 0.3, 0.3, 1]
)

create_box(
    [2.5, -2.5, 0.5],
    [2.0, 1.0, 1.0],
    [0.8, 0.3, 0.3, 1]
)


# =========================================================
# START AND GOAL
# =========================================================

start = (-4, -4)
goal = (4, 4)

start_position = [
    start[0],
    start[1],
    0.15
]

goal_position = [
    goal[0],
    goal[1],
    0.05
]


# Green destination marker
goal_visual = p.createVisualShape(
    p.GEOM_CYLINDER,
    radius=0.5,
    length=0.05,
    rgbaColor=[0.1, 0.9, 0.1, 1]
)

p.createMultiBody(
    baseMass=0,
    baseVisualShapeIndex=goal_visual,
    basePosition=goal_position
)


# =========================================================
# LOAD HUSKY ROBOT
# =========================================================

robot = p.loadURDF(
    "husky/husky.urdf",
    start_position
)


# =========================================================
# FIND WHEEL JOINTS
# =========================================================

left_wheels = []
right_wheels = []

for joint in range(p.getNumJoints(robot)):

    joint_name = p.getJointInfo(
        robot,
        joint
    )[1].decode("utf-8")

    if "left_wheel" in joint_name:
        left_wheels.append(joint)

    if "right_wheel" in joint_name:
        right_wheels.append(joint)


print("Left wheels:", left_wheels)
print("Right wheels:", right_wheels)


# =========================================================
# MOTOR CONTROL
# =========================================================

def set_wheel_speeds(left_speed, right_speed):

    for wheel in left_wheels:

        p.setJointMotorControl2(
            robot,
            wheel,
            p.VELOCITY_CONTROL,
            targetVelocity=left_speed,
            force=1000
        )

    for wheel in right_wheels:

        p.setJointMotorControl2(
            robot,
            wheel,
            p.VELOCITY_CONTROL,
            targetVelocity=right_speed,
            force=1000
        )


# =========================================================
# A* GRID MAP
# =========================================================

grid_min = -4
grid_max = 4


def is_blocked(x, y):

    # Obstacle 1
    if -2.5 <= x <= -0.5 and -3.5 <= y <= 1.5:
        return True

    # Obstacle 2
    if 0.5 <= x <= 2.5 and -0.5 <= y <= 3.5:
        return True

    # Obstacle 3
    if 1.0 <= x <= 4.0 and -3.5 <= y <= -1.5:
        return True

    return False


# =========================================================
# A* PATHFINDING
# =========================================================

def heuristic(a, b):

    return math.sqrt(
        (a[0] - b[0]) ** 2
        +
        (a[1] - b[1]) ** 2
    )


def get_neighbors(node):

    x, y = node

    possible_neighbors = [
        (x + 1, y),
        (x - 1, y),
        (x, y + 1),
        (x, y - 1)
    ]

    valid_neighbors = []

    for nx, ny in possible_neighbors:

        if nx < grid_min or nx > grid_max:
            continue

        if ny < grid_min or ny > grid_max:
            continue

        if is_blocked(nx, ny):
            continue

        valid_neighbors.append(
            (nx, ny)
        )

    return valid_neighbors


def a_star(start_node, goal_node):

    open_list = []

    heapq.heappush(
        open_list,
        (0, start_node)
    )

    came_from = {}

    g_score = {
        start_node: 0
    }

    while open_list:

        current = heapq.heappop(
            open_list
        )[1]

        if current == goal_node:

            path = [current]

            while current in came_from:

                current = came_from[current]
                path.append(current)

            path.reverse()

            return path

        for neighbor in get_neighbors(current):

            new_score = (
                g_score[current] + 1
            )

            if (
                neighbor not in g_score
                or new_score < g_score[neighbor]
            ):

                came_from[neighbor] = current

                g_score[neighbor] = new_score

                priority = (
                    new_score
                    +
                    heuristic(
                        neighbor,
                        goal_node
                    )
                )

                heapq.heappush(
                    open_list,
                    (
                        priority,
                        neighbor
                    )
                )

    return None


# =========================================================
# CALCULATE PATH
# =========================================================

path = a_star(
    start,
    goal
)

if path is None:

    print("ERROR: No path found!")

    p.disconnect()

    quit()


print("A* path found!")
print(path)


# =========================================================
# DRAW BLUE A* PATH
# =========================================================

for i in range(len(path) - 1):

    p.addUserDebugLine(
        [
            path[i][0],
            path[i][1],
            0.15
        ],
        [
            path[i + 1][0],
            path[i + 1][1],
            0.15
        ],
        lineColorRGB=[
            0,
            0,
            1
        ],
        lineWidth=5
    )


# =========================================================
# SIMULATED DISTANCE SENSORS
# =========================================================

def draw_sensors():

    position, orientation = (
        p.getBasePositionAndOrientation(robot)
    )

    yaw = p.getEulerFromQuaternion(
        orientation
    )[2]

    # Front, front-left, front-right
    sensor_angles = [
        yaw,
        yaw + 0.5,
        yaw - 0.5
    ]

    sensor_range = 3.0

    for angle in sensor_angles:

        start_ray = [
            position[0]
            + math.cos(angle) * 0.65,

            position[1]
            + math.sin(angle) * 0.65,

            position[2] + 0.65
        ]

        end_ray = [
            start_ray[0]
            + math.cos(angle) * sensor_range,

            start_ray[1]
            + math.sin(angle) * sensor_range,

            start_ray[2]
        ]

        result = p.rayTest(
            start_ray,
            end_ray
        )[0]

        hit_object = result[0]
        hit_fraction = result[2]

        # Ignore robot detecting itself
        if hit_object == robot:
            hit_fraction = 1.0

        distance = (
            hit_fraction * sensor_range
        )

        # -------------------------
        # SENSOR COLORS
        # -------------------------

        if distance < 1.5:

            # RED = obstacle close
            color = [1, 0, 0]

        elif distance < 2.5:

            # YELLOW = obstacle nearby
            color = [1, 1, 0]

        else:

            # GREEN = clear
            color = [0, 1, 0]

        p.addUserDebugLine(
            start_ray,
            end_ray,
            lineColorRGB=color,
            lineWidth=5,
            lifeTime=0.15
        )


# =========================================================
# ANGLE HELPER
# =========================================================

def normalize_angle(angle):

    while angle > math.pi:
        angle -= 2 * math.pi

    while angle < -math.pi:
        angle += 2 * math.pi

    return angle


# =========================================================
# FOLLOW A* PATH
# =========================================================

waypoint_index = 1
goal_reached = False

step_counter = 0

print()
print("Robot beginning delivery route!")
print("Sensor colors:")
print("GREEN = Clear")
print("YELLOW = Obstacle nearby")
print("RED = Obstacle close")
print()


while True:

    # -------------------------
    # DRAW SENSOR BEAMS
    # -------------------------

    step_counter += 1

    if step_counter % 12 == 0:
        draw_sensors()


    # -------------------------
    # GET ROBOT POSITION
    # -------------------------

    position, orientation = (
        p.getBasePositionAndOrientation(robot)
    )

    x = position[0]
    y = position[1]

    yaw = p.getEulerFromQuaternion(
        orientation
    )[2]


    # -------------------------
    # FOLLOW A* PATH
    # -------------------------

    if not goal_reached:

        target_x = path[
            waypoint_index
        ][0]

        target_y = path[
            waypoint_index
        ][1]

        dx = target_x - x
        dy = target_y - y

        distance = math.sqrt(
            dx ** 2
            +
            dy ** 2
        )

        target_angle = math.atan2(
            dy,
            dx
        )

        angle_error = normalize_angle(
            target_angle - yaw
        )


        # -------------------------
        # WAYPOINT REACHED
        # -------------------------

        if distance < 0.45:

            print(
                "Reached waypoint:",
                path[waypoint_index]
            )

            waypoint_index += 1

            if waypoint_index >= len(path):

                set_wheel_speeds(
                    0,
                    0
                )

                goal_reached = True

                print()
                print(
                    "DELIVERY COMPLETE!"
                )

                print(
                    "SmartPath reached "
                    "the destination."
                )


        # -------------------------
        # TURN TOWARD WAYPOINT
        # -------------------------

        elif abs(angle_error) > 0.25:

            # Faster turning
            turn_speed = 7.0

            if angle_error > 0:

                set_wheel_speeds(
                    -turn_speed,
                    turn_speed
                )

            else:

                set_wheel_speeds(
                    turn_speed,
                    -turn_speed
                )


        # -------------------------
        # DRIVE FORWARD
        # -------------------------

        else:

            # Faster forward speed
            forward_speed = 18.0

            steering = (
                angle_error * 2
            )

            left_speed = (
                forward_speed
                - steering
            )

            right_speed = (
                forward_speed
                + steering
            )

            set_wheel_speeds(
                left_speed,
                right_speed
            )


    # -------------------------
    # RUN PHYSICS
    # -------------------------

    p.stepSimulation()

    # Runs the simulation faster for the video
    time.sleep(1 / 480)