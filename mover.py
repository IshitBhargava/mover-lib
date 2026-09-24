import math
import time
import carParser
import matplotlib.pyplot as plt

# ---- internal state ----
_theta_calib = 0.0
_x_val = 0.0
_y_val = 0.0
_speed_x = 0.0
_speed_y = 0.0
_tar_x = 0.0
_tar_y = 0.0
_dist_at_beginning = None
_debug = False
_fig = None
_ax = None
_scatter = None
_target_scatter = None
_line = None

# ---- indices / offsets ----
back_index = 2
front_index = 1
right_index = 3
left_index = 0

left_offset = 125.25
right_offset = 125.25
front_offset = 117
back_offset = 177.5

width = 1181
length = 1143

# ---- tunables ----
base_speed = 100
MIN_SPEED = 15
STOP_THRESHOLD = 20


def _convert(dist_list, theta_deg):
    return [d * math.cos(math.radians(theta_deg)) for d in dist_list]


def _apply_offsets(array):
    result = list(array)
    result[left_index] += left_offset
    result[right_index] += right_offset
    result[front_index] += front_offset
    result[back_index] += back_offset
    return result


def calibrate(samples=1000):
    """Call once after carParser.init(), before move(). Averages IMU yaw for a zero-reference."""
    global _theta_calib
    total = 0
    for _ in range(samples):
        imu = carParser.getIMU()
        total += imu[8]
    _theta_calib = total / samples
    print("theta_calib:", _theta_calib)
    return _theta_calib


def setBasespeed(speed):
    global base_speed
    base_speed = speed


def setMinSpeed(speed):
    global MIN_SPEED
    MIN_SPEED = speed


def setThreshold(thresh):
    global STOP_THRESHOLD
    STOP_THRESHOLD = thresh


def pos():
    return [_x_val, _y_val]


def speeds():
    return [_speed_x, _speed_y]


def atTarget():
    dx = _tar_x - _x_val
    dy = _tar_y - _y_val
    return math.hypot(dx, dy) < STOP_THRESHOLD


def stop():
    carParser.move(0, 0, 0)


def Debug(enable):
    global _debug, _fig, _ax, _scatter, _target_scatter, _line
    _debug = enable
    if enable and _fig is None:
        plt.ion()
        _fig, _ax = plt.subplots()
        _ax.set_xlim(0, width)
        _ax.set_ylim(0, length)
        _scatter = _ax.scatter([0], [0], color='blue', label='robot')
        _target_scatter = _ax.scatter([0], [0], color='red', marker='x', s=100, label='target')
        _line, = _ax.plot([0, 0], [0, 0], 'g--', linewidth=1)
        _ax.legend()


def _updatePlot():
    if not _debug or _fig is None:
        return
    _scatter.set_offsets([[_x_val, _y_val]])
    _target_scatter.set_offsets([[_tar_x, _tar_y]])
    _line.set_data([_x_val, _tar_x], [_y_val, _tar_y])
    _fig.canvas.draw()
    _fig.canvas.flush_events()
    plt.pause(0.01)


def _step():
    """One iteration: read sensors, compute pos, send a move command. Internal use."""
    global _x_val, _y_val, _speed_x, _speed_y, _dist_at_beginning

    dist = carParser.getDIST()
    imu = carParser.getIMU()
    theta_deg = imu[8]

    theta = theta_deg - _theta_calib
    dists = _apply_offsets(_convert(dist, theta))

    w_left = 1 / dists[left_index]
    w_right = 1 / dists[right_index]
    w_front = 1 / dists[front_index]
    w_back = 1 / dists[back_index]

    x_from_left = dists[left_index]
    x_from_right = width - dists[right_index]
    y_from_front = length - dists[front_index]
    y_from_back = dists[back_index]

    _x_val = (w_left * x_from_left + w_right * x_from_right) / (w_left + w_right)
    _y_val = (w_front * y_from_front + w_back * y_from_back) / (w_front + w_back)

    theta_rad = -math.radians(theta)   # CW-IMU -> CCW-atan2 sign flip

    dx = _tar_x - _x_val
    dy = _tar_y - _y_val
    dist_to_target = math.hypot(dx, dy)

    if _dist_at_beginning is None:
        _dist_at_beginning = dist_to_target

    target_theta = math.atan2(dy, dx)
    turn_theta = target_theta - theta_rad
    turn_theta = (turn_theta + math.pi) % (2 * math.pi) - math.pi

    if dist_to_target < STOP_THRESHOLD:
        _speed_x, _speed_y = 0, 0
        carParser.move(0, 0, 0)
    else:
        speed = (dist_to_target / _dist_at_beginning) * base_speed
        speed = min(speed, base_speed)
        speed = max(speed, MIN_SPEED)
        _speed_x = speed * math.cos(turn_theta)
        _speed_y = speed * math.sin(turn_theta)
        carParser.move(_speed_x, _speed_y, 0)

    _updatePlot()
    return dist_to_target


def move(x_tar, y_tar, poll_delay=0.02):
    """Blocks until (x_tar, y_tar) is reached. carParser.init() and calibrate() must be called first."""
    global _tar_x, _tar_y, _dist_at_beginning
    _tar_x, _tar_y = x_tar, y_tar
    _dist_at_beginning = None   # fresh ramp for this target
    while True:
        _step()
        if atTarget():
            stop()
            return pos()
        time.sleep(poll_delay)