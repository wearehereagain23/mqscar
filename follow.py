"""
MQScar Curve Mode (follow.py, part of the combined MQScar add-on) = MQScar Follow v25 - v24 plus a 100 km/h speed limit on markers with Reverse checked (values above 100 are overridden, lower values are kept) and a wheelspin launch on the FIRST marker when its speed is high (rear wheels spin on the spot, then the car pulls away). v24 = v23 plus smooth reverse (drivable reverse line with a gradual turn-in after the gear change, front wheels follow the front axle path) and a realistic drift flip (full-length spin, speed fades, body roll and nose dip, short pause, then accelerates again). Forward driving and marker speed logic are unchanged.

Launch Control does not steer a physics car toward its curve. It puts the car on the curve and
animates the wheels from it, so the car is always exactly in the middle of the line.
This script does the same with your MQScar car:

  * ONE green line goes through your pins.
  * The point between the two FRONT tires is on the line, and so is the point between the two
    REAR tires, so the whole car lies along the line and turns with it.
  * The front wheels steer from how sharply the line bends (inner wheel turns more), and every
    wheel spins with the distance driven.
  * The car speeds up on straights, slows for sharp bends and stops exactly on the last pin.

HOW TO USE (Text Editor: open this file, press Alt+P)
  1. Generate the rig with MQScar as usual and choose its Ground Collection.
  2. Place route markers with Alt+Left-click. Snapping is restricted to meshes in that Ground Collection.
  3. Alt+Left-click to add markers. Click a marker to show only its speed and brake-hold controls.
  4. Press X or Delete with a marker selected to remove it; remaining markers renumber automatically.
  5. Change Marker Number to insert/reorder a marker in the route. Bake again after edits.

The ordered marker objects are the route source of truth. Baking reprojects them onto the selected
Ground Collection and rebuilds the connected green route. Markers can be shown/hidden manually;
a small Blender timer hides them during timeline playback and restores the manual visibility
preference when playback stops. Marker speeds define route targets. Each transition begins halfway between consecutive markers and reaches the next target at that marker. Acceleration and braking strengths shape physically reachable motion; marker speeds have no fixed upper limit. The car animation is a baked snapshot.
"""
import bpy
from bisect import bisect_left
from math import atan, atan2, sin, cos, tan, radians, pi, hypot, sqrt, ceil
from bpy.props import PointerProperty, StringProperty, BoolProperty, FloatProperty, IntProperty
from bpy.types import Panel, Operator, PropertyGroup
from mathutils import Matrix, Vector
from bpy_extras import view3d_utils
from bpy_extras.view3d_utils import location_3d_to_region_2d
from mathutils.geometry import intersect_line_plane, interpolate_bezier

# Names and values that come from the MQScar add-on (keep in sync with its __init__.py)
RIG_COLLECTION = "MQScar_Rig"
WHEEL_ATTRS = ("wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr")
FRONT_ATTRS = ("wheel_fl", "wheel_fr")
TAG_HUB = "mqscar_hub"
HOME_KEY = "mq_home"       # saved start pose (flat list of 16 numbers) on body, wheels and hubs
RUN_KEY = "mq_run"         # the raised start pose MQScar's Reset Car uses for the body

# Fixed values (edit here if needed)
MAX_WHEEL_ANGLE = radians(50.0)    # the front wheels never steer more than this
WHEEL_RATE = radians(150.0)        # the steering wheels turn at most this fast (deg/s): smooth
A_LAT = 4.0                        # m/s^2 of sideways acceleration: sets the speed in bends
MIN_CURVE_SPEED = 2.0              # m/s, never slower than this because of a bend
CURVE_LOOKAHEAD = 6.0              # metres ahead that are checked for a sharp bend
ACCEL = 2.5                        # m/s^2
DECEL = 3.0                        # m/s^2 (also used to stop exactly on the last pin)
CREEP = 0.5                        # m/s, crawl floor so a slow-down into a stop always arrives
LAUNCH_DELAY = 0.3                 # s, the car's reaction time before it pulls away (start and after each stop)
LAUNCH_TIME = 2.0                  # s, time to build up to the next marker's speed. Acceleration
                                   # is therefore proportional to the next marker's speed:
                                   # high value = hard pull away, low value = gentle pull away
LAUNCH_SPIN_MIN_KMH = 100.0        # first marker at or above this speed = wheelspin launch. Reaching 100 km/h in
                                   # LAUNCH_TIME (2 s) is about 1.4 g, more than road tyres can grip, so the
                                   # rear wheels really would spin. Below it the start is the normal smooth one.
LAUNCH_SPIN_DELAY = 0.4            # s, the car stays put while the rear wheels spin up (replaces LAUNCH_DELAY)
LAUNCH_SPIN_RATE = 35.0            # rad/s, extra rear-wheel spin at its peak (about 335 rpm)
LAUNCH_SPIN_FADE = 0.3             # fraction of the launch ramp over which the wheelspin dies away as the tyres bite
MAX_SECONDS = 180                  # give up after this long

# NOTE: names must NOT start with "MQScar_" (MQScar's Generate/Clear Rig deletes those)
PIN_PREFIX = "MQFollow_Pin_"
LEGACY_PIN = "MQFollow_Pin"
PATH_NAME = "MQFollow_Route"
SPEED_GRAPH_NAME = "MQFollow_SpeedSegments"
SPEED_KEY_PREFIX = "MQFollow_SpeedKey_"
SPEED_KEY = "mqfollow_speed_key"
SPEED_DISTANCE = "mqfollow_speed_distance"
SPEED_VALUE = "mqfollow_speed_kmh"
SPEED_ID = "mqfollow_speed_id"
DEFAULT_SPEED_KMH = 60.0
SPEED_GRAPH_SCALE = 0.025  # metres of graph height per km/h
PIN_KEY = "mqfollow_pin"
PIN_SMOOTH = "mqfollow_smoothness"
PIN_SPEED = "mqfollow_target_speed_kmh"
PIN_BRAKE_HOLD = "mqfollow_brake_hold_seconds"
REVERSE_MARK = "mqfollow_reverse_mark"   # checkbox on a marker: from this marker on, the gear is switched
                                          # (forward -> reverse, or reverse -> forward) until the next marked marker
DRIFT_MIN_KMH = 100.0                     # gear change at or above this speed = drift spin, no stop
DRIFT_SPIN_TIME = 0.7                     # s, the longest spin (shorter at very high speeds)
GEAR_PAUSE = 0.5                          # s, minimum pause when the car changes between forward and reverse
# Drift flip recovery (only used by the high-speed drift; stop-and-go and forward driving ignore these)
DRIFT_EXIT_FRACTION = 0.7                 # speed at the end of the spin as a fraction of the speed it entered with
DRIFT_HOLD_TIME = 0.10                    # s, short pause at the lowest speed after the spin (0.05 - 0.10 looks natural)
DRIFT_REACCEL_TIME = 1.2                  # s, time to build back up to the next marker's speed after the pause
DRIFT_ROLL = radians(4.0)                 # body lean during the drift flip (0 = none)
DRIFT_PITCH = radians(2.0)                # nose dip during the drift flip (0 = none)
REVERSE_MAX_KMH = 100.0                   # km/h, speed limit for markers that are in reverse (lower values are kept)
REVERSE_WHEEL_RATE = radians(240.0)       # deg/s, reverse steering can move a bit faster so the wheels keep up with the front axle
OLD_OBJECTS = ("MQFollow_FrontTrack", "MQFollow_RearTrack", "MQFollow_FrontMark")  # from v6
PARTS_KEY = "mqfollow_parts"       # names of the objects this script keyframed
OWN_ACTION = "mqf_own_action"

# Jump feature (integrated from the tested standalone override)
JUMP_FLAG = "mqfollow_jump_mark"
JUMP_AUTO = "mqfollow_jump_auto"
JUMP_HEIGHT = "mqfollow_jump_height"
JUMP_TILT = "mqfollow_jump_tilt"
JUMP_GRAVITY = 9.81
JUMP_MIN_AIR_FRAMES = 3
JUMP_MAX_AUTO_HEIGHT = 6.0
JUMP_COLOR = (1.0, 0.1, 0.9, 1.0)
JUMP_FIND_TOL = 1.5


# --------------------------------------------------------------------------
# Small 4x4 matrix helpers in plain Python (rows of 4 numbers), easy to test
# --------------------------------------------------------------------------
def m_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def m_tr(x, y, z):
    return [[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]]


def m_rot(axis, a):
    """Rotation by angle a about a unit axis (Rodrigues)."""
    x, y, z = axis
    c, s = cos(a), sin(a)
    t = 1.0 - c
    return [[t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0],
            [0, 0, 0, 1]]


def m_about(center, rotation):
    """Apply a rotation matrix about a point."""
    cx, cy, cz = center
    return m_mul(m_tr(cx, cy, cz), m_mul(rotation, m_tr(-cx, -cy, -cz)))


def m_pos(m):
    return (m[0][3], m[1][3], m[2][3])


def wrap(a):
    return atan2(sin(a), cos(a))


# --------------------------------------------------------------------------
# The line
# --------------------------------------------------------------------------
def smooth_curve(start, heading, pins, spacing=0.25, pin_angles=None, tension=1.0, resolution=16):
    """Build a cubic Hermite route through every pin, then resample by distance.

    Pin rotations can optionally define the tangent direction at each pin.  This gives
    users a practical way to adjust the curve's entry/exit angle without moving the pin.
    """
    pts = [start]
    accepted_angles = []
    for idx, q in enumerate(pins):
        if hypot(q[0] - pts[-1][0], q[1] - pts[-1][1]) > 0.05:
            pts.append(q)
            accepted_angles.append(pin_angles[idx] if pin_angles is not None and idx < len(pin_angles) else None)
    if len(pts) < 2:
        return pts

    tension = max(0.05, min(float(tension), 2.0))
    resolution = max(4, min(int(resolution), 128))
    seg_lens = [max(hypot(b[0]-a[0], b[1]-a[1]), 0.05) for a, b in zip(pts, pts[1:])]
    tangents = []
    for i, pt in enumerate(pts):
        if i == 0:
            ang = heading
            scale = seg_lens[0]
        elif i == len(pts)-1:
            ang = atan2(pt[1]-pts[i-1][1], pt[0]-pts[i-1][0])
            scale = seg_lens[-1]
        else:
            dx, dy = pts[i+1][0]-pts[i-1][0], pts[i+1][1]-pts[i-1][1]
            ang = atan2(dy, dx)
            scale = min(seg_lens[i-1], seg_lens[i])
        # pin_angles aligns with the original pins; start has no pin rotation.
        if i > 0 and i-1 < len(accepted_angles) and accepted_angles[i-1] is not None:
            ang = accepted_angles[i-1]
        tangents.append((cos(ang) * scale * tension, sin(ang) * scale * tension))

    dense = [pts[0]]
    for i, (a, b) in enumerate(zip(pts, pts[1:])):
        m0, m1 = tangents[i], tangents[i+1]
        count = max(resolution, int(seg_lens[i] * resolution))
        for j in range(1, count+1):
            t = j / count
            t2, t3 = t*t, t*t*t
            h00, h10 = 2*t3-3*t2+1, t3-2*t2+t
            h01, h11 = -2*t3+3*t2, t3-t2
            x = h00*a[0] + h10*m0[0] + h01*b[0] + h11*m1[0]
            y = h00*a[1] + h10*m0[1] + h01*b[1] + h11*m1[1]
            dense.append((x, y))

    out = [dense[0]]
    carried = 0.0
    for a, b in zip(dense, dense[1:]):
        seg = hypot(b[0]-a[0], b[1]-a[1])
        if seg < 1e-9:
            continue
        pos = spacing - carried
        while pos <= seg:
            f = pos / seg
            out.append((a[0] + (b[0]-a[0])*f, a[1] + (b[1]-a[1])*f))
            pos += spacing
        carried = seg - (pos - spacing)
    if hypot(out[-1][0]-pts[-1][0], out[-1][1]-pts[-1][1]) > 1e-6:
        out.append(pts[-1])
    return out


def build_line(rear, heading, wb, pins, pin_angles=None, tension=1.0, resolution=16):
    """The line the car drives along. It starts at the rear axle point 'rear', runs straight for
    one wheelbase (so the front axle starts ON it, exactly where the car already stands), and
    then curves through the pins. Returns (points, s) with s = distance along the line."""
    fx, fy = rear[0] + wb * cos(heading), rear[1] + wb * sin(heading)
    n = max(1, int(ceil(wb / 0.25)))
    lead = [(rear[0] + (fx - rear[0]) * i / n, rear[1] + (fy - rear[1]) * i / n) for i in range(n)]
    tail = smooth_curve((fx, fy), heading, pins, pin_angles=pin_angles, tension=tension, resolution=resolution)
    pts = lead + tail
    s = [0.0]
    for a, b in zip(pts, pts[1:]):
        s.append(s[-1] + hypot(b[0] - a[0], b[1] - a[1]))
    return pts, s


def point_at(pts, s, d):
    """Point on the line at distance d along it."""
    d = max(0.0, min(s[-1], d))
    i = min(max(bisect_left(s, d), 1), len(s) - 1)
    seg = s[i] - s[i - 1]
    f = 0.0 if seg < 1e-12 else (d - s[i - 1]) / seg
    a, b = pts[i - 1], pts[i]
    return (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f)


def rear_point(pts, s, d_front, wb):
    """Point on the line one wheelbase (straight-line distance) behind the front axle point."""
    pf = point_at(pts, s, d_front)
    i = min(max(bisect_left(s, d_front), 1), len(s) - 1)
    d_next = hypot(pts[i][0] - pf[0], pts[i][1] - pf[1])
    for k in range(i - 1, -1, -1):
        d_k = hypot(pts[k][0] - pf[0], pts[k][1] - pf[1])
        if d_k >= wb:
            if d_next >= d_k - 1e-12:
                return pts[k]
            t = (d_k - wb) / (d_k - d_next)
            return (pts[k][0] + (pts[k + 1][0] - pts[k][0]) * t,
                    pts[k][1] + (pts[k + 1][1] - pts[k][1]) * t)
        d_next = d_k
    return pts[0]


def rear_dist(pts, s, d_front, wb):
    """Distance along the line of the point rear_point() returns (same search, returns the distance)."""
    pf = point_at(pts, s, d_front)
    i = min(max(bisect_left(s, d_front), 1), len(s) - 1)
    d_next = hypot(pts[i][0] - pf[0], pts[i][1] - pf[1])
    for k in range(i - 1, -1, -1):
        d_k = hypot(pts[k][0] - pf[0], pts[k][1] - pf[1])
        if d_k >= wb:
            if d_next >= d_k - 1e-12:
                return s[k]
            t = (d_k - wb) / (d_k - d_next)
            return s[k] + (s[k + 1] - s[k]) * t
        d_next = d_k
    return 0.0


def heading_at(pts, s, d, half=0.3):
    """Direction the line points in at distance d (central difference over +-half metres)."""
    a = point_at(pts, s, d - half)
    b = point_at(pts, s, d + half)
    if hypot(b[0] - a[0], b[1] - a[1]) < 1e-9:
        return 0.0
    return atan2(b[1] - a[1], b[0] - a[0])


def resample_path(pts, step=0.2):
    """Evenly spaced points along a polyline (first and last point are kept exactly)."""
    s = [0.0]
    for a, b in zip(pts, pts[1:]):
        s.append(s[-1] + hypot(b[0] - a[0], b[1] - a[1]))
    total = s[-1]
    if total < 1e-9:
        return list(pts)
    n = max(2, int(ceil(total / step)))
    out = [pts[0]]
    for i in range(1, n):
        out.append(point_at(pts, s, total * i / n))
    out.append(pts[-1])
    return out


def drivable_reverse_line(start, heading, pins, wb, step=0.1, max_len=400.0):
    """v24: a smooth, drivable line for a leg that starts from rest after a gear change.

    start = the front axle point, heading = the direction the car will travel (it backs up that way),
    pins = the marker positions this leg must visit in order. The line starts with the straight lead-in
    (one wheelbase, so the car's real pose is exactly respected), then the REAR axle (which leads when
    reversing) is steered toward the markers like a real car: curvature never exceeds what the front
    wheels can do (85% of MAX_WHEEL_ANGLE), and the steering changes gradually (no snap turns), so a
    marker placed beside the car gives a wide, smooth swing instead of a fold in the route.
    Returns the list of (x, y) points, or None if the markers cannot be reached."""
    lead_n = max(1, int(ceil(wb / 0.25)))
    pts = [(start[0] + wb * cos(heading) * i / lead_n, start[1] + wb * sin(heading) * i / lead_n)
           for i in range(lead_n + 1)]
    x, y = pts[-1]
    th = heading                                   # direction the rear axle travels
    kmax = tan(MAX_WHEEL_ANGLE * 0.85) / wb        # tightest bend the car can make, 1/m
    kdot = 0.35                                    # how fast the bend may tighten, 1/m per metre driven
    targets = [(float(px), float(py)) for px, py in pins]
    # polyline the car follows: where it is now, then every marker
    poly = [(x, y)] + targets
    seg = 0
    kappa = 0.0
    look = max(1.5, wb * 1.1)
    travelled = 0.0
    for _ in range(int(max_len / step)):
        # advance to the segment whose end the car has not yet passed
        while seg < len(poly) - 2:
            ax, ay = poly[seg]
            bx, by = poly[seg + 1]
            sl = hypot(bx - ax, by - ay)
            along = ((x - ax) * (bx - ax) + (y - ay) * (by - ay)) / max(sl, 1e-9)
            if hypot(bx - x, by - y) < look * 0.8 or along > sl:
                seg += 1
            else:
                break
        ax, ay = poly[seg]
        bx, by = poly[seg + 1]
        sl = max(hypot(bx - ax, by - ay), 1e-9)
        ux, uy = (bx - ax) / sl, (by - ay) / sl
        along = (x - ax) * ux + (y - ay) * uy
        tgt_along = min(sl, max(along, 0.0) + look)
        tx, ty = ax + ux * tgt_along, ay + uy * tgt_along
        if seg == len(poly) - 2 and hypot(bx - x, by - y) <= look:
            tx, ty = bx, by                        # last stretch: aim straight at the final marker
        alpha = wrap(atan2(ty - y, tx - x) - th)
        dist_t = max(hypot(tx - x, ty - y), 0.3)
        want = max(-kmax, min(kmax, 2.0 * sin(alpha) / dist_t))
        if abs(alpha) > pi / 2.0:                  # target is behind the travel direction: turn as hard as possible
            want = kmax if alpha > 0 else -kmax
        kappa += max(-kdot * step, min(kdot * step, want - kappa))
        th += kappa * step
        x += step * cos(th)
        y += step * sin(th)
        pts.append((x, y))
        travelled += step
        fx, fy = targets[-1]
        if hypot(fx - x, fy - y) < 0.2 or (seg == len(poly) - 2 and
                ((x - poly[-2][0]) * (fx - poly[-2][0]) + (y - poly[-2][1]) * (fy - poly[-2][1]))
                >= (hypot(fx - poly[-2][0], fy - poly[-2][1]) ** 2)):
            pts[-1] = (fx, fy)                      # land exactly on the final marker
            return pts
    return None


def curvatures(pts, s, half=8):
    """Signed curvature (1/m, + = turning LEFT) at every point of the line."""
    n = len(pts)
    psi = []
    for i in range(n):
        a, b = (i, i + 1) if i + 1 < n else (i - 1, i)
        psi.append(atan2(pts[b][1] - pts[a][1], pts[b][0] - pts[a][0]) if n > 1 else 0.0)
    out = []
    for i in range(n):
        i0, i1 = max(0, i - half), min(n - 1, i + half)
        ds = max(s[i1] - s[i0], 1e-3)
        out.append(wrap(psi[i1] - psi[i0]) / ds)
    return out


# --------------------------------------------------------------------------
# The drive: where the car is on every frame
# --------------------------------------------------------------------------
def _speed_target_at_distance(speed_keys, distance, acceleration_strength, braking_strength,
                              default_speed_ms, current_speed=0.0, start_distance=0.0):
    """Marker target envelope with transitions beginning halfway to the next marker.

    A marker's speed is held from that marker until the midpoint of the following
    segment. From the midpoint to the next marker, a smoothstep blend moves toward
    the next marker's target. Physical acceleration/braking limits are then applied
    so the target curve does not demand impossible instantaneous changes.

    Speed keys are (distance metres, speed km/h).
    """
    accel = max(0.1, float(acceleration_strength))
    brake = max(0.1, float(braking_strength))
    if not speed_keys:
        return max(0.0, default_speed_ms)
    keys = sorted((max(0.0, float(d)), max(0.0, float(v)) / 3.6) for d, v in speed_keys)
    if distance < keys[0][0]:
        # Before the first marker the target is simply that marker's speed (no acceleration cap).
        return keys[0][1]
    if distance >= keys[-1][0]:
        return keys[-1][1]
    for (d0, v0), (d1, v1) in zip(keys, keys[1:]):
        if d0 <= distance <= d1:
            span = max(1e-8, d1 - d0)
            if v0 <= 1e-9:
                # Leaving a stop marker: the next marker's speed is the target straight away.
                # How fast the car gets there is handled by the launch ramp in plan_drive.
                return v1
            transition_start = d0 + 0.5 * span
            if distance <= transition_start or abs(v1 - v0) < 1e-9:
                return v0
            t = max(0.0, min(1.0, (distance - transition_start) / (d1 - transition_start)))
            if v1 <= 1e-9 and v0 > v1:
                # Slowing into a stop marker: steady braking that ends exactly on the marker.
                # The braking strength comes from the markers (speed and distance), not a cap.
                return sqrt(max(0.0, v1 * v1 + (v0 * v0 - v1 * v1) * (1.0 - t)))
            # Smoothstep: zero slope at both ends, reaching the next target at its marker.
            eased = t * t * (3.0 - 2.0 * t)
            requested = v0 + (v1 - v0) * eased
            if v1 > v0:
                # Speeding up: follow the marker blend directly. No acceleration limit (v18).
                return requested
            # v19: slowing down also follows the marker blend directly. No braking limit.
            return requested
    return keys[-1][1]


def _marker_response_rates(speed_keys, distance, acceleration_strength, braking_strength,
                           default_accel, default_decel, current_speed=0.0):
    """Return physical acceleration and braking limits in m/s^2."""
    return max(0.1, float(acceleration_strength)), max(0.1, float(braking_strength))


def plan_drive(pts, s, wb, cruise_ms, dt, track, max_steps, speed_keys=None, stop_holds=None,
               acceleration_strength=3.0, braking_strength=6.0, reverse=False, steer0=None,
               launch_time=None, v_init=0.0, launch_delay=0.0, spin_launch=False):
    """Plan motion using marker speed targets, tunable response times, and exact timed stops."""
    # Route curvature still controls steering, but it must not silently cap marker speed targets.
    kappa = curvatures(pts, s)
    total = s[-1]
    profile = sorted([(max(0.0, float(d)), max(0.0, float(v))) for d, v in (speed_keys or [])], key=lambda x: x[0])
    stops = [d for d, v in profile if v <= 1e-4 and wb - 1e-4 <= d <= total + 1e-4]
    stop_holds = stop_holds or {}
    peak_speed_ms = max([max(0.0, v) / 3.6 for _d, v in profile] or [cruise_ms])
    # Marker targets are authoritative at the route end. The old planner always
    # assumed a zero-speed endpoint, which silently capped higher marker speeds.
    end_target_ms = (max(0.0, float(profile[-1][1])) / 3.6) if profile else max(0.0, float(cruise_ms))
    default_accel_ms2 = max(0.1, float(acceleration_strength))
    default_decel_ms2 = max(0.1, float(braking_strength))
    hold_remaining = 0
    launching = True        # the car pulls away from rest at the start and again after every stop
    launch_age = 0.0
    spin_active = bool(spin_launch) and v_init <= 1e-6      # v25: wheelspin launch (first start, and after each stop hold)
    slip_angle = 0.0                                        # extra rear-wheel rotation so far (radians)
    sf = wb
    v = max(0.0, float(v_init))     # carried speed when a spin hands over to the next leg
    ltime = max(0.05, float(launch_time)) if launch_time else LAUNCH_TIME
    steer = 0.0 if steer0 is None else float(steer0)
    frames = []
    arrived = False
    for step in range(max_steps):
        rem = total - sf
        drive_dist = max(0.0, sf - wb)
        pf = point_at(pts, s, sf)
        pr = rear_point(pts, s, sf, wb)
        yaw = atan2(pf[1] - pr[1], pf[0] - pr[0])
        if reverse:
            # The line is followed by the REAR axle; the front axle trails behind it and the
            # body points opposite to the direction of travel.
            pf, pr = pr, pf
            yaw = wrap(yaw + pi)
        i_mid = min(max(bisect_left(s, max(sf - 0.5 * wb, 0.0)), 0), len(s) - 1)
        if reverse:
            # v24: while reversing, the steered FRONT axle trails along the line. Point the front
            # wheels exactly along the line at the front axle, relative to the body. The wheels then
            # roll the way the front axle is really moving, so the car does not drag or slide sideways.
            d_front_axle = rear_dist(pts, s, sf, wb)
            chord = wrap(yaw + pi)                      # direction of travel along the chord (yaw is already flipped)
            raw = wrap(heading_at(pts, s, d_front_axle) - chord)
            rate = REVERSE_WHEEL_RATE
        else:
            raw = atan(wb * kappa[i_mid])
            rate = WHEEL_RATE
        raw = max(-MAX_WHEEL_ANGLE, min(MAX_WHEEL_ANGLE, raw))
        if step > 0 or steer0 is not None:
            steer += max(-rate * dt, min(rate * dt, raw - steer))
        t = tan(steer)
        if abs(t) < 1e-6:
            dl = dr = 0.0
        else:
            r = wb / t
            dl = atan(wb / (r - track / 2.0))
            dr = atan(wb / (r + track / 2.0))
        frames.append(dict(front=pf, rear=pr, yaw=yaw, steer_left=dl, steer_right=dr,
                           dist=drive_dist, speed=v, steer=steer, gear=(-1 if reverse else 1),
                           slip_spin=slip_angle))
        # Keep the vehicle fixed at a stop marker for the requested number of frames,
        # including when the final marker is also the end of the route.
        if hold_remaining > 0:
            hold_remaining -= 1
            v = 0.0
            continue
        if rem <= 1e-4:
            arrived = True
            break

        j = min(max(bisect_left(s, sf), 0), len(s) - 1)
        acceleration_ms2, deceleration_ms2 = _marker_response_rates(
            profile, sf, acceleration_strength, braking_strength, default_accel_ms2, default_decel_ms2, v)
        predicted_distance = sf + max(v * dt, 0.01)
        # Only brake toward zero when the final marker requests zero. Otherwise
        # preserve its target speed while calculating the distance needed to reach it.
        terminal_remaining = max(0.0, rem - max(0.0, v * dt))
        terminal_limit = float('inf')   # v19: no braking envelope toward the route end
        profile_target = (_speed_target_at_distance(profile, predicted_distance, acceleration_strength,
                                                     braking_strength, cruise_ms, v, wb)
                          if profile else cruise_ms)
        target = min(terminal_limit, profile_target)
        # At a zero-speed key, begin the next segment's acceleration after the timed hold.
        at_stop = any(abs(sf - d) <= max(1e-4, dt*0.02) for d in stops)
        next_stop = next((d for d in stops if d > sf + 1e-5), None)
        if at_stop and v <= 1e-4:
            launch_lookahead = 0.05
            future_profile = (_speed_target_at_distance(profile, sf + launch_lookahead,
                                                        acceleration_strength, braking_strength,
                                                        cruise_ms, v, wb)
                              if profile else cruise_ms)
            acceleration_ms2, deceleration_ms2 = _marker_response_rates(
                profile, sf + launch_lookahead, acceleration_strength, braking_strength,
                default_accel_ms2, default_decel_ms2, v)
            target = min(future_profile, terminal_limit)
        # v19: no braking distance limit before stop markers. The car lands on the stop marker
        # exactly (see below) at whatever speed the marker blend gives it.
        # A small crawl floor stops the car freezing/creeping forever where a blend reaches zero.
        target = max(target, CREEP)
        if launching:
            # Launch ramp: short reaction delay, then ease up to the marker-defined target.
            # The ramp scales the target, so it never lowers the speed you typed: it only
            # decides how quickly the car gets there. Higher next-marker speed = harder pull.
            if spin_active:
                d_launch = LAUNCH_SPIN_DELAY
            else:
                d_launch = LAUNCH_DELAY if v_init <= 1e-6 else max(0.0, float(launch_delay))
            x = (launch_age - d_launch) / ltime
            if spin_active:
                # v25: rear wheels spin up while the car holds still, then the spin fades out as the
                # tyres bite and the car pulls away. Capped so the wheel never turns more than ~2 rad per frame.
                if launch_age < d_launch:
                    u = max(0.0, min(1.0, launch_age / max(d_launch, 1e-6)))
                    rate = LAUNCH_SPIN_RATE * u * u * (3.0 - 2.0 * u)
                else:
                    u = max(0.0, min(1.0, x / max(LAUNCH_SPIN_FADE, 1e-6)))
                    rate = LAUNCH_SPIN_RATE * (1.0 - u * u * (3.0 - 2.0 * u))
                slip_angle += (-1.0 if reverse else 1.0) * min(rate, 2.0 / max(dt, 1e-6)) * dt   # wheels spin backwards in reverse
            xc = max(0.0, min(1.0, x))
            f_ramp = xc * xc * (3.0 - 2.0 * xc)
            target = v_init + (target - v_init) * f_ramp if v_init > 1e-6 else target * f_ramp
            launch_age += dt
            if x >= 1.0:
                launching = False
        speed_before_step = v
        # v19: speed is simply the marker-defined target. No acceleration or braking limits.
        v = target
        # Trapezoidal integration keeps the braking distance consistent with the selected
        # transition time and avoids arriving at a stop marker with leftover speed.
        step_distance = max(0.0, 0.5 * (speed_before_step + v) * dt)

        # Land exactly on a stop key instead of stepping past it.
        next_event = next_stop
        if next_event is not None and sf + step_distance >= next_event - 1e-6:
            sf = min(total, next_event)
            v = 0.0
            hold_seconds = max(0.0, float(stop_holds.get(next_event, 0.0)))
            hold_frames = int(ceil(hold_seconds / max(dt, 1e-6)))
            # Hold for the full number of frame intervals after the first stationary sample.
            hold_remaining = hold_frames
            launching = True        # after the hold, pull away again using the next marker's speed
            launch_age = 0.0
            v_init = 0.0
            # v25: after the hold, look at the NEXT marker's speed. High = rear wheels spin before the
            # car pulls away; low = the normal gradual pull-away.
            nxt_kmh = next((vv for dd, vv in profile if dd > next_event + 1e-4), 0.0)
            spin_active = nxt_kmh >= LAUNCH_SPIN_MIN_KMH
        elif sf + step_distance >= total - 1e-6:
            # Finish at the route endpoint without inventing an instantaneous speed jump.
            # If the final target was not physically reachable, preserve the achieved speed.
            sf = total
        else:
            sf = min(total, sf + step_distance)
    return frames, arrived


def spin_frames(rear, yaw, travel, v0, wb, dt, track, steer_prev, sigma, spin_time):
    """Drift spin: the body turns 180 degrees while the car keeps travelling the same way.

    The car's centre keeps moving along 'travel', the body yaw swings from 'yaw' to
    'yaw' + sigma*180 deg with smooth start and end, and the front wheels counter-steer.
    v24: the car LOSES speed while it spins (it ends at DRIFT_EXIT_FRACTION of the entry speed, with
    a smooth, flat-bottomed finish) and the wheels come back to straight by the end, so the next
    leg can pause briefly and accelerate away cleanly.
    Returns (frames, new_rear, new_yaw, end_speed, end_steer)."""
    n = max(3, int(round(spin_time / dt)))
    half = wb * 0.5
    cx, cy = rear[0] + half * cos(yaw), rear[1] + half * sin(yaw)
    dx, dy = cos(travel), sin(travel)
    out = []
    dist = 0.0
    v = v0
    st = steer_prev
    exit_frac = max(0.1, min(1.0, DRIFT_EXIT_FRACTION))
    for i in range(1, n + 1):
        x = i / n
        e = x * x * (3.0 - 2.0 * x)
        yw = yaw + sigma * pi * e
        v = v0 * (1.0 - (1.0 - exit_frac) * e)                  # friction is lost during the spin
        cx += dx * v * dt
        cy += dy * v * dt
        c, sn = cos(yw), sin(yw)
        rr = (cx - half * c, cy - half * sn)
        ff = (cx + half * c, cy + half * sn)
        # steer into the spin, counter-steer as the car swings round, and unwind to straight at the end
        tail = max(0.0, (x - 0.55) / 0.45)
        env = 1.0 - tail * tail * (3.0 - 2.0 * tail)
        raw = sigma * MAX_WHEEL_ANGLE * 0.7 * cos(pi * x) * env
        w = min(1.0, x * 6.0)
        st = (1.0 - w) * steer_prev + w * raw
        t = tan(st)
        if abs(t) < 1e-6:
            dl = dr = 0.0
        else:
            r = wb / t
            dl = atan(wb / (r - track / 2.0))
            dr = atan(wb / (r + track / 2.0))
        dist += v * dt * cos(yw - travel)                       # wheels roll with the body's heading
        bell = sin(pi * x)
        roll = sigma * DRIFT_ROLL * bell                         # leans onto the outside of the slide
        pitch = DRIFT_PITCH * bell * cos(yw - travel)            # nose dips while the speed is scrubbed off
        out.append(dict(front=ff, rear=rr, yaw=wrap(yw), steer_left=dl, steer_right=dr,
                        dist=dist, speed=v, steer=st, gear=1, roll=roll, pitch=pitch))
    last = out[-1]
    return out, last["rear"], last["yaw"], v, st


def gear_states(marks):
    """Gear after each marker: False = forward, True = reverse. Every marked marker switches the gear
    and the new gear stays until another marked marker switches it back."""
    state = False
    out = []
    for m in marks:
        if m:
            state = not state
        out.append(state)
    return out


def reverse_marker_flags(marks):
    """True only for markers whose own Reverse Marker checkbox is ticked. Every other marker
    (including the ones driven in reverse after it) keeps no speed limit."""
    return [bool(m) for m in marks]


def enforce_reverse_speed_limit(pins):
    """Clamp the stored Target Speed of markers with Reverse checked to REVERSE_MAX_KMH. Only values above the
    limit are changed; lower values stay as they are. Forward markers are never touched."""
    if not pins:
        return
    flags = reverse_marker_flags([bool(getattr(p, "mqfollow_reverse_mark", False)) for p in pins])
    for pin, rev in zip(pins, flags):
        if not rev:
            continue
        try:
            if PIN_SPEED in pin and float(pin[PIN_SPEED]) > REVERSE_MAX_KMH:
                pin[PIN_SPEED] = REVERSE_MAX_KMH
        except Exception:
            pass


def plan_legs(markers, rest_rear, rest_yaw, wb, dt, track, max_steps, make_path, default_speed_kmh=DEFAULT_SPEED_KMH):
    """Plan a drive that contains reverse legs.

    markers: list of dicts {xy, speed, hold, reverse, obj} in route order. 'reverse' is the marker's
    Reverse checkbox: a marked marker switches the gear (forward -> reverse -> forward ...) for every
    leg after it, until the next marked marker. All speeds come from the markers' own speed values.
    Consecutive legs with the same gear form one group; each group is its own line, built from the
    pose the car has when the previous group ended.

    Gear change, two styles (chosen by the speed value of the marked marker):
      * below DRIFT_MIN_KMH: the car stops, pauses (Brake Hold, at least GEAR_PAUSE) and pulls away.
      * at or above it: the car does not stop. It arrives at that speed, spins 180 degrees while
        sliding on (a drift) and carries on the other way round. If the next marker is too close to
        spin before reaching it, the gear change falls back to stop-and-go.
    make_path(start_xy, heading, pins_xy, pin_objs, leg_index, reverse) -> (pts, s).
    Returns (frames, arrived, legs_info) where legs_info = [(pts, s, speed_pairs)].
    """
    n = len(markers)
    states = gear_states([bool(m.get("reverse")) for m in markers])
    # v25: markers with Reverse checked never go above REVERSE_MAX_KMH (lower values are kept as they are)
    markers = [dict(m) for m in markers]
    for m, rev in zip(markers, reverse_marker_flags([bool(m.get("reverse")) for m in markers])):
        if rev:
            m["speed"] = min(float(m["speed"]), REVERSE_MAX_KMH)
    groups = []
    cur = None
    for j in range(n):
        rev = bool(j > 0 and states[j - 1])
        if cur is None or cur["reverse"] != rev:
            cur = {"reverse": rev, "idx": [], "src": (j - 1 if j > 0 else None)}
            groups.append(cur)
        cur["idx"].append(j)

    # decide, for each gear change, whether it is a drift spin
    for k, g in enumerate(groups):
        g["drift_in"] = False
        if g["src"] is not None:
            m = markers[g["src"]]
            nxt = markers[g["idx"][0]]
            lv = max(0.0, float(m["speed"]))
            g["leave_kmh"] = lv
            gap = hypot(nxt["xy"][0] - m["xy"][0], nxt["xy"][1] - m["xy"][1])
            # room needed to finish the spin well before the next marker (a spin travels v * time)
            room = 2.0 * wb + 6.0 * (lv / 3.6) * dt
            g["drift_in"] = (lv >= DRIFT_MIN_KMH and gap >= room)

    rear = (rest_rear[0], rest_rear[1])
    yaw = rest_yaw
    steer = None
    carry_v = 0.0
    frames = []
    legs_info = []
    arrived = True
    for k, g in enumerate(groups):
        last_group = (k == len(groups) - 1)
        drift_out = (not last_group) and groups[k + 1]["drift_in"]
        if g["reverse"]:
            # Mirror image of a forward leg: the line starts at the front axle and the REAR axle leads.
            start = (rear[0] + wb * cos(yaw), rear[1] + wb * sin(yaw))
            heading = yaw + pi
        else:
            start = rear
            heading = yaw
        pins_xy = [markers[i]["xy"] for i in g["idx"]]
        pin_objs = [markers[i].get("obj") for i in g["idx"]]
        pts, s = make_path(start, heading, pins_xy, pin_objs, k, g["reverse"])

        pairs = []
        holds = {}
        for pos, i in enumerate(g["idx"]):
            d = nearest_route_distance(markers[i]["xy"][0], markers[i]["xy"][1], pts, s)
            v = max(0.0, float(markers[i]["speed"]))
            hold = max(0.0, min(60.0, float(markers[i].get("hold", 0.0))))
            if pos == len(g["idx"]) - 1 and not last_group:
                d = s[-1]                       # the gear change happens at the very end of this leg
                if not drift_out:
                    v = 0.0                     # stop-and-reverse: come to rest first
                    hold = max(hold, GEAR_PAUSE)
                else:
                    hold = 0.0                  # drift: arrive at the marker's own speed, no stop
            pairs.append((d, v))
            if v <= 1e-4 and hold > 0.0:
                holds[d] = max(holds.get(d, 0.0), hold)
        pairs.sort(key=lambda item: item[0])
        clean = []
        for d, v in pairs:
            if clean and abs(d - clean[-1][0]) < 1e-4:
                clean[-1] = (d, v)
            else:
                clean.append((d, v))
        pairs = clean
        if g["src"] is None and len(pairs) == 1 and pairs[0][1] <= 1e-4 and not last_group:
            # The first leg ends at a gear-change stop and has no earlier marker to set a speed:
            # pull away to that marker's own speed, then brake onto it.
            v_first = max(0.0, float(markers[g["idx"][0]]["speed"]))
            if v_first <= 1e-4:
                v_first = default_speed_kmh
            if pairs[0][0] > wb + 1e-4:
                pairs.insert(0, (wb, v_first))
        launch_time = None
        if g["src"] is not None:
            # The speed the car LEAVES the gear-change marker at.
            lv = g["leave_kmh"]
            if lv <= 1e-4:
                lv = next((v for _d, v in pairs if v > 1e-4), default_speed_kmh)
            if pairs[0][0] > wb + 1e-4:
                pairs.insert(0, (wb, lv))
            # Pull-away time follows the marker's speed: a high value gets there very fast, a low one slowly.
            launch_time = LAUNCH_TIME * max(0.1, min(1.5, 30.0 / max(lv, 1.0)))
            if g["drift_in"]:
                # v24: after the drift flip the car has lost speed: a short pause, then it builds
                # back up to the next marker's speed over DRIFT_REACCEL_TIME.
                launch_time = max(launch_time, DRIFT_REACCEL_TIME)
        legs_info.append((pts, s, list(pairs)))

        fr, ok = plan_drive(pts, s, wb, default_speed_kmh / 3.6, dt, track, max_steps,
                            speed_keys=pairs, stop_holds=holds, reverse=g["reverse"], steer0=steer,
                            launch_time=launch_time, v_init=(carry_v if g["drift_in"] else 0.0),
                            launch_delay=(DRIFT_HOLD_TIME if g["drift_in"] else 0.0),
                            spin_launch=((g["src"] is None
                                          and float(markers[g["idx"][0]]["speed"]) >= LAUNCH_SPIN_MIN_KMH)
                                         or (g["src"] is not None and not g["drift_in"]
                                             and g["leave_kmh"] >= LAUNCH_SPIN_MIN_KMH)))
        for f_ in fr:
            f_["leg"] = k
        frames.extend(fr if k == 0 else fr[1:])    # first frame repeats the previous pose
        if not ok:
            arrived = False
            break
        if drift_out and len(fr) >= 3:
            # v24: the leg's last frame is a truncated step (the car only covers the few cm left to the
            # marker in a full frame of time), which shows up as a one-frame stutter right before the
            # spin. Drop it: the spin continues from the previous frame at the same speed.
            frames.pop()
        rear = frames[-1]["rear"]
        yaw = frames[-1]["yaw"]
        steer = frames[-1]["steer"]
        carry_v = 0.0
        if drift_out:
            # Spin on the spot while still travelling the same way, then hand over to the next leg.
            travel = (yaw + pi) if g["reverse"] else yaw
            half = wb * 0.5
            cx, cy = rear[0] + half * cos(yaw), rear[1] + half * sin(yaw)
            nx, ny = markers[groups[k + 1]["idx"][0]]["xy"]
            cross = cos(travel) * (ny - cy) - sin(travel) * (nx - cx)
            sigma = 1.0 if cross >= 0.0 else -1.0
            v0 = max(frames[-1]["speed"], 0.0)
            kmh = max(v0 * 3.6, 1.0)
            spin_time = max(0.25, min(DRIFT_SPIN_TIME, DRIFT_SPIN_TIME * sqrt(60.0 / kmh)))
            # The car keeps travelling while it spins. Never let it travel further than 45% of the
            # way to the next marker, so the next leg always starts BEHIND that marker.
            ahead = cos(travel) * (nx - cx) + sin(travel) * (ny - cy)   # v24: x and y were swapped
            budget = 0.45 * max(ahead, 0.0)
            time_budget = budget / max(0.9 * v0, 1e-6)
            spin_time = max(3.0 * dt, min(spin_time, time_budget))
            sp, rear, yaw, carry_v, steer = spin_frames(rear, yaw, travel, v0, wb, dt, track, steer,
                                                        sigma, spin_time)
            for f_ in sp:
                f_["leg"] = k + 0.5
            frames.extend(sp)
    return frames, arrived, legs_info


# --------------------------------------------------------------------------
# Putting the car parts on the line
# --------------------------------------------------------------------------
def part_matrices(frame, rest, wb, wheel_radius, track, spin):
    """World matrices of every car part for one frame.
    rest: dict with 'rear0' (x,y,z), 'yaw0', 'axis0' (unit left axis), 'body' (4x4) and, per wheel
    attr, 'wheel' (4x4), 'hub' (4x4 or None), 'left' (bool).
    spin: wheel spin angle so far (radians). Returns {name: 4x4} with keys 'body',
    'wheel_xx', 'hub_xx'."""
    rx, ry, rz = rest["rear0"]
    pr = frame["rear"]
    G = m_mul(m_tr(pr[0], pr[1], rz), m_mul(m_rot((0, 0, 1), frame["yaw"] - rest["yaw0"]),
                                           m_tr(-rx, -ry, -rz)))
    body = rest["body"]
    roll, pitch = frame.get("roll", 0.0), frame.get("pitch", 0.0)
    if abs(roll) > 1e-6 or abs(pitch) > 1e-6:
        # v24: lean the BODY only (the wheels stay planted) about the middle of the car at ground level.
        fwd = (cos(rest["yaw0"]), sin(rest["yaw0"]), 0.0)
        pivot = (rx + 0.5 * wb * fwd[0], ry + 0.5 * wb * fwd[1], rz - wheel_radius)
        tilt = m_mul(m_rot(fwd, roll), m_rot(rest["axis0"], pitch))
        body = m_mul(m_about(pivot, tilt), body)
    out = {"body": m_mul(G, body)}
    for attr in WHEEL_ATTRS:
        W0 = rest[attr]["wheel"]
        c = m_pos(W0)
        wheel_spin = spin if attr in FRONT_ATTRS else spin + frame.get("slip_spin", 0.0)   # v25: rear wheels may spin extra
        M = m_mul(m_about(c, m_rot(rest["axis0"], wheel_spin)), W0)     # spin about the axle
        steer = 0.0
        if attr in FRONT_ATTRS:
            steer = frame["steer_left"] if rest[attr]["left"] else frame["steer_right"]
            M = m_mul(m_about(c, m_rot((0, 0, 1), steer)), M)           # steer about vertical
        out[attr] = m_mul(G, M)
        H0 = rest[attr]["hub"]
        if H0 is not None:
            Hm = H0
            if attr in FRONT_ATTRS:
                Hm = m_mul(m_about(c, m_rot((0, 0, 1), steer)), H0)
            out["hub_" + attr] = m_mul(G, Hm)
    return out


# --------------------------------------------------------------------------
# Reading the MQScar rig
# --------------------------------------------------------------------------
def rig_ready(scene):
    p = getattr(scene, "mqscar", None)
    if p is None:
        return False, "MQScar add-on is not installed"
    coll = bpy.data.collections.get(RIG_COLLECTION)
    if coll is None or len(coll.objects) == 0:
        return False, "Generate the MQScar rig first"
    if p.body is None or any(getattr(p, a) is None for a in WHEEL_ATTRS):
        return False, "MQScar Body / wheels are not set"
    return True, "Car found: " + p.body.name


def _rows(m):
    return [[float(v) for v in row] for row in m]


def home_rows(obj):
    """The pose MQScar saved for this object when the rig was generated."""
    if HOME_KEY in obj:
        vals = [float(v) for v in obj[HOME_KEY]]
        return [vals[i * 4:(i + 1) * 4] for i in range(4)]
    return _rows(obj.matrix_world)


def get_hubs():
    return {o[TAG_HUB]: o for o in bpy.data.objects if TAG_HUB in o}


def read_rest(p):
    """Everything about the car at its start. Returns (rest, wb, radius, track, parts) where
    parts maps part name -> Blender object."""
    hubs = get_hubs()
    wheels = {a: getattr(p, a) for a in WHEEL_ATTRS}
    W = {a: home_rows(w) for a, w in wheels.items()}
    c = {a: m_pos(W[a]) for a in WHEEL_ATTRS}
    front0 = ((c["wheel_fl"][0] + c["wheel_fr"][0]) / 2, (c["wheel_fl"][1] + c["wheel_fr"][1]) / 2)
    rear0 = ((c["wheel_rl"][0] + c["wheel_rr"][0]) / 2, (c["wheel_rl"][1] + c["wheel_rr"][1]) / 2)
    dx, dy = front0[0] - rear0[0], front0[1] - rear0[1]
    wb = hypot(dx, dy)
    if wb < 1e-4:
        return None
    yaw0 = atan2(dy, dx)
    axis0 = (-sin(yaw0), cos(yaw0), 0.0)                       # points to the car's LEFT
    z = (c["wheel_rl"][2] + c["wheel_rr"][2]) / 2
    rest = dict(rear0=(rear0[0], rear0[1], z), yaw0=yaw0, axis0=axis0, body=home_rows(p.body))
    radii = []
    parts = {"body": p.body}
    for a in WHEEL_ATTRS:
        left = ((c[a][0] - rear0[0]) * axis0[0] + (c[a][1] - rear0[1]) * axis0[1]) > 0
        hub = hubs.get(a)
        rest[a] = dict(wheel=W[a], hub=home_rows(hub) if hub is not None else None, left=left)
        parts[a] = wheels[a]
        if hub is not None:
            parts["hub_" + a] = hub
        radii.append(max(max(wheels[a].dimensions) / 2.0, 0.05))
    track = abs((c["wheel_fl"][0] - c["wheel_fr"][0]) * axis0[0]
                + (c["wheel_fl"][1] - c["wheel_fr"][1]) * axis0[1])
    return rest, wb, sum(radii) / len(radii), max(track, 0.1), parts


def ground_z(rest, radius):
    return min(rest[a]["wheel"][2][3] for a in WHEEL_ATTRS) - radius


def _set_mqscar_marker_mode(scene, enabled):
    """Hide/neutralize MQScar live controls while Follow marker mode owns the drive."""
    props = getattr(scene, "mqscar", None)
    if props is None:
        return
    keys = ("mqfollow_saved_speed", "mqfollow_saved_brake", "mqfollow_saved_hold")
    if enabled:
        if not scene.get("mqfollow_saved_live_controls", False):
            scene[keys[0]] = float(getattr(props, "speed", 0.0))
            scene[keys[1]] = bool(getattr(props, "brake", False))
            scene[keys[2]] = bool(getattr(props, "hold", False))
            scene["mqfollow_saved_live_controls"] = True
        props.speed = 0.0
        props.brake = False
        if hasattr(props, "hold"):
            props.hold = False
    else:
        if scene.get("mqfollow_saved_live_controls", False):
            try:
                props.speed = float(scene.get(keys[0], 0.0))
                props.brake = bool(scene.get(keys[1], False))
                if hasattr(props, "hold"):
                    props.hold = bool(scene.get(keys[2], False))
            finally:
                for key in keys + ("mqfollow_saved_live_controls",):
                    if key in scene:
                        del scene[key]


def set_ui_sections(scene, curve_open):
    """Accordion: Curve Mode open -> Rigging collapsed, and the other way round."""
    props = getattr(scene, "mqscar", None)
    if props is None:
        return
    try:
        if curve_open:
            props.show_rig_section = False
            props.show_curve_section = True
        else:
            props.show_curve_section = False
            props.show_rig_section = True
    except Exception:
        pass


def get_pins():
    """All pins in number order. A pin from an older version is adopted as pin 1."""
    old = bpy.data.objects.get(LEGACY_PIN)
    if old is not None and PIN_KEY not in old:
        old[PIN_KEY] = 1
        old.name = PIN_PREFIX + "1"
    pins = [o for o in bpy.data.objects if PIN_KEY in o]
    pins.sort(key=lambda o: o[PIN_KEY])
    return pins


def _get_active_marker_number(self):
    try:
        active = bpy.context.view_layer.objects.active
        return int(active.get(PIN_KEY, 1)) if active is not None and PIN_KEY in active else 1
    except Exception:
        return 1


def _set_active_marker_number(self, requested):
    """Move the active marker to a new ordinal, shifting intervening markers automatically."""
    try:
        active = bpy.context.view_layer.objects.active
        if active is None or PIN_KEY not in active:
            return
        pins = get_pins()
        if not pins:
            return
        old = int(active.get(PIN_KEY, 1))
        new = max(1, min(len(pins), int(requested)))
        if new == old:
            return
        if new < old:
            for pin in pins:
                idx = int(pin.get(PIN_KEY, 0))
                if pin != active and new <= idx < old:
                    pin[PIN_KEY] = idx + 1
        else:
            for pin in pins:
                idx = int(pin.get(PIN_KEY, 0))
                if pin != active and old < idx <= new:
                    pin[PIN_KEY] = idx - 1
        active[PIN_KEY] = new
        renumber_pins()
        self.info = f"Moved selected marker from {old} to {new}; route order updated."
    except Exception as exc:
        try:
            self.info = f"Could not reorder marker: {exc}"
        except Exception:
            pass


# --------------------------------------------------------------------------
# Properties
# --------------------------------------------------------------------------
class MQFOLLOW2_Props(PropertyGroup):
    info: StringProperty(name="Info", default="")
    surface_snap: BoolProperty(name="Snap to Ground Collection", default=True,
        description="Place markers only on mesh objects in MQScar's selected Ground Collection")
    show_markers: BoolProperty(name="Show Route Markers", default=True)
    show_lines: BoolProperty(name="Show Route Lines", default=True,
        description="Show or hide the route line, the reverse lines and the speed graph")
    marker_mode: BoolProperty(name="Marker Mode", default=False)
    placing_markers: BoolProperty(name="Placing Markers", default=False)
    marker_number: IntProperty(name="Marker Number", get=_get_active_marker_number,
        set=_set_active_marker_number, min=1, description="Move this marker to a new position in the route order; other markers shift automatically")
    acceleration_strength: FloatProperty(name="Acceleration Strength (m/s²)", default=3.0, min=0.5, max=20.0,
        precision=1, description="Higher values accelerate more strongly and reach the next marker speed sooner")
    braking_strength: FloatProperty(name="Braking Strength (m/s²)", default=6.0, min=0.5, max=30.0,
        precision=1, description="Higher values brake more strongly and shorten the stopping distance")


# --------------------------------------------------------------------------
# The one green line
# --------------------------------------------------------------------------
def _remove_obj(name):
    o = bpy.data.objects.get(name)
    if o is not None:
        data = o.data
        bpy.data.objects.remove(o, do_unlink=True)
        if isinstance(data, bpy.types.Curve) and data.users == 0:
            bpy.data.curves.remove(data)


def _new_curve_object(scene, name, material_name, color, bevel=0.025):
    _remove_obj(name)
    cu = bpy.data.curves.new(name, 'CURVE')
    cu.dimensions = '3D'
    cu.resolution_u = 24
    cu.bevel_depth = bevel
    cu.bevel_resolution = 3
    mat = bpy.data.materials.get(material_name)
    if mat is None:
        mat = bpy.data.materials.new(material_name)
    mat.diffuse_color = color
    cu.materials.append(mat)
    ob = bpy.data.objects.new(name, cu)
    scene.collection.objects.link(ob)
    ob.show_in_front = True
    return ob


def _pin_smoothness(pin):
    try:
        return max(0.0, min(2.0, float(pin.get(PIN_SMOOTH, 1.0))))
    except Exception:
        return 1.0


def ensure_pin_controls(pin):
    """Create marker-local driving controls; safe for pins from older extension versions."""
    if PIN_SPEED not in pin:
        pin[PIN_SPEED] = DEFAULT_SPEED_KMH
    if PIN_BRAKE_HOLD not in pin:
        pin[PIN_BRAKE_HOLD] = 0.0
    if PIN_SMOOTH not in pin:
        pin[PIN_SMOOTH] = 1.0
    try:
        pin.id_properties_ui(PIN_SPEED).update(
            min=0.0, max=1.0e30, soft_min=0.0, soft_max=300.0,
            precision=2,
            description="Target speed at this route marker in km/h. No practical upper speed limit; enter a value directly. Zero means a complete stop."
        )
        pin.id_properties_ui(PIN_BRAKE_HOLD).update(
            min=0.0, max=60.0, soft_min=0.0, soft_max=10.0,
            precision=2,
            description="How long the car remains stopped at this marker, in seconds."
        )
        pin.id_properties_ui(PIN_SMOOTH).update(
            min=0.0, max=2.0, soft_min=0.0, soft_max=2.0,
            description="How smoothly the route bends through this marker."
        )
    except Exception:
        pass


def renumber_pins():
    for i, pin in enumerate(get_pins(), 1):
        ensure_pin_controls(pin)
        pin[PIN_KEY] = i
        pin.name = f"{PIN_PREFIX}{i}"


def ground_collection(scene):
    """Return only the collection explicitly selected in MQScar's Ground picker."""
    props = getattr(scene, "mqscar", None)
    return getattr(props, "ground_collection", None) if props else None


def ground_meshes(scene):
    coll = ground_collection(scene)
    if coll is None:
        return []
    return [ob for ob in coll.all_objects if ob.type == 'MESH' and not ob.hide_render]


def raycast_ground(context, origin, direction):
    """Raycast exclusively against evaluated mesh objects in MQScar's Ground collection."""
    coll = ground_collection(context.scene)
    if coll is None:
        return None
    depsgraph = context.evaluated_depsgraph_get()
    best = None
    ray_dir = direction.normalized()
    for original in coll.all_objects:
        if original.type != 'MESH' or original.hide_get() or original.hide_viewport:
            continue
        try:
            evaluated = original.evaluated_get(depsgraph)
            inv = evaluated.matrix_world.inverted()
            local_origin = inv @ origin
            local_direction = (inv.to_3x3() @ ray_dir).normalized()
            hit, loc, normal, _face = evaluated.ray_cast(local_origin, local_direction)
            if not hit:
                continue
            world_loc = evaluated.matrix_world @ loc
            distance = (world_loc - origin).dot(ray_dir)
            if distance < 0:
                continue
            if best is None or distance < best[0]:
                best = (distance, world_loc.copy())
        except (ReferenceError, RuntimeError, ValueError):
            continue
    return best[1] if best else None


def project_point_to_ground(context, point):
    """Project a marker vertically onto the selected Ground Collection."""
    origin = Vector((point.x, point.y, point.z + 100000.0))
    return raycast_ground(context, origin, Vector((0.0, 0.0, -1.0)))


def create_route_curve(scene, rear, heading, wb, pins, z, name=None, pin_objs=None, color=(0.1, 1.0, 0.25, 1.0)):
    """Rebuild the generated route from the ordered marker positions.

    Pins are the source of truth. The generated curve is deliberately recreated on Bake,
    so dragging a marker cannot leave the route disconnected from it.
    """
    coords = [(rear[0], rear[1], z + 0.06),
              (rear[0] + wb*cos(heading), rear[1] + wb*sin(heading), z + 0.06)]
    coords.extend((float(x), float(y), z + 0.06) for x, y in pins)
    clean = []
    for co in coords:
        if not clean or hypot(co[0]-clean[-1][0], co[1]-clean[-1][1]) > 0.05:
            clean.append(co)
    if len(clean) < 2:
        raise ValueError("The route needs at least two distinct control points")
    ob = _new_curve_object(scene, name or PATH_NAME, "MQFollow_Green" if color[2] < 0.5 else "MQFollow_Blue", color, 0.045)
    pin_list = pin_objs if pin_objs is not None else get_pins()
    sp = ob.data.splines.new('BEZIER')
    sp.bezier_points.add(len(clean)-1)
    bps = list(sp.bezier_points)
    for bp, co in zip(bps, clean):
        bp.co = co
    # Per-marker smoothness: 0 = sharp/vector, 1 = normal smooth, 2 = extra soft.
    # The first two points are the rig's straight lead-in; following points map to pins.
    for i, bp in enumerate(bps):
        if i < 2:
            bp.handle_left_type = 'AUTO'
            bp.handle_right_type = 'AUTO'
            continue
        pin_index = i - 2
        smooth = _pin_smoothness(pin_list[pin_index]) if pin_index < len(pin_list) and pin_list[pin_index] is not None else 1.0
        if smooth <= 0.02:
            bp.handle_left_type = 'VECTOR'
            bp.handle_right_type = 'VECTOR'
        else:
            bp.handle_left_type = 'AUTO'
            bp.handle_right_type = 'AUTO'
            # Blender's AUTO is a stable default. Scale its handles around the knot
            # to give each marker an intuitive individual influence.
            bp.handle_left_type = 'FREE'
            bp.handle_right_type = 'FREE'
            left = bp.handle_left.copy()
            right = bp.handle_right.copy()
            bp.handle_left = bp.co + (left - bp.co) * smooth
            bp.handle_right = bp.co + (right - bp.co) * smooth
    sp.resolution_u = 24
    ob["mqfollow_editable_route"] = True
    ob["mqfollow_ground_z"] = float(z)
    ob["mqfollow_wheelbase"] = float(wb)
    ob["mqfollow_marker_driven"] = True
    ob.hide_select = False
    return ob


def create_poly_route_curve(scene, pts, z, wb, name, color):
    """v24: show a generated drivable line (list of x,y points) as the leg's route curve."""
    ob = _new_curve_object(scene, name, "MQFollow_Green" if color[2] < 0.5 else "MQFollow_Blue", color, 0.045)
    sp = ob.data.splines.new('POLY')
    sp.points.add(len(pts) - 1)
    for point, (x, y) in zip(sp.points, pts):
        point.co = (float(x), float(y), z + 0.06, 1.0)
    ob["mqfollow_ground_z"] = float(z)
    ob["mqfollow_wheelbase"] = float(wb)
    ob["mqfollow_marker_driven"] = True
    ob.hide_select = False
    return ob


def evaluated_route_samples(route_obj, depsgraph=None):
    """Sample the editable route using Blender's stable curve-control-point API.

    Spline.evaluated_points is not exposed in several Blender releases. For Bezier
    splines, interpolate each cubic segment from its knot and handle coordinates.
    This also means the sampled path follows the exact handles visible in the viewport.
    """
    if route_obj is None or route_obj.type != 'CURVE':
        raise ValueError("The driving route object is missing or is not a Curve")

    samples = []
    world = route_obj.matrix_world.copy()

    def append_local(local_co):
        co = world @ Vector((local_co[0], local_co[1], local_co[2]))
        xy = (float(co.x), float(co.y))
        if not samples or hypot(xy[0] - samples[-1][0], xy[1] - samples[-1][1]) > 1e-5:
            samples.append(xy)

    for spline in route_obj.data.splines:
        if spline.type == 'BEZIER':
            bez = spline.bezier_points
            count = len(bez)
            if count < 2:
                continue
            segment_count = count if spline.use_cyclic_u else count - 1
            resolution = max(2, int(spline.resolution_u or route_obj.data.resolution_u or 12))
            for i in range(segment_count):
                a = bez[i]
                b = bez[(i + 1) % count]
                # interpolate_bezier returns resolution + 1 samples, including both ends.
                segment = interpolate_bezier(
                    a.co, a.handle_right, b.handle_left, b.co, resolution
                )
                for point in segment:
                    append_local(point)
        elif spline.type == 'POLY':
            for point in spline.points:
                append_local(point.co)
        else:
            raise ValueError(
                f"Unsupported route spline type: {spline.type}. "
                "Use a Bezier or Poly spline for MQScar Follow."
            )

    if len(samples) < 2:
        raise ValueError("The driving curve has fewer than two sampled points")
    return samples


def distances_for_points(pts):
    dist = [0.0]
    for a, b in zip(pts, pts[1:]):
        dist.append(dist[-1] + hypot(b[0]-a[0], b[1]-a[1]))
    return dist


def get_route_samples(context):
    route = bpy.data.objects.get(PATH_NAME)
    if route is None or route.type != 'CURVE':
        raise ValueError("Create the driving curve first")
    pts = evaluated_route_samples(route, context.evaluated_depsgraph_get())
    return pts, distances_for_points(pts)


def nearest_route_distance(x, y, pts, dist):
    best_d, best_err = 0.0, float('inf')
    for i, (a, b) in enumerate(zip(pts, pts[1:])):
        dx, dy = b[0]-a[0], b[1]-a[1]
        den = dx*dx + dy*dy
        t = 0.0 if den < 1e-12 else max(0.0, min(1.0, ((x-a[0])*dx + (y-a[1])*dy)/den))
        px, py = a[0] + t*dx, a[1] + t*dy
        err = (x-px)**2 + (y-py)**2
        if err < best_err:
            best_err = err
            best_d = dist[i] + t*(dist[i+1]-dist[i])
    return best_d


def get_speed_keys():
    """Legacy speed-key objects from v10; retained only for backward compatibility."""
    keys = [o for o in bpy.data.objects if SPEED_KEY in o]
    keys.sort(key=lambda o: float(o.get(SPEED_DISTANCE, 0.0)))
    return keys


def _marker_speed_data(pins, pts, dist, default=DEFAULT_SPEED_KMH):
    """Return ordered (distance, speed_kmh) and (distance, hold_seconds) from route markers."""
    pairs = []
    holds = {}
    # Migrate older v10 speed-key profiles onto route markers once, without losing
    # the user's previously authored speed curve.
    legacy_pairs = [
        (float(key.get(SPEED_DISTANCE, 0.0)), float(key.get(SPEED_VALUE, default)))
        for key in get_speed_keys()
    ]
    legacy_pairs.sort(key=lambda item: item[0])
    for pin in pins:
        loc = pin.matrix_world.translation
        d = nearest_route_distance(loc.x, loc.y, pts, dist)
        if PIN_SPEED not in pin:
            pin[PIN_SPEED] = _speed_value_at(legacy_pairs, d, default) if legacy_pairs else default
        ensure_pin_controls(pin)
        speed = max(0.0, float(pin.get(PIN_SPEED, default)))
        hold = max(0.0, min(60.0, float(pin.get(PIN_BRAKE_HOLD, 0.0))))
        pairs.append((d, speed))
        if speed <= 1e-4 and hold > 0.0:
            holds[d] = max(holds.get(d, 0.0), hold)
    pairs.sort(key=lambda item: item[0])
    # Collapse duplicate/near-identical distances deterministically.
    clean = []
    for d, v in pairs:
        if clean and abs(d - clean[-1][0]) < 1e-4:
            clean[-1] = (d, v)
        else:
            clean.append((d, v))
    if not clean:
        clean = [(0.0, default), (dist[-1], default)]
    return clean, holds


def _speed_value_at(pairs, d, default=DEFAULT_SPEED_KMH):
    if not pairs:
        return default
    pairs = sorted(pairs)
    if d <= pairs[0][0]:
        return pairs[0][1]
    if d >= pairs[-1][0]:
        return pairs[-1][1]
    for (d0, v0), (d1, v1) in zip(pairs, pairs[1:]):
        if d0 <= d <= d1:
            t = 0.0 if d1-d0 < 1e-8 else (d-d0)/(d1-d0)
            return v0 + (v1-v0)*t
    return pairs[-1][1]


def _marker_profile_speed_at_distance(speed_keys, distance, default_speed_ms=DEFAULT_SPEED_KMH / 3.6):
    """Unconstrained target curve for display: flat to each segment midpoint, then smooth blend."""
    if not speed_keys:
        return max(0.0, float(default_speed_ms))
    keys = sorted((max(0.0, float(d)), max(0.0, float(v)) / 3.6) for d, v in speed_keys)
    if distance <= keys[0][0]:
        return keys[0][1]
    if distance >= keys[-1][0]:
        return keys[-1][1]
    for (d0, v0), (d1, v1) in zip(keys, keys[1:]):
        if d0 <= distance <= d1:
            midpoint = d0 + 0.5 * (d1 - d0)
            if distance <= midpoint or d1 - midpoint < 1e-9:
                return v0
            t = max(0.0, min(1.0, (distance - midpoint) / (d1 - midpoint)))
            eased = t * t * (3.0 - 2.0 * t)
            return v0 + (v1 - v0) * eased
    return keys[-1][1]


def rebuild_speed_graph(scene, pts, dist, z, default_speed=DEFAULT_SPEED_KMH, wheelbase=0.0,
                        acceleration_strength=3.0, braking_strength=6.0):
    """Draw the same marker-authoritative reachable speed profile used by the drive planner."""
    pins = get_pins()
    pairs, _holds = _marker_speed_data(pins, pts, dist, default_speed)
    # Old v10 speed-key empties are no longer editable; hide them after migrating their
    # values to route markers so the viewport contains one clear marker system.
    for legacy_key in get_speed_keys():
        try:
            legacy_key.hide_set(True)
            legacy_key.hide_render = True
        except Exception:
            pass
    graph = _new_curve_object(
        scene, SPEED_GRAPH_NAME, "MQFollow_SpeedGraph_Mat",
        (1.0, 0.42, 0.05, 1.0), 0.025
    )
    sp = graph.data.splines.new('POLY')
    sample_count = max(2, min(500, int(dist[-1] / 0.5) + 1))
    # Keep regular samples for a smooth line and add each marker's exact route distance.
    samples = [dist[-1] * i / (sample_count - 1) for i in range(sample_count)]
    samples.extend(max(0.0, min(dist[-1], float(d))) for d, _v in pairs)
    samples = sorted(set(round(d, 6) for d in samples))
    sp.points.add(len(samples) - 1)
    for bp, d in zip(sp.points, samples):
        # Draw the requested marker profile, not the physically reachable envelope.
        # This ensures the orange line actually meets every marker's speed stem.
        speed_ms = _marker_profile_speed_at_distance(pairs, d, default_speed / 3.6)
        speed = speed_ms * 3.6
        x, y = point_at(pts, dist, d)
        bp.co = (x, y, z + 0.12 + speed * SPEED_GRAPH_SCALE, 1.0)

    # Draw target-speed stems at marker positions, using the exact same projected distance.
    for pin in pins:
        loc = pin.matrix_world.translation
        d = nearest_route_distance(loc.x, loc.y, pts, dist)
        speed = max(0.0, float(pin.get(PIN_SPEED, default_speed)))
        x, y = point_at(pts, dist, d)
        stem = graph.data.splines.new('POLY')
        stem.points.add(1)
        stem.points[0].co = (x, y, z + 0.08, 1.0)
        stem.points[1].co = (x, y, z + 0.12 + speed * SPEED_GRAPH_SCALE, 1.0)
    graph.hide_select = True
    return graph


def rebuild_speed_graph_legs(scene, legs, z, default_speed=DEFAULT_SPEED_KMH):
    """Speed graph for a route that has reverse legs: one orange line and set of stems per leg."""
    for legacy_key in get_speed_keys():
        try:
            legacy_key.hide_set(True)
            legacy_key.hide_render = True
        except Exception:
            pass
    graph = _new_curve_object(scene, SPEED_GRAPH_NAME, "MQFollow_SpeedGraph_Mat",
                              (1.0, 0.42, 0.05, 1.0), 0.025)
    for pts, dist, pairs in legs:
        sample_count = max(2, min(500, int(dist[-1] / 0.5) + 1))
        samples = [dist[-1] * i / (sample_count - 1) for i in range(sample_count)]
        samples.extend(max(0.0, min(dist[-1], float(d))) for d, _v in pairs)
        samples = sorted(set(round(d, 6) for d in samples))
        sp = graph.data.splines.new('POLY')
        sp.points.add(len(samples) - 1)
        for bp, d in zip(sp.points, samples):
            speed = _marker_profile_speed_at_distance(pairs, d, default_speed / 3.6) * 3.6
            x, y = point_at(pts, dist, d)
            bp.co = (x, y, z + 0.12 + speed * SPEED_GRAPH_SCALE, 1.0)
        for d, v in pairs:
            x, y = point_at(pts, dist, d)
            stem = graph.data.splines.new('POLY')
            stem.points.add(1)
            stem.points[0].co = (x, y, z + 0.08, 1.0)
            stem.points[1].co = (x, y, z + 0.12 + max(0.0, v) * SPEED_GRAPH_SCALE, 1.0)
    graph.hide_select = True
    return graph


# --------------------------------------------------------------------------
# Keyframes
# --------------------------------------------------------------------------
def _rot_path(obj):
    return "rotation_quaternion" if obj.rotation_mode == 'QUATERNION' else "rotation_euler"


def key_object(obj, frame):
    obj.keyframe_insert("location", frame=frame)
    obj.keyframe_insert(_rot_path(obj), frame=frame)


def clear_object_keys(obj, f0, f1):
    ad = getattr(obj, "animation_data", None)
    if ad is None or ad.action is None:
        return
    if OWN_ACTION in obj:                       # the action was created by this script
        act = ad.action
        ad.action = None
        try:
            bpy.data.actions.remove(act)
        except Exception:
            pass
        del obj[OWN_ACTION]
    else:                                       # the object had its own animation: remove only our keys
        for fr in range(int(f0), int(f1) + 1):
            for path in ("location", _rot_path(obj)):
                try:
                    obj.keyframe_delete(path, frame=fr)
                except Exception:
                    pass


def clear_legacy(scene):
    """Remove what older versions of this script left behind."""
    for nm in OLD_OBJECTS:
        _remove_obj(nm)
    p = getattr(scene, "mqscar", None)
    frames = list(scene.get("mqfollow_frames", []))
    if p is not None and frames:
        for fr in frames:
            for prop in ("steer", "speed", "brake"):
                try:
                    p.keyframe_delete(prop, frame=int(fr))
                except Exception:
                    pass
    if "mqfollow_frames" in scene:
        del scene["mqfollow_frames"]


def clear_drive(scene, restore=True):
    names = list(scene.get(PARTS_KEY, []))
    span = scene.get("mqfollow_span", None)
    f0, f1 = (span if span else (scene.frame_start, scene.frame_end))
    for nm in names:
        obj = bpy.data.objects.get(nm)
        if obj is None:
            continue
        clear_object_keys(obj, f0, f1)
        if obj.rigid_body is not None:
            previous = obj.get("mqf_prev_kinematic", False)
            obj.rigid_body.kinematic = bool(previous)
            if "mqf_prev_kinematic" in obj:
                del obj["mqf_prev_kinematic"]
        if restore:
            key = RUN_KEY if RUN_KEY in obj else HOME_KEY
            if key in obj:
                vals = [float(v) for v in obj[key]]
                obj.matrix_world = Matrix([vals[i * 4:(i + 1) * 4] for i in range(4)])
    for k in (PARTS_KEY, "mqfollow_span"):
        if k in scene:
            del scene[k]
    # Keep the editable route and Speed Segments in the scene; only the vehicle bake is cleared.
    clear_legacy(scene)


# --------------------------------------------------------------------------
# Operators
# --------------------------------------------------------------------------
def viewport_region_and_coord(context, event):
    """Resolve the 3D WINDOW region even when a modal tool is launched from the N-panel."""
    area = context.area
    if area is None:
        return None, None, None
    region = next((r for r in area.regions if r.type == 'WINDOW'), None)
    space = area.spaces.active
    rv3d = getattr(space, 'region_3d', None)
    if region is None or rv3d is None:
        return None, None, None
    coord = (event.mouse_x - region.x, event.mouse_y - region.y)
    return region, rv3d, coord


def create_pin(scene, x, y, z, index=None, speed=DEFAULT_SPEED_KMH, brake_hold=0.0):
    pins = get_pins()
    if index is None:
        index = len(pins) + 1
    index = max(1, min(int(index), len(pins) + 1))
    for pin in reversed(pins):
        if pin[PIN_KEY] >= index:
            pin[PIN_KEY] = int(pin[PIN_KEY]) + 1
    pin = bpy.data.objects.new(f"{PIN_PREFIX}{index}", None)
    pin.empty_display_type = 'CONE'
    pin.empty_display_size = 0.65
    pin.show_in_front = True
    pin.show_name = True
    pin.color = (1.0, 0.35, 0.05, 1.0)
    pin[PIN_KEY] = index
    pin[PIN_SMOOTH] = 1.0
    pin[PIN_SPEED] = max(0.0, float(speed))
    pin[PIN_BRAKE_HOLD] = max(0.0, min(60.0, float(brake_hold)))
    ensure_pin_controls(pin)
    pin.location = (x, y, z)
    scene.collection.objects.link(pin)
    renumber_pins()
    return pin


def set_markers_visible(scene, visible):
    for pin in get_pins():
        try:
            pin.hide_set(not visible)
        except Exception:
            pin.hide_viewport = not visible


LINE_PREFIXES = (PATH_NAME, SPEED_GRAPH_NAME)   # route line (and reverse legs) + speed graph


def get_line_objects():
    return [o for o in bpy.data.objects if o.name.startswith(LINE_PREFIXES)]


def set_lines_visible(scene, visible):
    for ob in get_line_objects():
        try:
            ob.hide_set(not visible)
        except Exception:
            ob.hide_viewport = not visible


def surface_hit_from_event(context, event, flat_z=None):
    region, rv3d, coord = viewport_region_and_coord(context, event)
    if region is None or rv3d is None:
        return None
    origin = view3d_utils.region_2d_to_origin_3d(region, rv3d, coord)
    direction = view3d_utils.region_2d_to_vector_3d(region, rv3d, coord)
    if context.scene.mqfollow2.surface_snap:
        return raycast_ground(context, origin, direction)
    return intersect_line_plane(origin, origin + direction * 100000.0,
                                Vector((0.0, 0.0, float(flat_z or 0.0))), Vector((0.0, 0.0, 1.0)))


class MQFOLLOW2_OT_place_pins(Operator):
    bl_idname = "mqfollow2.place_pins"
    bl_label = "Place Pins (Alt + Click)"
    bl_description = "Enter placement mode, then Alt+Left-click the ground to add route pins; Esc or right-click to finish"
    bl_options = {'REGISTER', 'UNDO'}

    _ground_z = 0.0

    def invoke(self, context, event):
        if context.area is None or context.area.type != 'VIEW_3D':
            self.report({'ERROR'}, "Run this tool from a 3D View")
            return {'CANCELLED'}
        ok, msg = rig_ready(context.scene)
        if not ok:
            self.report({'ERROR'}, msg)
            return {'CANCELLED'}
        info = read_rest(context.scene.mqscar)
        if info is None:
            self.report({'ERROR'}, "Could not read the MQScar wheel positions")
            return {'CANCELLED'}
        rest, _wb, radius, _track, _parts = info
        self._ground_z = ground_z(rest, radius)
        if context.scene.mqfollow2.surface_snap and ground_collection(context.scene) is None:
            self.report({'ERROR'}, "Select a Ground Collection in the MQScar panel first.")
            return {'CANCELLED'}
        if not context.scene.mqfollow2.marker_mode:
            self.report({'ERROR'}, "Activate Marker Mode first")
            return {'CANCELLED'}
        context.scene.mqfollow2.placing_markers = True
        context.area.tag_redraw()
        context.window_manager.modal_handler_add(self)
        context.workspace.status_text_set("MQScar: PLACING MARKERS — Alt + Left-click on the selected Ground Collection")
        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        # A modal event can arrive after the originating editor has been closed.
        if context.area is None:
            try:
                context.scene.mqfollow2.placing_markers = False
            except Exception:
                pass
            try:
                if context.workspace is not None:
                    context.workspace.status_text_set(None)
            except Exception:
                pass
            return {'FINISHED'}
        if not context.scene.mqfollow2.marker_mode or not context.scene.mqfollow2.placing_markers:
            context.workspace.status_text_set(None)
            return {'FINISHED'}
        if event.type in {'ESC', 'RIGHTMOUSE'} and event.value == 'PRESS':
            context.workspace.status_text_set(None)
            context.scene.mqfollow2.placing_markers = False
            if context.area is not None:
                context.area.tag_redraw()
            context.scene.mqfollow2.info = (f"Marker Mode is active. {len(get_pins())} marker(s). "
                                             "Select a marker, edit its settings, or Bake.")
            return {'FINISHED'}
        if event.type == 'LEFTMOUSE' and event.value == 'PRESS' and event.alt:
            region, rv3d, coord = viewport_region_and_coord(context, event)
            if region is None or rv3d is None:
                return {'RUNNING_MODAL'}
            hit = surface_hit_from_event(context, event, self._ground_z)
            if hit is not None:
                create_pin(context.scene, hit.x, hit.y, hit.z)
                context.scene.mqfollow2.info = f"{len(get_pins())} marker(s). Continue with Alt+Click or finish placement."
                if context.area is not None:
                    context.area.tag_redraw()
            return {'RUNNING_MODAL'}
        return {'PASS_THROUGH'}

    def cancel(self, context):
        context.workspace.status_text_set(None)
        if context.scene:
            context.scene.mqfollow2.placing_markers = False


class MQFOLLOW2_OT_stop_placing(Operator):
    bl_idname = "mqfollow2.stop_placing"
    bl_label = "Stop Marker Placement"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        context.scene.mqfollow2.placing_markers = False
        context.workspace.status_text_set(None)
        context.scene.mqfollow2.info = "Placement paused. Marker Mode remains active."
        return {'FINISHED'}


class MQFOLLOW2_OT_toggle_marker_mode(Operator):
    bl_idname = "mqfollow2.toggle_marker_mode"
    bl_label = "Toggle Marker Mode"
    bl_description = "Temporarily hide and disable MQScar live drive controls while using marker-based driving"

    def execute(self, context):
        f = context.scene.mqfollow2
        if f.marker_mode:
            f.marker_mode = False
            f.placing_markers = False
            context.workspace.status_text_set(None)
            _set_mqscar_marker_mode(context.scene, False)
            set_ui_sections(context.scene, False)       # back to Rigging, Curve Mode collapses
            f.info = "Marker Mode off. MQScar live controls are available again."
            return {'FINISHED'}
        f.marker_mode = True
        _set_mqscar_marker_mode(context.scene, True)
        set_ui_sections(context.scene, True)            # Curve Mode open, Rigging collapses
        result = bpy.ops.mqfollow2.place_pins('INVOKE_DEFAULT')
        if 'CANCELLED' in result:
            f.marker_mode = False
            _set_mqscar_marker_mode(context.scene, False)
            set_ui_sections(context.scene, False)
            return {'CANCELLED'}
        f.info = "Marker Mode active. MQScar live controls are hidden and neutralized."
        return {'FINISHED'}


class MQFOLLOW2_OT_start_placing(Operator):
    bl_idname = "mqfollow2.start_placing"
    bl_label = "Start Marker Placement"
    bl_description = "Resume Alt-click marker placement without leaving Marker Mode"

    def execute(self, context):
        if not context.scene.mqfollow2.marker_mode:
            self.report({'ERROR'}, "Activate Marker Mode first")
            return {'CANCELLED'}
        result = bpy.ops.mqfollow2.place_pins('INVOKE_DEFAULT')
        return {'FINISHED'} if 'RUNNING_MODAL' in result else result


class MQFOLLOW2_OT_remove_pin(Operator):
    bl_idname = "mqfollow2.remove_pin"
    bl_label = "Delete Selected Marker"
    bl_description = "Delete selected route marker(s) and automatically renumber the remaining route"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return bool(context and context.mode == 'OBJECT' and any(PIN_KEY in o for o in context.selected_objects))

    def execute(self, context):
        targets = [o for o in context.selected_objects if PIN_KEY in o]
        if not targets:
            self.report({'INFO'}, "Select a route marker first")
            return {'CANCELLED'}
        for pin in targets:
            bpy.data.objects.remove(pin, do_unlink=True)
        renumber_pins()
        remaining = get_pins()
        bpy.ops.object.select_all(action='DESELECT')
        if remaining:
            active = remaining[min(len(remaining)-1, 0)]
            active.hide_set(False)
            active.select_set(True)
            context.view_layer.objects.active = active
        context.scene.mqfollow2.info = f"Deleted {len(targets)} marker(s); remaining markers renumbered."
        return {'FINISHED'}


class MQFOLLOW2_OT_toggle_markers(Operator):
    bl_idname = "mqfollow2.toggle_markers"
    bl_label = "Toggle Route Markers"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        p = context.scene.mqfollow2
        p.show_markers = not p.show_markers
        set_markers_visible(context.scene, p.show_markers)
        return {'FINISHED'}


class MQFOLLOW2_OT_toggle_lines(Operator):
    bl_idname = "mqfollow2.toggle_lines"
    bl_label = "Toggle Route Lines"
    bl_description = "Show or hide the route line, the reverse lines and the speed graph (markers are not affected)"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        p = context.scene.mqfollow2
        p.show_lines = not p.show_lines
        set_lines_visible(context.scene, p.show_lines)
        return {'FINISHED'}


class MQFOLLOW2_OT_select_markers(Operator):
    bl_idname = "mqfollow2.select_markers"
    bl_label = "Select Route Markers"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        pins = get_pins()
        bpy.ops.object.select_all(action='DESELECT')
        for pin in pins:
            pin.hide_set(False)
            pin.select_set(True)
        if pins:
            context.view_layer.objects.active = pins[0]
        context.scene.mqfollow2.show_markers = True
        context.scene.mqfollow2.info = "Move selected markers with G in Object Mode. Bake to rebuild the route."
        return {'FINISHED'}


# --------------------------------------------------------------------------
# Jump feature: add a height arc to the baked animation without changing the route
# --------------------------------------------------------------------------
def _jump_arc_height(t, peak):
    return peak * 4.0 * t * (1.0 - t)

def _jump_auto_peak(air_frames, fps):
    duration = max(float(air_frames), 0.0) / max(float(fps), 1e-6)
    return min(JUMP_MAX_AUTO_HEIGHT, 0.125 * JUMP_GRAVITY * duration * duration)

def _jump_tilt_angle(t, tilt):
    return tilt * sin(2.0 * pi * t)

def _jump_pairs(pins):
    return [i for i, pin in enumerate(pins[:-1]) if bool(getattr(pin, JUMP_FLAG, False))]

def _jump_selected_numbers(context):
    return [int(obj[PIN_KEY]) for obj in context.selected_objects if PIN_KEY in obj]

def _jump_plan_selected(numbers, count):
    nums = sorted(set(int(n) for n in numbers if 1 <= int(n) <= count))
    if len(nums) == 1:
        return [nums[0]] if nums[0] < count else []
    return [n for n in nums if n + 1 in nums]

def _jump_find_frame(positions, xy, start, tol=JUMP_FIND_TOL):
    if not positions:
        return 0, float("inf")
    start = max(0, min(int(start), len(positions) - 1))
    best, best_i = float("inf"), start
    for i in range(start, len(positions)):
        d = hypot(positions[i][0] - xy[0], positions[i][1] - xy[1])
        if d < best - 1e-9:
            best, best_i = d, i
        elif best < tol and d > best + tol:
            break
    return best_i, best

def _jump_eval_location(obj, frame):
    ad = obj.animation_data
    loc = list(obj.location)
    if ad is not None and ad.action is not None:
        for idx in range(3):
            fc = ad.action.fcurves.find("location", index=idx)
            if fc is not None:
                loc[idx] = fc.evaluate(frame)
    return loc

def _jump_axle_positions(wheels, frames, role_names, role_to_obj):
    result = []
    for frame in frames:
        xy = [0.0, 0.0]
        for role in role_names:
            loc = _jump_eval_location(role_to_obj[role], frame)
            xy[0] += loc[0]
            xy[1] += loc[1]
        result.append((xy[0] / len(role_names), xy[1] / len(role_names)))
    return result

def apply_jumps(context):
    """Apply configured jumps to the just-baked keys. Returns (count, message)."""
    scene = context.scene
    pins = get_pins()
    jump_starts = _jump_pairs(pins)
    if not jump_starts:
        return 0, ""

    span = scene.get("mqfollow_span")
    baked_names = list(scene.get(PARTS_KEY, []))
    if not span or not baked_names:
        return 0, "Jump: no baked vehicle animation was found."
    frame_start, frame_end = int(span[0]), int(span[1])
    frames = list(range(frame_start, frame_end + 1))
    fps = scene.render.fps / max(scene.render.fps_base, 1e-6)

    parts = {name: bpy.data.objects.get(name) for name in baked_names}
    parts = {name: obj for name, obj in parts.items() if obj is not None}
    props = scene.mqscar
    role_to_obj = {role: getattr(props, role, None) for role in WHEEL_ATTRS}
    missing_roles = [role for role, obj in role_to_obj.items() if obj is None]
    if missing_roles:
        return 0, "Jump: missing wheel assignments: " + ", ".join(missing_roles)
    role_to_name = {role: obj.name for role, obj in role_to_obj.items()}
    missing_names = [name for name in role_to_name.values() if name not in parts]
    if missing_names:
        return 0, "Jump: wheel objects are not part of the baked animation: " + ", ".join(missing_names)
    if any(obj.parent is not None for obj in parts.values()):
        return 0, "Jump: one or more baked car parts have parents; cannot safely apply world-space jump keys."

    reverse_marks = [bool(getattr(pin, "mqfollow_reverse_mark", False)) for pin in pins]
    gears = gear_states(reverse_marks)
    front_positions = _jump_axle_positions(parts, frames, FRONT_ATTRS, role_to_obj)
    rear_positions = _jump_axle_positions(parts, frames, ("wheel_rl", "wheel_rr"), role_to_obj)

    marker_frames = {}
    cursor = 0
    for index, pin in enumerate(pins):
        reverse_leg = bool(gears[index - 1]) if index > 0 else False
        positions = rear_positions if reverse_leg else front_positions
        xy = (pin.matrix_world.translation.x, pin.matrix_world.translation.y)
        found, distance = _jump_find_frame(positions, xy, cursor)
        marker_frames[index] = (found, distance)
        cursor = found

    jumps = []
    for index in jump_starts:
        takeoff = marker_frames[index][0]
        lead_positions = rear_positions if (index > 0 and gears[index - 1]) else front_positions
        while takeoff + 1 < len(frames) and hypot(
                lead_positions[takeoff + 1][0] - lead_positions[takeoff][0],
                lead_positions[takeoff + 1][1] - lead_positions[takeoff][1]) < 1e-4:
            takeoff += 1
        landing = marker_frames[index + 1][0]
        if landing - takeoff < JUMP_MIN_AIR_FRAMES:
            continue
        pin = pins[index]
        if bool(getattr(pin, JUMP_AUTO, True)):
            peak = _jump_auto_peak(landing - takeoff, fps)
        else:
            peak = max(0.0, float(getattr(pin, JUMP_HEIGHT, 1.0)))
        tilt = radians(float(getattr(pin, JUMP_TILT, 6.0)))
        if gears[index]:
            tilt = -tilt
        jumps.append((index, frames[takeoff], frames[landing], peak, tilt))

    if not jumps:
        return 0, "Jump: selected markers are too close together or were not reached."

    restore_frame = scene.frame_current
    affected = sorted({frame for _i, a, b, _h, _t in jumps for frame in range(a, b + 1)})
    snapshots = {}
    for frame in affected:
        scene.frame_set(frame)
        snapshots[frame] = {name: obj.matrix_world.copy() for name, obj in parts.items()}

    done = 0
    for _index, takeoff, landing, peak, tilt in jumps:
        duration = float(landing - takeoff)
        for frame in range(takeoff, landing + 1):
            t = (frame - takeoff) / duration
            dz = _jump_arc_height(t, peak)
            angle = _jump_tilt_angle(t, tilt)
            mats = snapshots[frame]
            # The baked part matrices are keyed by Blender object names, not wheel role IDs.
            fl = mats[role_to_name["wheel_fl"]].translation
            fr = mats[role_to_name["wheel_fr"]].translation
            rl = mats[role_to_name["wheel_rl"]].translation
            rr = mats[role_to_name["wheel_rr"]].translation
            center = (fl + fr + rl + rr) * 0.25
            left_axis = ((fl + rl) * 0.5) - ((fr + rr) * 0.5)
            left_axis.z = 0.0
            if left_axis.length < 1e-6:
                left_axis = Vector((0.0, 1.0, 0.0))
            left_axis.normalize()
            rotation = (Matrix.Translation(center) @ Matrix.Rotation(-angle, 4, left_axis)
                        @ Matrix.Translation(-center))
            translation = Matrix.Translation((0.0, 0.0, dz))
            for name, obj in parts.items():
                obj.matrix_world = translation @ rotation @ mats[name]
                key_object(obj, frame)
        done += 1

    try:
        scene.frame_set(restore_frame)
    except Exception:
        pass
    return done, f"Jump: added {done} jump(s)."


class MQFOLLOW2_OT_add_jump(Operator):
    bl_idname = "mqfollow2.add_jump"
    bl_label = "Add Jump Between Selected Markers"
    bl_description = "Select adjacent markers, or one marker, to jump from it to the next marker"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        pins = get_pins()
        numbers = _jump_selected_numbers(context)
        active = context.view_layer.objects.active
        if not numbers and active is not None and PIN_KEY in active:
            numbers = [int(active[PIN_KEY])]
        targets = _jump_plan_selected(numbers, len(pins))
        if not targets:
            self.report({'WARNING'}, "Select two neighbouring markers (e.g. 5 and 6), or one marker that has a next marker.")
            return {'CANCELLED'}
        for number in targets:
            setattr(pins[number - 1], JUMP_FLAG, True)
        _jump_recolor_markers()
        context.scene.mqfollow2.info = "Jump set: " + ", ".join(f"{n} to {n + 1}" for n in targets) + ". Bake again to apply."
        return {'FINISHED'}


class MQFOLLOW2_OT_remove_jump(Operator):
    bl_idname = "mqfollow2.remove_jump"
    bl_label = "Remove Jump"
    bl_description = "Remove jump settings from selected marker(s)"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        pins = get_pins()
        numbers = _jump_selected_numbers(context)
        active = context.view_layer.objects.active
        if not numbers and active is not None and PIN_KEY in active:
            numbers = [int(active[PIN_KEY])]
        for number in numbers:
            if 1 <= number <= len(pins):
                setattr(pins[number - 1], JUMP_FLAG, False)
        _jump_recolor_markers()
        context.scene.mqfollow2.info = "Jump removed. Bake again to apply."
        return {'FINISHED'}

def _jump_recolor_markers():
    pins = get_pins()
    reverse = [bool(getattr(pin, "mqfollow_reverse_mark", False)) for pin in pins]
    gears = gear_states(reverse)
    highlighted = set()
    for index in _jump_pairs(pins):
        highlighted.update((index, index + 1))
    for index, pin in enumerate(pins):
        try:
            pin.color = JUMP_COLOR if index in highlighted else (
                (0.2, 0.5, 1.0, 1.0) if gears[index] else (1.0, 0.35, 0.05, 1.0))
        except Exception:
            pass


class MQFOLLOW2_OT_drive(Operator):
    bl_idname = "mqfollow2.drive"
    bl_label = "Bake / Update Drive"
    bl_description = "Draw one line through the pins and animate the car along it"
    bl_options = {'REGISTER', 'UNDO'}

    def _execute_bake(self, context):
        sc = context.scene
        f = sc.mqfollow2
        ok, msg = rig_ready(sc)
        if not ok:
            self.report({'ERROR'}, msg)
            return {'CANCELLED'}
        pins = get_pins()
        p = sc.mqscar
        info = read_rest(p)
        if info is None:
            self.report({'ERROR'}, "Front and rear wheels are in the same place")
            return {'CANCELLED'}
        rest, wb, radius, track, parts = info
        z = ground_z(rest, radius)

        screen = context.screen
        if screen is not None and screen.is_animation_playing:
            bpy.ops.screen.animation_cancel(restore_frame=False)
        if not pins:
            self.report({'ERROR'}, "Place route markers first, then Bake / Update Drive")
            return {'CANCELLED'}
        # Remove the previous vehicle bake only after basic route validation.
        clear_drive(sc, restore=False)
        # Marker objects are the source of truth; re-project each marker onto the selected
        # Ground Collection before rebuilding so moved markers cannot float above/below it.
        if f.surface_snap:
            if ground_collection(sc) is None:
                self.report({'ERROR'}, "Select a Ground Collection in the MQScar panel first.")
                return {'CANCELLED'}
            for pin in pins:
                hit = project_point_to_ground(context, pin.matrix_world.translation)
                if hit is None:
                    self.report({'ERROR'}, f"Marker {pin[PIN_KEY]} is not above the selected Ground Collection.")
                    return {'CANCELLED'}
                pin.location = hit
        pin_xy = [(o.matrix_world.translation.x, o.matrix_world.translation.y) for o in pins]
        fps = sc.render.fps / sc.render.fps_base
        dt = 1.0 / fps
        def _is_mark(pin_):
            return bool(getattr(pin_, "mqfollow_reverse_mark", False))
        enforce_reverse_speed_limit(pins)
        reverse_mode = any(_is_mark(pin) for pin in pins[:-1])
        gear_after = gear_states([_is_mark(pin) for pin in pins])
        for pin, in_reverse in zip(pins, gear_after):
            try:
                pin.color = (0.2, 0.5, 1.0, 1.0) if in_reverse else (1.0, 0.35, 0.05, 1.0)
            except Exception:
                pass
        for ob_old in [o for o in bpy.data.objects if o.name.startswith(PATH_NAME + "_Leg")]:
            _remove_obj(ob_old.name)
        if reverse_mode:
            if context.object is not None and context.object.mode == 'EDIT':
                try:
                    bpy.ops.object.mode_set(mode='OBJECT')
                except Exception:
                    pass
            markers = []
            for pin in pins:
                markers.append(dict(
                    xy=(pin.matrix_world.translation.x, pin.matrix_world.translation.y),
                    speed=float(pin.get(PIN_SPEED, DEFAULT_SPEED_KMH)),
                    hold=float(pin.get(PIN_BRAKE_HOLD, 0.0)),
                    reverse=_is_mark(pin), obj=pin))

            def make_path(start_xy, heading, pins_xy, pin_objs, leg_index, is_reverse):
                name = PATH_NAME if leg_index == 0 else f"{PATH_NAME}_Leg{leg_index + 1}"
                color = (0.2, 0.5, 1.0, 1.0) if is_reverse else (0.1, 1.0, 0.25, 1.0)
                if is_reverse and leg_index > 0:
                    # v24: reverse legs get a drivable line (smooth start, no tighter bend than the
                    # front wheels can make) instead of a curve that can fold back on itself.
                    line_ = drivable_reverse_line(start_xy, heading, pins_xy, wb)
                    if line_ is not None:
                        create_poly_route_curve(sc, line_, z, wb, name, color)
                        return line_, distances_for_points(line_)
                route_obj = create_route_curve(sc, start_xy, heading, wb, pins_xy, z,
                                               name=name, pin_objs=pin_objs, color=color)
                pts_ = evaluated_route_samples(route_obj, context.evaluated_depsgraph_get())
                return pts_, distances_for_points(pts_)

            try:
                frames, arrived, legs_info = plan_legs(
                    markers, rest["rear0"][:2], rest["yaw0"], wb, dt, track,
                    int(MAX_SECONDS * fps), make_path)
            except Exception as exc:
                self.report({'ERROR'}, f"Could not build the reverse route: {exc}")
                return {'CANCELLED'}
            rebuild_speed_graph_legs(sc, [(p_, d_, pr_) for p_, d_, pr_ in legs_info], z, DEFAULT_SPEED_KMH)
            line_len = sum(max(0.0, d_[-1] - wb) for _p, d_, _pr in legs_info)
        else:
            route = create_route_curve(sc, rest["rear0"][:2], rest["yaw0"], wb, pin_xy, z)
            # Ensure Blender has evaluated the latest edit-mode curve before sampling.
            if context.object is not None and context.object.mode == 'EDIT':
                try:
                    bpy.ops.object.mode_set(mode='OBJECT')
                except Exception:
                    pass
            try:
                pts = evaluated_route_samples(route, context.evaluated_depsgraph_get())
                s = distances_for_points(pts)
            except Exception as exc:
                self.report({'ERROR'}, f"Could not evaluate driving curve: {exc}")
                return {'CANCELLED'}
            speed_pairs, stop_holds = _marker_speed_data(pins, pts, s, DEFAULT_SPEED_KMH)
            rebuild_speed_graph(sc, pts, s, z, DEFAULT_SPEED_KMH, wb,
                                f.acceleration_strength, f.braking_strength)

            fps = sc.render.fps / sc.render.fps_base
            dt = 1.0 / fps
            frames, arrived = plan_drive(
                pts, s, wb, DEFAULT_SPEED_KMH / 3.6, dt, track,
                int(MAX_SECONDS * fps), speed_keys=speed_pairs, stop_holds=stop_holds,
                acceleration_strength=f.acceleration_strength, braking_strength=f.braking_strength,
                spin_launch=(float(pins[0].get(PIN_SPEED, DEFAULT_SPEED_KMH)) >= LAUNCH_SPIN_MIN_KMH)
            )
            line_len = max(0.0, s[-1] - wb)

        # Write the animation. The car parts follow the keys exactly (Animated rigid bodies).
        start = sc.frame_start
        for name, obj in parts.items():
            if obj.rigid_body is not None:
                obj["mqf_prev_kinematic"] = bool(obj.rigid_body.kinematic)
                obj.rigid_body.kinematic = True
            ad = obj.animation_data
            if ad is None or ad.action is None:
                obj[OWN_ACTION] = 1
        spin = 0.0
        last = 0.0
        last_leg = 0
        for i, fr in enumerate(frames):
            if fr.get("leg", 0) != last_leg:
                last = 0.0              # distance restarts at the start of every leg
                last_leg = fr.get("leg", 0)
            spin += fr.get("gear", 1) * (fr["dist"] - last) / radius    # wheels spin backwards in reverse
            last = fr["dist"]
            mats = part_matrices(fr, rest, wb, radius, track, spin)
            for name, obj in parts.items():
                obj.matrix_world = Matrix(mats[name])
                key_object(obj, start + i)
        sc[PARTS_KEY] = [o.name for o in parts.values()]
        sc["mqfollow_span"] = [start, start + len(frames) - 1]

        end = start + len(frames) + int(fps)
        if sc.frame_end < end:
            sc.frame_end = end
        rbw = sc.rigidbody_world
        if rbw is not None:
            try:
                rbw.point_cache.frame_end = max(sc.frame_end, start + 1)
            except Exception:
                pass
        try:
            sc.frame_set(start)
        except Exception:
            pass

        if arrived:
            f.info = (f"Ready: line {line_len:.1f} m long, car arrives at frame "
                      f"{start + len(frames) - 1} (~{len(frames) / fps:.1f} s). Press Play.")
        else:
            f.info = "The route was not completed within the time limit. Shorten the route or raise the marker target speeds."
            self.report({'WARNING'}, f.info)
        return {'FINISHED'}


    def execute(self, context):
        try:
            result = self._execute_bake(context)
            if result == {'FINISHED'}:
                try:
                    jump_count, jump_message = apply_jumps(context)
                    _jump_recolor_markers()
                    if jump_message:
                        current_info = context.scene.mqfollow2.info or ""
                        context.scene.mqfollow2.info = (current_info + "  " + jump_message).strip()
                        if jump_count == 0:
                            self.report({'WARNING'}, jump_message)
                except Exception as jump_exc:
                    import traceback
                    traceback.print_exc()
                    warning = f"Drive baked, jump failed: {type(jump_exc).__name__}: {jump_exc}"
                    context.scene.mqfollow2.info = warning
                    self.report({'WARNING'}, warning)
                if context.area is not None:
                    context.area.tag_redraw()
            return result
        except Exception as exc:
            import traceback
            detail = traceback.format_exc(limit=5)
            message = f"Bake failed: {type(exc).__name__}: {exc}"
            self.report({'ERROR'}, message)
            context.scene.mqfollow2.info = message
            print("\n[MQScar Follow] BAKE FAILURE\n" + detail)
            return {'CANCELLED'}


class MQFOLLOW2_OT_clear(Operator):
    bl_idname = "mqfollow2.clear"
    bl_label = "Clear (back to physics)"
    bl_description = "Remove the drive and the green line. The car goes back to its start; press Reset Car"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        sc = context.scene
        clear_drive(sc, restore=True)
        sc.mqfollow2.info = "Vehicle bake cleared. Editable route markers were kept."
        try:
            sc.frame_set(sc.frame_start)
        except Exception:
            pass
        return {'FINISHED'}


# --------------------------------------------------------------------------
# UI
# --------------------------------------------------------------------------
def draw_follow(lay, context):
    """Curve Mode UI. Drawn inside the combined MQScar panel (see __init__.py)."""
    sc = context.scene
    f = sc.mqfollow2
    ok, msg = rig_ready(sc)
    lay.label(text=msg, icon='CHECKMARK' if ok else 'ERROR')
    if not ok:
        return

    pins = get_pins()
    if not f.marker_mode:
        lay.operator("mqfollow2.toggle_marker_mode", text="Activate Marker Mode", icon='EMPTY_AXIS')
        if pins:
            lay.label(text=f"Saved route: {len(pins)} marker(s)", icon='CURVE_BEZCURVE')
        return

    header = lay.row()
    header.alert = True
    header.label(text="MARKER MODE ACTIVE — MQScar live controls disabled", icon='RADIOBUT_ON')
    lay.operator("mqfollow2.toggle_marker_mode", text="Exit Marker Mode", icon='CANCEL')

    if f.placing_markers:
        row = lay.row()
        row.alert = True
        row.label(text="ALT + LEFT-CLICK ON GROUND TO ADD MARKERS", icon='MOUSE_LMB')
        lay.operator("mqfollow2.stop_placing", text="Pause Placement (Keep Marker Mode)", icon='CHECKMARK')
    else:
        lay.operator("mqfollow2.start_placing", text="Start / Resume Placement (Alt + Click)", icon='EMPTY_AXIS')

    lay.operator("mqfollow2.select_markers", text="Select All Markers", icon='RESTRICT_SELECT_OFF')
    row = lay.row(align=True)
    row.operator("mqfollow2.toggle_markers", text="Markers", icon='HIDE_OFF' if f.show_markers else 'HIDE_ON')
    row.operator("mqfollow2.toggle_lines", text="Lines", icon='HIDE_OFF' if f.show_lines else 'HIDE_ON')

    active = context.view_layer.objects.active
    active_pin = active if active is not None and PIN_KEY in active else None
    if active_pin is not None:
        ensure_pin_controls(active_pin)
        speed_box = lay.box()
        speed_box.label(text="Selected Marker", icon='EMPTY_AXIS')
        speed_box.prop(f, "marker_number", text="Marker Number")
        speed_box.prop(active_pin, f'["{PIN_SPEED}"]', text="Target Speed (km/h)", slider=False)
        if getattr(active_pin, "mqfollow_reverse_mark", False):
            speed_box.label(text=f"Reverse Marker: speed is limited to {REVERSE_MAX_KMH:.0f} km/h.", icon='INFO')
        else:
            speed_box.label(text="No fixed upper speed limit — type the value you need.", icon='INFO')
        if pins and active_pin is pins[0]:
            if float(active_pin.get(PIN_SPEED, 0.0)) >= LAUNCH_SPIN_MIN_KMH:
                speed_box.label(text="First marker, high speed: rear wheels spin, then launch.", icon='FORCE_VORTEX')
            else:
                speed_box.label(text=f"First marker: at {LAUNCH_SPIN_MIN_KMH:.0f} km/h or more the rear wheels spin before launch.", icon='INFO')
        speed_box.prop(active_pin, f'["{PIN_BRAKE_HOLD}"]', text="Brake Hold (seconds)")
        speed_box.prop(active_pin, "mqfollow_reverse_mark", text="Reverse Marker")
        if getattr(active_pin, "mqfollow_reverse_mark", False):
            if pins and active_pin is pins[-1]:
                speed_box.label(text="Last marker: nothing after it, so this has no effect.", icon='ERROR')
            else:
                speed_box.label(text="Gear switches here and stays until the next Reverse Marker.", icon='LOOP_BACK')
                if float(active_pin.get(PIN_SPEED, 0.0)) >= DRIFT_MIN_KMH:
                    speed_box.label(text="High speed: drift spin, slows a little, then accelerates again.", icon='FORCE_VORTEX')
                else:
                    speed_box.label(text="The car stops, pauses, then pulls away in the new gear.")
        speed_box.prop(active_pin, f'["{PIN_SMOOTH}"]', text="Route Smoothness", slider=True)
        speed_box.label(text="0.5 = 500 ms  |  1.0 = 1 second", icon='TIME')
        if float(active_pin.get(PIN_SPEED, DEFAULT_SPEED_KMH)) <= 0.001:
            speed_box.label(text="STOP — car brakes to zero at this marker.", icon='PAUSE')
            speed_box.label(text=f"Hold for {float(active_pin.get(PIN_BRAKE_HOLD, 0.0)):.2f} seconds.")
    else:
        lay.label(text="Click a route marker in the viewport to edit its settings.", icon='INFO')

    jump_box = lay.box()
    jump_box.label(text="Jump", icon='TRIA_UP_BAR')
    jump_row = jump_box.row(align=True)
    jump_row.scale_y = 1.2
    jump_row.operator("mqfollow2.add_jump", text="Add Jump (selected)", icon='ADD')
    jump_row.operator("mqfollow2.remove_jump", text="Remove", icon='X')
    jump_box.label(text="Select adjacent markers (e.g. 5 + 6), or one start marker.")
    if active_pin is not None:
        jump_box.prop(active_pin, JUMP_FLAG, text=f"Jump: marker {int(active_pin[PIN_KEY])} to next")
        if bool(getattr(active_pin, JUMP_FLAG, False)):
            if pins and active_pin is pins[-1]:
                jump_box.label(text="Last marker: there is no next marker.", icon='ERROR')
            else:
                jump_box.prop(active_pin, JUMP_AUTO, text="Auto Height (from air time)")
                height_col = jump_box.column()
                height_col.active = not bool(getattr(active_pin, JUMP_AUTO, True))
                height_col.prop(active_pin, JUMP_HEIGHT, text="Jump Height (m)")
                jump_box.prop(active_pin, JUMP_TILT, text="Nose Tilt (deg)")
    jump_count = len(_jump_pairs(pins))
    if jump_count:
        jump_box.label(text=f"{jump_count} jump(s) configured. Bake again after changes.", icon='INFO')

    dynamics = lay.box()
    dynamics.label(text="Drive Response", icon='TIME')
    dynamics.prop(f, "acceleration_strength", slider=True)
    dynamics.prop(f, "braking_strength", slider=True)
    dynamics.label(text="Higher acceleration = faster speed increase.")
    dynamics.label(text="Higher braking = shorter, stronger braking distance.")

    row = lay.row()
    row.scale_y = 1.5
    row.operator("mqfollow2.drive", text="Bake / Update Drive", icon='REC')
    lay.operator("mqfollow2.clear", text="Clear Vehicle Bake (Keep Route)", icon='X')
    if f.info:
        lay.label(text=f.info, icon='INFO')
    lay.label(text="After changing marker speeds or response times, Bake again.")


# Marker visibility follows timeline playback while preserving the manual preference.
def _mqfollow_marker_visibility_timer():
    try:
        screen = bpy.context.screen
        playing = bool(screen and screen.is_animation_playing)
        scene = bpy.context.scene
        if scene is not None and hasattr(scene, "mqfollow2"):
            desired = False if playing else bool(scene.mqfollow2.show_markers)
            all_pins = get_pins()
            enforce_reverse_speed_limit(all_pins)      # v25: markers with Reverse checked are capped at 100 km/h
            for pin in all_pins:
                if pin.hide_get() == desired:
                    pin.hide_set(not desired)
            want_lines = bool(scene.mqfollow2.show_lines)
            for ob in get_line_objects():
                try:
                    if ob.hide_get() == want_lines:
                        ob.hide_set(not want_lines)
                except Exception:
                    pass
    except Exception:
        pass
    return 0.25


# --------------------------------------------------------------------------
# Registration (safe to re-run from the Text Editor)
# --------------------------------------------------------------------------
classes = (MQFOLLOW2_Props, MQFOLLOW2_OT_place_pins, MQFOLLOW2_OT_stop_placing,
           MQFOLLOW2_OT_toggle_marker_mode, MQFOLLOW2_OT_start_placing,
           MQFOLLOW2_OT_remove_pin, MQFOLLOW2_OT_toggle_markers, MQFOLLOW2_OT_toggle_lines,
           MQFOLLOW2_OT_select_markers, MQFOLLOW2_OT_add_jump, MQFOLLOW2_OT_remove_jump,
           MQFOLLOW2_OT_drive, MQFOLLOW2_OT_clear)
OLD_CLASSES = ("MQFOLLOW2_OT_pin", "MQFOLLOW2_OT_bake", "MQFOLLOW2_OT_speed_segments", "MQFOLLOW2_PT_main")
addon_keymaps = []


def unregister():
    try:
        scene = bpy.context.scene
        if scene is not None and hasattr(scene, "mqfollow2") and scene.mqfollow2.marker_mode:
            _set_mqscar_marker_mode(scene, False)
    except Exception:
        pass
    for km, kmi in addon_keymaps:
        try:
            km.keymap_items.remove(kmi)
        except Exception:
            pass
    addon_keymaps.clear()
    try:
        if bpy.app.timers.is_registered(_mqfollow_marker_visibility_timer):
            bpy.app.timers.unregister(_mqfollow_marker_visibility_timer)
    except Exception:
        pass
    if hasattr(bpy.types.Scene, "mqfollow2"):
        del bpy.types.Scene.mqfollow2
    for prop_name in ("mqfollow_reverse_mark", JUMP_FLAG, JUMP_AUTO, JUMP_HEIGHT, JUMP_TILT):
        if hasattr(bpy.types.Object, prop_name):
            try:
                delattr(bpy.types.Object, prop_name)
            except Exception:
                pass
    for name in [c.__name__ for c in classes] + list(OLD_CLASSES):
        cls = getattr(bpy.types, name, None)
        if cls is not None:
            try:
                bpy.utils.unregister_class(cls)
            except Exception:
                pass


def register():
    unregister()
    for c in classes:
        bpy.utils.register_class(c)
    bpy.types.Scene.mqfollow2 = PointerProperty(type=MQFOLLOW2_Props)
    bpy.types.Object.mqfollow_reverse_mark = BoolProperty(
        name="Reverse Marker", default=False,
        description="Switch gear at this marker: forward becomes reverse, reverse becomes forward. "
                    "The new gear stays until the next Reverse Marker")
    bpy.types.Object.mqfollow_jump_mark = BoolProperty(
        name="Jump to next marker", default=False,
        description="The car leaves the ground at this marker and lands at the next marker")
    bpy.types.Object.mqfollow_jump_auto = BoolProperty(
        name="Auto Height", default=True,
        description="Calculate jump height from the time between take-off and landing")
    bpy.types.Object.mqfollow_jump_height = FloatProperty(
        name="Jump Height", default=1.0, min=0.0, soft_max=5.0, unit='LENGTH',
        description="Peak jump height when Auto Height is disabled")
    bpy.types.Object.mqfollow_jump_tilt = FloatProperty(
        name="Nose Tilt", default=6.0, min=0.0, max=45.0,
        description="Nose-up / nose-down tilt during the jump, in degrees")
    try:
        if not bpy.app.timers.is_registered(_mqfollow_marker_visibility_timer):
            bpy.app.timers.register(_mqfollow_marker_visibility_timer, first_interval=0.25, persistent=True)
        wm = bpy.context.window_manager
        kc = wm.keyconfigs.addon if wm else None
        if kc:
            km = kc.keymaps.new(name='3D View', space_type='VIEW_3D')
            for key in ('X', 'DEL'):
                kmi = km.keymap_items.new('mqfollow2.remove_pin', key, 'PRESS')
                addon_keymaps.append((km, kmi))
    except Exception:
        pass


