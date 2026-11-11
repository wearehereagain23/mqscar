bl_info = {
    "name": "MQScar",
    "author": "MQS",
    "version": (0, 21, 0),
    "blender": (3, 6, 0),
    "location": "View3D > Sidebar (N) > MQScar",
    "description": "Rigid body car rig generator with car-type presets",
    "category": "Rigging",
}

import time
import traceback
import bpy
from math import radians, degrees, sqrt, atan2, pi, floor, hypot, sin
from mathutils import Vector, Matrix
from bpy.props import (
    PointerProperty, FloatProperty, EnumProperty, BoolProperty, StringProperty,
)
from bpy.types import Panel, Operator, PropertyGroup
from bpy.app.handlers import persistent

# --------------------------------------------------------------------------
# Constants
# --------------------------------------------------------------------------
RIG_COLLECTION = "MQScar_Rig"
GROUND_COLLECTION = "MQScar_Ground"
GROUND_PLANE = "MQScar_GroundPlane"

TAG_RB = "mqscar_rb"            # objects we added a rigid body to
TAG_CON = "mqscar_constraint"   # constraint empties: 'susp' | 'axle' | 'motor'
TAG_HUB = "mqscar_hub"          # hub objects we created
TAG_OBS = "mqscar_obstacle"     # obstacle objects we gave a rigid body to

WHEEL_SLOTS = (
    ("wheel_fl", "Front Left"),
    ("wheel_fr", "Front Right"),
    ("wheel_rl", "Rear Left"),
    ("wheel_rr", "Rear Right"),
)
FRONT = ("wheel_fl", "wheel_fr")
REAR = ("wheel_rl", "wheel_rr")

# Rigid body collision layers (objects collide only if they share a layer)
LAYER_BODY, LAYER_WHEEL, LAYER_HUB = 0, 1, 19

GRAVITY = 9.81
STEER_STEP = 2.5  # degrees between the ready-made steering joints
STEER_K = 36  # joints cover -K..+K steps (= +-90 degrees), one switched on at a time
STEER_SOFT = 80.0  # slider drag range in degrees (drag all the way to 80); typing goes up to +-STEER_MAX
STEER_MAX = STEER_K * STEER_STEP
STEER_MAX_RAD = radians(STEER_MAX)
STEER_REF = 100.0  # km/h where 'calm steering at speed' halves the steering angle
BASE_SAG = 0.40  # static suspension sag as a fraction of travel
INSTANT_PUSH = 1.2  # engine torque per wheel in multiples of (mass * g * radius)
# Fixed values (these used to be Advanced sliders; a template can expose them later)
GRIP = 4.0  # tire friction: high enough that the car follows the typed speed at once
BRAKE_FORCE = 15.0  # brake strength in m/s2
SUSP_TRAVEL = 0.25  # suspension travel as a fraction of the wheel radius
SUSP_FIRMNESS = 1.0  # spring stiffness multiplier (1 = standard for the car's weight)
# Drift preset (tune these two if the drift is too wild or too tame)
DRIFT_REAR_GRIP = 0.35  # rear tire grip at full drift, as a fraction of GRIP
DRIFT_STEER = radians(15.0)  # steering angle at which the drift is fully applied
# Vibration (tune these if the shake is too weak or too wild)
VIB_SPEED = 80.0  # km/h at which the vibration reaches full strength
VIB_DRIFT_BOOST = 0.6  # extra vibration while drifting (1 = double)
YAW_RATE = radians(70.0)  # Hold: how fast (deg/s) the car keeps turning at full steering and vibration 1
SHAKE_SIDE = 0.004  # Hold: sideways shake size as a fraction of the wheelbase, per unit of vibration
SHAKE_STRAIGHT = 0.25  # share of the sideways shake that is left when the steering is straight
# Vibration while driving (the physics sim). Sideways only, through one extra joint at the rear axle.
SIM_SHAKE_VEL = 0.35  # m/s sideways wobble speed per unit of vibration
SIM_PUSH_VEL = 1.2  # m/s sideways push at full steering: the rear is pulled to the OPPOSITE side
SIM_FORCE = 0.25  # strength limit of that push in multiples of the car's weight
SIM_REAR_SLIP = 0.4  # rear tires lose up to this share of grip with vibration 1 (so the tail can slide)
SIM_BANK_STEP = 0.15  # m/s between the ready-made vibration joints
SIM_BANK_N = 20  # joints cover -N..+N steps (= +-3 m/s); one is switched on at a time
SIM_MOTOR_SIGN = 1.0  # set to -1.0 if the tail slides TOWARD the steering side instead of away
OBSTACLE_DENSITY = 250.0  # kg/m3 of bounding box: starting weight of an obstacle (edit it per object)
DRIFT_CALM = 0.5  # how much of the 'calm steering at speed' reduction drift removes (0..1)

# One built-in profile. Everything else is set in Advanced.
#   density    : body mass per m^3 of bounding box (kg/m3) -> starting weight estimate
#   wheel_mass : wheel mass as a fraction of body mass
PROFILE = dict(label="Car", density=115, wheel_mass=0.015)


# --------------------------------------------------------------------------
# Property callbacks (these only touch our own scene properties, never the sim)
# --------------------------------------------------------------------------
def body_update(self, context):
    # Starting weight estimate; you can type your own weight afterwards
    if self.body is not None:
        self.mass = auto_mass(self)


def reset_sim(context):
    """Force Blender's rigid-body world to evaluate from a clean start state.

    Merely assigning substeps_per_frame to itself does not reliably invalidate
    Bullet's cached transforms. Stepping to the frame before the cache start and
    then back to the start forces a fresh simulation evaluation.
    """
    scene = context.scene
    rbw = scene.rigidbody_world
    if rbw is None:
        return
    try:
        rbw.enabled = True
        # Keep the point-cache range valid and explicitly dirty the cache settings.
        cache = rbw.point_cache
        cache.frame_start = scene.frame_start
        cache.frame_end = max(scene.frame_end, scene.frame_start + 1)
        rbw.substeps_per_frame = max(rbw.substeps_per_frame, 20)
        # Blender's rigid-body cache is reset by crossing back over its start.
        scene.frame_set(scene.frame_start - 1)
        scene.frame_set(scene.frame_start)
        context.view_layer.update()
    except Exception:
        traceback.print_exc()


def sim_idle(context):
    """True when nothing is simulating (stopped, on the first frame)."""
    scene = context.scene
    screen = context.screen
    playing = screen is not None and screen.is_animation_playing
    return (not playing) and scene.frame_current <= scene.frame_start


def settings_update(self, context):
    # Weight and damping are baked into the joints when the sim starts. Changing joints while
    # the car is running is what makes Blender's physics jump or explode, so only refresh when
    # the sim is stopped at the start; otherwise the new value is used at the next restart.
    try:
        if has_rig() and not self.hold and sim_idle(context):
            reset_sim(context)
    except Exception:
        traceback.print_exc()


def apply_ride_height(context):
    """Put the body at its start height: modelled pose + spring sag + Ride Height."""
    scene = context.scene
    p = scene.mqscar
    body = p.body
    if body is None or not has_rig() or _HOME_KEY not in body or "mq_sag" not in body:
        return
    stop_playback(context)
    home = _unflat(list(body[_HOME_KEY]))
    run = Matrix.Translation((0.0, 0.0, body["mq_sag"] + p.ride_height)) @ home
    body.matrix_world = run  # the joint empties are children of the body and follow it
    body["mq_run"] = _flat(run)
    reset_sim(context)
    scene.frame_set(scene.frame_start)


def ride_height_update(self, context):
    try:
        if self.hold:
            return  # applied when Hold is released
        apply_ride_height(context)
    except Exception:
        traceback.print_exc()


def preset_update(self, context):
    # Drivers read this plain number, so no comparisons are needed there
    self.drift_on = 1.0 if self.preset == 'DRIFT' else 0.0


def drive_mode_update(self, context):
    # Drivers read these two plain numbers, so no comparisons are needed there
    self.drive_front = 1.0 if self.drive_mode in {'ALL', 'FRONT'} else 0.0
    self.drive_rear = 1.0 if self.drive_mode in {'ALL', 'REAR'} else 0.0


# --------------------------------------------------------------------------
# HOLD = test stand.
# Blender's rigid body sim cannot change joints or spin speed while it runs, so
# Hold switches the sim off for the car and moves it directly instead:
#   * the car is frozen exactly where it is (no jump back to the start)
#   * wheels spin at the Speed value and the front wheels follow the Steering
#     slider, live, whether or not the timeline is playing
# Releasing Hold puts the car back at its start position, ready to drive.
# --------------------------------------------------------------------------
_HOLD = {}


def _flat(m):
    return [v for row in m for v in row]


def _unflat(vals):
    return Matrix([vals[i * 4:(i + 1) * 4] for i in range(4)])


def _redraw():
    try:
        for win in bpy.context.window_manager.windows:
            for area in win.screen.areas:
                if area.type == 'VIEW_3D':
                    area.tag_redraw()
    except Exception:
        pass


def _center(m, obj):
    """World-space center of an object's bounding box."""
    pts = [m @ Vector(v) for v in obj.bound_box]
    return sum(pts, Vector((0.0, 0.0, 0.0))) / 8.0


def _about(center, rot3, m0):
    """m0 rotated by rot3 around the world point 'center'."""
    return Matrix.Translation(center) @ rot3.to_4x4() @ Matrix.Translation(-center) @ m0


def _solve3(m, v):
    """Solve a 3x3 linear system (Cramer's rule). None if it is singular."""
    def det(a):
        return (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
                - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
                + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
    d = det(m)
    if abs(d) < 1e-18:
        return None
    out = []
    for i in range(3):
        mi = [row[:] for row in m]
        for r in range(3):
            mi[r][i] = v[r]
        out.append(det(mi) / d)
    return out


def _fit_circle(pts):
    """Least-squares circle through 2D points -> (u, v, radius), or None."""
    n = len(pts)
    if n < 3:
        return None
    mu = sum(p[0] for p in pts) / n
    mv = sum(p[1] for p in pts) / n
    a = [[0.0] * 3 for _ in range(3)]
    b = [0.0, 0.0, 0.0]
    for pu, pv in pts:
        u, v = pu - mu, pv - mv
        row = (2.0 * u, 2.0 * v, 1.0)
        y = u * u + v * v
        for i in range(3):
            b[i] += row[i] * y
            for j in range(3):
                a[i][j] += row[i] * row[j]
    sol = _solve3(a, b)
    if sol is None:
        return None
    cu, cv, k = sol
    r2 = k + cu * cu + cv * cv
    if r2 <= 0.0:
        return None
    return cu + mu, cv + mv, sqrt(r2)


def wheel_geometry(w, lat, m=None):
    """Find the TRUE spin centre, spin axis and radius of a wheel mesh.

    The old code used the middle of the bounding box as the wheel centre. For a round
    wheel that is only right if the mesh is perfectly symmetric (a valve stem, a brake
    disc on one side, an odd vertex count... all pull the box off-centre), and spinning
    around a point that is even 1-2 mm off the real axle makes the tire wobble.
    Here the centre across the wheel is a circle fitted through the outer rim vertices.

    lat : world direction the axle should roughly point along (unit vector)
    m   : world matrix to use (defaults to the object's own)
    Returns dict(center=<local-space point>, axis=<local axis 0/1/2 or None>, radius=<world units>)
    """
    me = w.data
    if me is None or len(me.vertices) == 0:
        return None
    m = m if m is not None else w.matrix_world
    sc = m.to_scale()
    s = Vector((abs(sc.x) or 1.0, abs(sc.y) or 1.0, abs(sc.z) or 1.0))
    pts = [Vector((v.co.x * s.x, v.co.y * s.y, v.co.z * s.z)) for v in me.vertices]
    mn = Vector((min(p.x for p in pts), min(p.y for p in pts), min(p.z for p in pts)))
    mx = Vector((max(p.x for p in pts), max(p.y for p in pts), max(p.z for p in pts)))
    size = mx - mn
    mid = (mn + mx) / 2.0
    big = max(size)
    if big <= 1e-9:
        return None

    def to_local(v):
        return Vector((v.x / s.x, v.y / s.y, v.z / s.z))

    rot = m.to_3x3().normalized()
    if lat is not None and lat.length > 1e-9:
        lat = lat.normalized()
        dots = [abs((rot @ e).dot(lat)) for e in (Vector((1, 0, 0)), Vector((0, 1, 0)), Vector((0, 0, 1)))]
    else:
        dots = [1.0, 1.0, 1.0]

    # The two equal sizes are the wheel's diameter; the odd one out is its width (the axle)
    ax = min(((0, 1, 2), (0, 2, 1), (1, 2, 0)), key=lambda t: abs(size[t[0]] - size[t[1]]))[2]
    if (big - min(size)) < 0.03 * big or dots[ax] < 0.85:
        best = max(range(3), key=lambda i: dots[i])
        ax = best if dots[best] >= 0.85 else None
    if ax is None:
        return dict(center=to_local(mid), axis=None, radius=max(big / 2.0, 0.01))

    i, j = [k for k in range(3) if k != ax]
    dist = [hypot(p[i] - mid[i], p[j] - mid[j]) for p in pts]
    ref = sorted(dist)[int(0.9 * (len(dist) - 1))]
    cu, cv, r = mid[i], mid[j], ref
    if ref > 1e-9:
        rim = [(p[i], p[j]) for p, d in zip(pts, dist) if 0.9 * ref <= d <= 1.05 * ref]
        fit = _fit_circle(rim)
        if (fit is not None and hypot(fit[0] - mid[i], fit[1] - mid[j]) <= 0.25 * ref
                and 0.7 * ref <= fit[2] <= 1.3 * ref):
            cu, cv, r = fit
    cen = Vector((0.0, 0.0, 0.0))
    cen[ax] = mid[ax]
    cen[i] = cu
    cen[j] = cv
    return dict(center=to_local(cen), axis=ax, radius=max(r, 0.01))


def lateral_dir(p):
    """Unit vector pointing from the left wheels to the right wheels (in the body's flat plane)."""
    pos = {a: getattr(p, a).matrix_world.translation for a, _l in WHEEL_SLOTS}
    lat = (pos["wheel_fr"] - pos["wheel_fl"]) + (pos["wheel_rr"] - pos["wheel_rl"])
    up = p.body.matrix_world.to_quaternion().to_matrix() @ Vector((0.0, 0.0, 1.0)) \
        if p.body is not None else Vector((0.0, 0.0, 1.0))
    lat -= up * lat.dot(up)
    return lat.normalized() if lat.length > 1e-9 else Vector((1.0, 0.0, 0.0))


def vibration_amp(p):
    """How hard the car shakes right now (0 = not at all). Grows with the Vibration slider,
    with speed and with drifting; zero when braking or stopped. Shared by Hold
    and simulation so both modes use the same vibration response."""
    if p.brake or abs(p.speed) < 0.05 or p.vibration <= 0.0:
        return 0.0
    v = p.vibration + 2.0 * max(0.0, p.vibration - 1.0) * max(0.0, p.vibration - 1.0)  # extreme values shake much harder
    sf = 0.3 + 0.7 * min(1.0, abs(p.speed) / VIB_SPEED)
    dm = 1.0 + VIB_DRIFT_BOOST * p.drift_on * p.drift_amount * min(1.0, abs(p.steer) / DRIFT_STEER)
    return min(v * sf * dm, 8.0)


def slide_rate(p, rear_sign):
    """Turning speed (radians/second, about 'up', pivoting at the front axle) that swings the
    REAR to the side opposite to where the front wheels point. It is a SPEED, not an angle:
    the car keeps turning (even all the way round) for as long as the steering is held, and
    stops turning when the steering is straight. rear_sign: +1 if a positive yaw moves the
    rear toward the car's right side."""
    amp = vibration_amp(p)
    if amp <= 0.0:
        return 0.0
    st = max(-1.0, min(1.0, p.steer / radians(20.0)))  # steering, -1 (left) .. +1 (right)
    return -rear_sign * st * YAW_RATE * min(1.5, amp)


def _vib_noise(t, seed):
    """Smooth pseudo-random wobble in -1..1 (a few slow sine waves that never line up)."""
    return (sin(t * (19.0 + seed * 3.1)) + 0.6 * sin(t * (29.0 + seed * 2.3) + seed)
            + 0.3 * sin(t * (43.0 + seed * 1.3) + 2.0 * seed)) / 1.9


def steer_yaw(p):
    """Front-wheel steering angle (radians, about 'up') for the Steering slider.
    Same rule the steering joints use in normal mode: positive = turn right (negative
    yaw), and the angle is reduced at speed so the car stays stable."""
    d = p.drift_on * p.drift_amount * min(1.0, abs(p.steer) / DRIFT_STEER)
    f = 1.0 + p.speed * p.speed / (STEER_REF * STEER_REF) * (1.0 - DRIFT_CALM * d)
    return -max(-STEER_MAX_RAD, min(STEER_MAX_RAD, p.steer)) / f


_HOLD_GUARD = [False]  # True while we switch 'hold' off ourselves (skips the update callback)


def _force_hold_off(p):
    _HOLD_GUARD[0] = True
    try:
        p.hold = False
    finally:
        _HOLD_GUARD[0] = False


def hold_start(context):
    """Returns True when the test stand is running."""
    scene = context.scene
    p = scene.mqscar
    if p.body is None or not has_rig() or scene.rigidbody_world is None:
        return False
    stop_playback(context)
    _restore_hold_poses()  # a Hold that failed earlier must not leave stale start poses behind
    dg = context.evaluated_depsgraph_get()

    hubs = {o[TAG_HUB]: o for o in bpy.data.objects if o.get(TAG_HUB)}
    wheels = {a: getattr(p, a) for a, _l in WHEEL_SLOTS}
    if any(w is None for w in wheels.values()) or len(hubs) < 4:
        return False

    poses = {}
    for obj in [p.body] + list(wheels.values()) + list(hubs.values()):
        poses[obj] = obj.evaluated_get(dg).matrix_world.copy()

    body_rot = poses[p.body].to_quaternion().to_matrix()
    up = (body_rot @ Vector((0.0, 0.0, 1.0))).normalized()
    c = {a: poses[w].translation.copy() for a, w in wheels.items()}
    # average of the front and rear axle lines is steadier than the front pair alone
    lat = (c["wheel_fr"] - c["wheel_fl"]) + (c["wheel_rr"] - c["wheel_rl"])
    lat -= up * lat.dot(up)
    if lat.length < 1e-6:
        return False
    axis = lat.normalized()
    fwd = (c["wheel_fl"] + c["wheel_fr"]) / 2.0 - (c["wheel_rl"] + c["wheel_rr"]) / 2.0
    fwd -= up * fwd.dot(up)
    # spin direction (about 'axis') that rolls the car forward
    fwd_sign = 1.0 if axis.cross(up).dot(fwd) > 0 else -1.0

    # How far the front wheels are steered right now (so Hold keeps it as it is):
    # the rear hubs follow the body, so front-vs-rear hub rotation is the steering.
    yaw0 = {}
    for front, rear in (("wheel_fl", "wheel_rl"), ("wheel_fr", "wheel_rr")):
        try:
            qf = poses[hubs[front]].to_3x3()
            qr = poses[hubs[rear]].to_3x3()
            q = qr.inverted() @ qf
            yaw0[front] = atan2(q[1][0], q[0][0])
        except Exception:
            yaw0[front] = 0.0
    yaw0.setdefault("wheel_rl", 0.0)
    yaw0.setdefault("wheel_rr", 0.0)

    fwd_vec = fwd.normalized() if fwd.length > 1e-9 else axis.cross(up)
    front_c = (c["wheel_fl"] + c["wheel_fr"]) / 2.0
    rear_c = (c["wheel_rl"] + c["wheel_rr"]) / 2.0
    data = dict(up=up, axis=axis, fwd=fwd_sign, spin={}, wheels={}, hubs={},
                body=dict(obj=p.body, M0=poses[p.body]), fwd_vec=fwd_vec,
                pivot=front_c, wheelbase=max((front_c - rear_c).length, 0.5), vib_last=0.0,
                slide=0.0, yawv=0.0, rear_sign=(1.0 if up.cross(-fwd_vec).dot(axis) > 0 else -1.0),
                yaw0=yaw0, key0=(p.steer, p.speed, p.drift_on, p.drift_amount),
                scene=scene.name, t=time.perf_counter())
    for a, w in wheels.items():
        m = poses[w]
        # Where the axle of THIS wheel points right now. A steered front wheel's axle is
        # the plain left-right line turned by the current steering angle - using the plain
        # line for the front wheels made them spin about the wrong axis and wobble.
        lat_w = axis.copy() if a in REAR else (Matrix.Rotation(yaw0[a], 3, up) @ axis)
        g = wheel_geometry(w, lat_w, m)
        if g is not None:
            pivot = m @ g["center"]
            rad = g["radius"]
        else:
            pivot, rad = _center(m, w), 0.3
        if g is not None and g["axis"] is not None:
            e = Vector((0.0, 0.0, 0.0))
            e[g["axis"]] = 1.0
            ax_w = (m.to_3x3().normalized() @ e).normalized()
            if ax_w.dot(lat_w) < 0:
                ax_w = -ax_w
        else:
            ax_w = lat_w.normalized()
        data["wheels"][a] = dict(obj=w, M0=m, C=pivot, axis=ax_w, radius=rad)
        data["spin"][a] = 0.0
    for a, h in hubs.items():
        data["hubs"][a] = dict(obj=h, M0=poses[h], C=_center(poses[h], h))

    # Freeze: write the current pose into the real objects, then switch the physics
    # world off. Nothing is made kinematic, so no hidden velocity is stored.
    for obj, m in poses.items():
        if "mq_start" not in obj:
            obj["mq_start"] = _flat(obj.matrix_world)
        obj.matrix_world = m
    scene.rigidbody_world.enabled = False
    context.view_layer.update()

    _HOLD.clear()
    _HOLD.update(data)
    if not bpy.app.timers.is_registered(_hold_tick):
        bpy.app.timers.register(_hold_tick, first_interval=0.0)
    return True


def _restore_hold_poses():
    """Put every object Hold moved back to the pose it had before Hold."""
    for obj in list(bpy.data.objects):
        if "mq_start" in obj:
            try:
                obj.matrix_world = _unflat(list(obj["mq_start"]))
            except Exception:
                pass
            try:
                del obj["mq_start"]
            except Exception:
                pass


def hold_stop(context):
    _HOLD.clear()
    _restore_hold_poses()
    scene = context.scene
    rbw = scene.rigidbody_world
    if rbw is not None:
        rbw.enabled = True
        try:  # make sure Blender rebuilds the sim from the restored start pose
            rbw.point_cache.frame_end = rbw.point_cache.frame_end
        except Exception:
            pass
    try:
        scene.frame_set(scene.frame_start)
    except Exception:
        pass


def _hold_tick():
    try:
        scene = bpy.data.scenes.get(_HOLD.get("scene", "")) if _HOLD else None
        p = getattr(scene, "mqscar", None)
        if p is None or not p.hold or not _HOLD:
            return None
        now = time.perf_counter()
        dt = min(now - _HOLD["t"], 0.1)
        _HOLD["t"] = now
        up = _HOLD["up"]
        fwd = _HOLD["fwd"]

        # untouched steering = keep the wheels as they were when Hold was pressed
        moved = (p.steer, p.speed, p.drift_on, p.drift_amount) != _HOLD["key0"]
        target = steer_yaw(p)

        def delta(attr):
            return (target - _HOLD["yaw0"][attr]) if moved else 0.0

        # Vibration (Hold). Only ever SIDEWAYS (along the axle, flat on the ground): never
        # forward/back and never up/down. The sideways shake is small with the steering
        # straight and grows as the wheels turn. With the steering turned, the car also keeps
        # turning about its front axle so the REAR slides to the opposite side, round and
        # round, until the steering is changed. Straight steering = no turning at all.
        amp = vibration_amp(p)
        rate = slide_rate(p, _HOLD["rear_sign"])
        _HOLD["yawv"] += (rate - _HOLD["yawv"]) * min(1.0, dt * 4.0)  # eased, not abrupt
        _HOLD["slide"] = (_HOLD["slide"] + _HOLD["yawv"] * dt) % (2.0 * pi)
        active = amp > 0.0 or abs(_HOLD["yawv"]) > 1e-4
        D = None
        if active or _HOLD["vib_last"] > 0.0:
            L = _HOLD["wheelbase"]
            P = _HOLD["pivot"]
            st = min(1.0, abs(max(-1.0, min(1.0, p.steer / radians(20.0)))))
            side = SHAKE_SIDE * L * amp * (SHAKE_STRAIGHT + (1.0 - SHAKE_STRAIGHT) * st)
            shake = _HOLD["axis"] * (side * _vib_noise(now, 1))
            R = Matrix.Rotation(_HOLD["slide"], 3, up)
            D = Matrix.Translation(P) @ R.to_4x4() @ Matrix.Translation(-P) @ Matrix.Translation(shake)
            bd = _HOLD["body"]
            bd["obj"].matrix_world = D @ bd["M0"]
        _HOLD["vib_last"] = 1.0 if active else 0.0

        for attr, wd in _HOLD["wheels"].items():
            front = attr in FRONT
            driven = p.drive_front if front else p.drive_rear
            omega = 0.0 if p.brake else p.speed * driven * fwd / (3.6 * wd["radius"])
            _HOLD["spin"][attr] = (_HOLD["spin"][attr] + omega * dt) % (2.0 * pi)
            # spin about the wheel's own axle first, then steer the whole wheel about 'up'
            rot = Matrix.Rotation(_HOLD["spin"][attr], 3, wd["axis"])
            if front:
                rot = Matrix.Rotation(delta(attr), 3, up) @ rot
            wm = _about(wd["C"], rot, wd["M0"])
            if D is not None:
                if amp > 0.0:
                    # the rear tires shake the most - still only sideways (along the axle)
                    k = 1.0 if attr in REAR else 0.3
                    seed = 10 + 7 * list(_HOLD["wheels"]).index(attr)
                    wm = Matrix.Translation(_HOLD["axis"] * (0.010 * wd["radius"] * k * amp
                                                             * _vib_noise(now, seed))) @ wm
                wm = D @ wm
            wd["obj"].matrix_world = wm

        for attr, hd in _HOLD["hubs"].items():
            rot = Matrix.Rotation(delta(attr), 3, up) if attr in FRONT else Matrix.Identity(3)
            hm = _about(hd["C"], rot, hd["M0"])
            hd["obj"].matrix_world = (D @ hm) if D is not None else hm
        _redraw()
    except ReferenceError:
        _HOLD.clear()  # objects were deleted (e.g. Clear Rig)
        return None
    except Exception:
        traceback.print_exc()
        return None
    return 1.0 / 60.0


def hold_update(self, context):
    if _HOLD_GUARD[0]:
        return
    try:
        if self.hold:
            if not hold_start(context):
                print("MQScar: Hold needs a generated rig and all four wheels assigned")
                _force_hold_off(self)
        else:
            hold_stop(context)
            apply_ride_height(context)
    except Exception:
        traceback.print_exc()
        try:  # never leave the physics world switched off after a failed start
            _force_hold_off(self)
            if context.scene.rigidbody_world is not None:
                context.scene.rigidbody_world.enabled = True
        except Exception:
            pass


@persistent
def _on_load(_dummy):
    """A saved file can contain 'hold = on' but the test stand does not survive a reload."""
    _HOLD.clear()
    was_held = False
    for sc in bpy.data.scenes:
        p = getattr(sc, "mqscar", None)
        if p is not None and p.hold:
            was_held = True
            _force_hold_off(p)
    if was_held:
        try:
            hold_stop(bpy.context)
        except Exception:
            traceback.print_exc()


# --------------------------------------------------------------------------
# Properties
# --------------------------------------------------------------------------
def mesh_poll(self, obj):
    return obj.type == 'MESH'


class MQSCAR_Props(PropertyGroup):
    # Pickers
    body: PointerProperty(name="Body", type=bpy.types.Object, poll=mesh_poll, update=body_update)
    wheel_fl: PointerProperty(name="Front Left", type=bpy.types.Object, poll=mesh_poll)
    wheel_fr: PointerProperty(name="Front Right", type=bpy.types.Object, poll=mesh_poll)
    wheel_rl: PointerProperty(name="Rear Left", type=bpy.types.Object, poll=mesh_poll)
    wheel_rr: PointerProperty(name="Rear Right", type=bpy.types.Object, poll=mesh_poll)
    ground_collection: PointerProperty(
        name="Ground", type=bpy.types.Collection,
        description="Fixed things: roads, terrain, solid walls that never move")
    obstacle_collection: PointerProperty(
        name="Obstacles", type=bpy.types.Collection,
        description="Things that react to the car (walls, boxes, cones, barrels...). Each one "
                    "keeps its own weight: light ones get kicked away, heavy ones stop the car")

    # Live drive controls - read by drivers on the constraints, evaluated by
    # Blender every frame (even inside a running simulation).
    speed: FloatProperty(
        name="Speed", description="Target speed in km/h (negative = reverse)",
        default=0.0, min=-100.0, max=250.0,
    )
    steer: FloatProperty(
        name="Steering",
        description="Front wheel angle in degrees (negative = left, positive = right). "
                    "Drag for the normal range, or Ctrl+Click / double-click and type a "
                    "bigger value (up to 90) to turn further. Reduced a little at high "
                    "speed to keep the car stable",
        default=0.0, subtype='ANGLE', unit='ROTATION',
        min=-radians(STEER_MAX), max=radians(STEER_MAX),
        soft_min=-radians(STEER_SOFT), soft_max=radians(STEER_SOFT),
    )
    brake: BoolProperty(
        name="Brake", description="Hold every wheel at zero speed", default=False)
    hold: BoolProperty(
        name="Hold",
        description="Freeze the car where it is. Wheels spin at the Speed value and the "
                    "front wheels follow Steering, live. Release to go back to the start",
        default=False, update=hold_update,
    )
    drive_mode: EnumProperty(
        name="Drive Wheels",
        description="Which wheels the engine spins",
        items=[
            ('ALL', "All Wheels", "All four wheels are driven"),
            ('FRONT', "Front Wheels", "Only the front wheels are driven"),
            ('REAR', "Rear Wheels", "Only the rear wheels are driven"),
        ],
        default='REAR', update=drive_mode_update,
    )
    vibration: FloatProperty(
        name="Vibration",
        description="Shakes the car left and right in Hold and simulation. With steering "
                    "turned, the rear is pulled toward the opposite side. 0 = none. Drag for "
                    "the normal range, or Ctrl+Click and type up to 3 for a stronger shake. "
                    "Stops at speed 0 or with the Brake on",
        default=0.0, min=0.0, max=3.0, soft_min=0.0, soft_max=1.0, precision=2)
    preset: EnumProperty(
        name="Preset",
        description="Driving style preset. Works live while the sim runs",
        items=[
            ('NONE', "None", "Normal grip driving"),
            ('DRIFT', "Drift", "Turning the steering breaks the rear tires loose so the car drifts"),
        ],
        default='NONE', update=preset_update,
    )
    drift_on: FloatProperty(default=0.0, options={'HIDDEN'})
    drift_amount: FloatProperty(
        name="Drift Amount",
        description="How much the car drifts when it steers (0 = grips normally, 1 = full drift). "
                    "Live, and it can be keyframed: set it to 0 on stretches where the car must "
                    "NOT drift, even if it steers",
        default=1.0, min=0.0, max=1.0, subtype='FACTOR')
    drive_front: FloatProperty(default=0.0, options={'HIDDEN'})
    drive_rear: FloatProperty(default=1.0, options={'HIDDEN'})

    # Advanced (baked when the rig is generated)
    show_advanced: BoolProperty(name="Advanced", default=True)
    mass: FloatProperty(
        name="Weight",
        description="Car body weight in kg. Estimated when you pick the body, but you can "
                    "change it any time. Live: the car gets heavier or lighter as you drag",
        default=1300.0, min=1.0, soft_min=100.0, soft_max=3000.0, max=200000.0, unit='MASS',
        update=settings_update)
    susp_damping: FloatProperty(
        name="Suspension Damping",
        description="How bouncy the car is. Lower = bouncier, higher = firmer with less bounce. "
                    "Applies when the simulation restarts from the first frame",
        default=0.70, min=0.05, max=1.5, update=settings_update)
    ride_height: FloatProperty(
        name="Ride Height",
        description="Moves the car body up (+) or down (-) on its springs, away from or toward "
                    "the tires. Raise it if the tires poke through the bonnet when the car "
                    "bounces; lower it to sit the body closer to the ground. Updates live while "
                    "the sim is stopped (dragging it returns the car to the first frame)",
        default=0.0, subtype='DISTANCE', unit='LENGTH',
        min=-1.0, max=1.0, soft_min=-0.2, soft_max=0.2, step=1, precision=3,
        update=ride_height_update)


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------
def get_slots(p):
    slots = [("Body", p.body)]
    for attr, label in WHEEL_SLOTS:
        slots.append((label, getattr(p, attr)))
    return slots


def validate(p):
    errors, objs = [], []
    for label, obj in get_slots(p):
        if obj is None:
            errors.append(f"{label} not set")
        else:
            objs.append(obj)
    if len(set(objs)) != len(objs):
        errors.append("The same object is used in more than one slot")
    if p.ground_collection is not None and p.ground_collection == p.obstacle_collection:
        errors.append("Ground and Obstacles cannot be the same collection")
    return errors


def estimate_obstacle_mass(o):
    d = o.dimensions
    return max(1.0, min(d.x * d.y * d.z * OBSTACLE_DENSITY, 20000.0))


def auto_mass(p):
    d = p.body.dimensions
    return max(20.0, min(d.x * d.y * d.z * PROFILE["density"], 40000.0))


def final_mass(p):
    return p.mass


def has_rig():
    coll = bpy.data.collections.get(RIG_COLLECTION)
    return coll is not None and len(coll.objects) > 0


def world_bbox_points(obj):
    return [obj.matrix_world @ Vector(c) for c in obj.bound_box]


def world_center(obj):
    pts = world_bbox_points(obj)
    return sum(pts, Vector()) / len(pts)


def get_or_create_collection(scene, name):
    coll = bpy.data.collections.get(name)
    if coll is None:
        coll = bpy.data.collections.new(name)
    linked = {c.name for c in scene.collection.children_recursive}
    if coll.name not in linked:
        scene.collection.children.link(coll)
    return coll


def stop_playback(context):
    screen = context.screen
    if screen is not None and screen.is_animation_playing:
        try:
            bpy.ops.screen.animation_cancel(restore_frame=False)
        except Exception:
            traceback.print_exc()


def ensure_rigidbody_world(context):
    if context.scene.rigidbody_world is None:
        bpy.ops.rigidbody.world_add()
    return context.scene.rigidbody_world


def select_only(context, objs):
    for o in list(context.selected_objects):
        try:
            o.select_set(False)
        except RuntimeError:
            pass
    for o in objs:
        try:
            o.select_set(True)
        except RuntimeError:
            pass
    if objs:
        context.view_layer.objects.active = objs[0]


def override_for(context, obj):
    # Some rigidbody operators read the view layer's active object directly,
    # so a context override alone is not enough.
    select_only(context, [obj])
    return context.temp_override(
        object=obj, active_object=obj,
        selected_objects=[obj], selected_editable_objects=[obj],
    )


def add_rigid_body(context, obj, rb_type):
    if obj.rigid_body is None:
        with override_for(context, obj):
            bpy.ops.rigidbody.object_add(type=rb_type)
        obj[TAG_RB] = True
    rb = obj.rigid_body
    rb.type = rb_type
    return rb


def set_layers(rb, layers):
    rb.collision_collections = [i in layers for i in range(20)]


def detach_parent(obj):
    """Rigid bodies don't work properly when parented; keep world transform."""
    if obj.parent is not None:
        mw = obj.matrix_world.copy()
        obj.parent = None
        obj.matrix_world = mw
        return True
    return False


# --------------------------------------------------------------------------
# Safe removal (uses Blender's own operators so the physics world is updated;
# deleting rigid body objects with bpy.data.*.remove() crashes Blender)
# --------------------------------------------------------------------------
def delete_objects(context, objs):
    for o in [o for o in objs if o is not None]:
        try:
            bpy.data.objects.remove(o, do_unlink=True)
        except Exception:
            traceback.print_exc()


_HOME_KEY = "mq_home"


def store_home_poses(p):
    for o in [p.body] + [getattr(p, a) for a, _l in WHEEL_SLOTS]:
        if o is not None and _HOME_KEY not in o:
            o[_HOME_KEY] = _flat(o.matrix_world)


def restore_home_poses(delete=False, run=False):
    """delete=True: put everything back as modelled and forget it (Clear Rig).
    run=True: use the raised start pose of the body (Reset Car)."""
    for o in list(bpy.data.objects):
        key = "mq_run" if (run and "mq_run" in o) else _HOME_KEY
        if key in o:
            try:
                o.matrix_world = _unflat(list(o[key]))
            except Exception:
                traceback.print_exc()
        if delete:
            for k in (_HOME_KEY, "mq_run", "mq_sag"):
                if k in o:
                    del o[k]


def center_wheel_origins(p):
    """Move each wheel's origin to the true middle of the wheel (the look does not change),
    so it spins around its real centre. Returns a list of warning texts."""
    notes = []
    lat = lateral_dir(p)
    for attr, _l in WHEEL_SLOTS:
        w = getattr(p, attr)
        try:
            if w is None or w.type != 'MESH' or len(w.data.vertices) == 0:
                continue
            m = w.matrix_world.copy()
            g = wheel_geometry(w, lat, m)
            if g is None:
                continue
            c = g["center"]
            if (m.to_3x3() @ c).length < 0.0005 * g["radius"] * 2.0:
                continue  # already centred
            if w.modifiers:
                notes.append(f"{w.name} has modifiers: its centre was measured on the base mesh")
            kids = [(ch, ch.matrix_world.copy()) for ch in w.children]
            if w.data.users > 1:
                w.data = w.data.copy()
            w.data.transform(Matrix.Translation(-c), shape_keys=True)
            w.data.update()
            w.matrix_world = m @ Matrix.Translation(c)
            for ch, cm in kids:  # children must not move when the parent's origin does
                ch.matrix_world = cm
        except Exception:
            traceback.print_exc()
    return notes


def clear_rig(context):
    scene = context.scene
    stop_playback(context)
    try:
        if scene.mqscar.hold:
            scene.mqscar.hold = False  # releases the test stand and restores poses
        if scene.rigidbody_world is not None:
            scene.rigidbody_world.enabled = True
    except Exception:
        traceback.print_exc()
    if scene.rigidbody_world is not None:
        scene.frame_set(scene.frame_start)  # reset the simulation first

    # 1. Constraint empties (children first)
    empties = [o for o in bpy.data.objects if o.get(TAG_CON)]
    for e in empties:
        if e.rigid_body_constraint is not None:
            try:
                with override_for(context, e):
                    bpy.ops.rigidbody.constraint_remove()
            except Exception:
                pass
    delete_objects(context, empties)

    # 2. Hubs (and their meshes)
    hubs = [o for o in bpy.data.objects if o.get(TAG_HUB) or o.name.startswith("MQScar_Hub")]
    meshes = [h.data for h in hubs]
    delete_objects(context, hubs)
    for m in meshes:
        if m is not None and m.users == 0:
            bpy.data.meshes.remove(m)

    # 3. Rigid bodies we added to the user's objects
    for ob in [o for o in bpy.data.objects if o.get(TAG_RB)]:
        if ob.get(TAG_OBS):
            try:  # keep the weight the user typed, so Generate does not reset it
                if ob.rigid_body is not None:
                    ob["mq_mass"] = ob.rigid_body.mass
            except Exception:
                pass
            del ob[TAG_OBS]
        for path in ("rigid_body.kinematic", "rigid_body.mass", "rigid_body.friction"):
            try:
                ob.driver_remove(path)
            except Exception:
                pass
        if ob.rigid_body is not None:
            try:
                with override_for(context, ob):
                    bpy.ops.rigidbody.object_remove()
            except Exception:
                pass
        if ob.rigid_body is not None and scene.rigidbody_world is not None:
            try:  # fallback when the operator cannot run (e.g. object hidden)
                coll = scene.rigidbody_world.collection
                if coll is not None and ob.name in coll.objects:
                    coll.objects.unlink(ob)
            except Exception:
                pass
        del ob[TAG_RB]

    # 4. Anything else left over from an earlier rig (e.g. hidden or renamed pieces)
    for o in [o for o in bpy.data.objects
              if o.name.startswith("MQScar_") and not o.name.startswith(GROUND_PLANE)]:
        try:
            bpy.data.objects.remove(o, do_unlink=True)
        except Exception:
            pass

    # 5. Put the car back where it started
    restore_home_poses(delete=True)
    try:
        scene.frame_set(scene.frame_start)
    except Exception:
        pass

    rig = bpy.data.collections.get(RIG_COLLECTION)
    if rig is not None and not rig.objects:
        bpy.data.collections.remove(rig)


# --------------------------------------------------------------------------
# Building blocks
# --------------------------------------------------------------------------
def ensure_ground(context, wheels):
    """Returns (collection, auto_created)."""
    scene = context.scene
    p = scene.mqscar
    if p.ground_collection is not None:
        coll = p.ground_collection
        has_mesh = any(o.type == 'MESH' for o in coll.all_objects)
        if has_mesh or not coll.name.startswith(GROUND_COLLECTION):
            return coll, False
        # our own auto-made ground lost its plane: make it again below
    else:
        coll = get_or_create_collection(scene, GROUND_COLLECTION)
        p.ground_collection = coll

    # Large plane sitting at the bottom of the lowest wheel
    z = min(pt.z for w in wheels for pt in world_bbox_points(w))
    s = 200.0
    mesh = bpy.data.meshes.new(GROUND_PLANE)
    mesh.from_pydata([(-s, -s, 0), (s, -s, 0), (s, s, 0), (-s, s, 0)], [], [(0, 1, 2, 3)])
    mesh.update()
    plane = bpy.data.objects.new(GROUND_PLANE, mesh)
    plane.location.z = z
    coll.objects.link(plane)
    return coll, True


def create_hub(context, rig_coll, tag, center, mass, attr, size=0.05):
    """Small block that carries one wheel (collides with nothing)."""
    s = max(size, 0.05)
    verts = [(x * s, y * s, z * s) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    mesh = bpy.data.meshes.new(f"MQScar_Hub_{tag}")
    mesh.from_pydata(verts, [], faces)
    mesh.update()
    hub = bpy.data.objects.new(f"MQScar_Hub_{tag}", mesh)
    hub.location = center
    hub.display_type = 'WIRE'
    hub.hide_render = True
    hub[TAG_HUB] = attr
    hub[_HOME_KEY] = _flat(Matrix.Translation(center))  # Reset Car restores hubs as well
    rig_coll.objects.link(hub)

    rb = add_rigid_body(context, hub, 'ACTIVE')
    rb.mass = mass
    rb.collision_shape = 'BOX'
    rb.use_deactivation = False
    rb.angular_damping = 0.8  # calms the hub when the engine torque pushes back on it
    set_layers(rb, {LAYER_HUB})
    return hub


def add_driver(scene, obj, path, expr, names):
    """Driver reading scene.mqscar.<name> variables."""
    fc = obj.driver_add(path)
    drv = fc.driver
    drv.type = 'SCRIPTED'
    drv.expression = expr
    for name in names:
        var = drv.variables.new()
        var.name = name
        var.type = 'SINGLE_PROP'
        tgt = var.targets[0]
        tgt.id_type = 'SCENE'
        tgt.id = scene
        tgt.data_path = f"mqscar.{name}"
    for m in list(fc.modifiers):
        fc.modifiers.remove(m)
    return fc


def create_joint(context, rig_coll, name, kind, ctype, obj1, obj2,
                 center, rot, parent, parent_world, attr, susp=None):
    """Create a rigid body constraint empty.

    The empty is parented to a rigid body so that, if Blender rebuilds the
    constraint mid-simulation, the pivot follows the car.
    """
    empty = bpy.data.objects.new(name, None)
    empty.empty_display_type = 'ARROWS'
    empty.empty_display_size = 0.25
    rig_coll.objects.link(empty)
    empty.matrix_world = Matrix.Translation(center) @ rot
    empty.parent = parent
    empty.matrix_parent_inverse = parent_world.inverted()
    empty[TAG_CON] = kind

    with override_for(context, empty):
        bpy.ops.rigidbody.constraint_add(type=ctype)
    con = empty.rigid_body_constraint
    if con is None:
        raise RuntimeError(f"Could not add rigid body constraint to {empty.name}")
    con.object1 = obj1
    con.object2 = obj2
    con.disable_collisions = True

    def lock_lin(axes):
        for ax in axes:
            setattr(con, f"use_limit_lin_{ax}", True)
            setattr(con, f"limit_lin_{ax}_lower", 0.0)
            setattr(con, f"limit_lin_{ax}_upper", 0.0)

    def lock_ang(axes):
        for ax in axes:
            setattr(con, f"use_limit_ang_{ax}", True)
            setattr(con, f"limit_ang_{ax}_lower", 0.0)
            setattr(con, f"limit_ang_{ax}_upper", 0.0)

    if kind == 'susp':
        # body -> hub: vertical spring (local Z). Steering is local Z rotation.
        lock_lin("xy")
        con.use_limit_lin_z = True
        con.limit_lin_z_lower = -susp["travel"]
        con.limit_lin_z_upper = susp["travel"]
        con.use_spring_z = True
        con.spring_type = 'SPRING2'
        con.spring_stiffness_z = susp["stiffness"]
        con.spring_damping_z = susp["damping"]
        if susp.get("steered"):
            lock_ang("xy")  # yaw is handled by the steering joints
        else:
            lock_ang("xyz")
    elif kind == 'steer':
        # body -> hub: only fixes the yaw. One of these is switched on at a time.
        con.use_limit_ang_z = True
        con.limit_ang_z_lower = susp["angle"]
        con.limit_ang_z_upper = susp["angle"]
    elif kind == 'axle':
        # hub -> wheel: free spin around local X, everything else locked
        lock_lin("xyz")
        lock_ang("yz")
        con.use_limit_ang_x = False
    elif kind == 'vib':
        # body -> ground: sideways (local X) motor at the rear axle. Force is 0 until driven.
        con.use_motor_lin = True
        con.use_motor_ang = False
        con.motor_lin_target_velocity = (susp or {}).get("vel", 0.0)  # fixed per joint
        con.motor_lin_max_impulse = 0.0
        con.enabled = False
    elif kind == 'motor':
        # hub -> wheel: motor around local X. Force is 0 until driven.
        con.use_motor_lin = False
        con.use_motor_ang = True
        con.motor_ang_target_velocity = 0.0
        con.motor_ang_max_impulse = 0.0
    return empty


# --------------------------------------------------------------------------
# Operators
# --------------------------------------------------------------------------
class MQSCAR_OT_use_active(Operator):
    bl_idname = "mqscar.use_active"
    bl_label = "Use Active Object"
    bl_description = "Assign the active object to this slot"

    slot: StringProperty()

    def execute(self, context):
        obj = context.active_object
        if obj is None or obj.type != 'MESH':
            self.report({'WARNING'}, "Select a mesh object first")
            return {'CANCELLED'}
        setattr(context.scene.mqscar, self.slot, obj)
        return {'FINISHED'}


class MQSCAR_OT_generate_rig(Operator):
    bl_idname = "mqscar.generate_rig"
    bl_label = "Generate Rig"
    bl_description = "Build the rigid body car (creates the ground if missing)"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        p = context.scene.mqscar
        errors = validate(p)
        if errors:
            self.report({'ERROR'}, "; ".join(errors))
            return {'CANCELLED'}

        stop_playback(context)
        if p.hold:
            p.hold = False
        if context.mode != 'OBJECT' and context.object is not None:
            bpy.ops.object.mode_set(mode='OBJECT')

        try:
            clear_rig(context)
            for note in center_wheel_origins(p):
                self.report({'WARNING'}, note)
            context.view_layer.update()
            store_home_poses(p)
            self.build(context)
        except Exception as e:
            traceback.print_exc()
            try:
                clear_rig(context)
            except Exception:
                pass
            self.report({'ERROR'}, f"Rig failed: {e}")
            return {'CANCELLED'}
        return {'FINISHED'}

    def build(self, context):
        scene = context.scene
        p = scene.mqscar
        prof = PROFILE
        body = p.body
        wheels = [(attr, label, getattr(p, attr)) for attr, label in WHEEL_SLOTS]
        wheel_objs = [w for _, _, w in wheels]

        # --- physics world --------------------------------------------------
        rbw = ensure_rigidbody_world(context)
        rbw.enabled = True
        rbw.point_cache.frame_start = scene.frame_start
        rbw.point_cache.frame_end = max(scene.frame_end, scene.frame_start + 1)
        rbw.solver_iterations = max(rbw.solver_iterations, 50)
        rbw.substeps_per_frame = max(rbw.substeps_per_frame, 20)

        # --- prep -----------------------------------------------------------
        detached = [o.name for o in [body] + wheel_objs if detach_parent(o)]
        if detached:
            self.report({'INFO'}, f"Cleared parent (kept transform): {', '.join(detached)}")
        context.view_layer.update()

        # True wheel centres (fitted through the rim, not just the middle of the bounding box)
        lat0 = lateral_dir(p)
        centers = {}
        for a, _l, w in wheels:
            g = wheel_geometry(w, lat0)
            centers[a] = (w.matrix_world @ g["center"]) if g is not None else world_center(w)
            dims = max(w.dimensions)
            if dims > 0 and (centers[a] - w.matrix_world.translation).length > 0.02 * dims:
                self.report({'WARNING'}, f"{w.name}: origin is not at the wheel center")

        # Forward direction comes from where the front/rear wheels are
        front_c = (centers["wheel_fl"] + centers["wheel_fr"]) / 2.0
        rear_c = (centers["wheel_rl"] + centers["wheel_rr"]) / 2.0
        fwd = front_c - rear_c
        fwd.z = 0.0
        if fwd.length < 1e-4:
            raise RuntimeError("Front and rear wheels are in the same place - check the wheel slots")
        fwd.normalize()

        # Springs always sink the body by 'sag' under its own weight. Start the body
        # 'sag' higher so that, once settled, it sits exactly where you modelled it.
        radii = []
        for _a, _l, w in wheels:
            zs = [pt.z for pt in world_bbox_points(w)]
            radii.append(max((max(zs) - min(zs)) / 2.0, 0.01))
        sag_common = max(BASE_SAG * SUSP_TRAVEL * (sum(radii) / len(radii)), 1e-4)
        body.matrix_world = Matrix.Translation((0.0, 0.0, sag_common + p.ride_height)) @ body.matrix_world
        body["mq_run"] = _flat(body.matrix_world)  # where Reset Car puts the body
        body["mq_sag"] = sag_common  # so the Ride Height slider can move the body later
        context.view_layer.update()

        # Axle axis = body axis along the left->right wheel line
        body_world = body.matrix_world.copy()
        lat = body_world.to_3x3().inverted() @ (centers["wheel_fr"] - centers["wheel_fl"])
        axle_axis = 'X' if abs(lat.x) >= abs(lat.y) else 'Y'

        up_rot = body_world.to_quaternion().to_matrix().to_4x4()
        axle_rot = up_rot
        if axle_axis == 'Y':
            axle_rot = up_rot @ Matrix.Rotation(radians(90.0), 4, 'Z')

        # Which spin direction moves the car forward
        axle_dir = axle_rot.to_3x3() @ Vector((1.0, 0.0, 0.0))
        # Blender's motor spins the wheel the opposite way to the joint axis, so the
        # sign is flipped here once and positive speed always drives forward.
        drive_sign = -1.0 if axle_dir.cross(Vector((0.0, 0.0, 1.0))).dot(fwd) > 0 else 1.0

        # --- numbers from the Advanced settings -----------------------------
        mass = final_mass(p)
        wheel_mass = max(mass * prof["wheel_mass"], 0.5)
        hub_mass = max(wheel_mass * 2.0, 1.0)
        load = mass / 4.0

        # --- ground ---------------------------------------------------------
        ground, auto_created = ensure_ground(context, wheel_objs)
        if auto_created:
            self.report({'INFO'}, f"Created ground collection '{ground.name}' with a ground plane")
        ground_meshes = [
            o for o in ground.all_objects
            if o.type == 'MESH' and o is not body and o not in wheel_objs
        ]
        if not ground_meshes:
            self.report({'WARNING'}, "Ground collection has no mesh objects - the car will fall")
        for g in ground_meshes:
            rb = add_rigid_body(context, g, 'PASSIVE')
            rb.collision_shape = 'MESH'
            rb.friction = 1.0
            set_layers(rb, {LAYER_BODY, LAYER_WHEEL})

        # --- obstacles: things that react to the car -----------------------------
        obstacles = []
        if p.obstacle_collection is not None:
            skip = set(ground_meshes) | {body} | set(wheel_objs)
            for o in p.obstacle_collection.all_objects:
                if o.type != 'MESH' or o in skip or o.get(TAG_HUB) or o.name.startswith("MQScar_"):
                    continue
                obstacles.append(o)
            if not obstacles:
                self.report({'WARNING'}, "Obstacles collection has no mesh objects")
            moved = [o.name for o in obstacles if detach_parent(o)]
            if moved:
                self.report({'INFO'}, f"Cleared parent (kept transform): {', '.join(moved)}")
            context.view_layer.update()
            for o in obstacles:
                if o.rigid_body is None:
                    rb = add_rigid_body(context, o, 'ACTIVE')
                    rb.mass = o.get("mq_mass") or estimate_obstacle_mass(o)
                    rb.collision_shape = 'CONVEX_HULL'
                    rb.use_margin = True
                    rb.collision_margin = 0.005
                    rb.friction = 0.6
                    rb.linear_damping = 0.04
                    rb.use_deactivation = True  # settles and sleeps; the car wakes it on contact
                    o[TAG_OBS] = True
                else:
                    rb = o.rigid_body  # the user set this one up: keep their type and weight
                set_layers(rb, {LAYER_BODY, LAYER_WHEEL})
            if obstacles:
                self.report({'INFO'}, f"{len(obstacles)} obstacle(s) will react to the car")

        # --- body -----------------------------------------------------------
        rb = add_rigid_body(context, body, 'ACTIVE')
        rb.mass = mass
        rb.friction = 0.5
        rb.collision_shape = 'CONVEX_HULL'
        rb.use_margin = True
        rb.collision_margin = 0.005
        rb.use_deactivation = False
        set_layers(rb, {LAYER_BODY})
        add_driver(scene, body, "rigid_body.mass", "mass", ["mass"])

        # --- wheels ---------------------------------------------------------
        capped = None
        for attr, _l, w in wheels:
            grip = GRIP
            rb = add_rigid_body(context, w, 'ACTIVE')
            rb.mass = wheel_mass
            rb.friction = grip
            if rb.friction < grip - 0.01:
                capped = rb.friction
            rb.collision_shape = 'SPHERE'
            rb.use_margin = True
            rb.collision_margin = 0.0
            rb.use_deactivation = False
            set_layers(rb, {LAYER_WHEEL})
        if capped is not None:
            self.report({'WARNING'}, f"Blender limited tire grip to {capped:.2f}; "
                                     "acceleration may be traction-limited")

        # --- hubs, joints, drivers -----------------------------------------
        rig_coll = get_or_create_collection(scene, RIG_COLLECTION)

        for attr, label, w in wheels:
            tag = label.replace(" ", "")
            center = centers[attr]
            zs = [pt.z for pt in world_bbox_points(w)]
            radius = max((max(zs) - min(zs)) / 2.0, 0.01)

            # Suspension numbers from the body weight and wheel size
            travel = SUSP_TRAVEL * radius
            sag = sag_common
            stiffness = (load * GRAVITY / sag) * SUSP_FIRMNESS
            damping = 2.0 * p.susp_damping * sqrt(stiffness * load)
            susp = dict(travel=travel, stiffness=stiffness, damping=damping,
                        steered=attr in FRONT)

            # Live: wheel weight follows the Body Weight slider while the sim runs
            add_driver(scene, w, "rigid_body.mass", f"max(mass*{prof['wheel_mass']:.6f},0.5)", ["mass"])
            if attr in REAR:
                # Drift preset: the rear tires lose grip in proportion to how far the wheels
                # are turned (and to Drift Amount), so the tail slides out only while steering.
                add_driver(scene, w, "rigid_body.friction",
                           f"{GRIP}*(1-drift_on*drift_amount*min(1,abs(steer)/{DRIFT_STEER:.5f})"
                           f"*{1.0 - DRIFT_REAR_GRIP:.4f})"
                           f"*(1-{SIM_REAR_SLIP}*min(1,vibration)*(1-brake)*(abs(speed)>0.05))",
                           ["drift_on", "drift_amount", "steer", "vibration", "brake", "speed"])

            hub = create_hub(context, rig_coll, tag, center, hub_mass, attr, radius * 0.5)
            add_driver(scene, hub, "rigid_body.mass", f"max(mass*{prof['wheel_mass'] * 2.0:.6f},1.0)", ["mass"])
            hub_world = Matrix.Translation(center)

            susp_e = create_joint(
                context, rig_coll, f"MQScar_Susp_{tag}", 'susp', 'GENERIC_SPRING',
                body, hub, center, up_rot, body, body_world, attr, susp)
            create_joint(
                context, rig_coll, f"MQScar_Axle_{tag}", 'axle', 'GENERIC',
                hub, w, center, axle_rot, hub, hub_world, attr)
            motor_e = create_joint(
                context, rig_coll, f"MQScar_Motor_{tag}", 'motor', 'MOTOR',
                hub, w, center, axle_rot, hub, hub_world, attr)

            # Spring strength follows the weight, so the car sits at the same height at
            # any weight (applied whenever the sim starts from the first frame)
            add_driver(scene, susp_e, "rigid_body_constraint.spring_stiffness_z",
                       f"mass*{GRAVITY / (4.0 * sag) * SUSP_FIRMNESS:.5f}", ["mass"])
            add_driver(scene, susp_e, "rigid_body_constraint.spring_damping_z",
                       f"susp_damping*mass*{0.5 * sqrt(GRAVITY / sag * SUSP_FIRMNESS):.5f}",
                       ["susp_damping", "mass"])

            # Steering (front wheels): positive slider = turn right.
            # Blender cannot change a joint while the sim runs, but it CAN switch joints
            # on and off. So there is one ready-made steering joint per angle step and a
            # driver switches on the one matching the Steering slider, live.
            if attr in FRONT:
                # slider is in radians -> degrees -> steps; calmer at speed; clamped to the joints
                calm = (f"(1+speed*speed/{STEER_REF * STEER_REF:.1f}"
                        f"*(1-{DRIFT_CALM}*drift_on*drift_amount*min(1,abs(steer)/{DRIFT_STEER:.5f})))")
                idx = (f"max({-STEER_K},min({STEER_K},floor(steer*{180.0 / pi:.6f}"
                       f"/({STEER_STEP}*{calm})+0.5)))")
                names = ["steer", "speed", "drift_on", "drift_amount"]
                for k in range(-STEER_K, STEER_K + 1):
                    st = create_joint(
                        context, rig_coll, f"MQScar_Steer_{tag}_{k + STEER_K:02d}", 'steer',
                        'GENERIC', body, hub, center, up_rot, body, body_world, attr,
                        dict(angle=-radians(k * STEER_STEP)))
                    st.rigid_body_constraint.enabled = (k == 0)
                    add_driver(scene, st, "rigid_body_constraint.enabled", f"{idx}=={k}", names)

            # Motor / brake. 'driven' follows the Drive Wheels dropdown live.
            driven = "drive_front" if attr in FRONT else "drive_rear"
            # Far more than the tires can use: the wheel always reaches the typed
            # speed at once and only tire grip limits how fast the car follows.
            engine = f"mass*{GRAVITY * radius * INSTANT_PUSH:.5f}"
            k = drive_sign / (3.6 * radius)  # km/h -> wheel rad/s
            add_driver(
                scene, motor_e, "rigid_body_constraint.motor_ang_max_impulse",
                f"brake*mass*{BRAKE_FORCE * radius / 4.0:.5f}+(1-brake)*{driven}*(abs(speed)>0.05)*{engine}",
                ["speed", "brake", driven, "mass"])
            add_driver(
                scene, motor_e, "rigid_body_constraint.motor_ang_target_velocity",
f"(1-brake)*{driven}*speed*{k:.6f}",
                ["speed", "brake", driven])

        # Vibration while driving. Blender reads a joint's motor speed only when the sim
        # starts, so a single motor can never follow the Vibration / Steering / Speed values
        # live (it stayed at whatever they were at frame 1 - usually zero). What CAN change
        # live is a joint's 'enabled' flag, exactly like the steering joints. So there is one
        # ready-made sideways motor per speed step and a driver switches on the one that
        # matches the wanted shake right now. The joint's X axis is the axle line.
        if ground_meshes:
            right_sign = 1.0 if axle_dir.dot(lat0) > 0 else -1.0
            ks = SIM_MOTOR_SIGN * right_sign  # joint axis -> car's right side
            rear_mid = (centers["wheel_rl"] + centers["wheel_rr"]) / 2.0
            names = ["vibration", "speed", "brake", "steer", "drift_on", "drift_amount", "mass"]
            vf = "(vibration*(1+2*max(0,vibration-1)*max(0,vibration-1)))"
            sf = f"(0.3+0.7*min(1,abs(speed)/{VIB_SPEED}))"
            dm = (f"(1+{VIB_DRIFT_BOOST}*drift_on*drift_amount*"
                  f"min(1,abs(steer)/{DRIFT_STEER:.6f}))")
            amp = (f"(min({vf}*{sf}*{dm},8)"
                   f"*(1-brake)*(abs(speed)>0.05))")
            stx = f"max(-1,min(1,steer/{radians(20.0):.6f}))"
            nz = "((sin(frame*1.7)+sin(frame*2.9+1.3)+0.6*sin(frame*0.9+2.1))/2.6)"
            want = (f"{ks}*{amp}*({SIM_SHAKE_VEL}*{nz}*({SHAKE_STRAIGHT}+{1.0 - SHAKE_STRAIGHT}*abs({stx}))"
                    f"-{SIM_PUSH_VEL}*{stx})")
            idx = f"max({-SIM_BANK_N},min({SIM_BANK_N},floor(({want})/{SIM_BANK_STEP}+0.5)))"
            for k in range(-SIM_BANK_N, SIM_BANK_N + 1):
                if k == 0:
                    continue  # nothing switched on = no sideways force
                vib_e = create_joint(
                    context, rig_coll, f"MQScar_Vibration_{k + SIM_BANK_N:02d}", 'vib', 'MOTOR',
                    body, ground_meshes[0], rear_mid, axle_rot, body, body_world, "wheel_rl",
                    dict(vel=k * SIM_BANK_STEP))
                vib_e.rigid_body_constraint.enabled = False
                add_driver(scene, vib_e, "rigid_body_constraint.enabled", f"{idx}=={k}", names)
                add_driver(
                    scene, vib_e, "rigid_body_constraint.motor_lin_max_impulse",
                    f"mass*{GRAVITY * SIM_FORCE:.5f}", ["mass"])

        # Leave the user with the body selected, simulation at the start
        select_only(context, [body])
        scene.frame_set(scene.frame_start)
        self.report({'INFO'}, f"{prof['label']} rig built (~{mass:,.0f} kg). Press Play.")


class MQSCAR_OT_drift_mark(Operator):
    bl_idname = "mqscar.drift_mark"
    bl_label = "Drift Mark"
    bl_description = "Mark on the timeline where the car should drift (or not drift)"
    bl_options = {'REGISTER', 'UNDO'}

    action: EnumProperty(items=[
        ('ON', "Drift From Here", "Drift from this frame until the next mark"),
        ('OFF', "Stop Drift Here", "Grip normally from this frame until the next mark"),
        ('CLEAR', "Clear Marks", "Remove all drift marks"),
    ])

    def execute(self, context):
        sc = context.scene
        p = sc.mqscar
        path = "mqscar.drift_amount"
        ad = sc.animation_data
        fc = ad.action.fcurves.find(path) if (ad is not None and ad.action is not None) else None

        if self.action == 'CLEAR':
            if fc is not None:
                ad.action.fcurves.remove(fc)
            p.drift_amount = 1.0
            return {'FINISHED'}

        value = 1.0 if self.action == 'ON' else 0.0
        frame = sc.frame_current
        if fc is None and frame > sc.frame_start:
            # before the first mark the car does the opposite
            p.drift_amount = 1.0 - value
            p.keyframe_insert("drift_amount", frame=sc.frame_start)
        p.drift_amount = value
        p.keyframe_insert("drift_amount", frame=frame)

        ad = sc.animation_data
        fc = ad.action.fcurves.find(path) if (ad is not None and ad.action is not None) else None
        if fc is not None:
            for kp in fc.keyframe_points:
                kp.interpolation = 'CONSTANT'  # a mark is a hard on/off switch
            fc.update()
        return {'FINISHED'}


class MQSCAR_OT_clear_rig(Operator):
    bl_idname = "mqscar.clear_rig"
    bl_label = "Clear Rig"
    bl_description = "Remove the hubs, constraints and rigid bodies MQScar created"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        try:
            clear_rig(context)
        except Exception as e:
            traceback.print_exc()
            self.report({'ERROR'}, f"Clear failed: {e}")
            return {'CANCELLED'}
        self.report({'INFO'}, "MQScar rig cleared")
        return {'FINISHED'}


def repair_car_physics(context):
    """Make sure the body, wheels and hubs are still active members of the rigid body world
    (not Dynamic-off, not kinematic, in the right collision layers) and that the joints are
    still in the world's constraint list. Returns a list of problems that could not be fixed."""
    scene = context.scene
    p = scene.mqscar
    problems = []
    rbw = ensure_rigidbody_world(context)
    rbw.enabled = True
    hubs = [o for o in bpy.data.objects if o.get(TAG_HUB)]
    parts = [(p.body, {LAYER_BODY})] + [(getattr(p, a), {LAYER_WHEEL}) for a, _l in WHEEL_SLOTS] \
        + [(h, {LAYER_HUB}) for h in hubs]
    for o, layers in parts:
        if o is None:
            continue
        if o.rigid_body is None:
            problems.append(o.name)
            continue
        rb = o.rigid_body
        try:
            rb.type = 'ACTIVE'
            rb.enabled = True
            rb.kinematic = False
            set_layers(rb, layers)
            if rbw.collection is not None and o.name not in rbw.collection.objects:
                rbw.collection.objects.link(o)
        except Exception:
            traceback.print_exc()
    if rbw.constraints is not None:
        for e in bpy.data.objects:
            if e.get(TAG_CON) and e.rigid_body_constraint is not None \
                    and e.name not in rbw.constraints.objects:
                try:
                    rbw.constraints.objects.link(e)
                except Exception:
                    traceback.print_exc()
    return problems


class MQSCAR_OT_reset_drive(Operator):
    bl_idname = "mqscar.reset_drive"
    bl_label = "Reset Car"
    bl_description = "Zero speed and steering, release brake and hold, put the car back at its start"

    def execute(self, context):
        scene = context.scene
        p = scene.mqscar
        stop_playback(context)

        # Release Hold first; it restores the transforms it temporarily owned.
        if p.hold:
            p.hold = False
        else:
            _HOLD.clear()
            _restore_hold_poses()  # leftovers from a Hold that did not finish cleanly
        p.speed = 0.0
        p.steer = 0.0
        p.brake = False

        rbw = scene.rigidbody_world
        if rbw is not None:
            rbw.enabled = True

        # Restore the authored wheel poses and the raised body run pose.
        restore_home_poses(delete=False, run=True)
        context.view_layer.update()

        # Reset must never leave the car kinematic, 'Dynamic' off or outside the RB world.
        lost = repair_car_physics(context)
        if lost:
            self.report({'WARNING'}, "Lost its rigid body: " + ", ".join(lost)
                        + ". Press Generate Rig again.")
        if p.body is not None and p.body.rigid_body is not None:
            p.body.rigid_body.use_deactivation = False

        # Clear stale Bullet state by crossing the cache start, then evaluate
        # the restored starting pose before returning control to the user.
        reset_sim(context)
        return {'FINISHED'}


# --------------------------------------------------------------------------
# UI
# --------------------------------------------------------------------------
class MQSCAR_PT_main(Panel):
    bl_label = "MQScar"
    bl_idname = "MQSCAR_PT_main"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "MQScar"

    def draw(self, context):
        layout = self.layout
        p = context.scene.mqscar

        # Components
        box = layout.box()
        box.label(text="Car Components", icon='OUTLINER_OB_MESH')
        for attr, label in [("body", "Body")] + list(WHEEL_SLOTS):
            row = box.row(align=True)
            row.prop(p, attr, text=label)
            op = row.operator("mqscar.use_active", text="", icon='EYEDROPPER')
            op.slot = attr

        # Ground
        box = layout.box()
        box.label(text="Ground", icon='OUTLINER_COLLECTION')
        box.prop(p, "ground_collection", text="")
        if p.ground_collection is None:
            box.label(text="Auto-created on Generate", icon='INFO')

        # Obstacles
        box = layout.box()
        box.label(text="Obstacles (react to the car)", icon='PHYSICS')
        box.prop(p, "obstacle_collection", text="")
        if p.obstacle_collection is None:
            box.label(text="Optional: walls, boxes, cones, barrels...", icon='INFO')
        else:
            ob = context.active_object
            if (ob is not None and ob.rigid_body is not None
                    and ob.name in p.obstacle_collection.all_objects):
                box.prop(ob.rigid_body, "mass", text=f"{ob.name} Weight")
                box.label(text="Change weights while stopped; used from frame 1")

        col = layout.column(align=True)
        col.scale_y = 1.4
        col.operator("mqscar.generate_rig", icon='PHYSICS')
        row = col.row()
        row.scale_y = 0.8
        row.operator("mqscar.clear_rig", icon='TRASH')

        # Drive
        box = layout.box()
        box.label(text="Drive", icon='DRIVER')
        if has_rig():
            col = box.column(align=True)
            col.prop(p, "speed", slider=True)
            col.prop(p, "steer", slider=True)
            col.prop(p, "vibration", slider=True, text="Vibration")
            box.label(text="Spinning Wheels:")
            box.prop(p, "drive_mode", text="")
            row = box.row(align=True)
            row.scale_y = 1.5
            row.prop(p, "brake", toggle=True, icon='PAUSE', text="Brake")
            row.prop(p, "hold", toggle=True, icon='PINNED', text="Hold")
            if p.hold:
                box.label(text="Test stand: wheels spin and steer live", icon='PINNED')
            box.operator("mqscar.reset_drive", icon='LOOP_BACK')
            box.label(text="Press Play (Space) to drive")
        else:
            box.label(text="Generate the rig first", icon='INFO')

        # Preset
        box = layout.box()
        box.label(text="Preset", icon='PRESET')
        box.prop(p, "preset", text="")
        if p.preset == 'DRIFT':
            box.prop(p, "drift_amount", slider=True)
            row = box.row(align=True)
            row.operator("mqscar.drift_mark", text="Drift From Here", icon='KEYFRAME_HLT').action = 'ON'
            row.operator("mqscar.drift_mark", text="Stop Drift Here", icon='KEYFRAME').action = 'OFF'
            box.operator("mqscar.drift_mark", text="Clear Marks", icon='X').action = 'CLEAR'
            box.label(text="Drift builds as you steer. Rig must be generated.", icon='INFO')

        # Advanced
        box = layout.box()
        row = box.row()
        row.prop(p, "show_advanced", text="Advanced",
                 icon='TRIA_DOWN' if p.show_advanced else 'TRIA_RIGHT', emboss=False)
        if p.show_advanced:
            box.prop(p, "mass", text="Body Weight", slider=True)
            box.prop(p, "susp_damping", text="Suspension Damping", slider=True)
            box.prop(p, "ride_height", text="Ride Height", slider=True)
            box.label(text="Ride Height moves the body live (sim must be stopped)", icon='INFO')
            box.label(text="Weight and damping apply when the sim restarts at frame 1")


# --------------------------------------------------------------------------
# Registration
# --------------------------------------------------------------------------
classes = (
    MQSCAR_Props,
    MQSCAR_OT_use_active,
    MQSCAR_OT_generate_rig,
    MQSCAR_OT_drift_mark,
    MQSCAR_OT_clear_rig,
    MQSCAR_OT_reset_drive,
    MQSCAR_PT_main,
)


def register():
    for c in classes:
        bpy.utils.register_class(c)
    bpy.types.Scene.mqscar = PointerProperty(type=MQSCAR_Props)
    if _on_load not in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.append(_on_load)


def unregister():
    _HOLD.clear()
    if bpy.app.timers.is_registered(_hold_tick):
        bpy.app.timers.unregister(_hold_tick)
    if _on_load in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.remove(_on_load)
    if hasattr(bpy.types.Scene, "mqscar"):
        del bpy.types.Scene.mqscar
    for c in reversed(classes):
        try:
            bpy.utils.unregister_class(c)
        except Exception:
            pass


if __name__ == "__main__":
    # Safe to re-run from the Text Editor
    unregister()
    register()
