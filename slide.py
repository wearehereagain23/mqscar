"""
MQScar Slide - a small extension for the deployed MQScar add-on (rig + Curve Mode).

It adds SECONDARY MOTION to a drive you have already baked: the car slides in corners like a
drift, instead of moving like a rigid four-corner robot. It does not touch or replace any of the
add-on's code. It only reads the keys that Bake / Update Drive put on the car, adds the slide on
top, and keys the parts again.

HOW TO USE (MQScar tab > "Slide" section, part of the combined add-on)
  1. Bake the drive as usual (Curve Mode > Bake / Update Drive).
  2. Open the Slide section, set Slide Amount (and Body Lift if a tire pokes out), press Apply Slide.
  3. To change the amount or go back: press Apply Slide again (or Bake again first if Blender was
     restarted). Remove Slide also works while the add-on has stayed loaded since you applied it.

WHAT IT DOES
  * Corners: the harder and faster the car corners, the more the tail swings out (nose into the
    turn). The angle goes through a lightly damped spring, so the tail overshoots a little and settles.
  * Slide Amount goes from 0 to 2. Up to about 0.5 the car turns about the middle of its wheelbase.
    Above that the pivot moves toward the front axle, so at 2 the BACK slides out of the line while
    the FRONT stays on it.
  * The front wheels counter-steer: they turn against the slide (past straight at high amounts) and
    react while the slide is growing, like a driver catching the car. The body leans onto the outside.
  * Reversing on speed (the drift spin): the car swings on a little as it comes out of the spin.
  * Slow turns (under about 15 km/h) do not slide. Slide Amount 0 changes nothing.

PER MARKER
  Click a route marker in the viewport and its "Marker Slide" appears in the MQScar Slide panel.
  It is a share of the Slide Amount: 1.0 = the full amount, 0.5 = half, 0 = no slide at this marker.
  A marker's value applies to the stretch around it (from halfway to the previous marker to halfway
  to the next one) and blends smoothly between markers, so put a marker on the corner you want to
  change. Changing it needs Apply Slide again.
"""
import bpy
from math import atan2, sin, cos, tanh, radians, pi, hypot
from bpy.props import PointerProperty, FloatProperty, StringProperty
from bpy.types import Operator, PropertyGroup
from mathutils import Matrix, Euler, Quaternion

# Names that come from the MQScar add-on
WHEEL_ATTRS = ("wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr")
FRONT_ATTRS = ("wheel_fl", "wheel_fr")
TAG_HUB = "mqscar_hub"
PIN_KEY = "mqfollow_pin"            # custom property that marks a route marker (its value is the marker number)
SPAN_KEY = "mqfollow_span"          # [first frame, last frame] of the baked drive
SIG_KEY = "mqslide_sig"             # signature of the keys right after Apply Slide

# Tuning (edit here). Slide Amount in the panel scales all of it.
SLIDE_MAX_ANGLE = radians(22.0)     # tail-out angle in a hard corner at Slide Amount 1.0
SLIDE_A_REF = 4.0                   # m/s^2 of cornering that counts as a hard corner
SLIDE_SPEED_LO = 15.0               # km/h, no slide below this
SLIDE_SPEED_HI = 50.0               # km/h, full slide from here up
SLIDE_FREQ = 2.0                    # Hz, how quickly the tail swings
SLIDE_DAMPING = 0.45                # 0.3 = wobbly, 0.7 = settles fast
SLIDE_HARD_LIMIT = radians(40.0)    # the tail never swings out further than this
SLIDE_PIVOT_START = 0.5             # Slide Amount where the pivot starts moving from the middle to the front axle
SLIDE_PIVOT_END = 2.0               # Slide Amount where the pivot is exactly on the front axle
SLIDE_COUNTERSTEER = 0.6            # front-wheel countersteer as a fraction of the slide angle at low amounts
SLIDE_COUNTER_GAIN = 0.4            # extra countersteer fraction added per 1.0 of Slide Amount (1.0 = wheels keep their line)
SLIDE_COUNTER_LEAD = 0.07           # s, wheels also react to how fast the slide is growing (catching the car)
SLIDE_COUNTER_MAX = radians(55.0)   # countersteer never turns the wheels more than this
SLIDE_ROLL = radians(2.5)           # body lean onto the outside in a hard corner
SLIDE_REVERSE_SCALE = 0.5           # slide is gentler in reverse
SLIDE_FLIP_KICK = radians(14.0)     # extra swing as the car comes out of a drift spin
SPIN_DETECT = radians(22.0)         # heading this far off the travel direction = drift spin (no slide)

_ORIG = {"sig": None, "mats": None}  # the un-slid keys, kept while this script stays loaded


# --------------------------------------------------------------------------
# Small 4x4 matrix helpers (plain lists)
# --------------------------------------------------------------------------
def m_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def m_tr(x, y, z):
    return [[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]]


def m_rot(axis, a):
    x, y, z = axis
    n = (x * x + y * y + z * z) ** 0.5 or 1.0
    x, y, z = x / n, y / n, z / n
    c, s = cos(a), sin(a)
    t = 1.0 - c
    return [[t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0],
            [0, 0, 0, 1]]


def m_about(p, R):
    """Rotation R about the point p."""
    return m_mul(m_tr(p[0], p[1], p[2]), m_mul(R, m_tr(-p[0], -p[1], -p[2])))


def m_pos(m):
    return (m[0][3], m[1][3], m[2][3])


def wrap(a):
    return (a + pi) % (2.0 * pi) - pi


# --------------------------------------------------------------------------
# The slide itself (pure maths, no Blender)
# --------------------------------------------------------------------------
def pivot_for(amount):
    """0 = turn about the middle of the wheelbase, 1 = about the front axle."""
    return max(0.0, min(1.0, (amount - SLIDE_PIVOT_START) / (SLIDE_PIVOT_END - SLIDE_PIVOT_START)))


def compute_slide(front, rear, dt, amounts):
    """front/rear: per-frame (x, y) of the point between the front / rear wheels.
    amounts: the Slide Amount for every frame (a list; a single number is used for all frames).
    Returns (beta, lean, counter, pivot): the slip angle, the body lean, the front-wheel countersteer
    (radians) and the pivot position (0 = middle, 1 = front axle) for every frame."""
    n = len(front)
    if not isinstance(amounts, (list, tuple)):
        amounts = [amounts] * n
    amounts = [max(0.0, min(2.0, float(a))) for a in amounts]
    if max(amounts, default=0.0) <= 1e-6 or n < 3 or dt <= 0.0:
        return [0.0] * n, [0.0] * n, [0.0] * n, [0.0] * n

    cx = [0.5 * (f[0] + r[0]) for f, r in zip(front, rear)]
    cy = [0.5 * (f[1] + r[1]) for f, r in zip(front, rear)]
    yaw = [atan2(f[1] - r[1], f[0] - r[0]) for f, r in zip(front, rear)]

    # speed, travel direction and gear from how the car's middle moves
    speed, slip_fwd = [0.0] * n, [0.0] * n
    for i in range(n):
        a, b = max(0, i - 1), min(n - 1, i + 1)
        vx, vy = (cx[b] - cx[a]) / ((b - a) * dt), (cy[b] - cy[a]) / ((b - a) * dt)
        speed[i] = hypot(vx, vy)
        slip_fwd[i] = wrap(yaw[i] - atan2(vy, vx)) if speed[i] > 0.5 else 0.0
    gear, g_prev = [1] * n, 1
    for i in range(n):
        if speed[i] > 0.5:
            g_prev = 1 if abs(slip_fwd[i]) < pi / 2 else -1
        gear[i] = g_prev

    # drift-spin frames: heading well away from both "straight ahead" and "straight back"
    lo, hi = SPIN_DETECT, pi - SPIN_DETECT
    core = [speed[i] > 2.0 and lo < abs(slip_fwd[i]) < hi for i in range(n)]
    spin = list(core)
    for i in [k for k in range(n) if core[k]]:
        for j in range(max(0, i - 3), min(n, i + 8)):      # a little before, longer after (the heading is still settling)
            spin[j] = True

    # yaw rate, lightly smoothed
    rate = [0.0] * n
    for i in range(1, n - 1):
        rate[i] = wrap(yaw[i + 1] - yaw[i - 1]) / (2.0 * dt)
    rate[0], rate[-1] = rate[1], rate[-2]
    for i in range(n):
        if spin[i]:
            rate[i] = 0.0               # the spin itself is not cornering load
    sm, k_sm = 0.0, min(1.0, dt / 0.12)
    for i in range(n):
        sm += (rate[i] - sm) * k_sm
        rate[i] = sm

    w_n = 2.0 * pi * SLIDE_FREQ
    sub = 4
    h = dt / sub
    beta = beta_v = lean = 0.0
    k_lean = min(1.0, dt / 0.15)
    limit = SLIDE_HARD_LIMIT
    betas, leans, counters = [0.0] * n, [0.0] * n, [0.0] * n
    pivots = [pivot_for(a) for a in amounts]
    in_spin, peak, peak_dir = False, 0.0, 1.0
    for i in range(n):
        if spin[i]:
            if core[i] and abs(slip_fwd[i]) > peak:      # spin direction from mid-spin frames only
                peak, peak_dir = abs(slip_fwd[i]), (1.0 if slip_fwd[i] >= 0.0 else -1.0)
            in_spin = True
        elif in_spin:
            # out of the drift spin: the tail swings on, then settles
            in_spin, peak = False, 0.0
            beta, beta_v = peak_dir * min(1.0, amounts[i]) * SLIDE_FLIP_KICK, 0.0
        t = max(0.0, min(1.0, (speed[i] * 3.6 - SLIDE_SPEED_LO) / (SLIDE_SPEED_HI - SLIDE_SPEED_LO)))
        g = t * t * (3.0 - 2.0 * t)                                # slide only builds up with speed
        u = tanh(1.5 * speed[i] * rate[i] / SLIDE_A_REF)           # signed cornering load: + = turning left
        amount = amounts[i]
        k_counter = SLIDE_COUNTERSTEER + SLIDE_COUNTER_GAIN * amount
        target = 0.0 if spin[i] else amount * SLIDE_MAX_ANGLE * u * g * (1.0 if gear[i] >= 0 else -SLIDE_REVERSE_SCALE)
        target = max(-limit, min(limit, target))
        for _ in range(sub):
            acc = w_n * w_n * (target - beta) - 2.0 * SLIDE_DAMPING * w_n * beta_v
            beta_v += acc * h
            beta += beta_v * h
        beta = max(-limit, min(limit, beta))
        lean += ((0.0 if spin[i] else amount * SLIDE_ROLL * u * g) - lean) * k_lean
        betas[i], leans[i] = beta, lean
        # front wheels turn against the slide, and a bit more while it is still growing
        counters[i] = max(-SLIDE_COUNTER_MAX, min(SLIDE_COUNTER_MAX, k_counter * beta + SLIDE_COUNTER_LEAD * beta_v))
    return betas, leans, counters, pivots


def slide_matrices(mats, roles, front, rear, betas, leans, counters, ground_z, pivots, lift=0.0, half_track=0.0):
    """mats: {part: [frame matrices]}; roles: {part: 'body' | 'front' | 'other'}.
    pivots: per frame, 0 = the car turns about the middle of its wheelbase, 1 = about the front axle.
    lift: Body Lift in metres, the body only goes up by this much (the wheels stay where they are).
    half_track: half the distance between the left and right wheels. When the body leans, its low side
    goes down by about half_track * sin(lean), so the body is raised by that much automatically.
    Returns the same structure with the slide applied."""
    out = {name: [] for name in mats}
    n = len(front)
    for i in range(n):
        mx, my = 0.5 * (front[i][0] + rear[i][0]), 0.5 * (front[i][1] + rear[i][1])
        fx, fy = front[i][0] - mx, front[i][1] - my                    # middle -> front axle
        pivot = (mx + pivots[i] * fx, my + pivots[i] * fy, 0.0)       # slides toward the front axle at high amounts
        beta, lean = betas[i], leans[i]
        turn = m_about(pivot, m_rot((0, 0, 1), beta))
        yaw = atan2(front[i][1] - rear[i][1], front[i][0] - rear[i][0]) + beta
        for name, frames in mats.items():
            M = m_mul(turn, frames[i])                                  # whole car swings about its middle
            if roles[name] == 'front':                                  # wheels counter-steer
                M = m_mul(m_about(m_pos(M), m_rot((0, 0, 1), -counters[i])), M)
            elif roles[name] == 'body' and abs(lean) > 1e-6:            # body leans, wheels stay planted
                p = (mx, my, ground_z)
                M = m_mul(m_about(p, m_rot((cos(yaw), sin(yaw), 0.0), lean)), M)
            if roles[name] == 'body':                                   # Body Lift + lean compensation, body only
                up = lift + half_track * abs(sin(lean))
                if up > 1e-9:
                    M = m_mul(m_tr(0.0, 0.0, up), M)
            out[name].append(M)
    return out


# --------------------------------------------------------------------------
# Per-marker slide (pure maths)
# --------------------------------------------------------------------------
def marker_frame_indices(centers, marker_xy, reach=3.0):
    """For each marker (in route order) the frame where the car is at it. The search only moves
    forward, so a route that crosses itself still gives the right frame for every marker."""
    n = len(centers)
    out, start = [], 0
    for mx, my in marker_xy:
        d = [hypot(c[0] - mx, c[1] - my) for c in centers]
        i = next((k for k in range(start, n) if d[k] <= reach), None)
        if i is None:                                   # never within reach: closest frame after the last marker
            i = min(range(start, n), key=lambda k: d[k])
        else:                                           # walk down to the closest point of this pass
            while i + 1 < n and d[i + 1] < d[i]:
                i += 1
        out.append(i)
        start = i
    return out


def amount_per_frame(n, marker_frames, factors, global_amount, blend_frames=8):
    """Slide Amount for every frame. Each marker's value applies to the stretch around it (from halfway
    to the previous marker to halfway to the next one); between two markers it blends smoothly over
    about blend_frames, so the slide fades in and out instead of switching."""
    if not marker_frames:
        return [global_amount] * n
    mids = [0.5 * (marker_frames[k] + marker_frames[k + 1]) for k in range(len(marker_frames) - 1)]
    halves = []
    for k, mid in enumerate(mids):
        room = min([mid - (mids[k - 1] if k > 0 else marker_frames[0]),
                    (mids[k + 1] if k + 1 < len(mids) else marker_frames[-1]) - mid] + [1e9])
        halves.append(max(0.5, min(0.5 * blend_frames, 0.5 * room)))
    out = []
    for i in range(n):
        f = factors[0]
        for k, mid in enumerate(mids):
            t = max(0.0, min(1.0, (i - (mid - halves[k])) / (2.0 * halves[k])))
            f += (factors[k + 1] - f) * (t * t * (3.0 - 2.0 * t))
        out.append(global_amount * max(0.0, min(1.0, f)))
    return out


# --------------------------------------------------------------------------
# Reading and writing the baked keys
# --------------------------------------------------------------------------
def get_parts(sc):
    """{part name: object} for the body, the four wheels and the hubs. Same set Bake keys."""
    p = sc.mqscar
    parts = {"body": p.body}
    for a in WHEEL_ATTRS:
        parts[a] = getattr(p, a)
    for o in bpy.data.objects:
        attr = o.get(TAG_HUB)
        if attr in WHEEL_ATTRS:
            parts["hub_" + attr] = o
    return {k: v for k, v in parts.items() if v is not None}


def get_markers():
    """The route markers in route order."""
    pins = [o for o in bpy.data.objects if PIN_KEY in o]
    pins.sort(key=lambda o: o[PIN_KEY])
    return pins


def _rot_path(obj):
    return "rotation_quaternion" if obj.rotation_mode == 'QUATERNION' else "rotation_euler"


def _channel(obj, path, count, frame):
    ad = obj.animation_data
    out = list(getattr(obj, path))
    if ad is None or ad.action is None:
        return out
    for idx in range(count):
        fc = ad.action.fcurves.find(path, index=idx)
        if fc is not None:
            out[idx] = fc.evaluate(frame)
    return out


def sample(obj, frame):
    """The object's keyed pose at this frame, as a 4x4 list of lists."""
    loc = _channel(obj, "location", 3, frame)
    if obj.rotation_mode == 'QUATERNION':
        rot = Quaternion(_channel(obj, "rotation_quaternion", 4, frame)).to_matrix().to_4x4()
    else:
        rot = Euler(_channel(obj, "rotation_euler", 3, frame), obj.rotation_mode).to_matrix().to_4x4()
    M = Matrix.Translation(loc) @ rot
    return [[float(M[r][c]) for c in range(4)] for r in range(4)]


def signature(parts, f0, f1):
    """A number that changes when the keys change (so we know if a new bake replaced ours)."""
    total, count = 0.0, 0
    for obj in parts.values():
        ad = obj.animation_data
        if ad is None or ad.action is None:
            continue
        for fc in ad.action.fcurves:
            for kp in fc.keyframe_points:
                total += kp.co[1] * (1.0 + 0.001 * (kp.co[0] % 17))
                count += 1
    return f"{count}:{total:.5f}"


def write_pose(obj, M, frame):
    obj.matrix_world = Matrix(M)
    obj.keyframe_insert("location", frame=frame)
    obj.keyframe_insert(_rot_path(obj), frame=frame)


def _prepare(context):
    """Common checks. Returns (scene, parts, f0, f1, error)."""
    sc = context.scene
    if getattr(sc, "mqscar", None) is None:
        return sc, None, 0, 0, "The MQScar add-on is not installed"
    span = sc.get(SPAN_KEY)
    if span is None:
        return sc, None, 0, 0, "Bake the drive first (Curve Mode > Bake / Update Drive)"
    parts = get_parts(sc)
    if "body" not in parts or any(a not in parts for a in WHEEL_ATTRS):
        return sc, None, 0, 0, "MQScar Body / wheels are not set"
    if any(o.animation_data is None or o.animation_data.action is None for o in parts.values()):
        return sc, None, 0, 0, "The car has no baked animation. Bake the drive first"
    return sc, parts, int(span[0]), int(span[1]), ""


# --------------------------------------------------------------------------
# Properties, operators, panel
# --------------------------------------------------------------------------
class MQSLIDE_Props(PropertyGroup):
    amount: FloatProperty(name="Slide Amount", default=0.3, min=0.0, max=2.0, precision=2, subtype='FACTOR',
        description="How much the car slides in corners, like a drift. 0 = rigid, about 0.3 = subtle, 1 = strong, 2 = the back slides right out while the front stays on the line")
    lift: FloatProperty(name="Body Lift", default=0.0, min=0.0, max=1.0, soft_max=0.3, precision=3,
        subtype='DISTANCE', unit='LENGTH',
        description="Raises the car body (not the wheels) during the slide, so the tires do not poke through. "
                    "The body is already raised automatically when it leans; this adds extra on top. Press Apply Slide again to use it")
    info: StringProperty(name="Info", default="")


def _marker_prop():
    return FloatProperty(
        name="Marker Slide", default=1.0, min=0.0, max=1.0, precision=2, subtype='FACTOR',
        description="Share of the Slide Amount used at this route marker. 1 = full, 0.5 = half, 0 = no slide here")


class MQSLIDE_OT_apply(Operator):
    bl_idname = "mqslide.apply"
    bl_label = "Apply Slide"
    bl_description = "Add the slide to the baked drive (Bake first). Bake again to change the amount"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        sc, parts, f0, f1, err = _prepare(context)
        props = sc.mqslide
        if err:
            self.report({'ERROR'}, err)
            return {'CANCELLED'}
        cur = signature(parts, f0, f1)
        if sc.get(SIG_KEY) == cur:
            if _ORIG["sig"] == cur and _ORIG["mats"] is not None:
                mats = _ORIG["mats"]                     # already slid: start again from the original keys
            else:
                self.report({'ERROR'}, "Slide is already applied to this bake. Bake again to change it")
                return {'CANCELLED'}
        else:
            mats = {name: [sample(o, fr) for fr in range(f0, f1 + 1)] for name, o in parts.items()}
        fps = sc.render.fps / sc.render.fps_base
        front = [((a[0][3] + b[0][3]) / 2, (a[1][3] + b[1][3]) / 2) for a, b in
                 zip(mats["wheel_fl"], mats["wheel_fr"])]
        rear = [((a[0][3] + b[0][3]) / 2, (a[1][3] + b[1][3]) / 2) for a, b in
                zip(mats["wheel_rl"], mats["wheel_rr"])]
        centers = [((f[0] + r[0]) / 2, (f[1] + r[1]) / 2) for f, r in zip(front, rear)]
        pins = get_markers()
        mframes = marker_frame_indices(centers, [(o.matrix_world.translation.x, o.matrix_world.translation.y)
                                                 for o in pins]) if pins else []
        factors = [float(o.mqslide_marker) for o in pins]
        amounts = amount_per_frame(len(front), mframes, factors, props.amount, max(2, int(0.35 * fps)))
        betas, leans, counters, pivots = compute_slide(front, rear, 1.0 / fps, amounts)
        dims = sorted(parts["wheel_fl"].dimensions)
        radius = 0.5 * dims[1]
        ground_z = min(m_pos(M)[2] for name in WHEEL_ATTRS for M in mats[name]) - radius
        roles = {name: ('body' if name == "body" else
                        'front' if (name in FRONT_ATTRS or name.replace("hub_", "") in FRONT_ATTRS) else 'other')
                 for name in parts}
        half_track = 0.5 * hypot(mats["wheel_fl"][0][0][3] - mats["wheel_fr"][0][0][3],
                                 mats["wheel_fl"][1][0][3] - mats["wheel_fr"][1][0][3])
        new = slide_matrices(mats, roles, front, rear, betas, leans, counters, ground_z, pivots,
                             props.lift, half_track)
        for name, obj in parts.items():
            for i, M in enumerate(new[name]):
                write_pose(obj, M, f0 + i)
        _ORIG["mats"] = mats
        _ORIG["sig"] = signature(parts, f0, f1)
        sc[SIG_KEY] = _ORIG["sig"]
        try:
            sc.frame_set(sc.frame_current)
        except Exception:
            pass
        off = sum(1 for x in factors if x <= 1e-6)
        props.info = (f"Slide applied (amount {props.amount:.2f}) to frames {f0}-{f1}"
                      + (f"; {off} marker(s) set to 0" if off else ""))
        return {'FINISHED'}


class MQSLIDE_OT_remove(Operator):
    bl_idname = "mqslide.remove"
    bl_label = "Remove Slide"
    bl_description = "Put the baked drive back the way it was before Apply Slide"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        sc, parts, f0, f1, err = _prepare(context)
        if err:
            self.report({'ERROR'}, err)
            return {'CANCELLED'}
        cur = signature(parts, f0, f1)
        if sc.get(SIG_KEY) != cur:
            self.report({'INFO'}, "No slide is applied to this bake")
            return {'CANCELLED'}
        if _ORIG["sig"] != cur or _ORIG["mats"] is None:
            self.report({'ERROR'}, "The original keys were not kept. Bake again to remove the slide")
            return {'CANCELLED'}
        for name, obj in parts.items():
            for i, M in enumerate(_ORIG["mats"][name]):
                write_pose(obj, M, f0 + i)
        _ORIG["sig"] = None
        _ORIG["mats"] = None
        if SIG_KEY in sc:
            del sc[SIG_KEY]
        sc.mqslide.info = "Slide removed"
        return {'FINISHED'}


def draw_slide(lay, context):
    """Slide UI. Drawn inside the combined MQScar panel (see __init__.py)."""
    props = context.scene.mqslide
    lay.prop(props, "amount", slider=True)
    lay.prop(props, "lift")
    lay.label(text="0 rigid | 0.3 subtle | 1 strong | 2 back slides out, front holds")
    col = lay.column(align=True)
    col.scale_y = 1.4
    col.operator("mqslide.apply", icon='FORCE_VORTEX')
    col.operator("mqslide.remove", icon='LOOP_BACK')
    lay.label(text="Bake first. To change the amount: Apply again (Bake again after a restart).", icon='INFO')

    box = lay.box()
    ob = context.active_object
    if ob is not None and PIN_KEY in ob:
        box.label(text=f"Selected Marker {int(ob[PIN_KEY])}", icon='EMPTY_AXIS')
        box.prop(ob, "mqslide_marker", slider=True, text="Marker Slide")
        share = float(ob.mqslide_marker)
        if share <= 1e-6:
            box.label(text="No slide at this marker.", icon='X')
        else:
            box.label(text=f"Uses {share * 100:.0f}% of the Slide Amount ({props.amount * share:.2f}).")
        box.label(text="Then press Apply Slide again.")
    else:
        box.label(text="Click a route marker to set its own slide.", icon='INFO')
    if props.info:
        lay.label(text=props.info, icon='CHECKMARK')


classes = (MQSLIDE_Props, MQSLIDE_OT_apply, MQSLIDE_OT_remove)


def register():
    unregister()
    for c in classes:
        bpy.utils.register_class(c)
    bpy.types.Scene.mqslide = PointerProperty(type=MQSLIDE_Props)
    bpy.types.Object.mqslide_marker = _marker_prop()


def unregister():
    if hasattr(bpy.types.Object, "mqslide_marker"):
        del bpy.types.Object.mqslide_marker
    if hasattr(bpy.types.Scene, "mqslide"):
        del bpy.types.Scene.mqslide
    for c in reversed(classes):
        try:
            bpy.utils.unregister_class(c)
        except Exception:
            pass

