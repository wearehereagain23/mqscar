"""
MQScar EFFECTS  (independent test script, not part of the add-on zip)

HOW TO USE
  1. Open this file in Blender's Text Editor and press Alt+P ONCE per Blender session.
  2. From then on, every time you press Bake / Update Drive (or Clear), the effects update by
     themselves within about half a second. No need to run the script again.
  3. A "Tire Effects" panel appears in the 3D viewport sidebar (N) under the MQScar tab. Everything you can
     change (which tyres, strength, colour, how much of the drive is marked, softness...) is there and the
     marks update by themselves. Strength and colour are instant; the rest redraw after about half a second.
  4. To stop it: set AUTO_REBUILD = False below and run the script again (this also removes the effects and the panel).
  4. To SEE the realistic rubber (soft edges, tread grooves, streaks) switch the 3D viewport to
     Material Preview or Rendered. Solid mode only shows flat dark strips.
  Tip: in the Text Editor, tick "Register" (Text menu) to have it start when the .blend opens.

EFFECTS IN THIS FILE
  burnout_lines : tyre marks laid down ONLY where the tyres are pushed past their grip.
      The script reads the baked drive (where every wheel is, frame by frame) and works out what the
      car is doing: speed, acceleration, braking, cornering force, which way it points compared with
      which way it travels. From that it estimates, for every wheel and every frame:
          * how much weight the wheel carries (weight moves to the outside in corners, to the rear
            when accelerating, to the front when braking)
          * how much grip that wheel is asked for (cornering + engine/braking force, friction circle)
          * how far sideways the tyre is sliding (slip angle: drift, tail-out, understeer)
          * wheelspin (baked wheel rotation against the distance travelled)
      where the demand goes past the grip limit, rubber is laid down: faint broken patches first,
      solid black when it keeps going, fading out over a few metres when the tyre grips again.
      A car that drives gently leaves nothing.
      It works the same on a normal (rigid) bake and after Apply Slide: a rigid car leaves marks from
      hard corners, hard braking and hard launches (the outside tyres first); a sliding car also leaves
      drift marks, heaviest on the tyres that are sliding.
      The mark itself: near-black slightly glossy rubber, grainy and broken when thin, darker at the
      tyre edges, tread grooves, soft ragged edges, never ruler-straight. Wheels in the air draw nothing.

ADD MORE EFFECTS: write a build function and a remove function, then add them to EFFECTS at the bottom.
"""
import bpy
import traceback
import time
import textwrap
from math import sin, cos, acos, sqrt, exp, hypot, radians, floor
from mathutils import Vector

# ---- settings -------------------------------------------------------------
AUTO_REBUILD = True        # watch for new bakes and redraw the effects automatically
CHECK_SECONDS = 0.5        # how often the watcher looks for a new bake
ALL_ROLES = ("wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr")
WHEEL_ROLES = ALL_ROLES    # wheels that may leave marks (the physics decides which ones really do).
#                            rear only: ("wheel_rl", "wheel_rr")

BURN_MAX = 0.97            # darkest a mark can get (0-1)
BURN_WIDTH = 0.92          # mark width as a fraction of the tyre width (a bit narrower when faint, wider when heavy)
RISE_LENGTH = 1.3          # metres of travel for the mark to build up (bigger = longer, softer start)
FALL_LENGTH = 2.6          # metres for the mark to fade away after the slip stops
RISE_BASE, FALL_BASE = RISE_LENGTH, FALL_LENGTH
AUTO_GRIP = True           # judge the drive against itself: only its hardest MARK_SHARE of moments leave marks
MARK_SHARE = 0.30          # share of the moving drive (0-1) that is allowed to leave marks when AUTO_GRIP is on
MARK_MIN_DEMAND = 0.50     # a drive whose hardest moments stay below this (of the grip) is "gentle": no marks at all
VARIATION = 0.60           # 0 = even mark, 1 = very uneven: soft faint stretches and hard black stretches
SMUDGE = 0.50              # soft grey rubber dust around the mark (0 = none)
MIN_VISIBLE = 0.10         # anything fainter than this is not drawn at all (no ghost lines, no faint tails)
MIN_MARK_PEAK = 0.22       # a mark that never gets stronger than this is dropped (it would only be a smear)
MIN_MARK_LENGTH = 0.8      # metres: shorter flickers are dropped
ROLL_TRACE = 0.0           # darkness while just rolling. 0 = no mark at all; try 0.04 for a barely-there trace
WANDER = 0.010             # metres the mark drifts sideways (it is never perfectly straight)
DRIFT_MARKS = True         # marks from sideways tyre slide (drift, tail-out). Off = only grip demand + wheelspin

# --- how the car is modelled (stylised, tune to taste) -------------------------------------------
GRIP = 0.50                # g of grip the tyres give. LOWER = marks appear more easily, higher = only violent driving
WEIGHT_FRONT = 0.50        # share of the weight over the front axle (0.5 balanced, 0.6 nose heavy)
CG_HEIGHT = 1.6            # height of the centre of gravity, in wheel radii (taller = more weight transfer)
ROLL_FRONT = 0.55          # share of the side-to-side weight transfer carried by the front axle
LOAD_SENS = 0.25           # a heavily loaded tyre has a little less grip per kg (the outside tyre lets go first)
DRIVE_AXLE = "rear"        # which wheels get the engine force: "rear", "front" or "all"
BRAKE_BIAS = 0.60          # share of the braking done by the front wheels
CORNER_GAIN = 1.0          # multiply the cornering demand (0 = ignore corners)
DRIVE_GAIN = 1.0           # multiply the acceleration demand (0 = no launch / power marks)
BRAKE_GAIN = 1.0           # multiply the braking demand (0 = no braking marks)
ALPHA_DEAD = 3.0           # degrees of tyre slip that are normal rolling (ignored)
ALPHA_PEAK = 9.0           # degrees of tyre slip where the tyre is fully sliding
SMOOTH_TIME = 0.12         # seconds over which the measured accelerations are smoothed
FRICTION_ON = 0.85         # demand / grip where a tyre starts leaving rubber
FRICTION_FULL = 1.35       # demand / grip where it leaves a full black mark
SLIDE_SIG_KEY = "mqslide_sig"   # written by MQScar Slide when Apply Slide has been used
G = 9.81
BLOB_MIN_SLIP = 0.35       # wheelspin (0-1) above which a standing wheel leaves a rubber patch
BLOB_LENGTH = 0.32         # metres, length of one patch layer
BLOB_ALPHA = 0.22          # darkness added per frame by a patch layer (layers pile up)
SLIP_TOLERANCE = 1.08      # a wheel may cover this much more spin than distance before it counts as slipping
MIN_STEP = 0.003           # metres per frame; less than this = the wheel is standing still
AIR_TOLERANCE = 0.06       # metres a wheel may rise before it counts as airborne
LIFT = 0.01                # metres above the ground (stops flicker against the ground plane)
# ---------------------------------------------------------------------------

FX = "MQFX_"
BURN_PREFIX = FX + "Burnout_"
BURN_COLLECTION = FX + "Burnouts"
OLD_PREFIXES = (FX + "TireLine_",)           # from the previous version of this script
UV_NAME = "MQFX_UV"
ATTR_NAME = "MQFX_Burn"
PARTS_KEY = "mqfollow_parts"     # written by MQScar Curve Mode when it bakes
SPAN_KEY = "mqfollow_span"
STATE = bpy.app.driver_namespace  # survives re-running this script


# ---------------------------------------------------------------- baked car --
def loc_at(obj, frame):
    """World location of a baked part at a frame, read from its keys (no frame change needed)."""
    loc = list(obj.location)
    ad = obj.animation_data
    if ad and ad.action:
        for i in range(3):
            fc = ad.action.fcurves.find("location", index=i)
            if fc is not None:
                loc[i] = fc.evaluate(frame)
    return Vector(loc)


def baked_wheels(scene, roles=None):
    """[(role, wheel object)] of the baked car, plus how they were found."""
    roles = roles or WHEEL_ROLES
    names = list(scene.get(PARTS_KEY, []))
    found = []
    props = getattr(scene, "mqscar", None)
    if props is not None:
        for role in roles:
            ob = getattr(props, role, None)
            if ob is not None and ob.name in names:
                found.append((role, ob))
        if found:
            return found, "MQScar rig"
    for n in names:
        low = n.lower()
        if "wheel" in low and "hub" not in low:
            role = "wheel_" + low[-2:] if low[-2:] in ("fl", "fr", "rl", "rr") else n.replace(".", "_")
            if role in ALL_ROLES and role not in roles:
                continue
            ob = bpy.data.objects.get(n)
            if ob is not None:
                found.append((role, ob))
    return found, "wheel names"


def bake_signature(scene):
    """Changes whenever the car is baked differently (or cleared). None = no baked drive."""
    span = scene.get(SPAN_KEY)
    if not span or not list(scene.get(PARTS_KEY, [])):
        return None
    wheels, _how = baked_wheels(scene, ALL_ROLES)
    f0, f1 = int(span[0]), int(span[1])
    sig = [f0, f1, len(wheels), str(scene.get(SLIDE_SIG_KEY, ""))]
    for _role, w in wheels:
        for k in range(7):
            p = loc_at(w, f0 + (f1 - f0) * k // 6)
            sig += [round(p.x, 4), round(p.y, 4), round(p.z, 4)]
    return tuple(sig)


# ---------------------------------------------------------- wheelspin maths --
def _rot(ch, a):
    c, s = cos(a), sin(a)
    if ch == "X":
        return [[1, 0, 0], [0, c, -s], [0, s, c]]
    if ch == "Y":
        return [[c, 0, s], [0, 1, 0], [-s, 0, c]]
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def _mm(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _euler_matrix(e, order):
    R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    for ch in order:                       # 'XYZ' means R = Rz * Ry * Rx
        R = _mm(_rot(ch, e["XYZ".index(ch)]), R)
    return R


def rear_axle_info(scene, f0, f1):
    """(centres, axle directions) of the rear axle for every frame. The centre is the path the bake measures
    wheel spin against; the axle direction (flat, unit length) tells which way the car is pointing."""
    names = list(scene.get(PARTS_KEY, []))
    props = getattr(scene, "mqscar", None)
    rear = []
    for role in ("wheel_rl", "wheel_rr"):
        ob = getattr(props, role, None) if props is not None else None
        if ob is not None and ob.name in names:
            rear.append(ob)
    if len(rear) < 2:
        rear = [bpy.data.objects.get(n) for n in names
                if n.lower().startswith("wheel") and n.lower()[-2:] in ("rl", "rr")]
        rear = [o for o in rear if o is not None]
    if len(rear) < 2:
        return None, None
    out, axes = [], []
    for fr in range(f0, f1 + 1):
        a, b = loc_at(rear[0], fr), loc_at(rear[1], fr)
        out.append(Vector(((a.x + b.x) / 2, (a.y + b.y) / 2, (a.z + b.z) / 2)))
        d = Vector((b.x - a.x, b.y - a.y, 0.0))
        axes.append(d.normalized() if d.length > 1e-6 else (axes[-1] if axes else None))
    return out, axes


def _smooth(lo, hi, x):
    t = max(0.0, min(1.0, (x - lo) / (hi - lo)))
    return t * t * (3.0 - 2.0 * t)



def wheel_slip(wheel, locs, f0, f1, radius, ref=None):
    """Wheelspin per frame interval, 0 = rolling normally, 1 = spinning with no travel.
    Spin comes from how the baked wheel rotation changes about its axle. It is compared with the travel
    of `ref` (the rear axle centre, which is what the bake spins every wheel by), not the wheel's own path,
    so the inner wheel in a bend is not mistaken for wheelspin."""
    zero = [0.0] * (len(locs) - 1)
    order = str(getattr(wheel, "rotation_mode", ""))
    if len(order) != 3 or any(c not in "XYZ" for c in order):
        return zero, None
    ad = wheel.animation_data
    if not (ad and ad.action):
        return zero, None
    fcs = [ad.action.fcurves.find("rotation_euler", index=i) for i in range(3)]
    if any(fc is None for fc in fcs):
        return zero, None
    dims = list(wheel.dimensions)
    axle = dims.index(min(dims))           # the thin axis of a wheel is its axle
    Rs = [_euler_matrix([fc.evaluate(f) for fc in fcs], order) for f in range(f0, f1 + 1)]
    axes = []                              # which way the axle points (flat, unit) at every frame
    for R in Rs:
        ax, ay = R[0][axle], R[1][axle]
        ln = hypot(ax, ay)
        axes.append(Vector((ax / ln, ay / ln, 0.0)) if ln > 1e-6 else (axes[-1] if axes else None))
    out = []
    for i in range(len(locs) - 1):
        A, B = Rs[i], Rs[i + 1]
        Rr = _mm(B, [[A[c][r] for c in range(3)] for r in range(3)])     # B * A^T
        tr = Rr[0][0] + Rr[1][1] + Rr[2][2]
        th = acos(max(-1.0, min(1.0, (tr - 1.0) / 2.0)))
        k = 0.5 if th < 1e-6 else th / (2.0 * sin(th))
        rv = [(Rr[2][1] - Rr[1][2]) * k, (Rr[0][2] - Rr[2][0]) * k, (Rr[1][0] - Rr[0][1]) * k]
        spin = abs(sum(rv[j] * A[j][axle] for j in range(3))) * radius      # metres of surface per frame
        d = ((ref or locs)[i + 1] - (ref or locs)[i]).length
        excess = spin - SLIP_TOLERANCE * d - 0.01
        out.append(max(0.0, min(1.0, excess / (spin + 0.02))))
    return out, axes


# ------------------------------------------------------------ the friction model --
def _median(vals):
    v = sorted(vals)
    return v[len(v) // 2]


def _avg(vals, r):
    """Moving average with radius r (edge-safe)."""
    n = len(vals)
    out = []
    for i in range(n):
        lo, hi = max(0, i - r), min(n, i + r + 1)
        out.append(sum(vals[lo:hi]) / (hi - lo))
    return out


def _split(total, d):
    """Share `total` between the left and right wheel of an axle after moving `d` of it to the right."""
    left = min(max(total * 0.5 - d, 0.02), total - 0.02)
    return left, total - left


def _spin_util(slip):
    """Wheelspin 0..1 (see wheel_slip) expressed as demand / grip, so it lines up with FRICTION_ON/FULL."""
    if slip < 0.10:
        return slip * 8.5
    return FRICTION_ON + (slip - 0.10) * (FRICTION_FULL - FRICTION_ON) / 0.45


def slip_angle_util(locs, axes):
    """Sideways tyre slide per frame interval as demand / grip (1 = fully sliding).
    Compares where the wheel really travels with where its own axle says it is rolling."""
    dead, peak = sin(radians(ALPHA_DEAD)), sin(radians(ALPHA_PEAK))
    out = []
    for i in range(len(locs) - 1):
        vx, vy = locs[i + 1].x - locs[i].x, locs[i + 1].y - locs[i].y
        sp = hypot(vx, vy)
        ax = axes[i] if axes else None
        if ax is None or sp < MIN_STEP * 2:
            out.append(0.0)
            continue
        side = abs(vx * ax.x + vy * ax.y) / sp            # sin(slip angle)
        out.append(max(0.0, side - dead) / (peak - dead))
    return out


def car_dynamics(P, fps, radius, unit=1.0):
    """What the car is doing and what each tyre is asked to do, from the baked wheel paths.
    P: {role: [Vector per frame]} for all four wheels. Returns None if the car cannot be analysed, else
      util[role][frame] : force asked of the tyre / grip it has (1 = at the limit)
      load[role][frame] : weight on the tyre, 0.25 = its static share of the car's weight
      plus peak numbers (in g) for the report."""
    fl, fr, rl, rr = (P[k] for k in ALL_ROLES)
    n = len(fl)
    if n < 5 or fps <= 0:
        return None
    dt = 1.0 / fps
    fx = [(fl[i].x + fr[i].x) * 0.5 for i in range(n)]
    fy = [(fl[i].y + fr[i].y) * 0.5 for i in range(n)]
    rx = [(rl[i].x + rr[i].x) * 0.5 for i in range(n)]
    ry = [(rl[i].y + rr[i].y) * 0.5 for i in range(n)]
    L = _median([hypot(fx[i] - rx[i], fy[i] - ry[i]) for i in range(n)])
    T = _median([hypot(fl[i].x - fr[i].x, fl[i].y - fr[i].y) for i in range(n)])
    if L < 0.2 or T < 0.2:
        return None
    wf = min(0.9, max(0.1, WEIGHT_FRONT))
    hcg = CG_HEIGHT * radius
    hL, hT = hcg / L, hcg / T

    # centre of gravity path and which way the car points (rear axle -> front axle)
    cx = [rx[i] + (fx[i] - rx[i]) * wf for i in range(n)]
    cy = [ry[i] + (fy[i] - ry[i]) * wf for i in range(n)]
    hx, hy, last = [], [], (1.0, 0.0)
    for i in range(n):
        d = hypot(fx[i] - rx[i], fy[i] - ry[i])
        if d > 1e-6:
            last = ((fx[i] - rx[i]) / d, (fy[i] - ry[i]) / d)
        hx.append(last[0])
        hy.append(last[1])

    r = max(1, int(round(SMOOTH_TIME * fps * 0.5)))
    vx, vy = [0.0] * n, [0.0] * n
    for i in range(n):
        a, b = max(0, i - 1), min(n - 1, i + 1)
        vx[i], vy[i] = (cx[b] - cx[a]) / ((b - a) * dt), (cy[b] - cy[a]) / ((b - a) * dt)
    ax, ay = [0.0] * n, [0.0] * n
    for i in range(n):
        a, b = max(0, i - 1), min(n - 1, i + 1)
        ax[i], ay[i] = (vx[b] - vx[a]) / ((b - a) * dt), (vy[b] - vy[a]) / ((b - a) * dt)
    ax, ay = _avg(ax, r), _avg(ay, r)
    ax, ay = [a * unit for a in ax], [a * unit for a in ay]          # scene units -> metres per second squared
    speed = [hypot(vx[i], vy[i]) * unit for i in range(n)]

    util = {k: [0.0] * n for k in ALL_ROLES}
    load = {k: [0.25] * n for k in ALL_ROLES}
    peak_lat = peak_acc = peak_brk = 0.0
    for i in range(n):
        a_long = (ax[i] * hx[i] + ay[i] * hy[i]) / G                   # + = speeding up, in g
        gate = _smooth(1.0, 3.0, speed[i])                              # no cornering force from a standing car
        a_lat = (-ax[i] * hy[i] + ay[i] * hx[i]) / G * gate            # + = pulled to the left, in g
        peak_lat, peak_acc, peak_brk = max(peak_lat, abs(a_lat)), max(peak_acc, a_long), max(peak_brk, -a_long)

        # weight on each tyre (1 = whole car): fore-aft transfer, then side-to-side per axle
        nf = min(0.95, max(0.05, wf - hL * a_long))
        nr = 1.0 - nf
        n_fl, n_fr = _split(nf, hT * a_lat * ROLL_FRONT)
        n_rl, n_rr = _split(nr, hT * a_lat * (1.0 - ROLL_FRONT))
        n_w = {"wheel_fl": n_fl, "wheel_fr": n_fr, "wheel_rl": n_rl, "wheel_rr": n_rr}

        # cornering force: each axle takes its share, then each tyre in proportion to its load
        fy_f, fy_r = abs(wf * a_lat) * CORNER_GAIN, abs((1.0 - wf) * a_lat) * CORNER_GAIN
        side = {"wheel_fl": fy_f * n_fl / nf, "wheel_fr": fy_f * n_fr / nf,
                "wheel_rl": fy_r * n_rl / nr, "wheel_rr": fy_r * n_rr / nr}
        # engine / brake force
        if a_long >= 0.0:
            tot = a_long * DRIVE_GAIN
            if DRIVE_AXLE == "front":
                fwd = {"wheel_fl": tot / 2, "wheel_fr": tot / 2, "wheel_rl": 0.0, "wheel_rr": 0.0}
            elif DRIVE_AXLE == "all":
                fwd = {k: tot / 4 for k in ALL_ROLES}
            else:
                fwd = {"wheel_fl": 0.0, "wheel_fr": 0.0, "wheel_rl": tot / 2, "wheel_rr": tot / 2}
        else:
            tot = -a_long * BRAKE_GAIN
            fb, rb = tot * BRAKE_BIAS / 2, tot * (1.0 - BRAKE_BIAS) / 2
            fwd = {"wheel_fl": fb, "wheel_fr": fb, "wheel_rl": rb, "wheel_rr": rb}

        for k in ALL_ROLES:
            nk = n_w[k]
            mu = GRIP * max(0.5, 1.0 - LOAD_SENS * (nk / 0.25 - 1.0))
            util[k][i] = min(50.0, hypot(fwd[k], side[k]) / (mu * max(nk, 0.02)))    # friction circle
            load[k][i] = nk
    return {"util": util, "load": load, "lat": peak_lat, "acc": peak_acc, "brk": peak_brk}


# --------------------------------------------------------- burnout material --
def _burnout_material(halo=1.0, variation=0.6):
    """Dark, slightly glossy rubber. Opacity = mark intensity x soft ragged edge x tread grooves x uneven
    streaks x grain, plus a faint dust halo beside the mark. Thin marks are matte and broken up, heavy marks
    are solid, a bit glossy and slightly raised (bump). Colour and Strength are separate nodes so the panel
    can change them instantly."""
    name = BURN_PREFIX + "Mat"
    mat = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    mat.diffuse_color = (0.02, 0.02, 0.02, 1.0)
    mat.use_nodes = True
    try:
        mat.blend_method = 'BLEND'
        mat.shadow_method = 'NONE'
        mat.show_transparent_back = False
    except Exception:
        pass
    nt = mat.node_tree
    nodes, links = nt.nodes, nt.links
    nodes.clear()

    def node(kind, x, y, **props):
        n = nodes.new(kind)
        n.location = (x, y)
        for k, v in props.items():
            setattr(n, k, v)
        return n

    def feed(socket, value):
        if isinstance(value, (int, float)):
            socket.default_value = value
        else:                                                         # an output socket
            links.new(value, socket)

    def calc(op, a, b=None, c=None, clamp=False, x=0, y=0):
        n = node('ShaderNodeMath', x, y, operation=op, use_clamp=clamp)
        feed(n.inputs[0], a)
        if b is not None:
            feed(n.inputs[1], b)
        if c is not None:
            feed(n.inputs[2], c)
        return n.outputs[0]

    def noise(vec_socket, scale, detail, x, y):
        n = node('ShaderNodeTexNoise', x, y)
        n.inputs['Scale'].default_value = scale
        n.inputs['Detail'].default_value = detail
        n.inputs['Roughness'].default_value = 0.6
        links.new(vec_socket, n.inputs['Vector'])
        return n.outputs['Fac']

    def vec(xv, yv, zv, x, y):
        n = node('ShaderNodeCombineXYZ', x, y)
        feed(n.inputs['X'], xv)
        feed(n.inputs['Y'], yv)
        feed(n.inputs['Z'], zv)
        return n.outputs['Vector']

    attr = node('ShaderNodeAttribute', -1400, 500, attribute_type='GEOMETRY', attribute_name=ATTR_NAME)
    uv = node('ShaderNodeUVMap', -1400, 0, uv_map=UV_NAME)
    sep = node('ShaderNodeSeparateXYZ', -1200, 0)
    links.new(uv.outputs['UV'], sep.inputs['Vector'])
    across, along = sep.outputs['X'], sep.outputs['Y']            # across: 0..1, along: metres

    # the strip is wider than the tyre by `halo`: the tyre footprint is the middle part, the rest is dust
    abs_c = calc('ABSOLUTE', calc('MULTIPLY_ADD', across, 2.0, -1.0))          # 0 centre .. 1 strip edge
    core_c = calc('MULTIPLY', abs_c, halo)                                      # 0 centre .. 1 tyre edge
    centre = calc('SUBTRACT', 1.0, core_c)

    # soft ragged edge: distance from the edge of the footprint, broken up by noise
    edge_noise = noise(vec(calc('MULTIPLY', across, 14.0), calc('MULTIPLY', along, 2.5), 0.0, -900, -100),
                       1.0, 4.0, -700, -100)
    ragged = calc('ADD', centre, calc('MULTIPLY', calc('SUBTRACT', edge_noise, 0.5), 0.5))
    edge_map = node('ShaderNodeMapRange', -400, -100)
    edge_map.clamp = True
    links.new(ragged, edge_map.inputs['Value'])
    edge_map.inputs['From Min'].default_value = 0.08
    edge_map.inputs['From Max'].default_value = 0.45
    edge_alpha = edge_map.outputs['Result']

    # darker rim: a tyre presses hardest at its edges, so the middle of a heavy mark is a touch lighter
    rim = node('ShaderNodeMapRange', -400, -300)
    rim.clamp = True
    links.new(core_c, rim.inputs['Value'])
    rim.inputs['From Min'].default_value = 0.55
    rim.inputs['From Max'].default_value = 0.92
    rim_gain = calc('MULTIPLY_ADD', rim.outputs['Result'], 0.20, 0.82)

    # faint tread grooves running along the mark (4 per tyre width)
    tread = calc('MULTIPLY_ADD', calc('SINE', calc('MULTIPLY', calc('MULTIPLY_ADD', across, 1.0, -0.5), 25.13 * halo)),
                 0.09, 0.91)

    # uneven density along the length (more contrast with a higher Variation)
    amp = 0.25 + 0.70 * variation
    streak_noise = noise(vec(calc('MULTIPLY', across, 3.0), calc('MULTIPLY', along, 0.8), 5.0, -900, -400),
                         1.0, 3.0, -700, -400)
    streaks = calc('MULTIPLY_ADD', streak_noise, amp, 1.0 - 0.5 * amp)

    # grainy deposit: thin (low intensity) marks break up into scattered patches, heavy marks go solid
    grain = noise(vec(calc('MULTIPLY', across, 30.0), calc('MULTIPLY', along, 22.0), 11.0, -900, -650),
                  1.0, 5.0, -700, -650)
    inten = attr.outputs['Fac']
    gate_map = node('ShaderNodeMapRange', -400, 300)           # very faint rubber vanishes completely
    gate_map.clamp = True
    links.new(inten, gate_map.inputs['Value'])
    gate_map.inputs['From Min'].default_value = 0.03
    gate_map.inputs['From Max'].default_value = 0.18
    halo_map = node('ShaderNodeMapRange', -400, 450)           # dust only around strong marks
    halo_map.clamp = True
    links.new(inten, halo_map.inputs['Value'])
    halo_map.inputs['From Min'].default_value = 0.30
    halo_map.inputs['From Max'].default_value = 0.80
    solid = calc('MULTIPLY', inten, 1.4)
    gap = calc('MULTIPLY', calc('MULTIPLY', calc('SUBTRACT', 1.0, grain), calc('SUBTRACT', 1.0, inten)), 0.9)
    body = calc('SUBTRACT', solid, gap, clamp=True)

    core = calc('MULTIPLY', body, edge_alpha)
    core = calc('MULTIPLY', core, tread)
    core = calc('MULTIPLY', core, rim_gain)
    core = calc('MULTIPLY', core, streaks)
    core = calc('MULTIPLY', core, gate_map.outputs['Result'], clamp=True)

    # dust halo: soft, faint, only where there is rubber, broken up by the streaks
    shape = calc('POWER', calc('SUBTRACT', 1.0, abs_c, clamp=True), 1.6)
    halo_alpha = calc('MULTIPLY', calc('MULTIPLY', shape, inten), calc('MULTIPLY', streaks, 0.30))
    halo_alpha = calc('MULTIPLY', halo_alpha, halo_map.outputs['Result'])
    alpha = calc('MAXIMUM', core, halo_alpha)

    strength = node('ShaderNodeValue', 450, 300, name="MQFX_Strength", label="Strength")
    strength.outputs[0].default_value = 1.0
    alpha = calc('MULTIPLY', alpha, strength.outputs[0], clamp=True)

    colour = node('ShaderNodeRGB', 450, 100, name="MQFX_Color", label="Colour")
    colour.outputs[0].default_value = (0.012, 0.011, 0.010, 1.0)

    bump = node('ShaderNodeBump', 450, -150)
    bump.inputs['Strength'].default_value = 0.35
    bump.inputs['Distance'].default_value = 0.02
    links.new(core, bump.inputs['Height'])

    rubber = node('ShaderNodeBsdfPrincipled', 700, 100)
    links.new(colour.outputs[0], rubber.inputs['Base Color'])
    links.new(bump.outputs['Normal'], rubber.inputs['Normal'])
    links.new(calc('MULTIPLY_ADD', inten, -0.40, 0.85), rubber.inputs['Roughness'])      # thin = matte, heavy = glossy
    for sock in ('Specular', 'Specular IOR Level'):
        if sock in rubber.inputs:
            rubber.inputs[sock].default_value = 0.35
    clear = node('ShaderNodeBsdfTransparent', 700, -150)
    mix = node('ShaderNodeMixShader', 950, 0)
    out = node('ShaderNodeOutputMaterial', 1150, 0)
    links.new(alpha, mix.inputs['Fac'])
    links.new(clear.outputs['BSDF'], mix.inputs[1])
    links.new(rubber.outputs['BSDF'], mix.inputs[2])
    links.new(mix.outputs['Shader'], out.inputs['Surface'])
    return mat


def _simple_material():
    mat = bpy.data.materials.get(BURN_PREFIX + "Mat") or bpy.data.materials.new(BURN_PREFIX + "Mat")
    mat.diffuse_color = (0.02, 0.02, 0.02, 1.0)
    return mat


# ----------------------------------------------------------- burnout marks ---
def _jit(i, salt):
    """Repeatable pseudo-random number 0..1, so a rebuild always looks the same."""
    return (((i + 1) * 2654435761 + salt * 40503) & 0xFFFF) / 65535.0


def _vnoise(x, salt):
    """Smooth 1-D noise 0..1 (repeatable), used for the pressure of the rubber along the mark."""
    i = floor(x)
    t = x - i
    t = t * t * (3.0 - 2.0 * t)
    a, b = _jit(int(i), int(salt) + 5), _jit(int(i) + 1, int(salt) + 5)
    return a + (b - a) * t


def _pressure(v, salt):
    """How firmly the tyre presses at distance v along the mark: 1 = full, lower = soft stretch.
    Three wavelengths (about 4.5 m, 1.7 m, 0.6 m) so it is irregular, not a pattern."""
    n = 0.45 * _vnoise(v / 4.5, salt) + 0.35 * _vnoise(v / 1.7 + 10.0, salt) + 0.20 * _vnoise(v / 0.6 + 20.0, salt)
    return max(0.12, min(1.0, 1.0 - VARIATION * (1.0 - 1.45 * n)))


def _burnout_geometry(points, drawn, slip, util, load, half_w, v_offset, salt, full=None):
    """One quad per frame interval (quad i = the mark laid down between frame i and i+1). Frames with
    nothing to draw become zero-area quads, which keeps the numbering so the reveal stays in step with
    the car. Returns verts, faces, uvs (per face corner) and intensity (per vertex)."""
    n = len(points)
    dist, moved, dirs = [], [], []
    for i in range(n - 1):
        dx, dy = points[i + 1].x - points[i].x, points[i + 1].y - points[i].y
        ln = (dx * dx + dy * dy) ** 0.5
        dist.append(ln)
        moved.append(ln >= MIN_STEP)
        dirs.append((dx / ln, dy / ln) if ln > 1e-9 else None)
    v_acc = [0.0]
    for ln in dist:
        v_acc.append(v_acc[-1] + ln)

    last = next((d for d in dirs if d is not None), (1.0, 0.0))
    sides = []                                   # unit sideways vector per sample (points to the left)
    for i in range(n):
        cand = [d for d in (dirs[i - 1] if i > 0 else None, dirs[i] if i < n - 1 else None) if d is not None]
        if cand:
            tx, ty = sum(c[0] for c in cand), sum(c[1] for c in cand)
            ln = (tx * tx + ty * ty) ** 0.5
            if ln > 1e-9:
                last = (tx / ln, ty / ln)
        sides.append((-last[1], last[0]))

    # demand / grip per sample (`util` is per frame interval), lightly smoothed. `slip` (wheelspin) is only
    # used to spot a wheel spinning on the spot.
    raw = [0.0] * n
    for i in range(n):
        vals = [util[j] for j in (i - 1, i) if 0 <= j < n - 1]
        raw[i] = sum(vals) / len(vals) if vals else 0.0
    ss = [sum(raw[max(0, i - 2):i + 3]) / len(raw[max(0, i - 2):i + 3]) for i in range(n)]
    full = max(full or FRICTION_FULL, FRICTION_ON + 0.2)
    sev = [max(0.0, min(1.0, (v - FRICTION_ON) / (full - FRICTION_ON))) for v in ss]   # 0..1 how hard

    # rubber builds up while the tyre slips and fades after it grips again. Rolling = no rubber.
    rubber, inten, rub = 0.0, [0.0] * n, [0.0] * n
    for i in range(n):
        if i > 0:
            k = i - 1
            if not (drawn[k] and drawn[k + 1]):
                rubber = 0.0                                  # in the air: the mark ends
            else:
                # a tyre that carries little weight leaves a lighter mark
                target = _smooth(FRICTION_ON, full, ss[i]) * (0.55 + 0.45 * min(1.0, load[i] / 0.25))
                step = dist[k] + 0.03                         # a standing wheel still heats up, slowly
                length = RISE_LENGTH if target > rubber else FALL_LENGTH
                rubber += (target - rubber) * (1.0 - exp(-step / length))
        # soft and hard stretches: the pressure wanders along the mark, so it is never one even line
        rub[i] = max(rubber, ROLL_TRACE) * _pressure(v_offset + v_acc[i], salt)
        inten[i] = min(1.0, BURN_MAX * rub[i])

    # Draw only what is really there. Faint tails, specks and short flickers are removed, so in Solid mode you
    # see the same marks as in Material Preview and there is no ghost line between the real marks.
    inten = [max(0.0, (v - MIN_VISIBLE) / (1.0 - MIN_VISIBLE)) for v in inten]
    i = 0
    while i < n:
        if inten[i] <= 0.0:
            i += 1
            continue
        j = i
        while j + 1 < n and inten[j + 1] > 0.0:
            j += 1
        if max(inten[i:j + 1]) < MIN_MARK_PEAK or sum(dist[i:j]) < MIN_MARK_LENGTH:
            for k in range(i, j + 1):
                inten[k] = 0.0
        i = j + 1

    def wander(i):
        """Slow sideways wobble in metres along the mark, so it is never ruler-straight."""
        v = v_offset + v_acc[i]
        return WANDER * (0.6 * sin(v * 1.9 + salt) + 0.4 * sin(v * 5.3 + salt * 2.1))

    verts, faces, uvs, cols = [], [], [], []
    for i in range(n - 1):
        k = len(verts)
        a, b = points[i], points[i + 1]
        sa, sb = sides[i], sides[i + 1]
        halo = 1.0 + 0.4 * SMUDGE                     # the strip is wider than the tyre; the shader draws the dust there
        ha = half_w * halo * (BURN_WIDTH * (0.55 + 0.45 * min(1.0, rub[i] * 1.25)) + 0.12 * sev[i])
        hb = half_w * halo * (BURN_WIDTH * (0.55 + 0.45 * min(1.0, rub[i + 1] * 1.25)) + 0.12 * sev[i + 1])
        oa, ob = wander(i), wander(i + 1)
        va, vb = v_offset + v_acc[i], v_offset + v_acc[i + 1]
        if drawn[i] and drawn[i + 1] and moved[i] and (inten[i] > 0.0 or inten[i + 1] > 0.0):
            verts += [(a.x + sa[0] * (oa + ha), a.y + sa[1] * (oa + ha), a.z),
                      (a.x + sa[0] * (oa - ha), a.y + sa[1] * (oa - ha), a.z),
                      (b.x + sb[0] * (ob - hb), b.y + sb[1] * (ob - hb), b.z),
                      (b.x + sb[0] * (ob + hb), b.y + sb[1] * (ob + hb), b.z)]
            uvs += [(0.0, va), (1.0, va), (1.0, vb), (0.0, vb)]
            cols += [inten[i], inten[i], inten[i + 1], inten[i + 1]]
        elif drawn[i] and drawn[i + 1] and not moved[i] and slip[i] >= BLOB_MIN_SLIP:
            # wheel spinning on the spot: one more translucent layer of rubber, so it darkens over time
            tx, ty = sa[1], -sa[0]
            cx = a.x + sa[0] * ha * (_jit(i, salt) - 0.5) * 0.12 + tx * (_jit(i, salt + 7) - 0.5) * 0.1
            cy = a.y + sa[1] * ha * (_jit(i, salt) - 0.5) * 0.12 + ty * (_jit(i, salt + 7) - 0.5) * 0.1
            h = BLOB_LENGTH * (0.85 + 0.3 * _jit(i, salt + 3)) * 0.5
            verts += [(cx - tx * h + sa[0] * ha, cy - ty * h + sa[1] * ha, a.z),
                      (cx - tx * h - sa[0] * ha, cy - ty * h - sa[1] * ha, a.z),
                      (cx + tx * h - sa[0] * ha, cy + ty * h - sa[1] * ha, a.z),
                      (cx + tx * h + sa[0] * ha, cy + ty * h + sa[1] * ha, a.z)]
            uvs += [(0.0, va), (1.0, va), (1.0, va + BLOB_LENGTH), (0.0, va + BLOB_LENGTH)]
            c = min(1.0, BLOB_ALPHA * (0.4 + max(sev[i], slip[i])))
            cols += [c, c, c, c]
        else:
            verts += [(a.x, a.y, a.z)] * 4
            uvs += [(0.0, va)] * 4
            cols += [0.0] * 4
        faces.append((k, k + 1, k + 2, k + 3))
    return verts, faces, uvs, cols


def remove_burnouts(scene=None):
    for ob in [o for o in bpy.data.objects if o.name.startswith((BURN_PREFIX,) + OLD_PREFIXES)]:
        me = ob.data
        bpy.data.objects.remove(ob, do_unlink=True)
        if me is not None and me.users == 0:
            bpy.data.meshes.remove(me)


def build_burnouts(scene):
    """Returns a one-line message."""
    sync_settings(scene)
    props = getattr(scene, "mqfx", None)
    if props is not None and not props.enabled:
        return "tire effects are switched off in the panel"
    span = scene.get(SPAN_KEY)
    wheels, how = baked_wheels(scene)
    if not span or not wheels:
        return "burnout lines: no baked drive / wheels found"
    f0, f1 = int(span[0]), int(span[1])
    if f1 - f0 < 2:
        return "burnout lines: baked drive is too short"
    coll = bpy.data.collections.get(BURN_COLLECTION)
    if coll is None:
        coll = bpy.data.collections.new(BURN_COLLECTION)
    if coll.name not in scene.collection.children:
        scene.collection.children.link(coll)
    note = ""
    try:
        mat = _burnout_material(1.0 + 0.4 * SMUDGE, VARIATION)
    except Exception:
        traceback.print_exc()
        mat = _simple_material()
        note = " (simple material: the realistic shader failed, see console)"
    made, skipped = 0, 0
    fps = scene.render.fps / scene.render.fps_base
    unit = scene.unit_settings.scale_length or 1.0
    all_wheels, _how = baked_wheels(scene, ALL_ROLES)
    locs_all, spin, axes, radius_of = {}, {}, {}, {}
    ref, _axes = rear_axle_info(scene, f0, f1)
    for role, wheel in all_wheels:
        dims = sorted(wheel.dimensions)
        radius_of[role] = max(dims[-1] / 2.0, 0.05)
        locs_all[role] = [loc_at(wheel, fr) for fr in range(f0, f1 + 1)]
        spin[role], axes[role] = wheel_slip(wheel, locs_all[role], f0, f1, radius_of[role], ref)
    dyn = None
    if all(r in locs_all for r in ALL_ROLES):
        dyn = car_dynamics(locs_all, fps, radius_of["wheel_rl"], unit)
    mode = "slide" if scene.get(SLIDE_SIG_KEY) else "normal"

    # ---- pass 1: how hard is each chosen tyre pushed, frame by frame
    data = []
    for idx, (role, wheel) in enumerate(wheels):
        dims = sorted(wheel.dimensions)
        radius = radius_of.get(role, max(dims[-1] / 2.0, 0.05))
        locs = locs_all.get(role) or [loc_at(wheel, fr) for fr in range(f0, f1 + 1)]
        rest_z = min(p.z for p in locs)
        drawn = [(p.z - rest_z) <= AIR_TOLERANCE for p in locs]
        slip = spin.get(role) or [0.0] * (len(locs) - 1)
        n_int = len(locs) - 1
        u_spin = [_spin_util(v) for v in slip]
        u_slide = slip_angle_util(locs, axes.get(role)) if (DRIFT_MARKS and axes.get(role)) else [0.0] * n_int
        if dyn is not None and role in dyn["util"]:
            ud, load = dyn["util"][role], dyn["load"][role]
            u_dem = [(ud[k] + ud[k + 1]) * 0.5 for k in range(n_int)]
        else:
            u_dem, load = [0.0] * n_int, [0.25] * len(locs)
        util = [max(u_dem[k], u_slide[k], u_spin[k]) for k in range(n_int)]
        live = [drawn[k] and drawn[k + 1] and (hypot(locs[k + 1].x - locs[k].x, locs[k + 1].y - locs[k].y) >= MIN_STEP
                                              or slip[k] >= BLOB_MIN_SLIP) for k in range(n_int)]
        data.append(dict(idx=idx, role=role, radius=radius, half_w=max(dims[0], 0.05) * 0.5, locs=locs,
                         drawn=drawn, slip=slip, util=util, load=load, live=live))

    # ---- calibration: judge the drive against itself, so there are always hard and soft parts and quiet parts.
    # Only the hardest MARK_SHARE of the moving drive can leave marks. A drive that never gets hard stays clean.
    scale, full, calib = 1.0, FRICTION_FULL, "grip " + format(GRIP, ".2f") + " g (fixed)"
    pool = sorted(d["util"][k] for d in data for k in range(len(d["util"])) if d["live"][k])
    if AUTO_GRIP and pool:
        cut = pool[int(min(1.0, max(0.0, 1.0 - MARK_SHARE)) * (len(pool) - 1))]
        if cut >= MARK_MIN_DEMAND:
            scale = FRICTION_ON / cut
            calib = f"relative: hardest {MARK_SHARE * 100:.0f}% of the drive"
        else:
            calib = f"relative: the hardest {MARK_SHARE * 100:.0f}% of the drive is still gentle (demand {cut:.2f} of grip)"
        peak = pool[-1] * scale
        full = max(FRICTION_FULL, 0.65 * peak)
    marked = {}
    for d in data:
        d["util"] = [u * scale for u in d["util"]]
        marked[d["role"]] = sum(1 for k, u in enumerate(d["util"]) if u >= FRICTION_ON and d["live"][k]) / max(1, len(d["util"]))

    # ---- pass 2: build the marks
    for d in data:
        locs = d["locs"]
        ground = [Vector((p.x, p.y, p.z - d["radius"] + LIFT)) for p in locs]
        verts, faces, uvs, cols = _burnout_geometry(ground, d["drawn"], d["slip"], d["util"], d["load"], d["half_w"],
                                                    d["idx"] * 13.7, d["idx"] * 11 + 1, full)
        if not faces or max(cols) < 0.02:
            skipped += 1                       # this wheel never went past its grip: nothing to draw
            continue
        role = d["role"]
        me = bpy.data.meshes.new(BURN_PREFIX + role)
        me.from_pydata(verts, [], faces)
        me.update()
        uv_layer = me.uv_layers.new(name=UV_NAME)
        uv_layer.data.foreach_set("uv", [c for uv in uvs for c in uv])
        ca = me.color_attributes.new(ATTR_NAME, 'FLOAT_COLOR', 'POINT')
        ca.data.foreach_set("color", [c for v in cols for c in (v, v, v, 1.0)])
        me.materials.append(mat)
        ob = bpy.data.objects.new(BURN_PREFIX + role, me)
        coll.objects.link(ob)
        ob.hide_select = True
        # The Build modifier shows one quad per frame, in order: the mark grows behind the tire.
        mod = ob.modifiers.new("Reveal", 'BUILD')
        mod.frame_start = f0
        mod.frame_duration = len(faces)
        mod.use_random_order = False
        mod.use_reverse = False
        made += 1
    apply_look(scene)
    if dyn is not None:
        car = (f"peak cornering {dyn['lat']:.2f} g, braking {dyn['brk']:.2f} g, accel {dyn['acc']:.2f} g")
    else:
        car = "car physics unavailable (need all four wheels), using wheelspin/slide only"
    hot = ", ".join(f"{r[6:].upper()} {v * 100:.0f}%" for r, v in marked.items())
    out = (f"burnout lines [{mode} drive]: {made} tyre(s) marked, {skipped} stayed within grip (frames {f0}-{f1}, "
           f"wheels via {how}). {car}. Calibration {calib}. Marked share of the drive: {hot}.")
    if made == 0:
        out += " Nothing left a mark: raise Amount in the panel, or lower the grip."
    return out + note


# ------------------------------------------------------------ settings + look --
def sync_settings(scene):
    """Copy the panel values into the settings above (they are plain module settings the maths reads)."""
    global WHEEL_ROLES, DRIFT_MARKS, CORNER_GAIN, DRIVE_GAIN, BRAKE_GAIN, GRIP, AUTO_GRIP, MARK_SHARE
    global VARIATION, SMUDGE, BURN_WIDTH, RISE_LENGTH, FALL_LENGTH
    p = getattr(scene, "mqfx", None)
    if p is None:
        return
    WHEEL_ROLES = {"ALL": ALL_ROLES, "REAR": ("wheel_rl", "wheel_rr"), "FRONT": ("wheel_fl", "wheel_fr")}[p.wheels]
    DRIFT_MARKS = bool(p.use_slide)
    CORNER_GAIN, BRAKE_GAIN, DRIVE_GAIN = p.corner_gain, p.brake_gain, p.power_gain
    AUTO_GRIP, GRIP, MARK_SHARE = bool(p.auto_grip), p.grip, p.amount
    VARIATION, SMUDGE = p.variation, p.smudge
    BURN_WIDTH = 0.92 * p.width
    RISE_LENGTH, FALL_LENGTH = RISE_BASE * p.soft_ends, FALL_BASE * p.soft_ends


def apply_look(scene):
    """Strength and colour live in two shader nodes, so they change without redrawing anything."""
    p = getattr(scene, "mqfx", None)
    mat = bpy.data.materials.get(BURN_PREFIX + "Mat")
    if p is None or mat is None:
        return
    mat.diffuse_color = (p.color[0], p.color[1], p.color[2], 1.0)
    nt = getattr(mat, "node_tree", None)
    if nt is None:
        return
    n = nt.nodes.get("MQFX_Color")
    if n is not None:
        n.outputs[0].default_value = (p.color[0], p.color[1], p.color[2], 1.0)
    n = nt.nodes.get("MQFX_Strength")
    if n is not None:
        n.outputs[0].default_value = p.strength


def _look_changed(self, context):
    apply_look(context.scene)


def _redraw_later(self, context):
    STATE["mqfx_dirty"] = time.time()        # the watcher redraws shortly after the last change


# ------------------------------------------------------------------ the panel --
class MQFX_Props(bpy.types.PropertyGroup):
    enabled: bpy.props.BoolProperty(name="Tire marks", default=True, update=_redraw_later,
        description="Draw tyre marks on the ground for the baked drive")
    wheels: bpy.props.EnumProperty(name="Tires", default='ALL', update=_redraw_later,
        items=[('ALL', "All 4", "Every tyre can leave marks"),
               ('REAR', "Rear", "Only the two back tyres"),
               ('FRONT', "Front", "Only the two front tyres")],
        description="Which tyres can leave marks (the physics still decides when)")
    strength: bpy.props.FloatProperty(name="Strength", default=1.0, min=0.0, max=1.0, precision=2, subtype='FACTOR',
        update=_look_changed, description="How dark / opaque the marks are. Changes instantly")
    color: bpy.props.FloatVectorProperty(name="Color", subtype='COLOR', size=3, min=0.0, max=1.0,
        default=(0.012, 0.011, 0.010), update=_look_changed,
        description="Colour of the rubber. Changes instantly. Try a dark brown for old tyres or grey for dust")
    width: bpy.props.FloatProperty(name="Width", default=1.0, min=0.4, max=1.6, precision=2, update=_redraw_later,
        description="Mark width compared with the tyre (1 = the tyre's width)")
    smudge: bpy.props.FloatProperty(name="Soft Edge Dust", default=0.5, min=0.0, max=1.0, precision=2,
        subtype='FACTOR', update=_redraw_later, description="Faint rubber dust that spreads past the edges of the mark")
    amount: bpy.props.FloatProperty(name="Amount", default=0.30, min=0.03, max=1.0, precision=2, subtype='FACTOR',
        update=_redraw_later,
        description="How much of the drive may leave marks. The hardest moments (sharp corners, drifts, hard "
                    "braking and launches) are marked first. A gentle drive stays clean either way")
    variation: bpy.props.FloatProperty(name="Soft / Hard Variation", default=0.6, min=0.0, max=1.0, precision=2,
        subtype='FACTOR', update=_redraw_later,
        description="0 = even mark. Higher = the mark wanders between soft faint stretches and hard black ones")
    soft_ends: bpy.props.FloatProperty(name="Soft Start / End", default=1.0, min=0.3, max=3.0, precision=2,
        update=_redraw_later, description="How long a mark takes to build up and to fade away (1 = normal)")
    corner_gain: bpy.props.FloatProperty(name="Corners", default=1.0, min=0.0, max=2.0, precision=2,
        subtype='FACTOR', update=_redraw_later, description="How much cornering force counts (0 = corners leave nothing)")
    brake_gain: bpy.props.FloatProperty(name="Braking", default=1.0, min=0.0, max=2.0, precision=2,
        subtype='FACTOR', update=_redraw_later, description="How much braking counts (0 = no braking marks)")
    power_gain: bpy.props.FloatProperty(name="Launch / Power", default=1.0, min=0.0, max=2.0, precision=2,
        subtype='FACTOR', update=_redraw_later, description="How much acceleration counts (0 = no launch marks)")
    use_slide: bpy.props.BoolProperty(name="Sideways slide / drift", default=True, update=_redraw_later,
        description="Marks where a tyre travels sideways (drifting, tail-out, Apply Slide)")
    auto_grip: bpy.props.BoolProperty(name="Judge against this drive", default=True, update=_redraw_later,
        description="On: the hardest part of THIS drive is marked, whatever the car's speed or scale. "
                    "Off: use the fixed Grip value below")
    grip: bpy.props.FloatProperty(name="Grip", default=0.5, min=0.1, max=3.0, precision=2, update=_redraw_later,
        description="Fixed grip in g (used when 'Judge against this drive' is off). Lower = marks more easily")


class MQFX_OT_rebuild(bpy.types.Operator):
    bl_idname = "mqfx.rebuild"
    bl_label = "Redraw Now"
    bl_description = "Redraw the tyre marks from the current bake and settings"

    def execute(self, context):
        STATE["mqfx_dirty"] = None
        STATE["mqfx_sig"] = bake_signature(context.scene)
        for m in rebuild(context.scene):
            print("[MQScar effects]", m)
        return {'FINISHED'}


class MQFX_OT_clear(bpy.types.Operator):
    bl_idname = "mqfx.clear"
    bl_label = "Clear Marks"
    bl_description = "Remove the tyre marks (switch Tire marks back on, or bake again, to draw them)"

    def execute(self, context):
        for _n, _b, remove in EFFECTS:
            remove(context.scene)
        context.scene.mqfx.enabled = False
        return {'FINISHED'}


class MQFX_PT_panel(bpy.types.Panel):
    bl_label = "Tire Effects"
    bl_idname = "MQFX_PT_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "MQScar"

    def draw(self, context):
        lay = self.layout
        p = context.scene.mqfx
        lay.prop(p, "enabled", toggle=True, icon='CHECKBOX_HLT' if p.enabled else 'CHECKBOX_DEHLT')
        body = lay.column()
        body.active = p.enabled

        box = body.box()
        box.label(text="Which tires", icon='MOD_BUILD')
        box.row().prop(p, "wheels", expand=True)

        box = body.box()
        box.label(text="Look", icon='COLOR')
        box.prop(p, "strength", slider=True)
        box.prop(p, "color", text="Color")
        box.prop(p, "width")
        box.prop(p, "smudge", slider=True)

        box = body.box()
        box.label(text="Realism", icon='FORCE_TURBULENCE')
        box.prop(p, "amount", slider=True)
        box.prop(p, "variation", slider=True)
        box.prop(p, "soft_ends")

        box = body.box()
        box.label(text="What leaves marks", icon='AUTO')
        box.prop(p, "corner_gain", slider=True)
        box.prop(p, "brake_gain", slider=True)
        box.prop(p, "power_gain", slider=True)
        box.prop(p, "use_slide")
        box.prop(p, "auto_grip")
        sub = box.column()
        sub.active = not p.auto_grip
        sub.prop(p, "grip")

        row = lay.row(align=True)
        row.scale_y = 1.3
        row.operator("mqfx.rebuild", icon='FILE_REFRESH')
        row.operator("mqfx.clear", icon='TRASH')

        msgs = STATE.get("mqfx_msg") or []
        if msgs:
            info = lay.box()
            for m in msgs[:3]:
                for ln in textwrap.wrap(str(m), 46)[:9]:
                    info.label(text=ln)


def _panel_classes():
    return (MQFX_Props, MQFX_OT_rebuild, MQFX_OT_clear, MQFX_PT_panel)


def unregister_panel():
    if hasattr(bpy.types.Scene, "mqfx"):
        try:
            del bpy.types.Scene.mqfx
        except Exception:
            pass
    for c in reversed(STATE.get("mqfx_classes") or []):
        try:
            bpy.utils.unregister_class(c)
        except Exception:
            pass
    STATE["mqfx_classes"] = []


def register_panel():
    unregister_panel()
    for c in _panel_classes():
        bpy.utils.register_class(c)
    bpy.types.Scene.mqfx = bpy.props.PointerProperty(type=MQFX_Props)
    STATE["mqfx_classes"] = list(_panel_classes())


# ------------------------------------------------------------ the framework --
# (name, build(scene) -> message, remove(scene))
EFFECTS = [
    ("burnout_lines", build_burnouts, remove_burnouts),
]


def _rebuild_inner(scene):
    """Remove every effect, then redraw them all if a baked drive exists. Returns the messages."""
    for _name, _build, remove in EFFECTS:
        try:
            remove(scene)
        except Exception:
            traceback.print_exc()
    if bake_signature(scene) is None:
        return ["no baked drive in the scene (effects removed)"]
    msgs = []
    for name, build, _remove in EFFECTS:
        try:
            msgs.append(build(scene))
        except Exception:
            traceback.print_exc()
            msgs.append(f"{name}: FAILED - " + traceback.format_exc().strip().splitlines()[-1])
    return msgs


def rebuild(scene):
    sync_settings(scene)
    msgs = _rebuild_inner(scene)
    STATE["mqfx_msg"] = list(msgs)
    return msgs


# --------------------------------------------------------------- the watcher --
def _watch():
    try:
        scene = bpy.context.scene
        screen = bpy.context.screen
        if scene is None or (screen is not None and screen.is_animation_playing):
            return CHECK_SECONDS
        sig = bake_signature(scene)
        dirty = STATE.get("mqfx_dirty")
        due = dirty is not None and time.time() - dirty > 0.3        # a panel setting changed
        if due or sig != STATE.get("mqfx_sig"):
            STATE["mqfx_dirty"] = None
            STATE["mqfx_sig"] = sig
            for m in rebuild(scene):
                print("[MQScar effects]", m)
    except Exception:
        traceback.print_exc()
    return CHECK_SECONDS


def _stop_watcher():
    old = STATE.get("mqfx_watch")
    if old is not None:
        try:
            if bpy.app.timers.is_registered(old):
                bpy.app.timers.unregister(old)
        except Exception:
            pass
        STATE["mqfx_watch"] = None


def _popup(lines):
    try:
        def draw(self, _context):
            for ln in lines:
                self.layout.label(text=ln)
        bpy.context.window_manager.popup_menu(draw, title="MQScar effects", icon='INFO')
    except Exception:
        pass


def start():
    scene = bpy.context.scene
    _stop_watcher()
    if AUTO_REBUILD:
        register_panel()
    else:
        unregister_panel()
    msgs = rebuild(scene)
    STATE["mqfx_sig"] = bake_signature(scene)
    if AUTO_REBUILD:
        STATE["mqfx_watch"] = _watch
        bpy.app.timers.register(_watch, first_interval=CHECK_SECONDS, persistent=True)
        msgs.append("Watching for new bakes: effects update by themselves. Panel: N sidebar > MQScar > Tire Effects.")
    else:
        for _name, _build, remove in EFFECTS:
            remove(scene)
        STATE["mqfx_sig"] = None
        msgs = ["Auto update is OFF and the effects were removed."]
    for m in msgs:
        print("[MQScar effects]", m)
    _popup(msgs)


def register():
    # Called by the main MQScar add-on after its core properties are registered.
    start()


def unregister():
    # Stop background work and remove the panel/property classes without deleting
    # already-generated tyre-mark objects from the user's scene.
    _stop_watcher()
    unregister_panel()
