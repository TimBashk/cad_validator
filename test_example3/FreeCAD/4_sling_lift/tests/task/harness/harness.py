#!/usr/bin/env python3
"""Grading harness for 4_sling_lift.

    python3 tests/task/harness/harness.py CANDIDATE.FCStd
    python3 tests/task/harness/harness.py --freeze          # re-measure the reference
    python3 tests/task/harness/harness.py --score-from DIR  # score a stage output dir

The candidate is measured out of process by `measure_stage.py` under
freecadcmd (see that file for what is measured and how). This file never
imports FreeCAD: it turns two measurements -- the candidate's and the
reference's -- into scores, and every function that does so is a plain
function of those dicts, testable on a machine with no FreeCAD.

THE REFERENCE IS A MEASUREMENT, NOT A FILE TO DIFF

`tests/task/reference/baseline.json` holds the reference measured by the
same stage, plus a surface sample of its delivered geometry. It is keyed
to a hash of measure_stage.py: if the stage changes, the baseline is
stale and the reference is re-measured live (slower, never wrong).
`--freeze` rewrites it.

WHAT IS GRADED, AND WHAT IS LET GO

Nothing here looks at object names, labels, the document tree, the order
features were made in, or which of several valid ways a part was built.
Shapes are compared by invariants (volume, principal moments, extents in
the solid's own frame) and by a rigid registration of the whole that is
free to rotate and translate. Joints are judged by what they connect and
whether they hold, not by how many there are.

    geometry matches ground truth   half parts inventory (volume-weighted
                                    shape match, solid by solid), half the
                                    assembled pose after rigid registration.
                                    A model that only fits our reference
                                    once reflected keeps half the pose
                                    credit -- see MIRROR below.
    pantograph joints mated         the mechanism joints (not Fixed) that
                                    are mated and actually join two
                                    different rigid groups, against the
                                    reference's count, times the share of
                                    all mechanism joints that do so.
    mechanism articulates           a GUI-style drag moves the chain with
                                    every joint still mated, and the
                                    delivered pose is not parked against a
                                    joint limit (limits only matter for
                                    what they permit from where it stands).
    carries dimensional constraints dimensional parameters on delivered
                                    bodies, and the share of delivered
                                    volume that is parametric rather than
                                    dead imported solids.
    dimensions drive the geometry   of the parameters nudged by the stage,
                                    the share whose nudge changed geometry.
    signature dimensions present    pantograph pivot spacings (the link
                                    lengths) and the extents of every major
                                    part, each against the reference.
                                    Configuration-free: a lift posed
                                    differently keeps these.

MIRROR

The drawing is a set of orthographic views plus an isometric, and those
fix handedness: which side the indexing plungers sit on, which bar of
each scissor pair is in front. A mirrored assembly is therefore not the
drawing -- but its parts, joints, mechanism and parameters can all be
exactly right. So the reflection is charged once, in the one place it is
wrong: the pose half of "geometry matches ground truth" pays half. A
grader that failed the whole component (or worse, several) would be
charging an honest candidate who mis-read one view as if the lift were
broken; one that ignored it would accept a lift the drawing does not
show.

SCORE CONTINUOUSLY. Every component is a number in [0, 1]; only the gate
is binary.
"""
import hashlib
import json
import math
import os
import shutil
import sys
import tempfile
from itertools import product
from pathlib import Path

HERE = Path(__file__).resolve().parent


def _repo_root(start):
    d = start
    for _ in range(8):
        if (d / "common" / "__init__.py").is_file():
            return d
        if d.parent == d:
            break
        d = d.parent
    return start


REPO = _repo_root(HERE)
sys.path[:0] = [str(REPO), "/opt"]

from common.harness_base import (Harness, clamp01, score_error,  # noqa: E402
                                 score_ratio)

#: Weight per component; 0 makes it a gate when it is also in MUST_PASS.
#: The sum must equal task.toml's max_score (6).
ALL_CRITERIA = {
    "executes and builds geometry": 0,
    "geometry matches ground truth": 1,
    "pantograph joints mated": 1,
    "mechanism articulates": 1,
    "carries dimensional constraints": 1,
    "dimensions drive the geometry": 1,
    "signature dimensions present": 1,
}

REFERENCE_DIR = HERE.parent / "reference"
REFERENCE_DOC = REFERENCE_DIR / "solution.FCStd"
BASELINE = REFERENCE_DIR / "baseline.json"
STAGE = HERE / "measure_stage.py"

#: Surface points sampled from delivered.stl for the pose comparison.
SAMPLE_POINTS = 12000

#: Joint kinds that permit motion. Everything else holds parts together.
MECHANISM = {"Revolute", "Slider", "Cylindrical", "Ball", "Distance",
             "Parallel", "Perpendicular", "Angle", "RackPinion", "Screw",
             "Gears", "Belt"}

#: A joint is mated when its frames are this close (mm, degrees).
MATED_GAP = 0.05
MATED_TILT = 0.5

THRESHOLDS = {
    # one solid's shape invariants vs its match: full marks within 0.5%,
    # nothing at 5% (a link 4% short scores ~0.1 on that link alone)
    "shape_rel": (0.005, 0.05),
    # mean registered surface distance, mm. Two independent samplings of
    # the same lift land ~2-3 mm apart; 25 mm is a lift in another shape.
    "pose_mm": (4.0, 25.0),
    # reflected fit this much better than the direct one = mirror image.
    # The lift is nearly symmetric, so the margin is small but real: the
    # reference against itself fits 0.6x worse reflected.
    "mirror_ratio": 0.85,
    # signature dimension relative error
    "signature_rel": (0.005, 0.05),
    # credit for a parametric census at 80% of the reference's
    "census_full": 0.8,
    # driving share: full marks at 90% of the reference's own share
    "drive_full": 0.9,
    # articulation: chain moved, as a share of the reference's
    "moved_full": 0.8,
    # travel left before a joint limit, below which the pose is parked (mm/deg)
    "parked": 1.0,
    # a solid is "major" for the signature check above this volume share
    "major_share": 0.01,
}


# ---------------------------------------------------------------------------
# small geometry helpers (pure python; numpy only where noted)

def _sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def _norm(a):
    return math.sqrt(_dot(a, a))


def rel_err(a, b):
    return abs(a - b) / max(abs(b), 1e-9)


def mated(joint):
    r = joint.get("residual")
    return bool(r) and r["gap"] <= MATED_GAP and r["tilt"] <= MATED_TILT


# ---------------------------------------------------------------------------
# features: everything the scoring needs, derived from one measurement

def rigid_groups(meas):
    """part name -> group id, merging parts held by mated Fixed joints."""
    parent = {p["name"]: p["name"] for p in meas["delivered"]["parts"]}

    def find(x):
        while parent.setdefault(x, x) != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for j in meas["joints"]:
        if j.get("type") == "Fixed" and not j.get("suppressed") \
                and mated(j) and all(j.get("parts") or [None]):
            a, b = j["parts"]
            parent[find(a)] = find(b)
    return {p: find(p) for p in list(parent)}


def mechanism_joints(meas, groups):
    """Each non-Fixed joint, classified."""
    out = []
    for j in meas["joints"]:
        if j.get("type") not in MECHANISM or j.get("suppressed"):
            continue
        parts = j.get("parts") or [None, None]
        g = [groups.get(p) if p else None for p in parts]
        out.append({
            "name": j["name"], "type": j["type"],
            "mated": mated(j) if j.get("residual") is not None else None,
            "distinct": None not in g and g[0] != g[1],
            "groups": g, "frames": j.get("frames") or [],
            "value": j.get("value"), "limits": j.get("limits") or {},
        })
    return out


def pivot_spacings(mech):
    """Link lengths: on every rigid group, the distances between the
    parallel pivot axes of the revolute joints it carries. Coaxial pivots
    (the same pin seen from two joints) collapse to one. Independent of
    the pose the mechanism is in, and of a reflection."""
    axes = {}
    for m in mech:
        if m["type"] not in ("Revolute", "Cylindrical") or not m["distinct"]:
            continue
        # each group's pivot where its own part puts it: whether the joint
        # is mated is the joints component's business, not this one's
        for g, f in zip(m["groups"], m["frames"]):
            if f:
                axes.setdefault(g, []).append(f)
    spacings = []
    for frames in axes.values():
        lines = []
        for f in frames:
            o, a = f["origin"], f["axis"]
            if any(_norm(_cross(a, b)) < 1e-3
                   and _norm(_cross(_sub(o, p), b)) < 0.5 for p, b in lines):
                continue
            lines.append((o, a))
        for i in range(len(lines)):
            for k in range(i + 1, len(lines)):
                (p1, a1), (p2, a2) = lines[i], lines[k]
                if _norm(_cross(a1, a2)) > 1e-3:
                    continue
                d = _norm(_cross(_sub(p2, p1), a1)) / max(_norm(a1), 1e-12)
                if d > 0.5:
                    spacings.append(round(d, 3))
    return sorted(spacings)


def features(meas):
    groups = rigid_groups(meas)
    mech = mechanism_joints(meas, groups)
    d = meas["delivered"]
    par = meas.get("parametric", {})
    delivered_bodies = set(par.get("delivered_bodies", []))
    dims = sum(1 for s in par.get("sketches", [])
               if s.get("body") in delivered_bodies
               for c in s["dimensional"] if c.get("driving", True))
    dims += sum(1 for sh in par.get("spreadsheets", [])
                for c in sh["cells"] if _is_number(c["content"]))
    dims += sum(len(v["properties"]) for v in par.get("varsets", []))
    by_source = par.get("delivered_volume_by_source", {})
    total = sum(by_source.values()) or 1.0
    dead = sum(v for k, v in by_source.items()
               if k in ("Part::Feature", "Mesh::Feature", "Part::Part2DObject"))
    probe = [r for r in meas.get("drive_probe", {}).get("results", [])
             if r.get("body_delivered") is not False
             and r.get("drives") is not None]
    arts = meas.get("articulation", [])
    clean = [r for a in arts for r in a.get("drags", [])
             if r.get("rc") == 0 and r.get("worst_gap", 1e9) <= MATED_GAP
             and r.get("worst_tilt", 1e9) <= MATED_TILT]
    n_parts = max([a.get("parts", 0) for a in arts] or [0])
    return {
        "solids": d["solids"],
        "volume": d["total"].get("volume", 0.0),
        "mech": mech,
        "spacings": pivot_spacings(mech),
        "solve": meas.get("solve", []),
        "best_moved": max([r.get("moved", 0) for r in clean] or [0]),
        "drags": sum(len(a.get("drags", [])) for a in arts),
        "clean_drags": len(clean),
        "n_parts": n_parts,
        "dims": dims,
        "parametric_share": 1.0 - dead / total,
        "probed": len(probe),
        "driving": sum(1 for r in probe if r.get("drives")),
    }


def _is_number(text):
    try:
        float(str(text).lstrip("=").split()[0])
        return True
    except (ValueError, IndexError):
        return False


# ---------------------------------------------------------------------------
# geometry

def solid_error(a, b):
    """Worst relative error between two solids' shape invariants."""
    errs = [rel_err(a["volume"], b["volume"])]
    errs += [rel_err(x, y) for x, y in zip(a["extents"], b["extents"])]
    errs += [rel_err(x, y) for x, y in zip(a.get("principal_moments", []),
                                           b.get("principal_moments", []))]
    return max(errs)


def inventory_match(cand, ref):
    """Volume-weighted F1 of a greedy one-to-one match of solids by shape
    invariants. Missing parts cost recall, extra parts cost precision, a
    wrong-sized part costs on both in proportion to how wrong it is."""
    perfect, zero = THRESHOLDS["shape_rel"]
    free = list(range(len(cand)))
    got_ref = got_cand = 0.0
    missing = []
    for r in sorted(ref, key=lambda s: -s["volume"]):
        best, best_i = None, None
        for i in free:
            e = solid_error(cand[i], r)
            if best is None or e < best:
                best, best_i = e, i
        sim = score_error(best, perfect, zero) if best is not None else 0.0
        if sim > 0:
            free.remove(best_i)
            got_ref += r["volume"] * sim
            got_cand += cand[best_i]["volume"] * sim
        else:
            missing.append(r)
    vr = sum(s["volume"] for s in ref) or 1.0
    vc = sum(s["volume"] for s in cand) or 1.0
    recall, precision = got_ref / vr, got_cand / vc
    f1 = 0.0 if recall + precision == 0 else \
        2 * recall * precision / (recall + precision)
    return f1, recall, precision, len(missing), len(free)


def sample_points(stl_path, n=SAMPLE_POINTS, seed=0):
    import numpy as np
    import trimesh
    mesh = trimesh.load(str(stl_path), force="mesh")
    pts, _ = trimesh.sample.sample_surface(mesh, n, seed=seed)
    return np.asarray(pts, dtype=float)


def _kabsch(P, Q):
    import numpy as np
    cp, cq = P.mean(0), Q.mean(0)
    H = (P - cp).T @ (Q - cq)
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T)) or 1.0])
    R = Vt.T @ D @ U.T
    return R, cq - R @ cp


def register(P, Q, reflect=False, iters=40):
    """Mean symmetric surface distance (mm) between point sets P and Q
    after the best rigid alignment of P onto Q; `reflect` mirrors P
    first. Principal-axis starts in every proper orientation, then ICP."""
    import numpy as np
    from scipy.spatial import cKDTree
    P = np.asarray(P, dtype=float).copy()
    Q = np.asarray(Q, dtype=float)
    if reflect:
        P[:, 2] = -P[:, 2]
    tq = cKDTree(Q)
    pc, qc = P.mean(0), Q.mean(0)
    _, _, Vp = np.linalg.svd(P - pc, full_matrices=False)
    _, _, Vq = np.linalg.svd(Q - qc, full_matrices=False)
    best = math.inf
    for signs in product((1.0, -1.0), repeat=3):
        R0 = Vq.T @ np.diag(signs) @ Vp
        if np.linalg.det(R0) < 0:
            continue
        X = (P - pc) @ R0.T + qc
        for _ in range(iters):
            _, idx = tq.query(X)
            R, t = _kabsch(X, Q[idx])
            X = X @ R.T + t
        d_pq = tq.query(X)[0].mean()
        d_qp = cKDTree(X).query(Q)[0].mean()
        best = min(best, 0.5 * (d_pq + d_qp))
    return float(best)


def score_geometry(cf, rf, cand_pts, ref_pts):
    f1, rec, prec, missing, extra = inventory_match(cf["solids"],
                                                    rf["solids"])
    perfect, zero = THRESHOLDS["pose_mm"]
    if cand_pts is None or ref_pts is None:
        pose, pose_note = 0.0, "no delivered mesh to compare"
    else:
        direct = register(cand_pts, ref_pts)
        mirrored = register(cand_pts, ref_pts, reflect=True)
        s_direct = score_error(direct, perfect, zero)
        s_mirror = score_error(mirrored, perfect, zero)
        if mirrored < THRESHOLDS["mirror_ratio"] * direct:
            # Not max(s_direct, ...): the lift is close enough to symmetric
            # that even a mirror image fits "well" directly. Fitting
            # decisively better reflected is what makes it the mirror.
            pose = 0.5 * s_mirror
            pose_note = (f"fits the reference only when reflected "
                         f"({mirrored:.1f} mm vs {direct:.1f} mm direct): "
                         f"mirror-image assembly, half credit")
        else:
            pose = s_direct
            pose_note = (f"registered surface distance {direct:.1f} mm "
                         f"(reflected: {mirrored:.1f} mm)")
    score = 0.5 * f1 + 0.5 * pose
    return score, (f"parts inventory {f1:.2f} (recall {rec:.2f}, precision "
                   f"{prec:.2f}; {missing} reference solid(s) unmatched, "
                   f"{extra} extra); assembled pose {pose:.2f}: {pose_note}")


# ---------------------------------------------------------------------------
# joints and motion

def score_joints(cf, rf):
    ref_ok = sum(1 for m in rf["mech"] if m["mated"] and m["distinct"]) or 1
    mech = cf["mech"]
    ok = [m for m in mech if m["mated"] and m["distinct"]]
    if not mech:
        return 0.0, "no mechanism joints at all (only Fixed / grounding)"
    recall = score_ratio(len(ok), ref_ok)
    precision = len(ok) / len(mech)
    unmated = [m["name"] for m in mech if m["mated"] is False]
    noop = [m["name"] for m in mech if m["mated"] and not m["distinct"]]
    note = (f"{len(ok)} of {len(mech)} mechanism joints mated between "
            f"distinct rigid groups (reference: {ref_ok})")
    if unmated:
        note += f"; open at the delivered pose: {', '.join(unmated[:6])}"
    if noop:
        note += f"; joining parts already rigid together: {', '.join(noop[:6])}"
    return recall * precision, note


def parked(mech):
    """Worst remaining travel to a joint limit at the delivered pose, as a
    factor: 1 with room both ways, 0.5 parked against a stop, 0.25 outside
    the permitted range."""
    worst, notes = 1.0, []
    for m in mech:
        v, lim = m.get("value"), m.get("limits") or {}
        for lo_key, hi_key in (("LengthMin", "LengthMax"),
                               ("AngleMin", "AngleMax")):
            lo, hi = lim.get(lo_key), lim.get(hi_key)
            if v is None or (lo is None and hi is None):
                continue
            room = min(v - lo if lo is not None else math.inf,
                       hi - v if hi is not None else math.inf)
            if room < -THRESHOLDS["parked"]:
                f = 0.25
                notes.append(f"{m['name']} at {v:g} lies outside its "
                             f"limits [{lo}, {hi}]")
            else:
                f = 0.5 + 0.5 * score_ratio(room, THRESHOLDS["parked"])
                if f < 1:
                    notes.append(f"{m['name']} parked at {v:g} against its "
                                 f"limit [{lo}, {hi}]")
            worst = min(worst, f)
    return worst, notes


def score_articulation(cf, rf):
    if not any(m["type"] in ("Revolute", "Slider", "Cylindrical")
               for m in cf["mech"]):
        return 0.0, "nothing that can move: no revolute/slider joints"
    frac = cf["best_moved"] / max(cf["n_parts"], 1)
    ref_frac = rf["best_moved"] / max(rf["n_parts"], 1)
    moves = score_ratio(frac, THRESHOLDS["moved_full"] * ref_frac)
    stop, notes = parked(cf["mech"])
    note = (f"best drag moved {cf['best_moved']} of {cf['n_parts']} parts "
            f"with every joint mated ({cf['clean_drags']} clean of "
            f"{cf['drags']} drags; reference {rf['best_moved']} of "
            f"{rf['n_parts']})")
    if not cf["clean_drags"]:
        note += "; no drag solved with the joints intact"
    if notes:
        note += "; " + "; ".join(notes[:3])
    return moves * stop, note


# ---------------------------------------------------------------------------
# parametric structure

def score_census(cf, rf):
    full = THRESHOLDS["census_full"]
    a = score_ratio(cf["dims"], full * max(rf["dims"], 1))
    b = score_ratio(cf["parametric_share"], full * rf["parametric_share"])
    return 0.5 * a + 0.5 * b, (
        f"{cf['dims']} dimensional parameters on delivered bodies "
        f"(reference {rf['dims']}); {cf['parametric_share']:.0%} of the "
        f"delivered volume is parametric (reference "
        f"{rf['parametric_share']:.0%})")


def score_drive(cf, rf):
    if not cf["probed"]:
        return 0.0, "no parameter to nudge on the delivered geometry"
    share = cf["driving"] / cf["probed"]
    ref_share = rf["driving"] / max(rf["probed"], 1)
    return score_ratio(share, THRESHOLDS["drive_full"] * ref_share), (
        f"{cf['driving']} of {cf['probed']} nudged parameters changed the "
        f"geometry ({share:.0%}; reference {ref_share:.0%})")


def score_signature(cf, rf):
    """Pivot spacings and the extents of every major reference part, each
    found in the candidate or not, and how close."""
    perfect, zero = THRESHOLDS["signature_rel"]
    items = []
    pool = list(cf["spacings"])
    for s in rf["spacings"]:
        best = min(pool, key=lambda x: abs(x - s)) if pool else None
        sc = score_error(rel_err(best, s), perfect, zero) if best else 0.0
        if best is not None and sc > 0:
            pool.remove(best)
        items.append(("pivot spacing", s, sc))
    total = sum(s["volume"] for s in rf["solids"]) or 1.0
    major, seen = [], []
    for s in rf["solids"]:
        if s["volume"] < THRESHOLDS["major_share"] * total:
            continue
        key = tuple(round(x, 1) for x in s["extents"])
        if key in seen:
            continue
        seen.append(key)
        major.append(s)
    for s in major:
        best = min((max(rel_err(x, y) for x, y in
                        zip(c["extents"], s["extents"]))
                    for c in cf["solids"]), default=None)
        sc = score_error(best, perfect, zero) if best is not None else 0.0
        items.append(("part", "x".join(f"{x:g}" for x in s["extents"]), sc))
    if not items:
        return 0.0, "the reference has no signature dimensions to compare"
    score = sum(i[2] for i in items) / len(items)
    miss = [f"{k} {v}" for k, v, sc in items if sc < 0.5]
    n_piv = sum(1 for i in items if i[0] == "pivot spacing")
    note = (f"{sum(1 for i in items if i[2] >= 0.5)} of {len(items)} "
            f"signature dimensions present ({n_piv} pivot spacings, "
            f"{len(items) - n_piv} major part sizes)")
    if miss:
        note += "; missing or off: " + ", ".join(miss[:5])
    return score, note


# ---------------------------------------------------------------------------
# the whole score

def gate(meas, error):
    if error is not None:
        return False, f"the measurement stage failed: {error}"
    tot = meas.get("delivered", {}).get("total", {})
    if not tot.get("solids") or tot.get("volume", 0.0) <= 0:
        return False, "the document opens but delivers no solid geometry"
    bad = meas.get("build", {}).get("invalid", [])
    if bad:
        return True, (f"opened and built {tot['solids']} solids; "
                      f"{len(bad)} object(s) failed to recompute")
    return True, (f"opened, recomputed cleanly and delivered "
                  f"{tot['solids']} solids from "
                  f"{meas['delivered']['source']}")


def score_all(cand_meas, cand_pts, error, ref_meas, ref_pts):
    """{component: (name, score, description)} -- the harness contract."""
    out = {}
    ok, why = gate(cand_meas or {}, error)
    name = "executes and builds geometry"
    out[name] = (name, 1.0 if ok else 0.0, why)
    if not ok:
        for name in ALL_CRITERIA:
            out.setdefault(name, (name, 0.0, "not graded: the gate failed"))
        return out
    cf, rf = features(cand_meas), features(ref_meas)
    for name, fn in (
            ("geometry matches ground truth",
             lambda: score_geometry(cf, rf, cand_pts, ref_pts)),
            ("pantograph joints mated", lambda: score_joints(cf, rf)),
            ("mechanism articulates", lambda: score_articulation(cf, rf)),
            ("carries dimensional constraints", lambda: score_census(cf, rf)),
            ("dimensions drive the geometry", lambda: score_drive(cf, rf)),
            ("signature dimensions present",
             lambda: score_signature(cf, rf))):
        try:
            score, desc = fn()
        except Exception as exc:                            # noqa: BLE001
            score, desc = 0.0, f"could not be scored: " \
                               f"{type(exc).__name__}: {exc}"
        out[name] = (name, round(clamp01(score), 4), desc)
    return out


# ---------------------------------------------------------------------------
# measurement plumbing

def stage_hash():
    return hashlib.sha256(STAGE.read_bytes()).hexdigest()[:16]


def read_stage_dir(path):
    path = Path(path)
    meas = json.loads((path / "measure.json").read_text(encoding="utf-8"))
    stl = path / "delivered.stl"
    pts = sample_points(stl) if stl.is_file() else None
    return meas, pts


def baseline_payload(meas, pts):
    return {"stage_hash": stage_hash(), "measurement": meas,
            "points": [[round(float(x), 2) for x in p] for p in pts]
            if pts is not None else None}


class SlingLiftHarness(Harness):
    MUST_PASS = ("executes and builds geometry",)
    WEIGHTS = ALL_CRITERIA
    BUILD_TIMEOUT_S = 600

    def measure(self, doc_path):
        """(measurement, points, error) for one document."""
        out_dir = Path(tempfile.mkdtemp(prefix="sling_lift_"))
        try:
            try:
                meas, err = self.run_freecad_stage(doc_path, out_dir=out_dir)
            except SystemExit as exc:
                return None, None, str(exc)
            except Exception as exc:                        # noqa: BLE001
                return None, None, f"{type(exc).__name__}: {exc}"
            if err is not None:
                return None, None, err
            stl = out_dir / "delivered.stl"
            pts = sample_points(stl) if stl.is_file() else None
            return meas, pts, None
        finally:
            shutil.rmtree(out_dir, ignore_errors=True)

    def reference(self):
        """The reference measurement: frozen if current, else measured."""
        if BASELINE.is_file():
            data = json.loads(BASELINE.read_text(encoding="utf-8"))
            if data.get("stage_hash") == stage_hash():
                import numpy as np
                pts = data.get("points")
                return data["measurement"], (np.asarray(pts, dtype=float)
                                             if pts else None)
            print("baseline.json is stale (measure_stage.py changed); "
                  "measuring the reference live", file=sys.stderr)
        meas, pts, err = self.measure(REFERENCE_DOC)
        if err is not None:
            raise RuntimeError(f"cannot measure the reference: {err}")
        return meas, pts

    def build_state(self, candidate_path):
        meas, pts, err = self.measure(candidate_path)
        return {"candidate": candidate_path, "measurement": meas,
                "points": pts, "error": err}

    def checks(self, state):
        ref_meas, ref_pts = (None, None)
        if state["error"] is None:
            ref_meas, ref_pts = self.reference()
        return score_all(state["measurement"], state["points"],
                         state["error"], ref_meas, ref_pts)

    @classmethod
    def freeze(cls):
        h = cls()
        meas, pts, err = h.measure(REFERENCE_DOC)
        if err is not None:
            raise SystemExit(f"cannot measure the reference: {err}")
        BASELINE.write_text(json.dumps(baseline_payload(meas, pts)),
                            encoding="utf-8")
        print(f"wrote {BASELINE} (stage {stage_hash()})")

    @classmethod
    def score_from(cls, stage_dir, ref_dir=None):
        """Score a stage output directory without running FreeCAD for the
        candidate. The reference comes from baseline.json, or from another
        stage output directory when `ref_dir` is given."""
        from common.harness_base import finalize
        meas, pts = read_stage_dir(stage_dir)
        err = None if meas.get("ok", True) else meas.get("error", "stage failed")
        if ref_dir:
            ref_meas, ref_pts = read_stage_dir(ref_dir)
        else:
            ref_meas, ref_pts = cls().reference()
        checks = score_all(meas, pts, err, ref_meas, ref_pts)
        return finalize(cls.task_dir(), checks, must_pass=cls.MUST_PASS,
                        weights=cls.WEIGHTS)


main = SlingLiftHarness.as_main()

if __name__ == "__main__":
    if "--freeze" in sys.argv[1:]:
        SlingLiftHarness.freeze()
    elif "--score-from" in sys.argv[1:]:
        i = sys.argv.index("--score-from")
        ref = None
        if "--ref-from" in sys.argv:
            ref = sys.argv[sys.argv.index("--ref-from") + 1]
        print(json.dumps(SlingLiftHarness.score_from(sys.argv[i + 1], ref),
                         indent=1))
    else:
        SlingLiftHarness.cli()
