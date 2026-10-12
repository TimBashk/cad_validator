"""Measurement stage for 4_sling_lift.

Runs inside `freecadcmd`, not in the harness's interpreter. It is
launched by `Harness.run_freecad_stage` with three environment variables
already set:

    FC_STAGE_INPUT    the document to measure
    FC_STAGE_OUT      a directory to write measure.json into
    FC_STAGE_COMMON   the directory holding common/, so this can import
                      the Stage base

Optional knobs (environment):

    FC_STAGE_BUDGET_S    wall-clock budget for the two probes below,
                         split between them (default 240)
    FC_STAGE_MESH_TOL    linear deflection of delivered.stl (default 0.5)

**Stdlib and FreeCAD only.** Everything here measures; nothing here
scores. Every number the harness grades on is written to measure.json
as a plain fact, and the judgement of what is good enough lives in
harness.py, where it can be tested without FreeCAD.

WHAT IS MEASURED, AND WHY THIS WAY

`build`
    Does the document recompute, and which objects end up invalid. The
    gate.

`delivered`
    What the candidate actually hands over. That is the root assembly
    (or assemblies) -- not every body in the file. The corpus keeps
    source sub-assemblies parked in groups beside the root and pulls
    them in through flexible AssemblyLinks; counting both would count
    the lift twice. With no assembly at all, the visible root shapes
    are the deliverable.

    Per solid: volume, area, centre of mass, inertia tensor, and the
    extents in its own principal frame. None of those depend on names,
    tree shape or modelling order, and none of them change under a
    reflection, so the harness can compare shape without being fooled
    by a mirror -- and decide separately, from delivered.stl, whether
    the mirror matters.

`joints`
    Every joint in the delivered assemblies (recursing into
    AssemblyLinks), with what it connects as *moving parts* (so a joint
    between two faces of one rigid body is visibly a no-op), how far its
    two coordinate systems are apart at the delivered pose (mated or
    not), its current value and its limits. A Grounded joint records
    what it grounds.

`solve`
    Re-solve the delivered assembly once without touching it. A
    consistent assembly solves in place; joints that contradict each
    other -- which is what decorative joints tend to do -- make the
    solver fail or leave a gap.

`articulation`
    The question "does it move" is asked the way a user asks it in the
    GUI: drag. For each Slider/Revolute/Cylindrical joint, the part on
    one side is nudged along the joint's own axis, pinned there with a
    temporary ground, and the assembly is re-solved. A mechanism that
    articulates follows: the solve succeeds, every joint stays mated,
    and other parts move with it. A rigid one fails to solve. One that
    "moves" by flying apart solves with joints left open -- which is why
    the residual is recorded beside the count of parts that moved.

    (Assembly limits are not enforced by AssemblyObject.solve() in 1.1,
    and the kinematic Simulation needs the GUI module, so neither can
    serve as the drive. Dragging can.)

`parametric`
    Where the dimensions live -- sketch constraints, spreadsheet cells,
    VarSet properties, expressions -- and how much of the delivered
    volume comes from parametric features rather than dead imported
    solids.

`drive_probe`
    Whether a dimension *drives* anything is a different question from
    whether it exists. Each candidate parameter is nudged, the document
    recomputed, and the geometry it should own re-measured. A tidy set
    of named constraints that changes nothing is reported as exactly
    that. Values are restored afterwards.
"""
import math
import os
import sys
import time

sys.path.insert(0, os.environ["FC_STAGE_COMMON"])

from common.freecad_stage import Stage                    # noqa: E402

#: Joint kinds that permit motion and can be dragged, with the drag step
#: (mm along the axis for translation, degrees about it for rotation).
DRAGGABLE = {"Slider": ("translate", 5.0),
             "Revolute": ("rotate", 3.0),
             "Cylindrical": ("rotate", 3.0)}

#: Sketch constraint types that carry a dimension.
DIMENSIONAL = {"Distance", "DistanceX", "DistanceY", "Radius", "Diameter",
               "Angle", "Weight"}

#: A part counts as having moved in a drag beyond this distance (mm).
MOVED_MM = 0.5

#: Containers inside an assembly that are not parts.
NON_PART_TYPES = {"Assembly::JointGroup", "Assembly::ViewGroup",
                  "Assembly::BomGroup", "Assembly::SimulationGroup",
                  "Assembly::BomObject"}


def _r(x, nd=4):
    return round(float(x), nd)


def _vec(v, nd=4):
    return [_r(v.x, nd), _r(v.y, nd), _r(v.z, nd)]


def _matrix3(m):
    return [[_r(m.A11, 3), _r(m.A12, 3), _r(m.A13, 3)],
            [_r(m.A21, 3), _r(m.A22, 3), _r(m.A23, 3)],
            [_r(m.A31, 3), _r(m.A32, 3), _r(m.A33, 3)]]


class Budget:
    def __init__(self, seconds):
        self.end = time.time() + seconds

    def left(self):
        return self.end - time.time()

    def spent(self):
        return self.left() <= 0


class SlingLiftStage(Stage):

    # ------------------------------------------------------------------
    # opening

    def open(self):
        """FCStd opens as a document. A .py / .FCMacro candidate (task.toml
        accepts both) is executed here, inside freecadcmd -- never in the
        harness -- and whatever document it leaves active is measured."""
        import FreeCAD as App
        ext = os.path.splitext(self.doc_path)[1].lower()
        if ext not in (".py", ".fcmacro"):
            return App.openDocument(self.doc_path)
        before = set(App.listDocuments())
        with open(self.doc_path, encoding="utf-8") as f:
            code = compile(f.read(), self.doc_path, "exec")
        exec(code, {"__name__": "__main__", "__file__": self.doc_path})
        made = [d for n, d in App.listDocuments().items() if n not in before]
        doc = App.ActiveDocument or (made[-1] if made else None)
        if doc is None:
            raise RuntimeError("the script created no FreeCAD document")
        return doc

    # ------------------------------------------------------------------
    # top level

    def measure(self, doc):
        import FreeCAD as App
        t0 = time.time()
        budget_s = float(os.environ.get("FC_STAGE_BUDGET_S", "240"))
        out = {"freecad_version": ".".join(App.Version()[:3])}

        out["build"] = self.build(doc)
        roots = self.root_assemblies(doc)
        out["delivered"] = self.delivered(doc, roots)
        out["joints"] = [self.joint_record(j) for a in roots
                         for j in self.joints_of(a)]
        out["solve"] = [self.solve_in_place(a) for a in roots]
        out["parametric"] = self.parametric(doc, roots)

        # The two probes mutate the document; they run last and restore
        # what they touch, but nothing above may depend on running after.
        drag_budget = Budget(budget_s * 0.4)
        out["articulation"] = [self.articulation(a, drag_budget)
                               for a in roots]
        out["drive_probe"] = self.drive_probe(
            doc, roots, Budget(max(drag_budget.left(), 0) + budget_s * 0.6))
        out["elapsed_s"] = _r(time.time() - t0, 1)
        return out

    # ------------------------------------------------------------------
    # build

    def build(self, doc):
        doc.recompute()
        invalid = []
        for o in doc.Objects:
            state = list(getattr(o, "State", []) or [])
            if "Invalid" in state or "Error" in state:
                invalid.append({"name": o.Name, "label": o.Label,
                                "type": o.TypeId, "state": state})
        return {"objects": len(doc.Objects), "invalid": invalid}

    # ------------------------------------------------------------------
    # what is delivered

    @staticmethod
    def is_assembly(o):
        return o.TypeId == "Assembly::AssemblyObject"

    def root_assemblies(self, doc):
        """Assemblies nobody else pulls in. A nested assembly reached
        through an AssemblyLink is measured through that link, from the
        root, at the pose the root gives it."""
        out = []
        for a in doc.Objects:
            if not self.is_assembly(a):
                continue
            parents = [p for p in a.InList
                       if self.is_assembly(p)
                       or p.TypeId == "Assembly::AssemblyLink"]
            if not parents:
                out.append(a)
        return out

    def parts_of(self, container, seen=None, prefix=""):
        """Leaf parts of an assembly as (object, subpath-from-root) pairs,
        recursing into AssemblyLinks and nested assemblies; each leaf once.
        The subpath is what places a part in the world: a link inside a
        flexible AssemblyLink has no global placement of its own."""
        seen = set() if seen is None else seen
        out = []
        for c in getattr(container, "Group", []) or []:
            if c.TypeId in NON_PART_TYPES or hasattr(c, "JointType") \
                    or hasattr(c, "ObjectToGround"):
                continue
            if c.TypeId == "Assembly::AssemblyLink" or self.is_assembly(c):
                out += self.parts_of(c, seen, prefix + c.Name + ".")
                continue
            if not hasattr(c, "Shape") and not c.isDerivedFrom("App::Link"):
                continue
            if c.FullName in seen:
                continue
            seen.add(c.FullName)
            out.append((c, prefix + c.Name + "."))
        return out

    def joints_of(self, container, seen=None):
        seen = set() if seen is None else seen
        out = []
        for c in getattr(container, "Group", []) or []:
            if c.TypeId == "Assembly::JointGroup":
                out += self.joints_of(c, seen)
            elif c.TypeId == "Assembly::AssemblyLink":
                out += self.joints_of(c, seen)
            elif (hasattr(c, "JointType") or hasattr(c, "ObjectToGround")) \
                    and c.FullName not in seen:
                seen.add(c.FullName)
                out.append(c)
        return out

    @staticmethod
    def world_shape(root, sub):
        """Shape of `root`'s sub-object `sub` in world coordinates, or None."""
        import Part
        try:
            s = Part.getShape(root, sub, needSubElement=False, transform=True)
        except Exception:                                   # noqa: BLE001
            return None
        return None if s is None or s.isNull() else s

    @staticmethod
    def global_placement(obj, root, sub):
        import FreeCAD as App
        return App.GeoFeature.getGlobalPlacementOf(obj, root, sub)

    def delivered(self, doc, roots):
        import Part
        if roots:
            leaves = [(a, o, sub) for a in roots
                      for o, sub in self.parts_of(a)]
            source = "assembly"
        else:
            leaves = [(o, o, "") for o in doc.RootObjects
                      if getattr(o, "Visibility", True)
                      and hasattr(o, "Shape")]
            source = "visible root objects"
        solids, shapes, parts = [], [], []
        for root, leaf, sub in leaves:
            s = self.world_shape(root, sub)
            n = len(s.Solids) if s is not None else 0
            parts.append({"name": leaf.Name, "label": leaf.Label,
                          "type": leaf.TypeId,
                          "content": self.content_volumes(leaf),
                          "solids": n})
            if not n:
                continue
            shapes.append(s)
            for sol in s.Solids:
                solids.append(self.solid_record(sol, leaf))
        total = {"solids": len(solids)}
        if shapes:
            comp = Part.makeCompound(shapes)
            bb = comp.BoundBox
            vol = sum(x["volume"] for x in solids)
            com = [sum(x["com"][i] * x["volume"] for x in solids) / vol
                   for i in range(3)] if vol > 0 else [0.0, 0.0, 0.0]
            total.update({
                "volume": _r(vol, 2),
                "area": _r(sum(x["area"] for x in solids), 2),
                "com": [_r(c, 3) for c in com],
                "bbox": [_r(bb.XMin, 3), _r(bb.YMin, 3), _r(bb.ZMin, 3),
                         _r(bb.XMax, 3), _r(bb.YMax, 3), _r(bb.ZMax, 3)],
                "inertia": self.combined_inertia(solids, com),
            })
            total["mesh"] = self.export_mesh(comp)
        return {"source": source,
                "assemblies": [{"name": a.Name, "label": a.Label}
                               for a in roots],
                "parts": parts, "solids": solids, "total": total}

    @classmethod
    def content_volumes(cls, obj, depth=0):
        """{TypeId: volume} of the solid-bearing objects a part resolves
        to, looking through links and App::Part containers. A
        PartDesign::Body is parametric; a bare Part::Feature is dead
        geometry -- an import, or a refine-and-forget."""
        out = {}
        if depth > 8:
            return out
        try:
            tgt = obj.getLinkedObject(True)
        except Exception:                                   # noqa: BLE001
            tgt = obj
        if tgt.TypeId in ("App::Part", "App::DocumentObjectGroup") \
                or tgt.TypeId == "App::LinkGroup":
            for c in getattr(tgt, "Group", []) or []:
                if c.TypeId in ("App::Origin",) or c.isDerivedFrom(
                        "App::DatumElement") or c.isDerivedFrom(
                        "Part::Datum"):
                    continue
                if c.TypeId.startswith("Sketcher::"):
                    continue
                if c.TypeId != "PartDesign::Body" and any(
                        p.TypeId == "PartDesign::Body" for p in c.InList):
                    continue                 # a feature inside a body
                if not getattr(c, "Visibility", True) and \
                        c.TypeId != "App::Link":
                    continue                 # a hidden base, not delivered
                for k, v in cls.content_volumes(c, depth + 1).items():
                    out[k] = _r(out.get(k, 0.0) + v, 2)
            return out
        s = getattr(tgt, "Shape", None)
        if s is not None and not s.isNull() and s.Solids:
            out[tgt.TypeId] = _r(s.Volume, 2)
        return out

    @classmethod
    def bodies_in(cls, obj, depth=0):
        """Names of the PartDesign bodies a part resolves to."""
        if depth > 8:
            return set()
        try:
            tgt = obj.getLinkedObject(True)
        except Exception:                                   # noqa: BLE001
            tgt = obj
        if tgt.TypeId == "PartDesign::Body":
            return {tgt.Name}
        out = set()
        for c in getattr(tgt, "Group", []) or []:
            if c.TypeId in ("App::Part", "PartDesign::Body", "App::Link",
                            "App::DocumentObjectGroup", "App::LinkGroup"):
                out |= cls.bodies_in(c, depth + 1)
        return out

    @staticmethod
    def solid_record(sol, leaf):
        import FreeCAD as App
        rec = {"part": leaf.Name, "volume": _r(sol.Volume, 3),
               "area": _r(sol.Area, 3), "com": _vec(sol.CenterOfMass, 3),
               "inertia": _matrix3(sol.MatrixOfInertia)}
        # Extents in the solid's own principal frame: survives any rigid
        # motion and any reflection.
        try:
            pp = sol.PrincipalProperties
            axes = [pp["FirstAxisOfInertia"], pp["SecondAxisOfInertia"],
                    pp["ThirdAxisOfInertia"]]
            rot = App.Matrix(axes[0].x, axes[0].y, axes[0].z, 0,
                             axes[1].x, axes[1].y, axes[1].z, 0,
                             axes[2].x, axes[2].y, axes[2].z, 0,
                             0, 0, 0, 1)
            local = sol.copy()
            local.translate(sol.CenterOfMass * -1)
            local.transformShape(rot)
            bb = local.BoundBox
            rec["extents"] = sorted([_r(bb.XLength, 3), _r(bb.YLength, 3),
                                     _r(bb.ZLength, 3)])
            rec["principal_moments"] = sorted(
                _r(m, 1) for m in pp["Moments"])
        except Exception:                                   # noqa: BLE001
            bb = sol.BoundBox
            rec["extents"] = sorted([_r(bb.XLength, 3), _r(bb.YLength, 3),
                                     _r(bb.ZLength, 3)])
        return rec

    @staticmethod
    def combined_inertia(solids, com):
        """Inertia tensor of the whole delivered set about its own centre
        of mass (parallel-axis shift of each solid's tensor)."""
        tot = [[0.0] * 3 for _ in range(3)]
        for s in solids:
            d = [s["com"][i] - com[i] for i in range(3)]
            dd = sum(x * x for x in d)
            for i in range(3):
                for k in range(3):
                    shift = s["volume"] * ((dd if i == k else 0.0)
                                           - d[i] * d[k])
                    tot[i][k] += s["inertia"][i][k] + shift
        return [[_r(v, 1) for v in row] for row in tot]

    def export_mesh(self, comp):
        """delivered.stl beside measure.json, for whatever pose / chirality
        comparison the harness wants to make with trimesh."""
        try:
            import Mesh
            import MeshPart
            tol = float(os.environ.get("FC_STAGE_MESH_TOL", "0.5"))
            mesh = MeshPart.meshFromShape(Shape=comp, LinearDeflection=tol,
                                          AngularDeflection=0.5)
            path = os.path.join(self.out_dir, "delivered.stl")
            os.makedirs(self.out_dir, exist_ok=True)
            Mesh.Mesh(mesh).write(path)
            return {"file": "delivered.stl", "facets": mesh.CountFacets}
        except Exception as exc:                            # noqa: BLE001
            return {"error": f"{type(exc).__name__}: {exc}"}

    # ------------------------------------------------------------------
    # joints

    @staticmethod
    def moving_part(ref):
        import UtilsAssembly as U
        try:
            return U.getMovingPart(ref)
        except Exception:                                   # noqa: BLE001
            return None

    @staticmethod
    def jcs(joint, side):
        import UtilsAssembly as U
        ref = joint.Reference1 if side == 1 else joint.Reference2
        plc = joint.Placement1 if side == 1 else joint.Placement2
        return U.getJcsGlobalPlc(plc, ref)

    def residual(self, joint):
        """How far a joint is from being satisfied at the current pose:
        `gap` in mm, `tilt` in degrees between the axes it should align.
        None for kinds without a simple closed form here."""
        import FreeCAD as App
        kind = joint.JointType
        try:
            p1, p2 = self.jcs(joint, 1), self.jcs(joint, 2)
        except Exception:                                   # noqa: BLE001
            return None
        z = App.Vector(0, 0, 1)
        z1, z2 = p1.Rotation.multVec(z), p2.Rotation.multVec(z)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, abs(z1.dot(z2))))))
        d = p2.Base - p1.Base
        if kind in ("Fixed", "Revolute", "Ball"):
            gap = d.Length
        elif kind in ("Slider", "Cylindrical"):
            along = d.dot(z1)
            gap = (d - z1 * along).Length
        else:
            return None
        if kind == "Ball":
            tilt = 0.0
        if kind == "Fixed":
            rel = p1.inverse() * p2
            tilt = math.degrees(rel.Rotation.Angle)
            tilt = min(tilt, abs(180.0 - tilt))       # a flipped JCS is fine
        return {"gap": _r(gap, 4), "tilt": _r(tilt, 4)}

    def joint_value(self, joint):
        import UtilsAssembly as U
        try:
            if joint.JointType in ("Slider",):
                return _r(U.getJointDistance(joint), 4)
            if joint.JointType in ("Revolute", "Cylindrical"):
                return _r(math.degrees(U.getJointXYAngle(joint)), 4)
        except Exception:                                   # noqa: BLE001
            return None
        return None

    def joint_record(self, j):
        if hasattr(j, "ObjectToGround"):
            g = j.ObjectToGround
            return {"name": j.Name, "label": j.Label, "type": "Grounded",
                    "grounds": getattr(g, "Name", None)}
        rec = {"name": j.Name, "label": j.Label, "type": j.JointType,
               "suppressed": bool(getattr(j, "Suppressed", False))}
        parts = []
        for ref in (j.Reference1, j.Reference2):
            mp = self.moving_part(ref) if ref else None
            parts.append(mp.Name if mp is not None else None)
        rec["parts"] = parts
        rec["refs"] = [self.ref_text(j.Reference1), self.ref_text(j.Reference2)]
        rec["same_part"] = parts[0] is not None and parts[0] == parts[1]
        # Each side's coordinate system where that side's part puts it: on
        # a mated joint the two coincide; on an open one each still marks
        # where its own part's pivot is.
        rec["frames"] = []
        for side in (1, 2):
            try:
                import FreeCAD as App
                f = self.jcs(j, side)
                rec["frames"].append({"origin": _vec(f.Base, 3),
                                      "axis": _vec(f.Rotation.multVec(
                                          App.Vector(0, 0, 1)), 6)})
            except Exception:                               # noqa: BLE001
                rec["frames"].append(None)
        rec["residual"] = self.residual(j)
        rec["value"] = self.joint_value(j)
        lim = {}
        for key in ("LengthMin", "LengthMax", "AngleMin", "AngleMax"):
            if getattr(j, "Enable" + key, False):
                v = getattr(j, key)
                lim[key] = _r(getattr(v, "Value", v), 4)
        rec["limits"] = lim
        if j.JointType == "Distance":
            rec["distance"] = _r(getattr(j.Distance, "Value", j.Distance), 4)
        return rec

    @staticmethod
    def ref_text(ref):
        if not ref:
            return None
        try:
            obj, subs = ref[0], ref[1]
            return f"{obj.Name}:{subs[0] if subs else ''}"
        except Exception:                                   # noqa: BLE001
            return str(ref)

    # ------------------------------------------------------------------
    # solving

    def snapshot(self, asm, parts):
        out = {}
        for p, sub in parts:
            try:
                out[p.FullName] = self.global_placement(p, asm, sub)
            except Exception:                               # noqa: BLE001
                out[p.FullName] = p.Placement
        return out

    @staticmethod
    def local_placements(parts):
        import FreeCAD as App
        return {p.FullName: App.Placement(p.Placement) for p, _ in parts}

    @staticmethod
    def restore(parts, local):
        for p, _ in parts:
            if p.FullName in local and p.Placement != local[p.FullName]:
                p.Placement = local[p.FullName]

    def worst_residual(self, joints):
        gap = tilt = 0.0
        for j in joints:
            if not hasattr(j, "JointType") or getattr(j, "Suppressed", False):
                continue
            r = self.residual(j)
            if r:
                gap, tilt = max(gap, r["gap"]), max(tilt, r["tilt"])
        return _r(gap, 4), _r(tilt, 4)

    def solve_in_place(self, asm):
        parts = self.parts_of(asm)
        joints = self.joints_of(asm)
        local = self.local_placements(parts)
        before = self.snapshot(asm, parts)
        try:
            rc = asm.solve()
        except Exception as exc:                            # noqa: BLE001
            self.restore(parts, local)
            return {"assembly": asm.Name, "rc": None,
                    "error": f"{type(exc).__name__}: {exc}"}
        after = self.snapshot(asm, parts)
        shift = max([(before[k].Base - after[k].Base).Length
                     for k in before] or [0.0])
        gap, tilt = self.worst_residual(joints)
        self.restore(parts, local)
        return {"assembly": asm.Name, "rc": rc, "max_shift": _r(shift, 4),
                "worst_gap": gap, "worst_tilt": tilt}

    # ------------------------------------------------------------------
    # articulation

    def articulation(self, asm, budget):
        parts = self.parts_of(asm)
        joints = self.joints_of(asm)
        movable = [j for j in joints if getattr(j, "JointType", None)
                   in DRAGGABLE and not getattr(j, "Suppressed", False)]
        # Translational joints first: in a lift the slider is usually the
        # one input, and a drag there says the most about the whole.
        movable.sort(key=lambda j: DRAGGABLE[j.JointType][0] != "translate")
        drags = []
        for j in movable:
            # Four ways to drag a joint (either side, either direction).
            # One that moves the chain and keeps it mated settles the
            # question for that joint; the rest would only cost time.
            for side, sign in ((1, 1), (2, 1), (1, -1), (2, -1)):
                if budget.spent():
                    return {"assembly": asm.Name, "parts": len(parts),
                            "movable_joints": len(movable),
                            "drags": drags, "complete": False}
                rec = self.drag(asm, parts, joints, j, side, sign)
                drags.append(rec)
                if self.clean(rec) and rec.get("moved", 0) >= 2:
                    break
        return {"assembly": asm.Name, "parts": len(parts),
                "movable_joints": len(movable), "drags": drags,
                "complete": True}

    @staticmethod
    def clean(rec):
        """A drag the solver accepted with every joint still mated."""
        return (rec.get("rc") == 0 and rec.get("worst_gap", 1e9) < 0.01
                and rec.get("worst_tilt", 1e9) < 0.5)

    def drag(self, asm, parts, joints, j, side, sign):
        import FreeCAD as App
        import JointObject
        import UtilsAssembly as U
        mode, step = DRAGGABLE[j.JointType]
        rec = {"joint": j.Name, "type": j.JointType, "side": side,
               "delta": step * sign}
        ref = j.Reference1 if side == 1 else j.Reference2
        mp = self.moving_part(ref)
        if mp is None:
            rec["error"] = "no moving part"
            return rec
        rec["part"] = mp.Name
        paths = {p.FullName: sub for p, sub in parts}
        if mp.FullName not in paths:
            rec["error"] = "moving part is not a delivered part"
            return rec
        local = self.local_placements(parts)
        keep = App.Placement(mp.Placement)
        before = self.snapshot(asm, parts)
        v0 = self.joint_value(j)
        ground = None
        try:
            frame = self.jcs(j, side)
            axis = frame.Rotation.multVec(App.Vector(0, 0, 1))
            if mode == "translate":
                move = App.Placement(axis * (step * sign), App.Rotation())
            else:
                move = App.Placement(App.Vector(),
                                     App.Rotation(axis, step * sign),
                                     frame.Base)
            parent = (self.global_placement(mp, asm, paths[mp.FullName])
                      * mp.Placement.inverse())
            ground = U.getJointGroup(asm).newObject("App::FeaturePython",
                                                    "ProbeGround")
            JointObject.GroundedJoint(ground, mp)
            mp.Placement = parent.inverse() * move * parent * mp.Placement
            rec["rc"] = asm.solve()
            after = self.snapshot(asm, parts)
            moved = [k for k in before
                     if (before[k].Base - after[k].Base).Length > MOVED_MM
                     or abs(math.degrees((before[k].Rotation.inverted()
                                          * after[k].Rotation).Angle)) > 0.5]
            rec["moved"] = len(moved)
            rec["max_shift"] = _r(max([(before[k].Base - after[k].Base).Length
                                       for k in before] or [0.0]), 3)
            rec["worst_gap"], rec["worst_tilt"] = self.worst_residual(joints)
            v1 = self.joint_value(j)
            if v0 is not None and v1 is not None:
                rec["value_change"] = _r(v1 - v0, 4)
        except Exception as exc:                            # noqa: BLE001
            rec["error"] = f"{type(exc).__name__}: {exc}"
        finally:
            if ground is not None:
                try:
                    asm.Document.removeObject(ground.Name)
                except Exception:                           # noqa: BLE001
                    pass
            mp.Placement = keep
            self.restore(parts, local)
        return rec

    # ------------------------------------------------------------------
    # parametric structure

    def parametric(self, doc, roots):
        sketches, sheets, varsets, exprs = [], [], [], []
        for o in doc.Objects:
            if o.TypeId == "Sketcher::SketchObject":
                sketches.append(self.sketch_record(o))
            elif o.TypeId == "Spreadsheet::Sheet":
                sheets.append(self.sheet_record(o))
            elif o.TypeId == "App::VarSet":
                varsets.append({"name": o.Name, "label": o.Label,
                                "properties": self.varset_props(o)})
            for path, expr in getattr(o, "ExpressionEngine", []) or []:
                exprs.append({"object": o.Name, "type": o.TypeId,
                              "path": path, "expression": expr})
        # How much of the delivered volume is parametric.
        by_source, bodies = {}, set()
        for a in roots:
            for p, _sub in self.parts_of(a):
                bodies |= self.bodies_in(p)
                for k, v in self.content_volumes(p).items():
                    by_source[k] = _r(by_source.get(k, 0.0) + v, 2)
        return {"sketches": sketches, "spreadsheets": sheets,
                "varsets": varsets, "expressions": exprs,
                "delivered_volume_by_source": by_source,
                "delivered_bodies": sorted(bodies),
                "feature_types": self.feature_census(doc)}

    @staticmethod
    def feature_census(doc):
        out = {}
        for o in doc.Objects:
            if o.TypeId.startswith(("PartDesign::", "Part::", "Sketcher::")):
                out[o.TypeId] = out.get(o.TypeId, 0) + 1
        return out

    @staticmethod
    def body_of(obj):
        for p in obj.InList:
            if p.TypeId == "PartDesign::Body":
                return p
        return None

    def sketch_record(self, sk):
        cons = []
        for i, c in enumerate(sk.Constraints):
            if c.Type not in DIMENSIONAL:
                continue
            cons.append({"index": i, "type": c.Type, "name": c.Name or "",
                         "value": _r(c.Value, 4),
                         "driving": bool(getattr(c, "Driving", True)),
                         "expression": self.constraint_expression(sk, i)})
        body = self.body_of(sk)
        dof = None
        try:
            sk.solve()
            dof = getattr(sk, "FullyConstrained", None)
        except Exception:                                   # noqa: BLE001
            pass
        return {"name": sk.Name, "label": sk.Label,
                "body": body.Name if body else None,
                "constraints": len(sk.Constraints),
                "dimensional": cons, "fully_constrained": dof,
                "used": bool([x for x in sk.InList
                              if x.TypeId != "PartDesign::Body"])}

    @staticmethod
    def constraint_expression(sk, i):
        for path, expr in getattr(sk, "ExpressionEngine", []) or []:
            if path in (f"Constraints[{i}]", f".Constraints[{i}]") or \
                    path.endswith(f"Constraints[{i}]"):
                return expr
            c = sk.Constraints[i]
            if c.Name and path.endswith(f"Constraints.{c.Name}"):
                return expr
        return None

    @staticmethod
    def sheet_record(sh):
        cells = []
        for cell in sh.getUsedCells():
            content = sh.getContents(cell)
            alias = None
            try:
                alias = sh.getAlias(cell)
            except Exception:                               # noqa: BLE001
                pass
            cells.append({"cell": cell, "content": content, "alias": alias})
        return {"name": sh.Name, "label": sh.Label, "cells": cells}

    @staticmethod
    def varset_props(vs):
        out = []
        base = set(("Label", "Label2", "ExpressionEngine", "Visibility"))
        for p in vs.PropertiesList:
            if p in base:
                continue
            v = getattr(vs, p, None)
            v = getattr(v, "Value", v)
            if isinstance(v, (int, float)) and not isinstance(v, bool):
                out.append({"property": p, "value": _r(v, 4)})
        return out

    # ------------------------------------------------------------------
    # does a dimension drive anything?

    def drive_probe(self, doc, roots, budget):
        """Nudge every candidate parameter and see whether geometry
        follows. Sketch dimensions are judged against the body that owns
        the sketch, spreadsheet / VarSet values against the delivered
        geometry as a whole (they may feed any body)."""
        bodies_in_use = set()
        for a in roots:
            for p, _sub in self.parts_of(a):
                bodies_in_use |= self.bodies_in(p)

        def fingerprint(objs):
            vol, area, ext = 0.0, 0.0, []
            for o in objs:
                s = getattr(o, "Shape", None)
                if s is None or s.isNull():
                    continue
                vol += s.Volume
                area += s.Area
                bb = s.BoundBox
                # position as well as size: a constraint that only moves
                # a profile still drives the geometry
                ext += [bb.XMin, bb.YMin, bb.ZMin,
                        bb.XMax, bb.YMax, bb.ZMax]
            return vol, area, ext

        def changed(a, b):
            if abs(a[0] - b[0]) > 1e-6 * max(1.0, abs(a[0])) + 1e-6:
                return True
            if abs(a[1] - b[1]) > 1e-6 * max(1.0, abs(a[1])) + 1e-6:
                return True
            return any(abs(x - y) > 1e-6 for x, y in zip(a[2], b[2]))

        params = []
        for o in doc.Objects:
            if o.TypeId == "Sketcher::SketchObject":
                body = self.body_of(o)
                for i, c in enumerate(o.Constraints):
                    if c.Type in DIMENSIONAL and getattr(c, "Driving", True):
                        params.append(("sketch", o, i, body))
            elif o.TypeId == "Spreadsheet::Sheet":
                for cell in o.getUsedCells():
                    params.append(("cell", o, cell, None))
            elif o.TypeId == "App::VarSet":
                for p in self.varset_props(o):
                    params.append(("varset", o, p["property"], None))

        params = self.round_robin(params)
        everything = [o for o in doc.Objects
                      if o.TypeId == "PartDesign::Body"
                      or o.TypeId.startswith("Part::")]
        results, skipped = [], 0
        for kind, obj, key, body in params:
            if budget.spent():
                skipped += 1
                continue
            scope = [body] if body is not None else everything
            rec = {"kind": kind, "object": obj.Name, "key": key,
                   "body": body.Name if body is not None else None,
                   "body_delivered": (body.Name in bodies_in_use)
                   if body is not None else None}
            try:
                base = fingerprint(scope)
                undo = self.nudge(obj, kind, key, rec)
                if undo is None:
                    rec["drives"] = None
                    results.append(rec)
                    continue
                self.recompute(scope)
                rec["recompute_errors"] = sum(
                    1 for o in scope
                    if "Invalid" in (getattr(o, "State", []) or []))
                rec["drives"] = changed(base, fingerprint(scope))
                undo()
                self.recompute(scope)
            except Exception as exc:                        # noqa: BLE001
                rec["error"] = f"{type(exc).__name__}: {exc}"
            results.append(rec)
        return {"params": len(params), "probed": len(results),
                "skipped_for_time": skipped, "results": results}

    @staticmethod
    def recompute(objs):
        """Recompute just these objects and whatever they depend on. A
        whole-document recompute would also re-solve every assembly that
        links them, which costs seconds per probe and tells us nothing."""
        for o in objs:
            try:
                o.recompute(True)
            except Exception:                               # noqa: BLE001
                pass

    @staticmethod
    def round_robin(params):
        """Interleave parameters by owning object (one from each sketch,
        sheet or VarSet, then the next from each...), so that when the
        budget runs out every body has been probed about as often."""
        groups, order = {}, []
        for prm in params:
            owner = prm[1].Name
            if owner not in groups:
                groups[owner] = []
                order.append(owner)
            groups[owner].append(prm)
        out, i = [], 0
        while len(out) < len(params):
            for owner in order:
                if i < len(groups[owner]):
                    out.append(groups[owner][i])
            i += 1
        return out

    @staticmethod
    def nudge(obj, kind, key, rec):
        """Change one parameter a little; return a function undoing it, or
        None when the parameter is not a number we can nudge."""
        import FreeCAD as App

        def bumped(v):
            return v * 1.05 if abs(v) > 1e-9 else 1.0

        if kind == "sketch":
            c = obj.Constraints[key]
            rec["type"], rec["name"] = c.Type, c.Name or ""
            old = c.Value
            if c.Type in ("Angle",):
                new = old + math.radians(2.0)
            else:
                new = bumped(old)
            expr = None
            for path, e in getattr(obj, "ExpressionEngine", []) or []:
                if path.endswith(f"Constraints[{key}]") or (
                        c.Name and path.endswith(f"Constraints.{c.Name}")):
                    expr = (path, e)
            if expr is not None:
                # Bound to an expression: its own value is not a free
                # input; the upstream parameter is what gets probed.
                rec["bound_to"] = expr[1]
                return None
            unit = "deg" if c.Type == "Angle" else "mm"
            q = App.Units.Quantity(math.degrees(new) if unit == "deg"
                                   else new, App.Units.Unit(unit))
            obj.setDatum(key, q)

            def undo():
                q0 = App.Units.Quantity(math.degrees(old) if unit == "deg"
                                        else old, App.Units.Unit(unit))
                obj.setDatum(key, q0)
            return undo
        if kind == "cell":
            content = obj.getContents(key)
            try:
                v = float(content.lstrip("=").split()[0])
            except (ValueError, IndexError):
                return None
            rest = content.lstrip("=")[len(content.lstrip("=").split()[0]):]
            obj.set(key, f"{bumped(v)}{rest}")
            rec["name"] = obj.getAlias(key) if hasattr(obj, "getAlias") \
                else None

            def undo():
                obj.set(key, content)
            return undo
        if kind == "varset":
            old = getattr(obj, key)
            v = getattr(old, "Value", old)
            setattr(obj, key, bumped(v))
            rec["name"] = key

            def undo():
                setattr(obj, key, old)
            return undo
        return None


SlingLiftStage.run()
