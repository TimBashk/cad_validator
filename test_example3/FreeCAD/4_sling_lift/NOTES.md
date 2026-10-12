# Sling Lift harness — notes for reviewers

## Scores

Reference and all five examples, full verifier path (`tests/test.sh` in the
FreeCAD 1.1.3 Docker image) for the reference and the mirrored model,
`--score-from` for the rest:

| Model | Score | Geometry | Joints mated | Articulates | Dim. constraints | Dims drive | Signature dims |
|---|---:|---:|---:|---:|---:|---:|---:|
| **reference** | **6.00** | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 |
| half_finished_missing_features | 5.67 | 0.73 | 1.00 | 1.00 | 1.00 | 1.00 | 0.94 |
| assembly_mirrored_relative_to_ground_truth | 5.66 | 0.72 | 1.00 | 1.00 | 1.00 | 1.00 | 0.94 |
| one_link_missing_joints_bridge_gap | 5.27 | 0.77 | 1.00 | 0.50 | 1.00 | 1.00 | 1.00 |
| joint_limits_swapped | 4.96 | 0.52 | 1.00 | 0.50 | 1.00 | 1.00 | 0.94 |
| dummy_joints_inflate_the_count | 4.06 | 0.97 | 0.15 | 0.00 | 1.00 | 1.00 | 0.94 |

Each example loses points where its defect is, and the description next to
every subscore says what was measured (e.g. *"4 of 12 mechanism joints mated
between distinct rigid groups (reference: 9); open at the delivered pose:
Joint091, Joint042, …"*). The 0.06 on signature dimensions shared by
mirrored, swapped and dummy is a real difference from the reference, not
noise; see "Things I found in the corpus" below.

A full run takes about 3 minutes against a 600 s verifier budget.

## Components

I kept the seven shipped components, their names and their weights, so
`max_score` stays 6. What each one now means:

| Component | What is measured |
|---|---|
| executes and builds geometry (gate) | the stage opened the document and it delivers at least one solid |
| geometry matches ground truth | ½ **parts inventory**: a volume-weighted F1 from a one-to-one match of solids by shape invariants (volume, principal moments, extents in the solid's own principal frame). ½ **assembled pose**: mean surface distance after rigid registration (PCA starts + ICP) of 12k surface samples, with rotation and translation free |
| pantograph joints mated | mechanism joints (everything but Fixed/Grounded) that are mated at the delivered pose **and** join two different rigid groups (parts merged through mated Fixed joints). Score = recall against the reference count × the share of all mechanism joints that qualify |
| mechanism articulates | a GUI-style drag: nudge the part on one side of a joint along that joint's own axis, ground it temporarily, re-solve. It counts when the chain follows with every joint still mated. Multiplied by a limit factor: 1 with travel both ways, 0.5 parked against a stop, 0.25 outside its own limits |
| carries dimensional constraints | driving dimensional parameters on **delivered** bodies (sketch dimensions, numeric spreadsheet cells, VarSet properties), and the share of delivered volume that is parametric (PartDesign bodies) rather than dead `Part::Feature` imports. Full marks at 80 % of the reference |
| dimensions drive the geometry | every such parameter is nudged by +5 % (+2° for angles), its body recomputed, and its volume/area/bounding box (position included) compared. Score = driving share against the reference's own share (82 %), full marks at 90 % of it |
| signature dimensions present | pantograph pivot spacings (link lengths, from the axes of the revolute joints each rigid group carries) and the extents of every major reference part, each matched to the candidate. Pose-free on purpose |

None of this reads object names, labels, the document tree or feature order.
The shape comparisons stay the same under any rigid motion and any reflection.
The pose comparison is free in rotation and translation. Joints are judged
by what they connect and whether they hold, not by how many there are.

## The mirror call

**A mirror image is a defect, and it is charged once and only by half:
half of the pose half of "geometry matches ground truth" (−0.28 of 6).**

- **Why it's a defect.** The drawing's orthographic views and isometric fix
  the handedness: which side the indexing plungers are on, which bar of each
  scissor pair sits in front. A fabricator working from the mirrored model
  would not build the lift on the drawing.
- **Why it's charged so little.** Everything else about the mirrored model
  is right: its parts, joints, mechanism and parameters. Failing whole
  components (or several) would treat a candidate who mis-read one view as
  if the lift did not work.
- **How it's detected.** The lift is close to symmetric, so a mirror image
  also fits the reference "well" without reflection (4.6 mm here, inside
  tolerance). The signal is that the reflected fit is *decisively better*
  (2.8 mm). The rule is reflected < 0.85 × direct. The reference against a
  fresh sampling of itself gives 2.3 mm direct and 4.4 mm reflected.

## Attribution choices worth flagging

- **`joint_limits_swapped`** loses on *pose* (the limits push the pantograph
  into a different, folded configuration: 23 mm registered distance, and the
  lift lies on its side) and on *articulation* (the slider sits exactly at
  −130, the edge of its swapped range, so it can only move one way). I read
  "what the limits permit" as "what they permit from where the lift stands".
- **`one_link_missing_joints_bridge_gap`** loses on pose, and on
  articulation for the same parked-at-the-stop reason. The missing part is
  `Washer_A`, about 390 mm³, so the volume-weighted inventory barely sees it.
  The pose (the shackle standing in as a link, height 319 vs 348 mm) does.
  Its joints are mated, because the solver satisfied them, so "joints mated"
  does not punish it.
- **`dummy_joints`** is not charged on signature dimensions for its open
  joints. Pivot spacings are taken from each side's own frame, mated or not.
  Whether joints are mated is that component's business only.
- **Pose vs. signature.** Signature dimensions do not depend on pose, so a
  lift in the wrong configuration is charged once, in geometry, not twice.

## What I let go

- **Joint limit ranges as such.** The reference has no limits at all; only
  what the limits do to the delivered pose is graded.
- **Small extra parts.** The inventory is volume-weighted, so two spring pins
  (292 mm³ each) are close to free.
- **Modelling style.** There is no requirement on a feature count, on
  spreadsheets versus sketch constraints, or on a minimum number of joints.
  The census gives full marks at 80 % of the reference.
- **The LLM judge.** The harness is fully deterministic. Everything the brief
  asks for turned out to be measurable, and a judge would add variance
  without adding evidence. `requires_llm` in `task.toml` is now `false`.

## Things I found in the corpus

- **The reference may have the wrong ⌀33.7 tube.** The drawing calls for
  `Bar ⌀33,7, L=50`, 0.22 kg (≈ 28 000 mm³ of steel). The reference's
  tube is 54 mm long and 24 552 mm³; the mirrored, swapped and dummy
  examples have 58.4 mm and 27 409 mm³, closer to the BOM mass. I grade
  against the reference as instructed, which is where those three models'
  0.06 on signature dimensions and ~0.03 on geometry comes from. Worth a
  look.
- **The reference lacks two parts on the BOM.** It has no
  `Spring-typ pin ISO 8752 6×16` (qty 2); the examples include them.
- **`task.toml` notes are out of date.** They describe adversaries
  (`double_scale`, `decorative_dims`, `broken_mates`, …) that are not in the
  repo and not in the asset manifest.

## Measurement stage: implementation notes

- **What "delivered" means.** It is the root assembly or assemblies, the
  ones not pulled in by another assembly or `AssemblyLink`. The examples keep
  their source sub-assemblies parked beside the root, so counting every body
  in the file would count the lift twice.
- **Parts are addressed by their path from the root.** An `App::Link` inside
  a flexible `AssemblyLink` has no global placement of its own, so shapes
  come from `Part.getShape(root, path, transform=True)`. The resulting shape
  matches the root assembly's own shape exactly.
- **Why dragging is the articulation probe.** In FreeCAD 1.1,
  `AssemblyObject.solve()` ignores joint limits, so they cannot pin a joint.
  The kinematic Simulation does not generate without the GUI module, and
  temporary Distance/Angle joints do not express "move along this axis".
- **The drive probe recomputes only the owning body.** A whole-document
  recompute re-solves every linked assembly, about 25 s per probe on the
  nested examples. Parameters are visited round-robin across sketches, so a
  cut budget still covers every body.
- **Time budget.** The probes share a budget (`FC_STAGE_BUDGET_S`, default
  240 s) and restore everything they touch.

## Running it

```bash
# grade a candidate (needs FREECAD_CMD -> freecadcmd 1.1)
python3 tests/task/harness/harness.py path/to/candidate.FCStd

# re-freeze the reference measurement after changing measure_stage.py
python3 tests/task/harness/harness.py --freeze

# score an existing stage output dir (measure.json + delivered.stl), no FreeCAD needed
python3 tests/task/harness/harness.py --score-from OUT_DIR [--ref-from REF_OUT_DIR]
```

`tests/task/reference/baseline.json` is the reference measured by the stage,
plus 12k surface points. It is keyed to a hash of `measure_stage.py`: when
the stage changes, the harness says the baseline is stale and measures the
reference live, which is slower but never wrong.

## Files changed

- `tests/task/harness/measure_stage.py` — the measurement stage.
- `tests/task/harness/harness.py` — the scoring.
- `tests/task/reference/baseline.json` — new: the frozen reference measurement.
- `task.toml` — `requires_llm = false`.
