# Shooter: Limelight ranging + flywheel velocity

Everything the shooter decides lives in a small **pipeline of stages**. The Limelight gives
us a range to each goal AprilTag, the pipeline turns that into a flywheel speed and a
fire/hold decision, and the same pipeline runs in the browser sim and on the robot.

```
Limelight tags ──► range filters ──► calibration table ──► velocity modifiers ──► target rpm
                                                                               └─► fire gates ──► FIRE / hold reason
```

With **no stages** the brain uses the first tag it sees, looks the range up in the table,
and fires every cycle. That is the baseline. Every stage you add is one idea, with one job.

| Kind | Interface | Signature | Built-ins |
|---|---|---|---|
| range filter | `RangeFilter` | tags this frame + estimate in → estimate out (m) | `OutlierReject`, `TagMean`, `TagMedian`, `RollingAverage` |
| velocity modifier | `VelocityModifier` | shot context + rpm in → rpm out | `RangeTrim`, `FarShotBoost`, `ManualOffset` |
| fire gate | `FireGate` | shot context → `""` or a reason to hold | `TagCountGate`, `RecentTagGate`, `VelocityToleranceGate` |

A pipeline is written as one line, the **spec**, and that string is the contract between the sim and the robot:

```
OutlierReject(maxDevM=0.3); TagMedian; RollingAverage(frames=5); !FarShotBoost(startM=2.5, pctPerM=2); RecentTagGate(maxAgeMs=300); VelocityToleranceGate(tolRpm=60)
```

`;` separates stages, `!` means present but switched off, parentheses hold tunables.

## Files

| File | What it is |
|---|---|
| `ShooterConfig.java` | The tunables: `PIPELINE` spec, calibration table, hardware names. The sim's **Export to TeamCode** panel generates this file. |
| `ShooterBrain.java` | Pure Java decision logic. `observe(tagRanges)` every frame, `evaluate(actualRpm)` every loop. No SDK imports, so it runs in JUnit. |
| `Pipeline.java` | Ordered stages with on/off switches, `fromSpec` / `toSpec`, and `lastEffects()` for telemetry. |
| `Stage`, `RangeFilter`, `VelocityModifier`, `FireGate` | The three stage interfaces and their common base. |
| `ShooterStages.java` | The built-in stages and the `REGISTRY` the spec parser uses. **Add new stages here.** |
| `CalibrationTable.java` | Distance → rpm with linear interpolation. |
| `Tunable.java` | A named, clamped number a stage exposes for live tweaking. |
| `LimelightRanger.java` | The only class that talks to the Limelight. New frame → list of horizontal ranges to the goal tags. |
| `CalibrationStore.java` | Saves/loads the table on the Control Hub and prints it as Java. |
| `ShooterCalibration.java` | OpMode **Shooter: Calibration**. First-run procedure that builds the table from real shots. |
| `ShooterTeleOp.java` | OpMode **Shooter: TeleOp**. Drive + auto-ranged shooter, live stage telemetry, gamepad tunable menu. |
| `../../../../../../../test/java/.../shooter/ShooterBrainTest.java` | Laptop tests. |
| `TeamCode/sim/shooter-brain-testbed.html` | The simulator. Open it in a browser. |
| `TeamCode/sim/parity-vectors.txt` | Known inputs and expected outputs for every stage. The Java tests run them; the sim runs them on load and shows `parity N/N ok` in its header. The sim embeds a copy, and a test fails if the copy drifts from the file. |

## Run the sim locally

Double-click `TeamCode/sim/shooter-brain-testbed.html`. It is one self-contained file: no server, no build.
Your pipeline is remembered in that browser. The published copy is at
<https://claude.ai/artifact/XGNJu4H8anUFcdKBQHtyu9>.

The sim and the Java have the **same stage names, tunables and math**. In the page, each stage is an
entry in the `DEFS` object in the script; on the robot it is a class in `ShooterStages.java`.

## Run the tests

```
./gradlew :TeamCode:testDebugUnitTest
```

Results land in `TeamCode/build/reports/tests/testDebugUnitTest/index.html`. No robot needed.

## Sim → robot

1. Build a pipeline in the sim. Copy Java from the Export panel.
2. Paste it over `ShooterConfig.java` (or just the spec into `PIPELINE`). Build and deploy.
3. Run **Shooter: Calibration** once (see the procedure in its Javadoc). It saves the table on the
   hub and prints two Java lines; paste those into `ShooterConfig` so the table is in git too.
4. Run **Shooter: TeleOp**. Right trigger spins to target, right bumper requests a shot, the Driver
   Station shows each stage and what it did this loop. Dpad selects and nudges tunables, LB toggles a stage.

## Writing a new stage

1. Pick the kind. Implement `RangeFilter`, `VelocityModifier` or `FireGate` in `ShooterStages.java`.
   Name tunables as identifiers (`maxAgeMs`), give them sane min/max/step.
2. Add one line to `REGISTRY`.
3. Add the twin to `DEFS` in the sim with the same name, tunables and math. Try it there first.
4. Add a line or two to `TeamCode/sim/parity-vectors.txt` with a known input and output, and paste the
   file's new contents into the sim's `PARITY_VECTORS` block. `simDefsMatchJavaRegistry` checks names and
   tunables match; `parityVectorsPassInJavaAndMatchTheSimCopy` checks both sides compute the same answers.

Keep stages pure: no hardware, no sleeping. Return `rpmIn` / `rangeIn` unchanged to be a no-op; a
`NaN` is ignored by the pipeline, so a broken stage never breaks the shot.

## Limelight setup

Pipeline type AprilTag, family 36h11, tag size 3.25 in. Under **3D**, enter the camera's offset from
robot centre so "target pose in robot space" is measured from the robot, not the lens. Without it,
robot space equals camera space. Goal tag IDs: red scoring 30–33, blue scoring 42–45 (set `TARGET_TAG_IDS`).

## What the sim does not know yet

The table, launch angle, exit height, goal height and flywheel constants in the sim are placeholders
from drag-free physics. The calibration OpMode is what makes the robot right; the sim is for the logic.
