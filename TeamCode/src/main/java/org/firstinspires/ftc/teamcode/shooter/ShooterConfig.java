package org.firstinspires.ftc.teamcode.shooter;

import org.firstinspires.ftc.shooterlib.CalibrationStore;
import org.firstinspires.ftc.shooterlib.CalibrationTable;
import org.firstinspires.ftc.shooterlib.Pipeline;
import org.firstinspires.ftc.shooterlib.StageRegistry;

/**
 * The team's shooter settings. The "Export to TeamCode" panel in the Shooter Brain Testbed
 * generates this file; the calibration table gets replaced by the output of the
 * "Shooter: Calibration" OpMode run on the real robot (or loaded from the hub).
 *
 * Library code lives in org.firstinspires.ftc.shooterlib and is not edited for team changes.
 * Team stages live in {@link TeamStages}.
 */
public final class ShooterConfig {

    // --- Pipeline: range filters, velocity modifiers, fire gates, in order. See Pipeline.fromSpec(). ---
    // '!' = present but switched off. Built-in stage names are in BuiltInStages; team stages in TeamStages.
    public static String PIPELINE =
            "OutlierReject(maxDevM=0.3); TagMedian; RollingAverage(frames=5); "
          + "!RangeTrim(slopeRpmPerM=0, pivotM=3); !FarShotBoost(startM=2.5, pctPerM=2); ManualOffset(offsetRpm=0); "
          + "!TagCountGate(minTags=2); RecentTagGate(maxAgeMs=300); VelocityToleranceGate(tolRpm=60)";

    // --- Calibration table: distance to goal (m) -> flywheel rpm. Must be sorted by distance. ---
    // SIM PLACEHOLDER VALUES. Run "Shooter: Calibration" on the robot and paste its output here,
    // or leave these and let loadCalibrationFromHub() override them from the saved robot file.
    public static double[] CALIBRATION_METERS = { 1.00, 1.75, 2.50, 3.25, 4.00, 5.00 };
    public static double[] CALIBRATION_RPM    = { 1751, 2001, 2256, 2512, 2746, 3031 };

    /** The values above as shipped, so a deleted hub table really restores them. */
    static final double[] DEFAULT_CALIBRATION_METERS = CALIBRATION_METERS.clone();
    static final double[] DEFAULT_CALIBRATION_RPM = CALIBRATION_RPM.clone();

    // --- Hardware ---
    public static String FLYWHEEL_MOTOR = "shooter";
    public static double FLYWHEEL_TICKS_PER_REV = 28;      // encoder ticks per flywheel revolution (28 for a bare goBILDA 5202/5203 shaft)
    public static String LIMELIGHT = "limelight";
    public static int LIMELIGHT_PIPELINE = 0;
    public static int[] TARGET_TAG_IDS = { 30, 31, 32, 33 }; // red scoring cluster; blue scoring is 42-45

    /** Built-in stages plus the team's own. */
    public static StageRegistry registry() { return TeamStages.register(StageRegistry.builtIns()); }

    /** The pipeline the robot runs. Throws IllegalArgumentException if PIPELINE names an unknown stage or tunable. */
    public static Pipeline pipeline() { return registry().parse(PIPELINE); }

    public static CalibrationTable table() { return new CalibrationTable(CALIBRATION_METERS, CALIBRATION_RPM); }

    /** Replaces the table with the one saved on the hub, if there is a valid one. CalibrationStore.lastLoadMessage() says what happened. */
    public static boolean loadCalibrationFromHub() {
        CalibrationTable t = CalibrationStore.load();
        if (t == null) return false;
        CALIBRATION_METERS = t.meters();
        CALIBRATION_RPM = t.rpm();
        return true;
    }

    /** Puts the shipped placeholder table back in force for this run. */
    public static void restoreDefaultTable() {
        CALIBRATION_METERS = DEFAULT_CALIBRATION_METERS.clone();
        CALIBRATION_RPM = DEFAULT_CALIBRATION_RPM.clone();
    }

    /** True while the table in use is the sim placeholder rather than one measured on this robot. */
    public static boolean usingPlaceholderTable() {
        return java.util.Arrays.equals(CALIBRATION_METERS, DEFAULT_CALIBRATION_METERS)
            && java.util.Arrays.equals(CALIBRATION_RPM, DEFAULT_CALIBRATION_RPM);
    }

    public static double rpmToTicksPerSecond(double rpm) { return rpm * FLYWHEEL_TICKS_PER_REV / 60.0; }
    public static double ticksPerSecondToRpm(double tps) { return tps * 60.0 / FLYWHEEL_TICKS_PER_REV; }

    private ShooterConfig() {}
}
