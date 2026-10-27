package org.firstinspires.ftc.teamcode.shooter;

/**
 * Shooter tunables. The "Export to TeamCode" panel in the Shooter Brain Testbed generates this
 * file from the sim settings; the calibration table gets replaced by the output of the
 * "Shooter: Calibration" OpMode run on the real robot (or loaded from the hub by CalibrationStore).
 */
public final class ShooterConfig {

    // --- Pipeline: range filters, velocity modifiers, fire gates, in order. See Pipeline.fromSpec(). ---
    // '!' = present but switched off. Stage names and tunables are defined in ShooterStages.
    public static String PIPELINE =
            "OutlierReject(maxDevM=0.3); TagMedian; RollingAverage(frames=5); "
          + "!RangeTrim(slopeRpmPerM=0, pivotM=3); !FarShotBoost(startM=2.5, pctPerM=2); ManualOffset(offsetRpm=0); "
          + "!TagCountGate(minTags=2); RecentTagGate(maxAgeMs=300); VelocityToleranceGate(tolRpm=60)";

    // --- Calibration table: distance to goal (m) -> flywheel rpm. Must be sorted by distance. ---
    // SIM PLACEHOLDER VALUES. Run "Shooter: Calibration" on the robot and paste its output here,
    // or leave these and let CalibrationStore.loadIntoConfig() override them from the saved robot file.
    public static double[] CALIBRATION_METERS = { 1.00, 1.75, 2.50, 3.25, 4.00, 5.00 };
    public static double[] CALIBRATION_RPM    = { 1751, 2001, 2256, 2512, 2746, 3031 };

    /** The values above as shipped, so CalibrationStore can put them back after a saved table is deleted. */
    static final double[] DEFAULT_CALIBRATION_METERS = CALIBRATION_METERS.clone();
    static final double[] DEFAULT_CALIBRATION_RPM = CALIBRATION_RPM.clone();

    /** True while the table in use is the sim placeholder rather than one measured on this robot. */
    public static boolean usingPlaceholderTable() {
        return java.util.Arrays.equals(CALIBRATION_METERS, DEFAULT_CALIBRATION_METERS)
            && java.util.Arrays.equals(CALIBRATION_RPM, DEFAULT_CALIBRATION_RPM);
    }

    // --- Hardware ---
    public static String FLYWHEEL_MOTOR = "shooter";
    public static double FLYWHEEL_TICKS_PER_REV = 28;      // encoder ticks per flywheel revolution (28 for a bare goBILDA 5202/5203 shaft)
    public static String LIMELIGHT = "limelight";
    public static int LIMELIGHT_PIPELINE = 0;
    public static int[] TARGET_TAG_IDS = { 30, 31, 32, 33 }; // red scoring cluster; blue scoring is 42-45

    public static double rpmToTicksPerSecond(double rpm) { return rpm * FLYWHEEL_TICKS_PER_REV / 60.0; }
    public static double ticksPerSecondToRpm(double tps) { return tps * 60.0 / FLYWHEEL_TICKS_PER_REV; }

    private ShooterConfig() {}
}
