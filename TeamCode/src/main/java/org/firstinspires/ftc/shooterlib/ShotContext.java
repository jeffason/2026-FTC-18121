package org.firstinspires.ftc.shooterlib;

/** Everything a velocity modifier or fire gate is allowed to know about the current shot. */
public final class ShotContext {
    public final double rangeMeters;   // output of the range filters
    public final int tagCount;         // tags used in the most recent observation
    public final long tagAgeMs;        // time since a tag was last seen
    public final double tableRpm;      // raw calibration-table value for this range
    public final double targetRpm;     // after the velocity modifiers; NaN while the modifiers themselves run
    public final double actualRpm;     // measured flywheel speed, NaN if unknown
    public final double batteryVolts;  // NaN if unknown

    public ShotContext(double rangeMeters, int tagCount, long tagAgeMs, double tableRpm,
                       double targetRpm, double actualRpm, double batteryVolts) {
        this.rangeMeters = rangeMeters; this.tagCount = tagCount; this.tagAgeMs = tagAgeMs;
        this.tableRpm = tableRpm; this.targetRpm = targetRpm; this.actualRpm = actualRpm; this.batteryVolts = batteryVolts;
    }

    ShotContext withTarget(double target) {
        return new ShotContext(rangeMeters, tagCount, tagAgeMs, tableRpm, target, actualRpm, batteryVolts);
    }
}
