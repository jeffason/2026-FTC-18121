package org.firstinspires.ftc.shooterlib;

import java.util.List;

/**
 * The shooter decision logic, with no FTC SDK dependencies so it runs in plain JUnit on a
 * laptop. Same algorithm as the "Your brain" lane in the Shooter Brain Testbed page.
 *
 * Every frame:   observe(tagRanges)  ->  range filters  ->  range estimate
 * Every loop:    evaluate(actualRpm) ->  table lookup   ->  velocity modifiers  ->  target rpm
 *                                    ->  fire gates     ->  fire / hold reason
 *
 * With an empty {@link Pipeline} this is the baseline: first tag, table value, always fire.
 */
public final class ShooterBrain {

    /** Result of evaluate(). holdReason is empty when fire is true. */
    public static final class Decision {
        public final boolean fire;
        public final String holdReason;
        public final double rangeMeters;   // NaN when no target has been seen yet
        public final double tableRpm;      // NaN when no target has been seen yet
        public final double targetRpm;     // NaN when no target has been seen yet

        Decision(boolean fire, String holdReason, double rangeMeters, double tableRpm, double targetRpm) {
            this.fire = fire; this.holdReason = holdReason; this.rangeMeters = rangeMeters;
            this.tableRpm = tableRpm; this.targetRpm = targetRpm;
        }
    }

    private final CalibrationTable table;
    private final Pipeline pipeline;
    private double estimate = Double.NaN;
    private long lastSeenMs = Long.MIN_VALUE;
    private int lastTagCount = 0;

    public ShooterBrain(CalibrationTable table, Pipeline pipeline) { this.table = table; this.pipeline = pipeline; }

    /** Baseline brain: no stages. */
    public ShooterBrain(CalibrationTable table) { this(table, new Pipeline()); }

    public Pipeline getPipeline() { return pipeline; }
    public CalibrationTable getTable() { return table; }

    /**
     * Feed one camera frame. Pass an empty list when no goal tag was visible; the brain keeps
     * its last estimate and the RecentTagGate decides whether that is still trustworthy.
     */
    public void observe(List<Double> tagRangesMeters, long nowMs) {
        if (tagRangesMeters == null) return;
        List<Double> clean = new java.util.ArrayList<>();
        for (Double r : tagRangesMeters) if (r != null && Double.isFinite(r) && r > 0) clean.add(r);
        if (clean.isEmpty()) return;
        RangeFrame frame = new RangeFrame(clean, nowMs);
        estimate = pipeline.filterRange(frame, clean.get(0));
        lastSeenMs = nowMs;
        lastTagCount = frame.tags.size();
    }

    public boolean hasTarget() { return !Double.isNaN(estimate); }
    public double getRangeMeters() { return estimate; }
    public long getLastSeenMs() { return lastSeenMs; }
    public int getLastTagCount() { return lastTagCount; }

    /** Raw calibration-table value for the current range, before any modifier. NaN until a target has been seen. */
    public double getTableRpm() { return hasTarget() ? table.rpmFor(estimate) : Double.NaN; }

    /** Table value run through the velocity modifiers. NaN until a target has been seen. */
    public double computeTargetRpm(double actualRpm, long nowMs, double batteryVolts) {
        if (!hasTarget()) return Double.NaN;
        double t = getTableRpm();
        return pipeline.adjustRpm(context(t, Double.NaN, actualRpm, nowMs, batteryVolts), t);
    }

    /** Convenience for callers without flywheel or battery feedback (tests, calibration readouts). */
    public double getTargetRpm() { return computeTargetRpm(Double.NaN, lastSeenMs, Double.NaN); }

    public Decision evaluate(double actualRpm, long nowMs) { return evaluate(actualRpm, nowMs, Double.NaN); }

    /** Should we fire right now, given the measured flywheel speed? */
    public Decision evaluate(double actualRpm, long nowMs, double batteryVolts) {
        if (!hasTarget()) { pipeline.noTarget(); return new Decision(false, "no target yet", Double.NaN, Double.NaN, Double.NaN); }
        double tableRpm = getTableRpm();
        double target = pipeline.adjustRpm(context(tableRpm, Double.NaN, actualRpm, nowMs, batteryVolts), tableRpm);
        String hold = pipeline.checkGates(context(tableRpm, target, actualRpm, nowMs, batteryVolts));
        return new Decision(hold.isEmpty(), hold, estimate, tableRpm, target);
    }

    /** Forget the target, e.g. after the robot turns away. */
    public void reset() { estimate = Double.NaN; lastSeenMs = Long.MIN_VALUE; lastTagCount = 0; pipeline.reset(); }

    private ShotContext context(double tableRpm, double targetRpm, double actualRpm, long nowMs, double volts) {
        return new ShotContext(estimate, lastTagCount, nowMs - lastSeenMs, tableRpm, targetRpm, actualRpm, volts);
    }
}
