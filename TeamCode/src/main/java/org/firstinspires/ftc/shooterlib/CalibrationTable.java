package org.firstinspires.ftc.shooterlib;

/** Distance (m) -> flywheel rpm, with linear interpolation between measured points. */
public final class CalibrationTable {
    private final double[] meters, rpm;

    public CalibrationTable(double[] meters, double[] rpm) {
        if (meters.length != rpm.length || meters.length < 2)
            throw new IllegalArgumentException("Calibration table needs at least 2 matching points");
        for (int i = 0; i < meters.length; i++)
            if (!Double.isFinite(meters[i]) || !Double.isFinite(rpm[i]) || meters[i] <= 0 || rpm[i] < 0)
                throw new IllegalArgumentException("Calibration values must be finite, distances > 0 and rpm >= 0");
        for (int i = 1; i < meters.length; i++)
            if (meters[i] <= meters[i - 1])
                throw new IllegalArgumentException("Calibration distances must be strictly increasing");
        this.meters = meters.clone();
        this.rpm = rpm.clone();
    }

    /** Copies, so a caller cannot break the validated ordering. */
    public double[] meters() { return meters.clone(); }
    public double[] rpm() { return rpm.clone(); }
    public int size() { return meters.length; }

    /** NaN in, NaN out; a non-finite range is not a shot. */
    public double rpmFor(double rangeMeters) { return Double.isFinite(rangeMeters) ? interpolate(meters, rpm, rangeMeters) : Double.NaN; }

    /** Piecewise-linear lookup, clamped to the table ends. xs must be strictly increasing. */
    public static double interpolate(double[] xs, double[] ys, double x) {
        if (x <= xs[0]) return ys[0];
        int last = xs.length - 1;
        if (x >= xs[last]) return ys[last];
        for (int i = 1; i <= last; i++) {
            if (x <= xs[i]) {
                double f = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
                return ys[i - 1] + f * (ys[i] - ys[i - 1]);
            }
        }
        return ys[last];
    }
}
