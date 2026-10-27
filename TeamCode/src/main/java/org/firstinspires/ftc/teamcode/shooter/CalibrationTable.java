package org.firstinspires.ftc.teamcode.shooter;

/** Distance (m) -> flywheel rpm, with linear interpolation between measured points. */
public final class CalibrationTable {
    public final double[] meters, rpm;

    public CalibrationTable(double[] meters, double[] rpm) {
        if (meters.length != rpm.length || meters.length < 2)
            throw new IllegalArgumentException("Calibration table needs at least 2 matching points");
        for (int i = 1; i < meters.length; i++)
            if (meters[i] <= meters[i - 1])
                throw new IllegalArgumentException("Calibration distances must be strictly increasing");
        this.meters = meters.clone();
        this.rpm = rpm.clone();
    }

    public static CalibrationTable fromConfig() {
        return new CalibrationTable(ShooterConfig.CALIBRATION_METERS, ShooterConfig.CALIBRATION_RPM);
    }

    public double rpmFor(double rangeMeters) { return interpolate(meters, rpm, rangeMeters); }

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
