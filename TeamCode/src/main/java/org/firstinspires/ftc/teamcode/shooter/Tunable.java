package org.firstinspires.ftc.teamcode.shooter;

/** A named number a modifier exposes for live tweaking (gamepad in TeleOp, slider in the sim). */
public final class Tunable {
    public final String name;
    public final double min, max, step;
    private double value;

    public Tunable(String name, double value, double min, double max, double step) {
        this.name = name; this.min = min; this.max = max; this.step = step;
        this.value = clamp(value);
    }

    public double get() { return value; }

    /** Clamps to [min, max] and snaps to the nearest step from min, so an integer tunable stays an integer. */
    public void set(double v) { value = clamp(v); }
    public void nudge(int steps) { set(value + steps * step); }

    private double clamp(double v) {
        if (!Double.isFinite(v)) return value;
        double c = Math.max(min, Math.min(max, v));
        if (step > 0) c = Math.min(max, min + Math.round((c - min) / step) * step);
        return c;
    }

    @Override public String toString() { return String.format("%s=%.2f", name, value); }
}
