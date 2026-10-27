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
    public void set(double v) { value = clamp(v); }
    public void nudge(int steps) { set(value + steps * step); }

    private double clamp(double v) { return Math.max(min, Math.min(max, v)); }

    @Override public String toString() { return String.format("%s=%.2f", name, value); }
}
