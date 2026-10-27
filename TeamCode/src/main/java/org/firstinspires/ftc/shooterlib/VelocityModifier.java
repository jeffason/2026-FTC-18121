package org.firstinspires.ftc.shooterlib;

/** Nudges the flywheel target. The first modifier receives the raw calibration-table rpm. */
public interface VelocityModifier extends Stage {
    /** Return the adjusted target rpm. Return rpmIn unchanged to be a no-op. */
    double adjustRpm(ShotContext ctx, double rpmIn);
}
