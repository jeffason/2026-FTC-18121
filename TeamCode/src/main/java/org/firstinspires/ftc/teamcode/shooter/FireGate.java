package org.firstinspires.ftc.teamcode.shooter;

/** Decides whether it is safe to fire right now. Gates can only hold a shot, never force one. */
public interface FireGate extends Stage {
    /** Return "" to allow the shot, or a short reason (shown on the Driver Station) to hold it. */
    String check(ShotContext ctx);
}
