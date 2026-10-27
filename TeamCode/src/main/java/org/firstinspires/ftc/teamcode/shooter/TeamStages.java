package org.firstinspires.ftc.teamcode.shooter;

import org.firstinspires.ftc.shooterlib.StageRegistry;

/**
 * The team's own stages. Write a stage here (or in its own file in this package), register it
 * below, add its twin to the DEFS object in TeamCode/sim/shooter-brain-testbed.html with the
 * same name and tunables, and add a line to TeamCode/sim/parity-vectors.txt. The tests then
 * check the two sides agree.
 *
 * A stage is one of three kinds:
 *   RangeFilter      filterRange(RangeFrame frame, double rangeIn)  -> a better range in meters
 *   VelocityModifier adjustRpm(ShotContext ctx, double rpmIn)       -> a better rpm
 *   FireGate         check(ShotContext ctx)                         -> "" to fire, or a reason to hold
 *
 * Template (copy, rename, fill in):
 *
 *   public static final class WindDrift implements VelocityModifier {
 *       final Tunable gust = new Tunable("gustRpm", 0, -200, 200, 10);
 *       @Override public String name() { return "WindDrift"; }
 *       @Override public List<Tunable> tunables() { return Collections.singletonList(gust); }
 *       @Override public double adjustRpm(ShotContext c, double rpm) { return rpm + gust.get(); }
 *   }
 *   ... and in register():  r.register("WindDrift", WindDrift::new);
 */
public final class TeamStages {

    /** Registers every team stage into the given registry and returns it. */
    public static StageRegistry register(StageRegistry r) {
        // r.register("WindDrift", WindDrift::new);
        return r;
    }

    private TeamStages() {}
}
