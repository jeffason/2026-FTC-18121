package org.firstinspires.ftc.teamcode.shooter;

import java.util.List;

/**
 * One step of the shooter pipeline. A stage is one of three kinds, each with its own method:
 *
 *   {@link RangeFilter}      tag ranges in  -> range estimate out      (what distance are we at?)
 *   {@link VelocityModifier} rpm in         -> rpm out                 (what speed does that need?)
 *   {@link FireGate}         shot context   -> "" or a reason to hold  (is it safe to fire now?)
 *
 * Every stage has a name, can be switched off, and exposes {@link Tunable}s for live tweaking.
 * The {@link Pipeline} runs enabled stages in order, kind by kind. With no stages at all the brain
 * uses the first tag it sees, looks the range up in the table, and always fires: that is the baseline.
 *
 * To add your own stage: implement one of the three interfaces, register it in
 * {@link ShooterStages#REGISTRY}, and mirror it in the sim's STAGES array.
 */
public interface Stage {
    /** Registry name, also used in the pipeline spec string. Letters and digits only. */
    String name();

    /** Live-adjustable parameters. Tunable names are identifiers (e.g. "maxAgeMs") so they can appear in a spec. */
    List<Tunable> tunables();

    default Tunable tunable(String tunableName) {
        for (Tunable t : tunables()) if (t.name.equals(tunableName)) return t;
        return null;
    }
}
