package org.firstinspires.ftc.teamcode.shooter;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

/**
 * An ordered list of {@link Stage}s with on/off switches. The brain runs the enabled range filters
 * in order, then the enabled velocity modifiers, then the enabled fire gates. After each run,
 * {@link #lastEffects()} says what every stage did, for telemetry.
 *
 * Pipelines are described by a one-line spec so the sim can export one and the robot can parse it:
 *
 *   OutlierReject(maxDevM=0.3); TagMedian; RollingAverage(frames=5); !FarShotBoost(startM=2.5, pctPerM=2); RecentTagGate(maxAgeMs=300)
 *
 * Stages are separated by ';'. A leading '!' means present but switched off. Parentheses hold tunable values.
 */
public final class Pipeline {
    private final List<Stage> stages = new ArrayList<>();
    private final Set<Stage> off = new HashSet<>();
    private final Map<Stage, String> effects = new LinkedHashMap<>();

    public Pipeline add(Stage s) { return add(s, true); }
    public Pipeline add(Stage s, boolean enabled) {
        if (!(s instanceof RangeFilter) && !(s instanceof VelocityModifier) && !(s instanceof FireGate))
            throw new IllegalArgumentException(s.name() + " implements none of RangeFilter, VelocityModifier or FireGate");
        stages.add(s); if (!enabled) off.add(s); return this;
    }

    public List<Stage> stages() { return Collections.unmodifiableList(stages); }
    public boolean isEnabled(Stage s) { return !off.contains(s); }
    public void setEnabled(Stage s, boolean on) { if (on) off.remove(s); else off.add(s); }
    public void toggle(Stage s) { setEnabled(s, !isEnabled(s)); }

    /** Runs the range filters. rangeIn is the first tag's range. */
    public double filterRange(RangeFrame frame, double rangeIn) {
        double r = rangeIn;
        for (Stage s : stages) {
            if (!(s instanceof RangeFilter)) continue;
            if (!isEnabled(s)) { effects.put(s, "off"); continue; }
            double out = ((RangeFilter) s).filterRange(frame, r);
            boolean ok = Double.isFinite(out) && out > 0;
            effects.put(s, ok ? String.format(Locale.US, "%.2f -> %.2f m", r, out) : "ignored (bad output)");
            if (ok) r = out;
        }
        return r;
    }

    /** Runs the velocity modifiers. rpmIn is the calibration-table value. */
    public double adjustRpm(ShotContext ctx, double rpmIn) {
        double rpm = rpmIn;
        for (Stage s : stages) {
            if (!(s instanceof VelocityModifier)) continue;
            if (!isEnabled(s)) { effects.put(s, "off"); continue; }
            double out = ((VelocityModifier) s).adjustRpm(ctx, rpm);
            boolean ok = Double.isFinite(out) && out >= 0;
            effects.put(s, ok ? String.format(Locale.US, "%+.0f rpm", out - rpm) : "ignored (bad output)");
            if (ok) rpm = out;
        }
        return rpm;
    }

    /** Runs every enabled gate (so telemetry shows all of them) and returns the first hold reason, or "". */
    public String checkGates(ShotContext ctx) {
        String hold = "";
        for (Stage s : stages) {
            if (!(s instanceof FireGate)) continue;
            if (!isEnabled(s)) { effects.put(s, "off"); continue; }
            String why = ((FireGate) s).check(ctx);
            if (why == null) why = "";
            effects.put(s, why.isEmpty() ? "pass" : "HOLD " + why);
            if (hold.isEmpty() && !why.isEmpty()) hold = why;
        }
        return hold;
    }

    /** Marks every stage's effect as waiting for a target, so telemetry never shows a verdict from an earlier target. */
    public void noTarget() { for (Stage s : stages) effects.put(s, isEnabled(s) ? "no target" : "off"); }

    /** What each stage did on the most recent run, in pipeline order. */
    public Map<Stage, String> lastEffects() { return Collections.unmodifiableMap(effects); }

    /** Tunables of enabled stages, flattened in pipeline order. */
    public List<Tunable> liveTunables() {
        List<Tunable> out = new ArrayList<>();
        for (Stage s : stages) if (isEnabled(s)) out.addAll(s.tunables());
        return out;
    }

    /** Tunables of every stage, enabled or not, so a gamepad menu can reach and switch on a disabled stage. */
    public List<Tunable> allTunables() {
        List<Tunable> out = new ArrayList<>();
        for (Stage s : stages) out.addAll(s.tunables());
        return out;
    }

    /** Clears every stage's per-run memory. */
    public void reset() { for (Stage s : stages) s.reset(); effects.clear(); }

    public Stage ownerOf(Tunable t) {
        for (Stage s : stages) if (s.tunables().contains(t)) return s;
        return null;
    }

    // ---------- spec string ----------

    public String toSpec() {
        StringBuilder sb = new StringBuilder();
        for (Stage s : stages) {
            if (sb.length() > 0) sb.append("; ");
            if (!isEnabled(s)) sb.append('!');
            sb.append(s.name());
            if (!s.tunables().isEmpty()) {
                sb.append('(');
                boolean first = true;
                for (Tunable t : s.tunables()) {
                    if (!first) sb.append(", ");
                    first = false;
                    sb.append(t.name).append('=').append(fmt(t.get()));
                }
                sb.append(')');
            }
        }
        return sb.toString();
    }

    /** Builds a pipeline from a spec string. Throws IllegalArgumentException on an unknown stage or tunable. */
    public static Pipeline fromSpec(String spec, Map<String, Supplier<Stage>> registry) {
        Pipeline p = new Pipeline();
        if (spec == null || spec.trim().isEmpty()) return p;
        for (String raw : spec.split(";")) {
            String item = raw.trim();
            if (item.isEmpty()) continue;
            boolean enabled = true;
            if (item.startsWith("!")) { enabled = false; item = item.substring(1).trim(); }
            String name = item, args = "";
            int paren = item.indexOf('(');
            if (paren >= 0) {
                if (!item.endsWith(")")) throw new IllegalArgumentException("Missing ')' in: " + raw);
                name = item.substring(0, paren).trim();
                args = item.substring(paren + 1, item.length() - 1);
            }
            Supplier<Stage> factory = registry.get(name);
            if (factory == null) throw new IllegalArgumentException("Unknown stage '" + name + "'. Known: " + registry.keySet());
            Stage s = factory.get();
            for (String kv : args.split(",")) {
                if (kv.trim().isEmpty()) continue;
                String[] parts = kv.split("=");
                if (parts.length != 2) throw new IllegalArgumentException("Expected name=value in: " + raw);
                Tunable t = s.tunable(parts[0].trim());
                if (t == null) throw new IllegalArgumentException("Stage " + name + " has no tunable '" + parts[0].trim() + "'");
                t.set(Double.parseDouble(parts[1].trim()));
            }
            p.add(s, enabled);
        }
        return p;
    }

    private static String fmt(double v) {
        String s = String.format(Locale.US, "%.3f", v);
        s = s.replaceAll("0+$", "").replaceAll("\\.$", "");
        return s.isEmpty() || s.equals("-") ? "0" : s;
    }
}
