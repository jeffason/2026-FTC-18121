package org.firstinspires.ftc.shooterlib;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Deque;
import java.util.List;

/**
 * The library's built-in stages. Each has a twin in the simulator's DEFS object with the same
 * name, tunables and math, and a line or two in parity-vectors.txt.
 *
 * Team stages do not go here: they go in the team's own class (see TeamStages in the team package),
 * which registers them into the same {@link StageRegistry}.
 */
public final class BuiltInStages {

    /** Registers every built-in stage into the given registry and returns it. */
    public static StageRegistry register(StageRegistry r) {
        // range filters
        r.register("OutlierReject", OutlierReject::new);
        r.register("TagMean", TagMean::new);
        r.register("TagMedian", TagMedian::new);
        r.register("RollingAverage", RollingAverage::new);
        // velocity modifiers
        r.register("RangeTrim", RangeTrim::new);
        r.register("FarShotBoost", FarShotBoost::new);
        r.register("ManualOffset", ManualOffset::new);
        // fire gates
        r.register("TagCountGate", TagCountGate::new);
        r.register("RecentTagGate", RecentTagGate::new);
        r.register("VelocityToleranceGate", VelocityToleranceGate::new);
        return r;
    }

    // =========================== range filters ===========================

    /** Drops tags farther than maxDevM from the median of this frame, then returns the mean of the survivors. */
    public static final class OutlierReject implements RangeFilter {
        final Tunable maxDev = new Tunable("maxDevM", 0.30, 0.05, 1.0, 0.05);
        @Override public String name() { return "OutlierReject"; }
        @Override public List<Tunable> tunables() { return Collections.singletonList(maxDev); }
        @Override public double filterRange(RangeFrame f, double rangeIn) {
            if (f.tags.size() < 3) return rangeIn;                 // can't tell which one is the liar
            double med = median(f.tags);
            List<Double> keep = new ArrayList<>();
            for (double t : f.tags) if (Math.abs(t - med) <= maxDev.get()) keep.add(t);
            if (keep.isEmpty()) return rangeIn;
            f.tags.clear(); f.tags.addAll(keep);
            return mean(keep);
        }
    }

    /** Average of every visible tag. Four tags cut random noise roughly in half. */
    public static final class TagMean implements RangeFilter {
        @Override public String name() { return "TagMean"; }
        @Override public List<Tunable> tunables() { return Collections.emptyList(); }
        @Override public double filterRange(RangeFrame f, double rangeIn) { return mean(f.tags); }
    }

    /** Median of every visible tag. Like the mean, but one bad tag cannot drag it. */
    public static final class TagMedian implements RangeFilter {
        @Override public String name() { return "TagMedian"; }
        @Override public List<Tunable> tunables() { return Collections.emptyList(); }
        @Override public double filterRange(RangeFrame f, double rangeIn) { return median(f.tags); }
    }

    /** Average of the last N frames' estimates. Smoother, but lags a moving robot. */
    public static final class RollingAverage implements RangeFilter {
        final Tunable frames = new Tunable("frames", 5, 1, 30, 1);
        private final Deque<Double> hist = new ArrayDeque<>();
        @Override public String name() { return "RollingAverage"; }
        @Override public List<Tunable> tunables() { return Collections.singletonList(frames); }
        @Override public void reset() { hist.clear(); }
        @Override public double filterRange(RangeFrame f, double rangeIn) {
            hist.addLast(rangeIn);
            while (hist.size() > (int) frames.get()) hist.removeFirst();
            return mean(hist);
        }
    }

    // ========================== velocity modifiers ==========================

    /** Adds slope × (range − pivot). Fixes a table that is right near the pivot but drifts with distance. */
    public static final class RangeTrim implements VelocityModifier {
        final Tunable slope = new Tunable("slopeRpmPerM", 0, -200, 200, 5);
        final Tunable pivot = new Tunable("pivotM", 3.0, 1, 5, 0.25);
        @Override public String name() { return "RangeTrim"; }
        @Override public List<Tunable> tunables() { return Arrays.asList(slope, pivot); }
        @Override public double adjustRpm(ShotContext c, double rpm) { return rpm + slope.get() * (c.rangeMeters - pivot.get()); }
    }

    /** Beyond startM, adds pctPerM percent per extra meter. Compensates for air drag on long shots. */
    public static final class FarShotBoost implements VelocityModifier {
        final Tunable start = new Tunable("startM", 2.5, 1, 5, 0.25);
        final Tunable pct = new Tunable("pctPerM", 2.0, 0, 10, 0.25);
        @Override public String name() { return "FarShotBoost"; }
        @Override public List<Tunable> tunables() { return Arrays.asList(start, pct); }
        @Override public double adjustRpm(ShotContext c, double rpm) {
            double extra = Math.max(0, c.rangeMeters - start.get());
            return rpm * (1 + extra * pct.get() / 100.0);
        }
    }

    /** Flat driver trim, meant to be nudged from the gamepad during a match. */
    public static final class ManualOffset implements VelocityModifier {
        final Tunable offset = new Tunable("offsetRpm", 0, -400, 400, 10);
        @Override public String name() { return "ManualOffset"; }
        @Override public List<Tunable> tunables() { return Collections.singletonList(offset); }
        @Override public double adjustRpm(ShotContext c, double rpm) { return rpm + offset.get(); }
    }

    // ============================= fire gates =============================

    /** Hold unless at least minTags tags went into the current estimate. */
    public static final class TagCountGate implements FireGate {
        final Tunable minTags = new Tunable("minTags", 2, 1, 4, 1);
        @Override public String name() { return "TagCountGate"; }
        @Override public List<Tunable> tunables() { return Collections.singletonList(minTags); }
        @Override public String check(ShotContext c) { return c.tagCount >= (int) minTags.get() ? "" : "only " + c.tagCount + " tag(s)"; }
    }

    /** Hold if the last tag sighting is older than maxAgeMs. Stops shots at where the goal used to be. */
    public static final class RecentTagGate implements FireGate {
        final Tunable maxAge = new Tunable("maxAgeMs", 300, 50, 2000, 25);
        @Override public String name() { return "RecentTagGate"; }
        @Override public List<Tunable> tunables() { return Collections.singletonList(maxAge); }
        @Override public String check(ShotContext c) { return c.tagAgeMs <= maxAge.get() ? "" : "stale target (" + c.tagAgeMs + " ms)"; }
    }

    /** Hold until the measured flywheel speed is within tolRpm of the target. */
    public static final class VelocityToleranceGate implements FireGate {
        final Tunable tol = new Tunable("tolRpm", 60, 5, 400, 5);
        @Override public String name() { return "VelocityToleranceGate"; }
        @Override public List<Tunable> tunables() { return Collections.singletonList(tol); }
        @Override public String check(ShotContext c) {
            if (Double.isNaN(c.actualRpm)) return "no flywheel reading";
            double dv = c.actualRpm - c.targetRpm;
            return Math.abs(dv) <= tol.get() ? "" : String.format("flywheel %+.0f rpm", dv);
        }
    }

    // ---------- helpers ----------
    static double mean(Iterable<Double> xs) { double s = 0; int n = 0; for (double x : xs) { s += x; n++; } return n == 0 ? Double.NaN : s / n; }
    static double median(List<Double> xs) {
        List<Double> s = new ArrayList<>(xs); Collections.sort(s); int n = s.size();
        return n == 0 ? Double.NaN : n % 2 == 1 ? s.get(n / 2) : (s.get(n / 2 - 1) + s.get(n / 2)) / 2.0;
    }

    private BuiltInStages() {}
}
