package org.firstinspires.ftc.teamcode.shooter;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import org.junit.Test;

/**
 * Laptop tests for the shooter brain and its pipeline stages.
 * Run with: ./gradlew :TeamCode:testDebugUnitTest   (no robot or Android device needed)
 */
public class ShooterBrainTest {

    private static final CalibrationTable TABLE = new CalibrationTable(
            new double[] { 1.0, 2.0, 3.0, 4.0, 5.0 }, new double[] { 1750, 2080, 2430, 2750, 3030 });

    private static ShooterBrain brain(String spec) {
        return new ShooterBrain(TABLE, Pipeline.fromSpec(spec, ShooterStages.REGISTRY));
    }

    // ---------- calibration table ----------

    @Test public void interpolationHitsTablePointsAndMidpoints() {
        assertEquals(2430, TABLE.rpmFor(3.0), 1e-9);
        assertEquals((2080 + 2430) / 2.0, TABLE.rpmFor(2.5), 1e-9);
    }

    @Test public void interpolationClampsOutsideTable() {
        assertEquals(1750, TABLE.rpmFor(0.2), 1e-9);
        assertEquals(3030, TABLE.rpmFor(9.0), 1e-9);
    }

    @Test(expected = IllegalArgumentException.class)
    public void unsortedCalibrationIsRejected() {
        new CalibrationTable(new double[] { 1, 3, 2 }, new double[] { 1, 2, 3 });
    }

    // ---------- baseline (empty pipeline) ----------

    @Test public void baselineUsesFirstTagAndAlwaysFires() {
        ShooterBrain b = new ShooterBrain(TABLE);
        b.observe(Arrays.asList(4.0, 3.0, 3.0, 3.0), 0);
        assertEquals(4.0, b.getRangeMeters(), 1e-9);
        ShooterBrain.Decision d = b.evaluate(0, 99999);
        assertTrue(d.fire);
        assertEquals(2750, d.targetRpm, 1e-9);
    }

    @Test public void noTargetNeverFires() {
        ShooterBrain.Decision d = new ShooterBrain(TABLE).evaluate(2500, 0);
        assertFalse(d.fire);
        assertEquals("no target yet", d.holdReason);
    }

    @Test public void emptyFrameKeepsLastEstimate() {
        ShooterBrain b = new ShooterBrain(TABLE);
        b.observe(Collections.singletonList(2.5), 0);
        b.observe(Collections.emptyList(), 33);
        assertEquals(2.5, b.getRangeMeters(), 1e-9);
        assertEquals(0, b.getLastSeenMs());
    }

    // ---------- range filters ----------

    @Test public void tagMeanAndMedian() {
        List<Double> tags = Arrays.asList(3.9, 3.0, 3.1, 3.0);
        ShooterBrain mean = brain("TagMean"), median = brain("TagMedian");
        mean.observe(tags, 0); median.observe(tags, 0);
        assertEquals(3.25, mean.getRangeMeters(), 1e-9);
        assertEquals(3.05, median.getRangeMeters(), 1e-9);
    }

    @Test public void outlierRejectDropsTheLiarThenAverages() {
        ShooterBrain b = brain("OutlierReject(maxDevM=0.3)");
        b.observe(Arrays.asList(3.0, 3.02, 2.98, 3.8), 0);
        assertEquals(3.0, b.getRangeMeters(), 1e-9);
        assertEquals(3, b.getLastTagCount());
        b.observe(Arrays.asList(3.0, 3.8), 33);          // two tags: cannot tell which is wrong, keep first
        assertEquals(3.0, b.getRangeMeters(), 1e-9);
        assertEquals(2, b.getLastTagCount());
    }

    @Test public void rollingAverageAveragesFramesAndDropsOld() {
        ShooterBrain b = brain("RollingAverage(frames=3)");
        b.observe(Collections.singletonList(1.0), 0);
        b.observe(Collections.singletonList(2.0), 33);
        b.observe(Collections.singletonList(3.0), 66);
        assertEquals(2.0, b.getRangeMeters(), 1e-9);
        b.observe(Collections.singletonList(6.0), 99);
        assertEquals(11.0 / 3.0, b.getRangeMeters(), 1e-9);
    }

    @Test public void filtersRunInOrder() {
        // median first, then rolling average over 2 frames
        ShooterBrain b = brain("TagMedian; RollingAverage(frames=2)");
        b.observe(Arrays.asList(2.0, 2.0, 9.0), 0);      // median 2.0
        b.observe(Arrays.asList(4.0, 4.0, 0.5), 33);     // median 4.0 -> average 3.0
        assertEquals(3.0, b.getRangeMeters(), 1e-9);
    }

    // ---------- velocity modifiers ----------

    @Test public void modifiersCascadeInOrderAndRespectEnableFlags() {
        ShooterBrain b = brain("ManualOffset(offsetRpm=100); FarShotBoost(startM=2, pctPerM=10); !RangeTrim(slopeRpmPerM=100, pivotM=1)");
        b.observe(Collections.singletonList(3.0), 0);
        assertEquals(2430, b.getTableRpm(), 1e-9);
        assertEquals((2430 + 100) * 1.1, b.getTargetRpm(), 1e-6);          // offset, then +10% for 1 m past 2 m
        Pipeline p = b.getPipeline();
        p.setEnabled(p.stages().get(2), true);                              // + 100 * (3 - 1)
        assertEquals((2430 + 100) * 1.1 + 200, b.getTargetRpm(), 1e-6);
    }

    @Test public void brokenStageOutputIsIgnored() {
        VelocityModifier broken = new VelocityModifier() {
            @Override public String name() { return "Broken"; }
            @Override public List<Tunable> tunables() { return Collections.emptyList(); }
            @Override public double adjustRpm(ShotContext ctx, double rpmIn) { return Double.NaN; }
        };
        ShooterBrain b = new ShooterBrain(TABLE, new Pipeline().add(broken));
        b.observe(Collections.singletonList(3.0), 0);
        assertEquals(2430, b.getTargetRpm(), 1e-9);
        assertEquals("ignored (bad output)", b.getPipeline().lastEffects().get(broken));
    }

    // ---------- fire gates ----------

    @Test public void recentTagGateHoldsOnStaleTarget() {
        ShooterBrain b = brain("RecentTagGate(maxAgeMs=300)");
        b.observe(Collections.singletonList(3.0), 1000);
        assertTrue(b.evaluate(2430, 1200).fire);
        ShooterBrain.Decision stale = b.evaluate(2430, 1400);
        assertFalse(stale.fire);
        assertTrue(stale.holdReason.startsWith("stale target"));
    }

    @Test public void velocityToleranceGateHoldsUntilFlywheelIsClose() {
        ShooterBrain b = brain("VelocityToleranceGate(tolRpm=60)");
        b.observe(Collections.singletonList(3.0), 0);    // target 2430
        assertFalse(b.evaluate(2300, 0).fire);
        assertFalse(b.evaluate(2500, 0).fire);
        assertTrue(b.evaluate(2400, 0).fire);
        assertTrue(b.evaluate(2489, 0).fire);
    }

    @Test public void toleranceGateComparesAgainstModifiedTarget() {
        ShooterBrain b = brain("ManualOffset(offsetRpm=200); VelocityToleranceGate(tolRpm=20)");
        b.observe(Collections.singletonList(3.0), 0);    // table 2430, target 2630
        assertFalse(b.evaluate(2430, 0).fire);
        assertTrue(b.evaluate(2630, 0).fire);
    }

    @Test public void tagCountGate() {
        ShooterBrain b = brain("TagCountGate(minTags=2)");
        b.observe(Collections.singletonList(3.0), 0);
        assertEquals("only 1 tag(s)", b.evaluate(2430, 0).holdReason);
        b.observe(Arrays.asList(3.0, 3.0), 33);
        assertTrue(b.evaluate(2430, 33).fire);
    }

    @Test public void firstFailingGateWinsButAllAreEvaluated() {
        ShooterBrain b = brain("RecentTagGate(maxAgeMs=100); VelocityToleranceGate(tolRpm=10)");
        b.observe(Collections.singletonList(3.0), 0);
        ShooterBrain.Decision d = b.evaluate(1000, 500);
        assertTrue(d.holdReason.startsWith("stale target"));
        assertEquals(2, b.getPipeline().lastEffects().size());
        assertTrue(b.getPipeline().lastEffects().get(b.getPipeline().stages().get(1)).startsWith("HOLD flywheel"));
    }

    // ---------- spec string ----------

    @Test public void specRoundTrips() {
        String spec = "OutlierReject(maxDevM=0.3); TagMedian; !FarShotBoost(startM=2.5, pctPerM=2); RecentTagGate(maxAgeMs=300)";
        Pipeline p = Pipeline.fromSpec(spec, ShooterStages.REGISTRY);
        assertEquals(4, p.stages().size());
        assertFalse(p.isEnabled(p.stages().get(2)));
        assertEquals(spec, p.toSpec());
    }

    @Test public void everyRegisteredStageParsesWithDefaults() {
        StringBuilder spec = new StringBuilder();
        for (String name : ShooterStages.REGISTRY.keySet()) spec.append(name).append("; ");
        Pipeline p = Pipeline.fromSpec(spec.toString(), ShooterStages.REGISTRY);
        assertEquals(ShooterStages.REGISTRY.size(), p.stages().size());
        assertEquals(p.toSpec(), Pipeline.fromSpec(p.toSpec(), ShooterStages.REGISTRY).toSpec());
    }

    @Test public void defaultConfigPipelineHoldsStaleTargetsAndUnreadyFlywheel() {
        ShooterBrain b = new ShooterBrain(TABLE, ShooterStages.defaultPipeline());
        b.observe(Arrays.asList(3.0, 3.0, 3.0), 1000);
        double target = b.getTargetRpm();
        assertTrue(b.evaluate(target, 1100).fire);                              // fresh tag, wheel on target
        ShooterBrain.Decision slow = b.evaluate(target - 500, 1100);
        assertFalse(slow.fire);
        assertTrue(slow.holdReason, slow.holdReason.startsWith("flywheel"));      // wheel 500 rpm low
        ShooterBrain.Decision stale = b.evaluate(target, 5000);
        assertFalse(stale.fire);
        assertTrue(stale.holdReason, stale.holdReason.startsWith("stale target")); // tag 4 s old
    }

    @Test public void tunableSnapsToItsStep() {
        Pipeline p = Pipeline.fromSpec("RollingAverage(frames=2.6); TagCountGate(minTags=1.4)", ShooterStages.REGISTRY);
        assertEquals(3, p.stages().get(0).tunable("frames").get(), 1e-9);
        assertEquals(1, p.stages().get(1).tunable("minTags").get(), 1e-9);
        Tunable t = new Tunable("x", 0, 0, 10, 2);
        t.set(Double.NaN);
        assertEquals(0, t.get(), 1e-9);
    }

    @Test public void resetClearsRollingWindow() {
        ShooterBrain b = brain("RollingAverage(frames=5)");
        b.observe(Collections.singletonList(1.0), 0);
        b.observe(Collections.singletonList(1.0), 33);
        b.reset();
        assertFalse(b.hasTarget());
        b.observe(Collections.singletonList(4.0), 66);
        assertEquals(4.0, b.getRangeMeters(), 1e-9);                            // no blend with the forgotten 1.0 m
    }

    @Test public void allTunablesReachesDisabledStages() {
        Pipeline p = Pipeline.fromSpec("TagMedian; !RecentTagGate(maxAgeMs=300)", ShooterStages.REGISTRY);
        assertTrue(p.liveTunables().isEmpty());
        assertEquals(1, p.allTunables().size());
        assertEquals(p.stages().get(1), p.ownerOf(p.allTunables().get(0)));
    }

    @Test public void calibrationStoreMergesSameSpotAndRoundsDistances() {
        double[][] t = CalibrationStore.normalize(new double[] { 3.004, 1.0, 2.0, 3.0 }, new double[] { 2500, 1750, 2080, 2400 });
        assertEquals(3, t[0].length);
        assertEquals(1.0, t[0][0], 1e-9);
        assertEquals(3.0, t[0][2], 1e-9);
        assertEquals(2450, t[1][2], 1e-9);                                       // two shots at 3.00 m averaged
        new CalibrationTable(t[0], t[1]);                                        // what save() writes always loads
    }

    @Test(expected = IllegalArgumentException.class)
    public void calibrationRejectsNaNDistance() {
        new CalibrationTable(new double[] { 1, Double.NaN, 3 }, new double[] { 1, 2, 3 });
    }

    @Test(expected = IllegalArgumentException.class)
    public void calibrationRejectsNaNRpm() {
        new CalibrationTable(new double[] { 1, 2 }, new double[] { Double.NaN, 2 });
    }

    @Test(expected = IllegalArgumentException.class)
    public void unknownStageIsRejected() { Pipeline.fromSpec("MagicAim", ShooterStages.REGISTRY); }

    @Test(expected = IllegalArgumentException.class)
    public void unknownTunableIsRejected() { Pipeline.fromSpec("TagMean(frames=3)", ShooterStages.REGISTRY); }

    @Test public void tunableClampsAndNudges() {
        Tunable t = new Tunable("x", 5, 0, 10, 2);
        t.nudge(3); assertEquals(10, t.get(), 1e-9);
        t.nudge(-7); assertEquals(0, t.get(), 1e-9);
    }
}
