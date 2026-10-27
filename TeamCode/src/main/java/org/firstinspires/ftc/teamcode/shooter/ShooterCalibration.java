package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * First-run calibration. Builds the distance -> flywheel rpm table from real shots.
 *
 * Procedure (one student drives, one catches balls, one reads the screen):
 *   1. Park the robot facing the goal with the tag cluster in view. The screen shows the
 *      Limelight range. Check it against a tape measure the first time.
 *   2. Hold X to spin the flywheel. Dpad up/down changes rpm by 25, bumpers by 100.
 *   3. Feed a ball by hand. Adjust rpm until balls land in the goal.
 *   4. Press A to record (range, rpm). Press B to undo the last point.
 *   5. Move to a new distance and repeat. Five or six points from closest to farthest is plenty.
 *   6. Press Y to save to the Control Hub. The screen prints two Java lines to paste into ShooterConfig.
 *
 * Back clears all points. Start deletes the saved file and puts the ShooterConfig defaults back in force.
 */
@TeleOp(name = "Shooter: Calibration", group = "Shooter")
public class ShooterCalibration extends LinearOpMode {

    private final List<double[]> points = new ArrayList<>();   // {meters, rpm}
    private static final int MAX_POINTS = 12;
    private static final long FRESH_MS = 300;

    /** A point within CalibrationStore.SAME_SPOT_METERS of an existing one replaces it; otherwise it is added, up to MAX_POINTS. */
    private void recordPoint(double meters, double rpm) {
        for (double[] p : points) if (Math.abs(p[0] - meters) < CalibrationStore.SAME_SPOT_METERS) { p[1] = rpm; return; }
        if (points.size() < MAX_POINTS) points.add(new double[] { meters, rpm });
    }

    @Override
    public void runOpMode() {
        LimelightRanger ranger = new LimelightRanger(hardwareMap);
        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, ShooterConfig.FLYWHEEL_MOTOR);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean hadSaved = CalibrationStore.loadIntoConfig();

        // A smooth readout: median of visible tags, averaged over 10 frames, no fire gates.
        ShooterBrain readout = new ShooterBrain(CalibrationTable.fromConfig(),
                Pipeline.fromSpec("TagMedian; RollingAverage(frames=10)", ShooterStages.REGISTRY));
        telemetry.addLine("Shooter calibration ready. Press Play.");
        telemetry.addData("Saved table on hub", hadSaved ? "yes (Start deletes it)" : CalibrationStore.lastLoadMessage());
        telemetry.update();
        waitForStart();
        ranger.start();

        Gamepad prev = new Gamepad();
        double targetRpm = 2000;
        String javaOut = "", hint = "";
        int tagsSeen = 0;
        long lastFrameMs = 0;

        while (opModeIsActive()) {
            long now = System.currentTimeMillis();
            List<Double> ranges = ranger.pollRangesMeters();
            if (ranges != null) { readout.observe(ranges, now); tagsSeen = ranges.size(); lastFrameMs = now; }

            // --- rpm adjust (edge-triggered) ---
            if (gamepad1.dpad_up && !prev.dpad_up) targetRpm += 25;
            if (gamepad1.dpad_down && !prev.dpad_down) targetRpm -= 25;
            if (gamepad1.right_bumper && !prev.right_bumper) targetRpm += 100;
            if (gamepad1.left_bumper && !prev.left_bumper) targetRpm -= 100;
            targetRpm = Math.max(0, targetRpm);

            flywheel.setVelocity(gamepad1.x ? ShooterConfig.rpmToTicksPerSecond(targetRpm) : 0);
            double actualRpm = ShooterConfig.ticksPerSecondToRpm(flywheel.getVelocity());

            // --- record / undo / clear / save ---
            boolean fresh = tagsSeen > 0 && now - lastFrameMs <= FRESH_MS;
            if (gamepad1.a && !prev.a) {
                if (readout.hasTarget() && fresh) { recordPoint(readout.getRangeMeters(), targetRpm); hint = ""; gamepad1.rumble(150); }
                else hint = "Not recorded: no goal tag in the last " + FRESH_MS + " ms. Face the goal and try again.";
            }
            if (gamepad1.b && !prev.b && !points.isEmpty()) points.remove(points.size() - 1);
            if (gamepad1.back && !prev.back) { points.clear(); javaOut = ""; }
            if (gamepad1.start && !prev.start) { CalibrationStore.delete(); javaOut = CalibrationStore.lastLoadMessage(); }
            if (gamepad1.y && !prev.y && points.size() >= 2) {
                double[] m = new double[points.size()], r = new double[points.size()];
                for (int i = 0; i < points.size(); i++) { m[i] = points.get(i)[0]; r[i] = points.get(i)[1]; }
                try {
                    CalibrationStore.save(m, r);
                    CalibrationStore.loadIntoConfig();
                    javaOut = CalibrationStore.asJava(ShooterConfig.CALIBRATION_METERS, ShooterConfig.CALIBRATION_RPM);
                    gamepad1.rumble(500);
                } catch (IllegalArgumentException e) {
                    javaOut = "Not saved: " + e.getMessage();
                }
            }
            prev.copy(gamepad1);

            // --- screen ---
            telemetry.addLine("X hold: spin  |  dpad ±25  bumpers ±100  |  A record  B undo  Y save  Back clear");
            telemetry.addData("Limelight", ranger.status());
            if (!hint.isEmpty()) telemetry.addLine(hint);
            telemetry.addData("Range", readout.hasTarget()
                    ? String.format("%.2f m  (%d tags, last frame %d ms ago)", readout.getRangeMeters(), tagsSeen, now - lastFrameMs)
                    : "no target tags in view");
            telemetry.addData("Flywheel", "target %.0f  actual %.0f rpm", targetRpm, actualRpm);
            telemetry.addData("Points", "%d of %d (same spot replaces)", points.size(), MAX_POINTS);
            List<double[]> sorted = new ArrayList<>(points);
            Collections.sort(sorted, (p, q) -> Double.compare(p[0], q[0]));
            for (double[] p : sorted) telemetry.addLine(String.format("   %.2f m  ->  %.0f rpm", p[0], p[1]));
            if (points.size() >= 2 && javaOut.isEmpty()) telemetry.addLine("Press Y to save and print Java.");
            if (!javaOut.isEmpty()) { telemetry.addLine("Paste into ShooterConfig:"); telemetry.addLine(javaOut); }
            telemetry.update();
        }
        ranger.stop();
    }
}
