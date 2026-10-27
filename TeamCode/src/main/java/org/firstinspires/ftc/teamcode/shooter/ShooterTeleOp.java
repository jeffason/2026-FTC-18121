package org.firstinspires.ftc.teamcode.shooter;

import org.firstinspires.ftc.shooterlib.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.List;
import java.util.Map;

/**
 * Drive + auto-ranged shooter. Everything decided here mirrors the "Your brain" lane in the
 * Shooter Brain Testbed; the sim's Export panel generates ShooterConfig (incl. the PIPELINE spec) for this OpMode.
 * The Driver Station shows every pipeline stage and what it did this loop.
 *
 * Gamepad 1:  sticks = mecanum drive (robot-centric)
 *             right trigger = spin flywheel to the brain's target
 *             right bumper  = request a shot (fires only when the brain's gates pass)
 *             dpad left/right = select a modifier tunable, dpad up/down = nudge it
 *             left bumper   = enable/disable the stage that owns the selected tunable
 */
@TeleOp(name = "Shooter: TeleOp", group = "Shooter")
public class ShooterTeleOp extends OpMode {

    private DcMotor fl, fr, bl, br;
    private DcMotorEx flywheel;
    private Servo feeder;                 // optional: configure a servo named "feeder" to auto-feed
    private VoltageSensor battery;
    private LimelightRanger ranger;
    private ShooterBrain brain;
    private final Gamepad prev = new Gamepad();
    private int tunableIndex = 0;
    private String pipelineError = "";
    private long feedUntilMs = 0;

    private static final double FEEDER_REST = 0.0, FEEDER_PUSH = 0.6;
    private static final long FEED_MS = 300;

    @Override
    public void init() {
        // Motor names match the team's TELE_RoosterDrive_v1 configuration.
        fl = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        fr = hardwareMap.get(DcMotor.class, "frontRightDrive");
        bl = hardwareMap.get(DcMotor.class, "backLeftDrive");
        br = hardwareMap.get(DcMotor.class, "backRightDrive");
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        flywheel = hardwareMap.get(DcMotorEx.class, ShooterConfig.FLYWHEEL_MOTOR);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        feeder = hardwareMap.tryGet(Servo.class, "feeder");
        battery = hardwareMap.voltageSensor.iterator().hasNext() ? hardwareMap.voltageSensor.iterator().next() : null;

        ShooterConfig.loadCalibrationFromHub();
        ranger = new LimelightRanger(hardwareMap, ShooterConfig.LIMELIGHT, ShooterConfig.LIMELIGHT_PIPELINE, ShooterConfig.TARGET_TAG_IDS);
        Pipeline pipeline;
        try { pipeline = ShooterConfig.pipeline(); }
        catch (IllegalArgumentException e) { pipeline = new Pipeline(); pipelineError = e.getMessage(); }
        brain = new ShooterBrain(ShooterConfig.table(), pipeline);
        if (!pipelineError.isEmpty()) telemetry.addLine("PIPELINE SPEC INVALID; shots are blocked until it is fixed: " + pipelineError);

        telemetry.addData("Calibration", CalibrationStore.lastLoadMessage());
        if (ShooterConfig.usingPlaceholderTable())
            telemetry.addLine("UNCALIBRATED: table is the sim placeholder. Run Shooter: Calibration before relying on range shots.");
        telemetry.addData("Feeder", feeder == null ? "none configured (feed by hand)" : "servo ready");
    }

    @Override public void start() { ranger.start(); }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();
        drive();

        // --- perception ---
        List<Double> ranges = ranger.pollRangesMeters();
        if (ranges != null) brain.observe(ranges, now);

        // --- decision ---
        double actualRpm = ShooterConfig.ticksPerSecondToRpm(flywheel.getVelocity());
        double volts = battery == null ? Double.NaN : battery.getVoltage();
        ShooterBrain.Decision d = brain.evaluate(actualRpm, now, volts);

        // --- flywheel ---
        boolean spin = gamepad1.right_trigger > 0.3 && !Double.isNaN(d.targetRpm);
        flywheel.setVelocity(spin ? ShooterConfig.rpmToTicksPerSecond(d.targetRpm) : 0);

        // --- shot request ---
        String shotStatus = "idle";
        if (gamepad1.right_bumper && !pipelineError.isEmpty()) shotStatus = "holding: PIPELINE spec invalid, fix ShooterConfig";
        else if (gamepad1.right_bumper && !spin) shotStatus = "hold right trigger to spin up first";
        else if (gamepad1.right_bumper) {
            if (d.fire && now >= feedUntilMs) { feedOneBall(now); shotStatus = "FIRE"; }
            else if (!d.fire) shotStatus = "holding: " + d.holdReason;
            else shotStatus = "feeding";
        }
        if (feeder != null) feeder.setPosition(now < feedUntilMs ? FEEDER_PUSH : FEEDER_REST);

        tweakMenu();
        prev.copy(gamepad1);

        // --- screen ---
        if (!pipelineError.isEmpty()) telemetry.addLine("PIPELINE SPEC INVALID; shots are blocked until it is fixed: " + pipelineError);
        telemetry.addData("Limelight", ranger.status());
        telemetry.addData("Range", brain.hasTarget() ? String.format("%.2f m", brain.getRangeMeters()) : "no target");
        telemetry.addData("Flywheel", "table %.0f  target %.0f  actual %.0f rpm", d.tableRpm, d.targetRpm, actualRpm);
        telemetry.addData("Shot", shotStatus);
        telemetry.addData("Tune", tuneLine());
        for (Map.Entry<Stage, String> e : brain.getPipeline().lastEffects().entrySet())
            telemetry.addData("  " + e.getKey().name(), e.getValue());
        telemetry.addData("Battery", Double.isNaN(volts) ? "?" : String.format("%.1f V", volts));
    }

    @Override public void stop() { ranger.stop(); flywheel.setVelocity(0); }

    private void feedOneBall(long now) {
        feedUntilMs = now + FEED_MS;
        gamepad1.rumble(100);
        // If you have no feeder servo, this just rumbles; the operator feeds by hand when "FIRE" shows.
    }

    private void drive() {
        double y = -gamepad1.left_stick_y, x = gamepad1.left_stick_x, rx = gamepad1.right_stick_x;
        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        fl.setPower((y + x + rx) / denom);
        bl.setPower((y - x + rx) / denom);
        fr.setPower((y - x - rx) / denom);
        br.setPower((y + x - rx) / denom);
    }

    /** One row of the gamepad menu: a stage, and one of its tunables (null for a stage that has none). */
    private static final class MenuItem {
        final Stage stage; final Tunable tunable;
        MenuItem(Stage stage, Tunable tunable) { this.stage = stage; this.tunable = tunable; }
    }

    /** Every stage appears at least once, enabled or not, so LB can switch any of them. */
    private List<MenuItem> menuItems() {
        List<MenuItem> items = new java.util.ArrayList<>();
        for (Stage s : brain.getPipeline().stages()) {
            if (s.tunables().isEmpty()) items.add(new MenuItem(s, null));
            for (Tunable t : s.tunables()) items.add(new MenuItem(s, t));
        }
        return items;
    }

    private MenuItem selectedItem(List<MenuItem> items) {
        if (items.isEmpty()) return null;
        tunableIndex = ((tunableIndex % items.size()) + items.size()) % items.size();
        return items.get(tunableIndex);
    }

    private void tweakMenu() {
        List<MenuItem> items = menuItems();
        if (gamepad1.dpad_right && !prev.dpad_right) tunableIndex++;
        if (gamepad1.dpad_left && !prev.dpad_left) tunableIndex--;
        MenuItem sel = selectedItem(items);
        if (sel == null) return;
        if (sel.tunable != null) {
            if (gamepad1.dpad_up && !prev.dpad_up) sel.tunable.nudge(+1);
            if (gamepad1.dpad_down && !prev.dpad_down) sel.tunable.nudge(-1);
        }
        if (gamepad1.left_bumper && !prev.left_bumper) brain.getPipeline().toggle(sel.stage);
    }

    private String tuneLine() {
        MenuItem sel = selectedItem(menuItems());
        if (sel == null) return "pipeline has no stages";
        String state = brain.getPipeline().isEnabled(sel.stage) ? "on" : "OFF";
        String what = sel.tunable == null ? "(no tunables)" : sel.tunable.toString();
        return "> " + sel.stage.name() + " [" + state + "] " + what + "  (dpad L/R select, U/D nudge, LB toggle)";
    }
}
