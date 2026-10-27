package org.firstinspires.ftc.teamcode.shooter;

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
    private boolean calibrationFromHub;
    private long feedUntilMs = 0;

    private static final double FEEDER_REST = 0.0, FEEDER_PUSH = 0.6;
    private static final long FEED_MS = 300;

    @Override
    public void init() {
        fl = hardwareMap.get(DcMotor.class, "front_left_drive");
        fr = hardwareMap.get(DcMotor.class, "front_right_drive");
        bl = hardwareMap.get(DcMotor.class, "back_left_drive");
        br = hardwareMap.get(DcMotor.class, "back_right_drive");
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        flywheel = hardwareMap.get(DcMotorEx.class, ShooterConfig.FLYWHEEL_MOTOR);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        feeder = hardwareMap.tryGet(Servo.class, "feeder");
        battery = hardwareMap.voltageSensor.iterator().hasNext() ? hardwareMap.voltageSensor.iterator().next() : null;

        calibrationFromHub = CalibrationStore.loadIntoConfig();
        ranger = new LimelightRanger(hardwareMap);
        brain = new ShooterBrain(CalibrationTable.fromConfig(), ShooterStages.defaultPipeline());

        telemetry.addData("Calibration", calibrationFromHub ? "loaded from hub" : "ShooterConfig defaults");
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
        if (gamepad1.right_bumper) {
            if (d.fire && now >= feedUntilMs) { feedOneBall(now); shotStatus = "FIRE"; }
            else if (!d.fire) shotStatus = "holding: " + d.holdReason;
            else shotStatus = "feeding";
        }
        if (feeder != null) feeder.setPosition(now < feedUntilMs ? FEEDER_PUSH : FEEDER_REST);

        tweakMenu();
        prev.copy(gamepad1);

        // --- screen ---
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

    private void tweakMenu() {
        List<Tunable> tunables = brain.getPipeline().liveTunables();
        if (gamepad1.dpad_right && !prev.dpad_right) tunableIndex++;
        if (gamepad1.dpad_left && !prev.dpad_left) tunableIndex--;
        if (!tunables.isEmpty()) {
            tunableIndex = ((tunableIndex % tunables.size()) + tunables.size()) % tunables.size();
            Tunable t = tunables.get(tunableIndex);
            if (gamepad1.dpad_up && !prev.dpad_up) t.nudge(+1);
            if (gamepad1.dpad_down && !prev.dpad_down) t.nudge(-1);
        }
        if (gamepad1.left_bumper && !prev.left_bumper) {
            // Toggle the stage that owns the selected tunable (or the first disabled stage if none are live).
            Pipeline pipe = brain.getPipeline();
            Stage target = tunables.isEmpty() ? null : pipe.ownerOf(tunables.get(tunableIndex));
            if (target == null) for (Stage st : pipe.stages()) if (!pipe.isEnabled(st)) { target = st; break; }
            if (target != null) pipe.toggle(target);
        }
    }

    private String tuneLine() {
        List<Tunable> t = brain.getPipeline().liveTunables();
        if (t.isEmpty()) return "no live tunables (LB enables a stage)";
        return "> " + t.get(tunableIndex) + "  (dpad L/R select, U/D nudge, LB toggle)";
    }
}
