package org.firstinspires.ftc.shooterlib;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.ArrayList;
import java.util.List;

/**
 * The only class that talks to the Limelight. Turns each new camera frame into a list of
 * horizontal ranges (meters) to the goal's AprilTags, which is exactly what
 * {@link ShooterBrain#observe} wants.
 *
 * Setup on the Limelight web UI: pipeline type AprilTag, family 36h11, tag size 3.25 in,
 * and set the camera's offset from robot center under "3D" so "target pose in robot space"
 * is measured from the robot, not the lens. Without that offset, robot space equals camera space.
 */
public final class LimelightRanger {
    private final Limelight3A limelight;
    private final int[] tagIds;
    private long lastStamp = Long.MIN_VALUE;
    private LLResult latest;
    private String lastStatus = "no result yet";
    private long lastFrameMs = 0;

    public LimelightRanger(HardwareMap hw, String deviceName, int pipeline, int[] tagIds) {
        limelight = hw.get(Limelight3A.class, deviceName);
        limelight.pipelineSwitch(pipeline);
        this.tagIds = tagIds.clone();
    }

    public void start() { limelight.setPollRateHz(50); limelight.start(); }
    public void stop() { limelight.stop(); }
    public boolean isConnected() { return limelight.isConnected(); }
    public LLResult latestResult() { return latest; }

    /** One line for the Driver Station: connection, then what the last poll produced. */
    public String status() {
        String age = lastFrameMs == 0 ? "" : " · " + (System.currentTimeMillis() - lastFrameMs) + " ms since a new frame";
        return (isConnected() ? "" : "NOT CONNECTED · ") + lastStatus + age;
    }

    /**
     * Call every loop. Returns null when the Limelight has not produced a new frame since the
     * last call (so the brain's rolling window is not fed duplicates), otherwise the ranges of
     * the wanted tags seen in that frame, possibly empty.
     */
    public List<Double> pollRangesMeters() {
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) { lastStatus = "no valid result (is the Limelight pipeline type AprilTag?)"; return null; }
        long stamp = r.getControlHubTimeStamp();
        if (stamp == lastStamp) return null;
        lastStamp = stamp;
        latest = r;
        lastFrameMs = System.currentTimeMillis();

        List<Double> out = new ArrayList<>();
        for (LLResultTypes.FiducialResult fr : r.getFiducialResults()) {
            if (!wanted(fr.getFiducialId())) continue;
            Pose3D pose = fr.getTargetPoseRobotSpace();
            if (pose == null) continue;
            Position p = pose.getPosition();
            double x = p.unit.toMeters(p.x), y = p.unit.toMeters(p.y);
            double range = Math.hypot(x, y);           // horizontal distance; ignore height
            if (range > 0.05) out.add(range);          // 0,0 means "no 3D solve"
        }
        int all = r.getFiducialResults().size();
        lastStatus = out.size() + " goal tag(s) of " + all + " tag(s) in the last frame";
        return out;
    }

    private boolean wanted(int id) {
        for (int t : tagIds) if (t == id) return true;
        return false;
    }
}
