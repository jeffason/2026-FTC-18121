package org.firstinspires.ftc.teamcode.shooter;

import java.util.ArrayList;
import java.util.List;

/** One camera frame as seen by the range filters. */
public final class RangeFrame {
    /** Horizontal range in meters to each goal tag seen this frame. Filters may remove entries but never leave it empty. */
    public final List<Double> tags;
    public final long nowMs;

    public RangeFrame(List<Double> tags, long nowMs) {
        this.tags = new ArrayList<>(tags);
        this.nowMs = nowMs;
    }
}
