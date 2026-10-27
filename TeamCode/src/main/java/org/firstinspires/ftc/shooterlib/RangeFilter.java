package org.firstinspires.ftc.shooterlib;

/** Turns this frame's tag ranges into a better range estimate. Runs only on frames where at least one tag was seen. */
public interface RangeFilter extends Stage {
    /**
     * @param frame   the tags seen this frame (the list may be edited, e.g. to drop outliers) and the time
     * @param rangeIn the estimate so far: the first tag's range for the first filter, or the previous filter's output
     * @return the new estimate in meters
     */
    double filterRange(RangeFrame frame, double rangeIn);
}
