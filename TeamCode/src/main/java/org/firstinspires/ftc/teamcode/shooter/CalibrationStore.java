package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

/**
 * Saves the distance -> rpm table on the Control Hub so a calibration survives redeploys,
 * and formats it as Java so it can be pasted into {@link ShooterConfig} for version control.
 *
 * File lives in /sdcard/FIRST/settings/shooter_calibration.txt and looks like:
 *   meters=1.00,2.00,3.00
 *   rpm=1750,2080,2430
 */
public final class CalibrationStore {
    public static final String FILE_NAME = "shooter_calibration.txt";

    /** Distances closer than this are the same measuring spot; their rpm values are averaged. */
    public static final double SAME_SPOT_METERS = 0.01;

    private static String lastLoadMessage = "not loaded yet";

    public static File file() { return AppUtil.getInstance().getSettingsFile(FILE_NAME); }

    /** Why the last loadIntoConfig() answered as it did, for telemetry. */
    public static String lastLoadMessage() { return lastLoadMessage; }

    /**
     * Sorts by distance, merges points recorded at the same spot (averaging their rpm) and writes the
     * file. The result always satisfies CalibrationTable; throws IllegalArgumentException if it cannot.
     */
    public static void save(double[] meters, double[] rpm) {
        double[][] table = normalize(meters, rpm);
        new CalibrationTable(table[0], table[1]);   // validates before anything is written
        ReadWriteFile.writeFile(file(), "meters=" + join(table[0], "%.2f") + "\nrpm=" + join(table[1], "%.0f") + "\n");
        RobotLog.ii("Shooter", "Saved calibration: " + asJava(table[0], table[1]));
    }

    /** Sorted, deduplicated {meters, rpm} with two-decimal distances, ready for CalibrationTable. */
    static double[][] normalize(double[] meters, double[] rpm) {
        if (meters.length != rpm.length) throw new IllegalArgumentException("meters and rpm differ in length");
        int n = meters.length;
        Integer[] idx = new Integer[n];
        for (int i = 0; i < n; i++) idx[i] = i;
        Arrays.sort(idx, (a, b) -> Double.compare(meters[a], meters[b]));
        List<Double> m = new ArrayList<>(), r = new ArrayList<>(), count = new ArrayList<>();
        for (int k = 0; k < n; k++) {
            double d = Math.round(meters[idx[k]] * 100.0) / 100.0, v = rpm[idx[k]];
            if (!Double.isFinite(d) || !Double.isFinite(v)) throw new IllegalArgumentException("Calibration values must be finite");
            int last = m.size() - 1;
            if (last >= 0 && d - m.get(last) < SAME_SPOT_METERS) {
                double c = count.get(last);
                r.set(last, (r.get(last) * c + v) / (c + 1));
                count.set(last, c + 1);
            } else { m.add(d); r.add(v); count.add(1.0); }
        }
        double[] mm = new double[m.size()], rr = new double[r.size()];
        for (int i = 0; i < mm.length; i++) { mm[i] = m.get(i); rr[i] = Math.round(r.get(i)); }
        return new double[][] { mm, rr };
    }

    /**
     * Loads the saved table into ShooterConfig. Returns false and leaves the config alone if there is
     * no file or it is invalid; lastLoadMessage() says which.
     */
    public static boolean loadIntoConfig() {
        File f = file();
        if (!f.exists()) { lastLoadMessage = "no saved table on the hub"; return false; }
        try {
            double[] m = null, r = null;
            for (String line : ReadWriteFile.readFile(f).split("\n")) {
                line = line.trim();
                if (line.startsWith("meters=")) m = parse(line.substring(7));
                else if (line.startsWith("rpm=")) r = parse(line.substring(4));
            }
            if (m == null || r == null) throw new IllegalArgumentException("missing meters= or rpm= line");
            CalibrationTable table = new CalibrationTable(m, r);   // rejects NaN, unsorted, short or mismatched rows
            ShooterConfig.CALIBRATION_METERS = table.meters;
            ShooterConfig.CALIBRATION_RPM = table.rpm;
            lastLoadMessage = "loaded " + m.length + " points from " + f.getName();
            RobotLog.ii("Shooter", "Loaded calibration from " + f.getName() + ": " + asJava(m, r));
            return true;
        } catch (RuntimeException e) {
            lastLoadMessage = "INVALID saved table (" + e.getMessage() + "); using ShooterConfig defaults";
            RobotLog.ee("Shooter", e, lastLoadMessage);
            return false;
        }
    }

    public static boolean delete() { return file().delete(); }

    /** Two Java lines ready to paste into ShooterConfig. */
    public static String asJava(double[] meters, double[] rpm) {
        return "CALIBRATION_METERS = { " + join(meters, "%.2f") + " };\n"
             + "CALIBRATION_RPM    = { " + join(rpm, "%.0f") + " };";
    }

    static double[] parse(String csv) {
        String[] parts = csv.split(",");
        double[] out = new double[parts.length];
        for (int i = 0; i < parts.length; i++) out[i] = Double.parseDouble(parts[i].trim());
        return out;
    }

    static String join(double[] v, String fmt) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < v.length; i++) { if (i > 0) sb.append(", "); sb.append(String.format(Locale.US, fmt, v[i])); }
        return sb.toString();
    }

    private CalibrationStore() {}
}
