package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Arrays;
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

    public static File file() { return AppUtil.getInstance().getSettingsFile(FILE_NAME); }

    /** Sorts by distance and writes the file. */
    public static void save(double[] meters, double[] rpm) {
        int n = meters.length;
        Integer[] idx = new Integer[n];
        for (int i = 0; i < n; i++) idx[i] = i;
        Arrays.sort(idx, (a, b) -> Double.compare(meters[a], meters[b]));
        double[] m = new double[n], r = new double[n];
        for (int i = 0; i < n; i++) { m[i] = meters[idx[i]]; r[i] = rpm[idx[i]]; }
        ReadWriteFile.writeFile(file(), "meters=" + join(m, "%.2f") + "\nrpm=" + join(r, "%.0f") + "\n");
        RobotLog.ii("Shooter", "Saved calibration: " + asJava(m, r));
    }

    /** Loads the saved table into ShooterConfig. Returns false (and leaves the config alone) if there is none or it is invalid. */
    public static boolean loadIntoConfig() {
        File f = file();
        if (!f.exists()) return false;
        try {
            double[] m = null, r = null;
            for (String line : ReadWriteFile.readFile(f).split("\n")) {
                line = line.trim();
                if (line.startsWith("meters=")) m = parse(line.substring(7));
                else if (line.startsWith("rpm=")) r = parse(line.substring(4));
            }
            if (m == null || r == null || m.length != r.length || m.length < 2) return false;
            for (int i = 1; i < m.length; i++) if (m[i] <= m[i - 1]) return false;
            ShooterConfig.CALIBRATION_METERS = m;
            ShooterConfig.CALIBRATION_RPM = r;
            RobotLog.ii("Shooter", "Loaded calibration from " + f.getName() + ": " + asJava(m, r));
            return true;
        } catch (RuntimeException e) {
            RobotLog.ee("Shooter", e, "Bad calibration file, using ShooterConfig defaults");
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
