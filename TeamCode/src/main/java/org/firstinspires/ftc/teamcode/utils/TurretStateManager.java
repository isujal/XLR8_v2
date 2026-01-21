package org.firstinspires.ftc.teamcode.utils;

import android.content.Context;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.*;

public class TurretStateManager {
    private static final String FILE_NAME = "turret_state.txt";
    private final File file;

    public TurretStateManager(HardwareMap hardwareMap) {
        Context context = hardwareMap.appContext;
        file = new File(context.getFilesDir(), FILE_NAME);
    }

    // Save turret position and angle
    public void saveState(double encoderPos, double angle) {
        try (FileWriter fw = new FileWriter(file)) {
            fw.write(encoderPos + "," + angle);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    // Load turret position and angle
    public double[] loadState() {
        try (BufferedReader br = new BufferedReader(new FileReader(file))) {
            String[] parts = br.readLine().split(",");
            return new double[]{Double.parseDouble(parts[0]), Double.parseDouble(parts[1])};
        } catch (Exception e) {
            return new double[]{0, 0};  // default if file missing
        }
    }

    public void clearState() {
        if (file.exists()) file.delete();
    }
}
