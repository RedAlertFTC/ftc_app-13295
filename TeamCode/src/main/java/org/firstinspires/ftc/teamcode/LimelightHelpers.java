package org.firstinspires.ftc.teamcode;



import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;

import java.net.HttpURLConnection;
import java.net.URL;
import java.util.Scanner;

public class LimelightHelpers {

    private static final Gson gson = new Gson();

    // --------------------------------------------------
    // Basic Target Valid
    // --------------------------------------------------
    public static boolean getTV(String limelightName) {
        return getLimelightValue(limelightName, "tv") == 1;
    }

    // --------------------------------------------------
    // AprilTag pose (target space)
    // --------------------------------------------------
    public static double[] getBotPose_TargetSpace(String limelightName) {
        return getLimelightDoubleArray(limelightName, "botpose_targetspace");
    }

    // --------------------------------------------------
    // Internal HTTP getter (FTC compatible)
    // --------------------------------------------------
    private static double getLimelightValue(String name, String key) {
        try {
            URL url = new URL("http://" + name + ".local:5807/results");
            HttpURLConnection conn = (HttpURLConnection) url.openConnection();
            conn.setConnectTimeout(50);
            conn.setReadTimeout(50);

            Scanner scanner = new Scanner(conn.getInputStream());
            String json = scanner.useDelimiter("\\A").next();
            scanner.close();

            JsonObject obj = gson.fromJson(json, JsonObject.class);
            return obj.get(key).getAsDouble();

        } catch (Exception e) {
            return 0;
        }
    }

    private static double[] getLimelightDoubleArray(String name, String key) {
        try {
            URL url = new URL("http://" + name + ".local:5807/results");
            HttpURLConnection conn = (HttpURLConnection) url.openConnection();
            conn.setConnectTimeout(50);
            conn.setReadTimeout(50);

            Scanner scanner = new Scanner(conn.getInputStream());
            String json = scanner.useDelimiter("\\A").next();
            scanner.close();

            JsonObject obj = gson.fromJson(json, JsonObject.class);
            JsonArray arr = obj.getAsJsonArray(key);

            double[] out = new double[arr.size()];
            for (int i = 0; i < arr.size(); i++) {
                out[i] = arr.get(i).getAsDouble();
            }
            return out;

        } catch (Exception e) {
            return new double[6];
        }
    }
}
