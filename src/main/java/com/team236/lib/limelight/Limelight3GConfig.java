package com.team236.lib.limelight;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The {@code Limelight3GConfig} configures port forwarding and NetworkTable publishing for the
 * Limelight 3G camera streams.
 */
public class Limelight3GConfig {
    private static HttpCamera mLimelight3G;

    private static String[] mHttpUrls = {
        "http://10.2.36.2:5800/?action=stream", "http://10.2.36.2:5800/stream.mjpg"
    };

    private static String[] mDashboardUrls = {
        "mjpg:http://10.2.36.2:5800/?action=stream", "mjpg:http://10.2.36.2:5800/stream.mjpg"
    };

    private static String[] mModes = {"160x160 MJPEG 40 fps"};

    public static final void configureLimelight3G() {
        // Only used when tethering.
        LimelightHelpers.setupPortForwardingUSB(0);

        // Only used when connected over wifi.
        mLimelight3G = new HttpCamera("Limelight-3G", mHttpUrls);

        NetworkTable limelightTable =
                NetworkTableInstance.getDefault()
                        .getTable("CameraPublisher")
                        .getSubTable("Limelight-3G");

        limelightTable.getStringArrayTopic("streams").publish().set(mDashboardUrls);
        limelightTable.getStringTopic("source").publish().set("ip:http://10.2.36.2:5800");
        limelightTable.getStringTopic("description").publish().set("Limelight 3G");
        limelightTable.getStringArrayTopic("modes").publish().set(mModes);
        limelightTable.getStringTopic("mode").publish().set("");
    }
}
