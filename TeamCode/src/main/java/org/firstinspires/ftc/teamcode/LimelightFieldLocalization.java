package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * Utility class for AprilTag-based field localization using a Limelight 3A.
 *
 * The Limelight already computes robot pose in FTC field coordinates. This class wraps that pose
 * into a simple API you can use each loop to update the robot's field position and heading.
 *
 * Coordinate convention:
 *   +x = right on the field
 *   +y = forward on the field
 *   heading = yaw in degrees, normalized to [-180, 180)
 *
 * Limelight reports the pose in meters. We convert to inches for convenience when using the field
 * coordinate system in FTC, but the raw meters remain available if you prefer to do math in SI.
 */
public class LimelightFieldLocalization {
    public static final double METERS_TO_INCHES = 39.3700787402;

    private final Limelight3A limelight;

    private boolean valid = false;
    private double xMeters = 0.0;
    private double yMeters = 0.0;
    private double headingDegrees = 0.0;
    private int lastTagId = -1;
    private long lastUpdateMs = 0L;

    public LimelightFieldLocalization(HardwareMap hardwareMap) {
        this(hardwareMap, "limelight");
    }

    public LimelightFieldLocalization(HardwareMap hardwareMap, String deviceName) {
        this.limelight = hardwareMap.get(Limelight3A.class, deviceName);
        limelight.pipelineSwitch(0);
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    public void setPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
    }

    /**
     * Polls the Limelight and updates the robot's field pose if a valid AprilTag result exists.
     *
     * @return true if pose data was updated, false if no valid tag was seen.
     */
    public boolean update() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            valid = false;
            return false;
        }

        Pose3D botpose = result.getBotpose();
        if (botpose == null) {
            valid = false;
            return false;
        }

        xMeters = botpose.getPosition().x;
        yMeters = botpose.getPosition().y;
        headingDegrees = normalizeDegrees(botpose.getOrientation().getYaw(AngleUnit.DEGREES));

        lastTagId = getBestTagId(result);
        lastUpdateMs = System.currentTimeMillis();
        valid = true;
        return true;
    }

    public boolean isValid() {
        return valid;
    }

    public double getXMeters() {
        return xMeters;
    }

    public double getYMeters() {
        return yMeters;
    }

    public double getXInches() {
        return xMeters * METERS_TO_INCHES;
    }

    public double getYInches() {
        return yMeters * METERS_TO_INCHES;
    }

    public double getHeadingDegrees() {
        return headingDegrees;
    }

    public double getHeadingRadians() {
        return Math.toRadians(headingDegrees);
    }

    public int getLastTagId() {
        return lastTagId;
    }

    public long getLastUpdateMs() {
        return lastUpdateMs;
    }

    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    public static double normalizeDegrees(double degrees) {
        while (degrees > 180.0) {
            degrees -= 360.0;
        }
        while (degrees <= -180.0) {
            degrees += 360.0;
        }
        return degrees;
    }

    public FieldPose getPoseInches() {
        return new FieldPose(getXInches(), getYInches(), headingDegrees);
    }

    public FieldPose getPoseMeters() {
        return new FieldPose(xMeters * METERS_TO_INCHES, yMeters * METERS_TO_INCHES, headingDegrees);
    }

    private int getBestTagId(LLResult result) {
        List<com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult> tagList = result.getFiducialResults();
        if (tagList == null || tagList.isEmpty()) {
            return -1;
        }
        return tagList.get(0).getFiducialId();
    }

    public static class FieldPose {
        private final double xInches;
        private final double yInches;
        private final double headingDegrees;

        public FieldPose(double xInches, double yInches, double headingDegrees) {
            this.xInches = xInches;
            this.yInches = yInches;
            this.headingDegrees = normalizeDegrees(headingDegrees);
        }

        public double getXInches() {
            return xInches;
        }

        public double getYInches() {
            return yInches;
        }

        public double getHeadingDegrees() {
            return headingDegrees;
        }
    }
}
