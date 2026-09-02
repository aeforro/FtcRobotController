package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.externalhardware.RobotHardware;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Fuses three sources of localization:
 *   1) Pinpoint odometry + IMU for continuous dead-reckoning
 *   2) Limelight AprilTag detection for absolute correction
 *   3) The robot's current estimated pose for target following and autonomous navigation
 *
 * This class keeps the robot pose in inch-space, because FTC field positioning is usually easier to
 * reason about in inches. When a valid AprilTag is seen, we snap the pose to the AprilTag solution
 * and then re-seed the Pinpoint. When no tag is visible, we continue from the pinpoint odometry.
 */
public class FieldLocalizationFusion {
    private final LimelightFieldLocalization limelight;
    private final GoBildaPinpointDriver pinpoint;

    private double xInches = 0.0;
    private double yInches = 0.0;
    private double headingDegrees = 0.0;
    private boolean valid = false;

    private boolean useLimelight = true;
    private boolean usePinpoint = true;

    // Tunable values for your robot. Start conservative, then tune on the field.
    private double xPodOffsetMm = -84.0;
    private double yPodOffsetMm = -168.0;
    private double limelightCorrectionWeight = 0.85;

    public FieldLocalizationFusion(HardwareMap hardwareMap) {
        this(hardwareMap, "limelight", "pinpoint");
    }

    public FieldLocalizationFusion(HardwareMap hardwareMap, String limelightName, String pinpointName) {
        this.limelight = new LimelightFieldLocalization(hardwareMap, limelightName);
        this.pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, pinpointName);
        configurePinpoint();
        resetPose(0.0, 0.0, 0.0);
    }

    /**
     * Prefer a Pinpoint instance from RobotHardware when available.  Falls back to the HardwareMap name otherwise.
     */
    public FieldLocalizationFusion(RobotHardware robotHardware, HardwareMap hardwareMap, String limelightName, String pinpointName) {
        this.limelight = new LimelightFieldLocalization(hardwareMap, limelightName);
        GoBildaPinpointDriver tempPinpoint = null;
        if (robotHardware != null && robotHardware.hasPinpoint()) {
            tempPinpoint = robotHardware.getPinpoint();
        } else {
            try {
                tempPinpoint = hardwareMap.get(GoBildaPinpointDriver.class, pinpointName);
            } catch (Exception e) {
                tempPinpoint = null;
            }
        }
        this.pinpoint = tempPinpoint;

        if (this.pinpoint != null) {
            configurePinpoint();
            resetPose(0.0, 0.0, 0.0);
        }
    }

    public void startLimelight() {
        limelight.start();
    }

    public void stopLimelight() {
        limelight.stop();
    }

    public void configurePinpoint() {
        pinpoint.setOffsets(xPodOffsetMm, yPodOffsetMm, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }

    public void setPoseOffsetsMm(double xPodOffsetMm, double yPodOffsetMm) {
        this.xPodOffsetMm = xPodOffsetMm;
        this.yPodOffsetMm = yPodOffsetMm;
        pinpoint.setOffsets(this.xPodOffsetMm, this.yPodOffsetMm, DistanceUnit.MM);
    }

    public void setLimelightCorrectionWeight(double weight) {
        this.limelightCorrectionWeight = clamp(weight, 0.0, 1.0);
    }

    public void setUseLimelight(boolean useLimelight) {
        this.useLimelight = useLimelight;
    }

    public void setUsePinpoint(boolean usePinpoint) {
        this.usePinpoint = usePinpoint;
    }

    public void resetPose(double xInches, double yInches, double headingDegrees) {
        this.xInches = xInches;
        this.yInches = yInches;
        this.headingDegrees = normalizeDegrees(headingDegrees);
        valid = true;

        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, xInches, yInches, AngleUnit.DEGREES, headingDegrees));
    }

    public void update() {
        if (usePinpoint) {
            pinpoint.update();
            Pose2D pinpointPose = pinpoint.getPosition();

            double odomX = pinpointPose.getX(DistanceUnit.INCH);
            double odomY = pinpointPose.getY(DistanceUnit.INCH);
            double odomHeading = normalizeDegrees(pinpointPose.getHeading(AngleUnit.DEGREES));

            xInches = odomX;
            yInches = odomY;
            headingDegrees = odomHeading;
        }

        if (useLimelight && limelight.update()) {
            LimelightFieldLocalization.FieldPose tagPose = limelight.getPoseInches();
            double llX = tagPose.getXInches();
            double llY = tagPose.getYInches();
            double llHeading = tagPose.getHeadingDegrees();

            xInches = lerp(xInches, llX, limelightCorrectionWeight);
            yInches = lerp(yInches, llY, limelightCorrectionWeight);
            headingDegrees = normalizeDegrees(lerpDegrees(headingDegrees, llHeading, limelightCorrectionWeight));

            // Re-seed the Pinpoint so future odometry continues from the corrected field pose.
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, xInches, yInches, AngleUnit.DEGREES, headingDegrees));
            valid = true;
        } else {
            valid = usePinpoint;
        }
    }

    public boolean isValid() {
        return valid;
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

    public double getHeadingRadians() {
        return Math.toRadians(headingDegrees);
    }

    public GoBildaPinpointDriver getPinpoint() {
        return pinpoint;
    }

    public LimelightFieldLocalization getLimelightLocalization() {
        return limelight;
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

    private double clamp(double value, double min, double max) {
        if (value < min) {
            return min;
        }
        if (value > max) {
            return max;
        }
        return value;
    }

    private double lerp(double a, double b, double weight) {
        return a + (b - a) * weight;
    }

    private double lerpDegrees(double a, double b, double weight) {
        double delta = normalizeDegrees(b - a);
        return a + delta * weight;
    }
}
