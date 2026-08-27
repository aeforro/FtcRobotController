package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;

public class LimelightDriveController {
    private final MecanumDrive drive;
    private final LimelightFieldLocalization localization;

    private double linearGain = 0.025;
    private double turnGain = 0.04;
    private double maxLinearSpeed = 0.55;
    private double maxTurnSpeed = 0.45;

    public LimelightDriveController(MecanumDrive drive, LimelightFieldLocalization localization) {
        this.drive = drive;
        this.localization = localization;
    }

    public void setGains(double linearGain, double turnGain) {
        this.linearGain = linearGain;
        this.turnGain = turnGain;
    }

    public void setMaxSpeeds(double maxLinearSpeed, double maxTurnSpeed) {
        this.maxLinearSpeed = maxLinearSpeed;
        this.maxTurnSpeed = maxTurnSpeed;
    }

    public boolean driveToTarget(double targetXInches, double targetYInches, double targetHeadingDeg,
                                double linearToleranceInches, double headingToleranceDeg) {
        if (!localization.update()) {
            drive.stop();
            return false;
        }

        LimelightFieldLocalization.FieldPose currentPose = localization.getPoseInches();
        double dx = targetXInches - currentPose.getXInches();
        double dy = targetYInches - currentPose.getYInches();
        double headingDeg = currentPose.getHeadingDegrees();

        double headingRad = Math.toRadians(headingDeg);
        double fieldToRobotCos = Math.cos(-headingRad);
        double fieldToRobotSin = Math.sin(-headingRad);

        double robotRelativeX = dx * fieldToRobotCos - dy * fieldToRobotSin;
        double robotRelativeY = dx * fieldToRobotSin + dy * fieldToRobotCos;

        double turnError = LimelightFieldLocalization.normalizeDegrees(targetHeadingDeg - headingDeg);
        double forwardCommand = clamp(robotRelativeY * linearGain, -maxLinearSpeed, maxLinearSpeed);
        double strafeCommand = clamp(robotRelativeX * linearGain, -maxLinearSpeed, maxLinearSpeed);
        double turnCommand = clamp(turnError * turnGain, -maxTurnSpeed, maxTurnSpeed);

        double distanceError = Math.hypot(dx, dy);
        if (distanceError <= linearToleranceInches && Math.abs(turnError) <= headingToleranceDeg) {
            drive.stop();
            return true;
        }

        drive.drive(forwardCommand, strafeCommand, turnCommand);
        return false;
    }

    public void stop() {
        drive.stop();
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
}
