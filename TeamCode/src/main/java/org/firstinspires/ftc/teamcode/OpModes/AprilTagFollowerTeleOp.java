package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;

import java.util.List;

@TeleOp(name = "AprilTag Follower", group = "Team")
public class AprilTagFollowerTeleOp extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = -1; // -1 = any tag
    private static final double DESIRED_DISTANCE_INCHES = 12.0;

    private static final double SPEED_GAIN = 0.02;
    private static final double STRAFE_GAIN = 0.015;
    private static final double TURN_GAIN = 0.01;

    private static final double MAX_AUTO_SPEED = 0.5;
    private static final double MAX_AUTO_STRAFE = 0.5;
    private static final double MAX_AUTO_TURN = 0.3;

    private final MecanumDrive drive = new MecanumDrive();
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private AprilTagDetection desiredTag = null;

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);
        initAprilTag();

        telemetry.addLine("AprilTag follower ready");
        telemetry.addLine("Hold left bumper to auto-follow a tag");
        telemetry.addLine("Use left stick to drive manually when not auto-following");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            desiredTag = null;
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null) {
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        desiredTag = detection;
                        break;
                    }
                }
            }

            if (gamepad1.left_bumper && desiredTag != null) {
                double rangeError = desiredTag.ftcPose.range - DESIRED_DISTANCE_INCHES;
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;

                double forward = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                double rotate = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                drive.drive(forward, strafe, rotate);

                telemetry.addData("Target ID", desiredTag.id);
                telemetry.addData("Range", "%5.1f in", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f deg", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f deg", desiredTag.ftcPose.yaw);
                telemetry.addData("Auto drive", "F %.2f S %.2f R %.2f", forward, strafe, rotate);
            } else {
                double forward = -gamepad1.left_stick_y / 2.0;
                double strafe = gamepad1.left_stick_x / 2.0;
                double rotate = -gamepad1.right_stick_x / 3.0;

                drive.drive(forward, strafe, rotate);

                telemetry.addData("Manual drive", "F %.2f S %.2f R %.2f", forward, strafe, rotate);
                if (desiredTag == null) {
                    telemetry.addLine("No AprilTag found");
                } else {
                    telemetry.addData("Target ID", desiredTag.id);
                    telemetry.addData("Range", "%5.1f in", desiredTag.ftcPose.range);
                }
            }

            telemetry.update();
            sleep(20);
        }

        drive.stop();
        visionPortal.close();
    }

    private void initAprilTag() {
        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        aprilTagProcessor.setDecimation(2);

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTagProcessor)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTagProcessor)
                    .build();
        }
    }
}
