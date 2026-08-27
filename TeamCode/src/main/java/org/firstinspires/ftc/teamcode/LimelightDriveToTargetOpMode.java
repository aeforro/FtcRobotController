package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;

@TeleOp(name = "Limelight: Drive to Target", group = "Concept")
@Disabled
public class LimelightDriveToTargetOpMode extends LinearOpMode {
    private final MecanumDrive drive = new MecanumDrive();
    private LimelightFieldLocalization localization;
    private LimelightDriveController controller;

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);
        localization = new LimelightFieldLocalization(hardwareMap, "limelight");
        localization.start();

        controller = new LimelightDriveController(drive, localization);
        controller.setGains(0.024, 0.03);
        controller.setMaxSpeeds(0.45, 0.35);

        telemetry.addLine("Limelight target-drive ready. Press START.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            boolean reachedTarget = controller.driveToTarget(0.0, 48.0, 0.0, 2.5, 5.0);

            if (localization.isValid()) {
                LimelightFieldLocalization.FieldPose pose = localization.getPoseInches();
                telemetry.addData("X (in)", pose.getXInches());
                telemetry.addData("Y (in)", pose.getYInches());
                telemetry.addData("Heading (deg)", pose.getHeadingDegrees());
                telemetry.addData("Target reached", reachedTarget);
            } else {
                telemetry.addData("Target reached", false);
                telemetry.addData("Status", "Waiting for valid AprilTag pose");
            }

            telemetry.update();
            sleep(20);
        }

        controller.stop();
        localization.stop();
    }
}
