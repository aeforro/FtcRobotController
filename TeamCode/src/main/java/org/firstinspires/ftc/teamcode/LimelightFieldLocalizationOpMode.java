package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Limelight: Field Localization", group = "Concept")
@Disabled
public class LimelightFieldLocalizationOpMode extends LinearOpMode {
    private LimelightFieldLocalization localization;

    @Override
    public void runOpMode() {
        localization = new LimelightFieldLocalization(hardwareMap, "limelight");
        localization.start();

        telemetry.addLine("Limelight ready. Press START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (localization.update()) {
                LimelightFieldLocalization.FieldPose pose = localization.getPoseInches();
                telemetry.addData("Valid", true);
                telemetry.addData("Tag ID", localization.getLastTagId());
                telemetry.addData("X (in)", pose.getXInches());
                telemetry.addData("Y (in)", pose.getYInches());
                telemetry.addData("Heading (deg)", pose.getHeadingDegrees());
            } else {
                telemetry.addData("Valid", false);
                telemetry.addData("Status", "No valid AprilTag pose yet");
            }

            telemetry.update();
            sleep(20);
        }

        localization.stop();
    }
}
