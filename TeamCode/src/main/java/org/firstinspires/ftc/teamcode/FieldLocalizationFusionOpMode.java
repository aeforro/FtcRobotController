package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;

@TeleOp(name = "Fusion: Limelight + Pinpoint", group = "Concept")
@Disabled
public class FieldLocalizationFusionOpMode extends LinearOpMode {
    private final MecanumDrive drive = new MecanumDrive();
    private FieldLocalizationFusion localization;

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);
        localization = new FieldLocalizationFusion(hardwareMap, "limelight", "pinpoint");
        localization.startLimelight();

        telemetry.addLine("Fusion localization ready. Press START.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            localization.update();

            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            drive.driveFieldRelative(forward, strafe, rotate);

            telemetry.addData("Pose valid", localization.isValid());
            telemetry.addData("X (in)", localization.getXInches());
            telemetry.addData("Y (in)", localization.getYInches());
            telemetry.addData("Heading (deg)", localization.getHeadingDegrees());

            if (localization.getLimelightLocalization().isValid()) {
                telemetry.addData("Limelight tag", localization.getLimelightLocalization().getLastTagId());
            }

            telemetry.update();
            sleep(20);
        }

        drive.stop();
        localization.stopLimelight();
    }
}
