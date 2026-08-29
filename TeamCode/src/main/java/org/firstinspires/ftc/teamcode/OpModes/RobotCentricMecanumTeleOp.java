package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;

@TeleOp(name = "Robot Centric Mecanum", group = "Team")
public class RobotCentricMecanumTeleOp extends LinearOpMode {
    private final MecanumDrive drive = new MecanumDrive();

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);

        telemetry.addLine("Robot-centric mecanum ready");
        telemetry.addLine("Left stick = drive");
        telemetry.addLine("Right stick X = rotate");
        telemetry.addLine("Left bumper = slow mode");
        telemetry.addLine("Right bumper = full speed");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            double scale = 0.7;
            if (gamepad1.left_bumper) {
                scale = 0.35;
            } else if (gamepad1.right_bumper) {
                scale = 1.0;
            }

            drive.drive(forward * scale, strafe * scale, rotate * scale);

            telemetry.addData("Forward", forward);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Rotate", rotate);
            telemetry.addData("Speed scale", scale);
            telemetry.update();

            sleep(20);
        }

        drive.stop();
    }
}
