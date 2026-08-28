package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;

@TeleOp(name = "Gamepad TeleOp", group = "Team")
public class GamepadTeleop extends LinearOpMode {
    private final MecanumDrive drive = new MecanumDrive();

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);

        telemetry.addLine("Gamepad TeleOp ready. Y toggles field-relative. Left bumper = slow.");
        telemetry.update();

        boolean fieldRelative = true;
        boolean lastY = false;

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // Toggle field-relative mode on Y press
            if (gamepad1.y && !lastY) {
                fieldRelative = !fieldRelative;
            }
            lastY = gamepad1.y;

            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            // Speed scaling: hold left bumper for slow precision, right bumper for full speed
            double scale = 0.7;
            if (gamepad1.left_bumper) scale = 0.4;
            else if (gamepad1.right_bumper) scale = 1.0;

            forward *= scale;
            strafe *= scale;
            rotate *= scale;

            if (fieldRelative) {
                drive.driveFieldRelative(forward, strafe, rotate);
            } else {
                drive.drive(forward, strafe, rotate);
            }

            telemetry.addData("Field-relative", fieldRelative);
            telemetry.addData("Speed scale", scale);
            telemetry.update();

            sleep(20); // loop at ~50Hz
        }

        drive.stop();
    }
}
