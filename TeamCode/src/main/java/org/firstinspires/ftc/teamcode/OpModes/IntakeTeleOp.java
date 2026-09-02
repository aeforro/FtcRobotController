package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;

@TeleOp(name = "Intake TeleOp", group = "Team")
public class IntakeTeleOp extends LinearOpMode {
    private final MecanumDrive drive = new MecanumDrive();
    private final Intake intake = new Intake();

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);
        intake.init(hardwareMap);

        telemetry.addLine("Intake ready");
        telemetry.addLine("Gamepad 1: left stick = drive, right stick X = rotate");
        telemetry.addLine("Gamepad 2: left trigger = intake roller, right trigger = eject");
        telemetry.addLine("Gamepad 2: left bumper = side wheels in, right bumper = side wheels out");
        telemetry.addData("Side wheel type", String.valueOf(intake.getSideWheelType()));
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

            double rollerPower = 0.0;
            if (gamepad2.left_trigger > 0.1) {
                rollerPower = gamepad2.left_trigger;
            } else if (gamepad2.right_trigger > 0.1) {
                rollerPower = -gamepad2.right_trigger;
            }
            intake.setRollerPower(rollerPower);

            double sideWheelPower = 0.0;
            if (gamepad2.left_bumper) {
                sideWheelPower = -0.75;
            } else if (gamepad2.right_bumper) {
                sideWheelPower = 0.75;
            }
            intake.setSideWheelPower(sideWheelPower, sideWheelPower);

            telemetry.addData("Drive scale", scale);
            telemetry.addData("Roller power", rollerPower);
            telemetry.addData("Side wheel power", sideWheelPower);
            telemetry.addData("Roller connected", intake.isRollerConnected());
            telemetry.update();

            sleep(20);
        }

        drive.stop();
        intake.stop();
    }
}
