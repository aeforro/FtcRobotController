package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;

@TeleOp(name = "Robot Centric Mecanum", group = "Team")
public class RobotCentricMecanumTeleOp extends LinearOpMode {
    private final MecanumDrive drive = new MecanumDrive();
    private final Intake intake = new Intake();

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);
        intake.init(hardwareMap);

        telemetry.addLine("Robot-centric mecanum ready");
        telemetry.addLine("Gamepad 1: left stick = drive, right stick X = rotate");
        telemetry.addLine("Gamepad 1: left bumper = slow, right bumper = full speed");
        telemetry.addLine("Gamepad 1: hold right trigger = intake, hold left trigger = eject");
        telemetry.addLine("Gamepad 1: right bumper = full-speed intake");
        telemetry.addLine("Gamepad 2: backup intake/eject controls");
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
            double sideWheelPower = 0.0;

            // Primary controller controls intake behavior directly, but only while held.
            if (gamepad1.right_trigger > 0.1) {
                rollerPower = 1.0;
                sideWheelPower = gamepad1.right_bumper ? 1.0 : 0.75;
            } else if (gamepad1.left_trigger > 0.1) {
                rollerPower = -1.0;
                sideWheelPower = gamepad1.left_bumper ? -1.0 : -0.75;
            }

            // Backup controls on secondary controller.
            if (rollerPower == 0.0 && sideWheelPower == 0.0) {
                if (gamepad2.right_trigger > 0.1) {
                    rollerPower = 1.0;
                    sideWheelPower = 0.75;
                } else if (gamepad2.left_trigger > 0.1) {
                    rollerPower = -1.0;
                    sideWheelPower = -0.75;
                } else if (gamepad2.right_bumper) {
                    rollerPower = 1.0;
                    sideWheelPower = 1.0;
                } else if (gamepad2.left_bumper) {
                    rollerPower = -1.0;
                    sideWheelPower = -1.0;
                }
            }

            intake.setRollerPower(rollerPower);
            intake.setSideWheelPower(sideWheelPower, sideWheelPower);

            telemetry.addData("Forward", forward);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Rotate", rotate);
            telemetry.addData("Speed scale", scale);
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
