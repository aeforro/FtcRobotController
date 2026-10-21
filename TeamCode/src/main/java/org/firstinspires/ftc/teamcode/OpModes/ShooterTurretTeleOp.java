package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@TeleOp(name = "Shooter Turret TeleOp", group = "Team")
public class ShooterTurretTeleOp extends LinearOpMode {
    private final MecanumDrive drive = new MecanumDrive();
    private final Shooter shooter = new Shooter();
    private final Turret turret = new Turret();
    private final Intake intake = new Intake();
    private final Transfer transfer = new Transfer();

    private boolean previousDpadUp;
    private boolean previousDpadDown;
    private boolean previousDpadRight;
    private boolean previousDpadLeft;

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);
        shooter.init(hardwareMap);
        turret.init(hardwareMap);
        intake.init(hardwareMap);
        transfer.init(hardwareMap);

        telemetry.addLine("Shooter and turret tele-op ready");
        telemetry.addLine("Gamepad 1: mecanum drive (left stick), turn (right stick X)");
        telemetry.addLine("Gamepad 1: right trigger intake, left trigger outtake");
        telemetry.addLine("Gamepad 2: hold right trigger to shoot and feed");
        telemetry.addLine("Either gamepad: bumpers rotate turret");
        telemetry.addLine("Gamepad 2: up/down flywheel, right/left counter RPM");
        telemetry.addLine("D-pad changes shooter targets by 50 RPM (capped at 6000)");
        telemetry.addLine("Hardware names: transfer_roller_motor, turret_left_servo, turret_right_servo");
        telemetry.update();

        waitForStart();
        turret.start();

        while (opModeIsActive() && !isStopRequested()) {
            drive.drive(
                    -gamepad1.left_stick_y * 0.7,
                    gamepad1.left_stick_x * 0.7,
                    gamepad1.right_stick_x * 0.7);

            adjustShooterTargets();
            boolean turnLeft = gamepad1.left_bumper || gamepad2.left_bumper;
            boolean turnRight = gamepad1.right_bumper || gamepad2.right_bumper;
            turret.update(turnLeft, turnRight);

            boolean shooting = gamepad2.right_trigger > 0.1;
            boolean shooterAtSpeed = false;
            if (shooting) {
                shooter.spinUp();
                intake.setPower(1.0);
                shooterAtSpeed = shooter.isAtSpeed();
                transfer.setPower(shooterAtSpeed ? 1.0 : 0.0);
            } else {
                shooter.stop();
                if (gamepad1.right_trigger > 0.1) {
                    intake.setPower(gamepad1.right_trigger);
                    transfer.setPower(gamepad1.right_trigger);
                } else if (gamepad1.left_trigger > 0.1) {
                    intake.setPower(-gamepad1.left_trigger);
                    transfer.setPower(-gamepad1.left_trigger);
                } else {
                    intake.stop();
                    transfer.stop();
                }
            }

            telemetry.addData("Flywheel target / actual RPM", "%.0f / %.0f",
                    shooter.getFlywheelTargetRpm(), shooter.getFlywheelRpm());
            telemetry.addData("Counter target / actual RPM", "%.0f / %.0f",
                    shooter.getCounterRollerTargetRpm(), shooter.getCounterRollerRpm());
            telemetry.addData("Encoder calibration", "%.0f ticks/rev",
                    Shooter.ENCODER_TICKS_PER_REV);
            telemetry.addData("Shooter", shooting ? "Shooting" : "Stopped");
            telemetry.addData("Shooter at speed", shooterAtSpeed);
            telemetry.addData("Transfer roller", shooting && !shooterAtSpeed
                    ? "Waiting for shooter speed" : (shooterAtSpeed ? "Feeding" : "Stopped"));
            telemetry.addData("Intake / transfer power", "%.2f / %.2f",
                    intake.getPower(), transfer.getPower());
            telemetry.addData("Ball handling", shooting
                    ? (shooterAtSpeed ? "Feeding" : "Intaking; transfer waiting")
                    : (gamepad1.right_trigger > 0.1 ? "Intaking" :
                            (gamepad1.left_trigger > 0.1 ? "Outtaking" : "Stopped")));
            telemetry.addData("Turret position", "%.2f", turret.getPosition());
            telemetry.addData("Turret bumpers", "left %b / right %b", turnLeft, turnRight);
            telemetry.update();

            sleep(20);
        }

        drive.stop();
        shooter.stop();
        intake.stop();
        transfer.stop();
    }

    private void adjustShooterTargets() {
        if (gamepad2.dpad_up && !previousDpadUp) {
            shooter.increaseFlywheelRpm();
        }
        if (gamepad2.dpad_down && !previousDpadDown) {
            shooter.decreaseFlywheelRpm();
        }
        if (gamepad2.dpad_right && !previousDpadRight) {
            shooter.increaseCounterRollerRpm();
        }
        if (gamepad2.dpad_left && !previousDpadLeft) {
            shooter.decreaseCounterRollerRpm();
        }

        previousDpadUp = gamepad2.dpad_up;
        previousDpadDown = gamepad2.dpad_down;
        previousDpadRight = gamepad2.dpad_right;
        previousDpadLeft = gamepad2.dpad_left;
    }
}
