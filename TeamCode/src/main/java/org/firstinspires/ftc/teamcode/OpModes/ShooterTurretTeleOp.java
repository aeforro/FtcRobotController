package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;

@TeleOp(name = "Shooter Turret TeleOp", group = "Team")
public class ShooterTurretTeleOp extends LinearOpMode {
    private static final double DEFAULT_FLYWHEEL_RPM = 2350.0;
    private static final double DEFAULT_COUNTER_ROLLER_RPM = 5550.0;
    private static final double MAX_SHOOTER_RPM = 6000.0;
    private static final double RPM_ADJUSTMENT = 50.0;
    private static final double SHOOTER_READY_FRACTION = 0.95;
    private static final double SHOOTER_ENCODER_TICKS_PER_REV = 28.0;
    private static final double TURRET_MIN_POSITION = 0.05;
    private static final double TURRET_MAX_POSITION = 0.95;
    private static final double TURRET_SPEED = 5.0;

    private final MecanumDrive drive = new MecanumDrive();
    private final ElapsedTime loopTimer = new ElapsedTime();

    private DcMotorEx flywheel;
    private DcMotorEx counterRoller;
    private DcMotor intakeRoller;
    private DcMotor transferRoller;
    private Servo leftTurretServo;
    private Servo rightTurretServo;

    private double flywheelTargetRpm = DEFAULT_FLYWHEEL_RPM;
    private double counterRollerTargetRpm = DEFAULT_COUNTER_ROLLER_RPM;
    private double turretPosition = 0.5;
    private boolean previousDpadUp;
    private boolean previousDpadDown;
    private boolean previousDpadRight;
    private boolean previousDpadLeft;

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        counterRoller = hardwareMap.get(DcMotorEx.class, "counter_roller");
        intakeRoller = hardwareMap.get(DcMotor.class, "intake_roller_motor");
        transferRoller = hardwareMap.get(DcMotor.class, "transfer_roller_motor");
        leftTurretServo = hardwareMap.get(Servo.class, "turret_left_servo");
        rightTurretServo = hardwareMap.get(Servo.class, "turret_right_servo");

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        counterRoller.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        counterRoller.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        counterRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        transferRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRoller.setPower(0.0);
        transferRoller.setPower(0.0);

        leftTurretServo.setDirection(Servo.Direction.FORWARD);
        rightTurretServo.setDirection(Servo.Direction.FORWARD);
        setTurretPosition(turretPosition);
        stopShooter();

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
        loopTimer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            drive.drive(
                    -gamepad1.left_stick_y * 0.7,
                    gamepad1.left_stick_x * 0.7,
                    gamepad1.right_stick_x * 0.7);

            adjustShooterTargets();
            boolean turnLeft = gamepad1.left_bumper || gamepad2.left_bumper;
            boolean turnRight = gamepad1.right_bumper || gamepad2.right_bumper;
            updateTurret(turnLeft, turnRight);

            boolean shooting = gamepad2.right_trigger > 0.1;
            boolean shooterAtSpeed = false;
            if (shooting) {
                flywheel.setVelocity(rpmToTicksPerSecond(flywheelTargetRpm));
                counterRoller.setVelocity(rpmToTicksPerSecond(counterRollerTargetRpm));
                intakeRoller.setPower(1.0);
                shooterAtSpeed = isShooterAtSpeed();
                transferRoller.setPower(shooterAtSpeed ? 1.0 : 0.0);
            } else {
                stopShooter();
                if (gamepad1.right_trigger > 0.1) {
                    intakeRoller.setPower(gamepad1.right_trigger);
                    transferRoller.setPower(gamepad1.right_trigger);
                } else if (gamepad1.left_trigger > 0.1) {
                    intakeRoller.setPower(-gamepad1.left_trigger);
                    transferRoller.setPower(-gamepad1.left_trigger);
                } else {
                    intakeRoller.setPower(0.0);
                    transferRoller.setPower(0.0);
                }
            }

            telemetry.addData("Flywheel target / actual RPM", "%.0f / %.0f",
                    flywheelTargetRpm, ticksPerSecondToRpm(flywheel.getVelocity()));
            telemetry.addData("Counter target / actual RPM", "%.0f / %.0f",
                    counterRollerTargetRpm, ticksPerSecondToRpm(counterRoller.getVelocity()));
            telemetry.addData("Encoder calibration", "%.0f ticks/rev",
                    SHOOTER_ENCODER_TICKS_PER_REV);
            telemetry.addData("Shooter", shooting ? "Shooting" : "Stopped");
            telemetry.addData("Shooter at speed", shooterAtSpeed);
            telemetry.addData("Transfer roller", shooting && !shooterAtSpeed
                    ? "Waiting for shooter speed" : (shooterAtSpeed ? "Feeding" : "Stopped"));
            telemetry.addData("Intake / transfer power", "%.2f / %.2f",
                    intakeRoller.getPower(), transferRoller.getPower());
            telemetry.addData("Ball handling", shooting
                    ? (shooterAtSpeed ? "Feeding" : "Intaking; transfer waiting")
                    : (gamepad1.right_trigger > 0.1 ? "Intaking" :
                            (gamepad1.left_trigger > 0.1 ? "Outtaking" : "Stopped")));
            telemetry.addData("Turret position", "%.2f", turretPosition);
            telemetry.addData("Turret servo commands", "left %.2f / right %.2f",
                        turretPosition, turretPosition);
            telemetry.addData("Turret bumpers", "left %b / right %b", turnLeft, turnRight);
            telemetry.update();

            sleep(20);
        }

        drive.stop();
        stopShooter();
        intakeRoller.setPower(0.0);
        transferRoller.setPower(0.0);
    }

    private void adjustShooterTargets() {
        if (gamepad2.dpad_up && !previousDpadUp) {
            flywheelTargetRpm = Math.min(MAX_SHOOTER_RPM, flywheelTargetRpm + RPM_ADJUSTMENT);
        }
        if (gamepad2.dpad_down && !previousDpadDown) {
            flywheelTargetRpm = Math.max(0.0, flywheelTargetRpm - RPM_ADJUSTMENT);
        }
        if (gamepad2.dpad_right && !previousDpadRight) {
            counterRollerTargetRpm = Math.min(
                    MAX_SHOOTER_RPM, counterRollerTargetRpm + RPM_ADJUSTMENT);
        }
        if (gamepad2.dpad_left && !previousDpadLeft) {
            counterRollerTargetRpm = Math.max(0.0, counterRollerTargetRpm - RPM_ADJUSTMENT);
        }

        previousDpadUp = gamepad2.dpad_up;
        previousDpadDown = gamepad2.dpad_down;
        previousDpadRight = gamepad2.dpad_right;
        previousDpadLeft = gamepad2.dpad_left;
    }

    private void updateTurret(boolean turnLeft, boolean turnRight) {
        double elapsedSeconds = Math.min(loopTimer.seconds(), 0.05);
        loopTimer.reset();

        if (turnLeft && !turnRight) {
            turretPosition += TURRET_SPEED * elapsedSeconds;
        } else if (turnRight && !turnLeft) {
            turretPosition -= TURRET_SPEED * elapsedSeconds;
        }

        turretPosition = Math.max(TURRET_MIN_POSITION,
                Math.min(TURRET_MAX_POSITION, turretPosition));
        setTurretPosition(turretPosition);
    }

    private void setTurretPosition(double position) {
        leftTurretServo.setPosition(position);
        rightTurretServo.setPosition(position);
    }

    private double rpmToTicksPerSecond(double rpm) {
        return rpm * SHOOTER_ENCODER_TICKS_PER_REV / 60.0;
    }

    private double ticksPerSecondToRpm(double ticksPerSecond) {
        return Math.abs(ticksPerSecond) * 60.0 / SHOOTER_ENCODER_TICKS_PER_REV;
    }

    private boolean isShooterAtSpeed() {
        double flywheelRpm = ticksPerSecondToRpm(flywheel.getVelocity());
        double counterRollerRpm = ticksPerSecondToRpm(counterRoller.getVelocity());
        return flywheelTargetRpm > 0.0
                && counterRollerTargetRpm > 0.0
                && flywheelRpm >= flywheelTargetRpm * SHOOTER_READY_FRACTION
                && counterRollerRpm >= counterRollerTargetRpm * SHOOTER_READY_FRACTION;
    }

    private void stopShooter() {
        flywheel.setVelocity(0.0);
        counterRoller.setVelocity(0.0);
    }
}
