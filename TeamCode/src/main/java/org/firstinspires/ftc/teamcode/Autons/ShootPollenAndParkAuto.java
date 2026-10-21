package org.firstinspires.ftc.teamcode.Autons;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Shoot Pollen and Park", group = "Saline")
public class ShootPollenAndParkAuto extends LinearOpMode {
    private static final double LAUNCHER_TARGET_VELOCITY = 1700;

    private static final double POLLEN_GATE_CLOSED_POSITION = 0.8;
    private static final double POLLEN_GATE_OPEN_POSITION = 0.2;
    private static final double INITIAL_GATE_POSITION = 1.0;

    private static final double DRIVE_POWER = 0.5;
    private static final double BACKUP_SECONDS = 0.8;
    private static final double FIRST_STRAFE_RIGHT_SECONDS = 1.4;
    private static final double SECOND_BACKUP_SECONDS = 2.7;
    private static final double FINAL_STRAFE_RIGHT_SECONDS = 0.6;
    private static final double SHOOTER_SPINUP_SECONDS = 1.0;
    private static final double POLLEN_FEED_SECONDS = 4.0;

    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotorEx pollenLauncher;
    private DcMotorEx nectarLauncher;
    private Servo pollenGate;

    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        pollenLauncher = hardwareMap.get(DcMotorEx.class, "pollenLauncher");
        nectarLauncher = hardwareMap.get(DcMotorEx.class, "nectarLauncher");
        pollenGate = hardwareMap.get(Servo.class, "pollen_gate");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        setDriveZeroPowerBehavior(BRAKE);

        pollenLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        nectarLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        pollenLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        nectarLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients launcherPidf = new PIDFCoefficients(40, 0, 0, 12.5);
        pollenLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherPidf);
        nectarLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherPidf);
        pollenGate.setPosition(INITIAL_GATE_POSITION);

        telemetry.addLine("Ready: shoot pollen, then back up, strafe right, drive forward, and strafe right");
        telemetry.update();

        try {
            waitForStart();
            if (!opModeIsActive()) {
                return;
            }

            pollenLauncher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            nectarLauncher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            sleep((long) (SHOOTER_SPINUP_SECONDS * 1000));

            if (opModeIsActive()) {
                pollenGate.setPosition(POLLEN_GATE_OPEN_POSITION);
                sleep((long) (POLLEN_FEED_SECONDS * 1000));
                pollenGate.setPosition(POLLEN_GATE_CLOSED_POSITION);
            }

            pollenLauncher.setVelocity(0);
            nectarLauncher.setVelocity(0);
            driveFor(-DRIVE_POWER, 0.0, BACKUP_SECONDS);
            driveFor(0.0, DRIVE_POWER, FIRST_STRAFE_RIGHT_SECONDS);
            driveFor(-DRIVE_POWER, 0.0, SECOND_BACKUP_SECONDS);
            driveFor(0.0, DRIVE_POWER, FINAL_STRAFE_RIGHT_SECONDS);
        } finally {
            stopDrive();
            pollenLauncher.setVelocity(0);
            nectarLauncher.setVelocity(0);
            pollenGate.setPosition(POLLEN_GATE_CLOSED_POSITION);
        }
    }

    private void driveFor(double forward, double strafe, double durationSeconds) {
        double leftFront = forward + strafe;
        double rightFront = forward - strafe;
        double leftBack = forward - strafe;
        double rightBack = forward + strafe;
        double maxPower = Math.max(
                1.0,
                Math.max(
                        Math.max(Math.abs(leftFront), Math.abs(rightFront)),
                        Math.max(Math.abs(leftBack), Math.abs(rightBack))));

        leftFrontDrive.setPower(leftFront / maxPower);
        rightFrontDrive.setPower(rightFront / maxPower);
        leftBackDrive.setPower(leftBack / maxPower);
        rightBackDrive.setPower(rightBack / maxPower);

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < durationSeconds) {
            idle();
        }
        stopDrive();
    }

    private void setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFrontDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
        leftBackDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
    }

    private void stopDrive() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}
