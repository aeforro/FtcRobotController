package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Flywheel Counter Roller", group = "Shooter")
public class FlywheelCounterRollerTeleOp extends OpMode {
    private static final double DEFAULT_FLYWHEEL_TARGET_RPM = 3000.0;
    private static final double DEFAULT_COUNTER_ROLLER_TARGET_RPM = 5000.0;
    private static final double RPM_ADJUSTMENT = 50.0;
    private static final double ENCODER_TICKS_PER_REV = 28.0;

    private DcMotorEx flywheel;
    private DcMotorEx counterRoller;
    private double flywheelTargetRpm = DEFAULT_FLYWHEEL_TARGET_RPM;
    private double counterRollerTargetRpm = DEFAULT_COUNTER_ROLLER_TARGET_RPM;
    private boolean previousDpadUp;
    private boolean previousDpadDown;
    private boolean previousDpadRight;
    private boolean previousDpadLeft;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        counterRoller = hardwareMap.get(DcMotorEx.class, "counter_roller");

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        counterRoller.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        counterRoller.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stopShooters();

        telemetry.addData("Status", "Ready");
        telemetry.addData("Controls", "D-pad adjusts targets by 50 RPM; hold right bumper to run");
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up && !previousDpadUp) {
            flywheelTargetRpm += RPM_ADJUSTMENT;
        }
        if (gamepad1.dpad_down && !previousDpadDown) {
            flywheelTargetRpm = Math.max(0.0, flywheelTargetRpm - RPM_ADJUSTMENT);
        }
        if (gamepad1.dpad_right && !previousDpadRight) {
            counterRollerTargetRpm += RPM_ADJUSTMENT;
        }
        if (gamepad1.dpad_left && !previousDpadLeft) {
            counterRollerTargetRpm = Math.max(0.0, counterRollerTargetRpm - RPM_ADJUSTMENT);
        }
        previousDpadUp = gamepad1.dpad_up;
        previousDpadDown = gamepad1.dpad_down;
        previousDpadRight = gamepad1.dpad_right;
        previousDpadLeft = gamepad1.dpad_left;

        if (gamepad1.right_bumper) {
            flywheel.setVelocity(rpmToTicksPerSecond(flywheelTargetRpm));
            counterRoller.setVelocity(rpmToTicksPerSecond(counterRollerTargetRpm));
        } else {
            stopShooters();
        }

        double flywheelTicksPerSecond = flywheel.getVelocity();
        double counterRollerTicksPerSecond = counterRoller.getVelocity();
        telemetry.addData("Flywheel", "Target: %.0f RPM | Encoder: %.0f RPM",
                flywheelTargetRpm, ticksPerSecondToRpm(flywheelTicksPerSecond));
        telemetry.addData("Flywheel encoder", "%.0f ticks/s | calibration: %.0f ticks/rev | SDK type: %.1f",
                flywheelTicksPerSecond, ENCODER_TICKS_PER_REV, flywheel.getMotorType().getTicksPerRev());
        telemetry.addData("Flywheel power", "%.2f", flywheel.getPower());
        telemetry.addData("Counter roller", "Target: %.0f RPM | Encoder: %.0f RPM",
                counterRollerTargetRpm, ticksPerSecondToRpm(counterRollerTicksPerSecond));
        telemetry.addData("Counter encoder", "%.0f ticks/s | calibration: %.0f ticks/rev | SDK type: %.1f",
                counterRollerTicksPerSecond, ENCODER_TICKS_PER_REV,
                counterRoller.getMotorType().getTicksPerRev());
        telemetry.addData("Counter power", "%.2f", counterRoller.getPower());
        telemetry.addData("Shooter", gamepad1.right_bumper ? "Running" : "Stopped (hold right bumper)");
        telemetry.addData("Controls", "Flywheel: up/down | Roller: right/left (50 RPM)");
        telemetry.update();
    }

    @Override
    public void stop() {
        stopShooters();
    }

    private double rpmToTicksPerSecond(double rpm) {
        return rpm * ENCODER_TICKS_PER_REV / 60.0;
    }

    private double ticksPerSecondToRpm(double ticksPerSecond) {
        return Math.abs(ticksPerSecond) * 60.0 / ENCODER_TICKS_PER_REV;
    }

    private void stopShooters() {
        flywheel.setVelocity(0);
        counterRoller.setVelocity(0);
    }
}
