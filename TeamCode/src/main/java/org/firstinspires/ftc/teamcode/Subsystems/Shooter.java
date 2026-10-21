package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Shooter subsystem: a flywheel and a counter roller, both running closed-loop velocity control.
 */
public class Shooter {
    public static final double DEFAULT_FLYWHEEL_RPM = 2350.0;
    public static final double DEFAULT_COUNTER_ROLLER_RPM = 5550.0;
    public static final double MAX_RPM = 6000.0;
    public static final double RPM_ADJUSTMENT = 50.0;
    public static final double READY_FRACTION = 0.95;
    public static final double ENCODER_TICKS_PER_REV = 28.0;

    private DcMotorEx flywheel;
    private DcMotorEx counterRoller;

    private double flywheelTargetRpm = DEFAULT_FLYWHEEL_RPM;
    private double counterRollerTargetRpm = DEFAULT_COUNTER_ROLLER_RPM;

    public void init(HardwareMap hwMap) {
        flywheel = hwMap.get(DcMotorEx.class, "flywheel");
        counterRoller = hwMap.get(DcMotorEx.class, "counter_roller");

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        counterRoller.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        counterRoller.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        counterRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        stop();
    }

    public void spinUp() {
        flywheel.setVelocity(rpmToTicksPerSecond(flywheelTargetRpm));
        counterRoller.setVelocity(rpmToTicksPerSecond(counterRollerTargetRpm));
    }

    public void stop() {
        flywheel.setVelocity(0.0);
        counterRoller.setVelocity(0.0);
    }

    public boolean isAtSpeed() {
        return flywheelTargetRpm > 0.0
                && counterRollerTargetRpm > 0.0
                && getFlywheelRpm() >= flywheelTargetRpm * READY_FRACTION
                && getCounterRollerRpm() >= counterRollerTargetRpm * READY_FRACTION;
    }

    public void increaseFlywheelRpm() {
        flywheelTargetRpm = Math.min(MAX_RPM, flywheelTargetRpm + RPM_ADJUSTMENT);
    }

    public void decreaseFlywheelRpm() {
        flywheelTargetRpm = Math.max(0.0, flywheelTargetRpm - RPM_ADJUSTMENT);
    }

    public void increaseCounterRollerRpm() {
        counterRollerTargetRpm = Math.min(MAX_RPM, counterRollerTargetRpm + RPM_ADJUSTMENT);
    }

    public void decreaseCounterRollerRpm() {
        counterRollerTargetRpm = Math.max(0.0, counterRollerTargetRpm - RPM_ADJUSTMENT);
    }

    public double getFlywheelTargetRpm() {
        return flywheelTargetRpm;
    }

    public double getCounterRollerTargetRpm() {
        return counterRollerTargetRpm;
    }

    public double getFlywheelRpm() {
        return ticksPerSecondToRpm(flywheel.getVelocity());
    }

    public double getCounterRollerRpm() {
        return ticksPerSecondToRpm(counterRoller.getVelocity());
    }

    private double rpmToTicksPerSecond(double rpm) {
        return rpm * ENCODER_TICKS_PER_REV / 60.0;
    }

    private double ticksPerSecondToRpm(double ticksPerSecond) {
        return Math.abs(ticksPerSecond) * 60.0 / ENCODER_TICKS_PER_REV;
    }
}
