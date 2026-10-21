package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Intake subsystem for the shooter turret robot: one motor that pulls balls in.
 */
public class Intake {
    private DcMotor intakeRoller;

    public void init(HardwareMap hwMap) {
        intakeRoller = hwMap.get(DcMotor.class, "intake_roller_motor");

        intakeRoller.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRoller.setPower(0.0);
    }

    public void setPower(double power) {
        intakeRoller.setPower(power);
    }

    public double getPower() {
        return intakeRoller.getPower();
    }

    public void stop() {
        setPower(0.0);
    }
}
