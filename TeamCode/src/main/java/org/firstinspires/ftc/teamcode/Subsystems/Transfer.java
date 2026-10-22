package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Transfer subsystem: the roller that moves balls from the intake up into the shooter.
 */
public class Transfer {
    private DcMotor transferRoller;

    public void init(HardwareMap hwMap) {
        transferRoller = hwMap.get(DcMotor.class, "transfer_roller_motor");

        transferRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        transferRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferRoller.setPower(0.0);
    }

    public void setPower(double power) {
        transferRoller.setPower(power);
    }

    public double getPower() {
        return transferRoller.getPower();
    }

    public void stop() {
        setPower(0.0);
    }
}
