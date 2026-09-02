package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Intake subsystem that supports continuous-rotation side wheels and a roller connected to either
 * a motor, CRServo, or regular Servo (for continuous-rotation servos).
 */
public class Intake {
    private enum SideWheelType { NONE, CRSERVO, DCMOTOR, POS_SERVO }

    private CRServo leftCR, rightCR;
    private DcMotor leftMotor, rightMotor;
    private Servo leftPosServo, rightPosServo;

    private DcMotor intakeRollerMotor;
    private CRServo intakeRollerCR;
    private Servo intakeRollerServo;
    private boolean rollerConnected;

    private SideWheelType sideWheelType = SideWheelType.NONE;

    public void init(HardwareMap hwMap) {
        // Side wheels: prefer CRServo, then DC motor, then regular servo fallback
        leftCR = hwMap.tryGet(CRServo.class, "left_intake_servo");
        rightCR = hwMap.tryGet(CRServo.class, "right_intake_servo");
        if (leftCR != null && rightCR != null) {
            sideWheelType = SideWheelType.CRSERVO;
            leftCR.setDirection(CRServo.Direction.FORWARD);
            rightCR.setDirection(CRServo.Direction.REVERSE);
        } else {
            leftMotor = hwMap.tryGet(DcMotor.class, "left_intake_servo");
            rightMotor = hwMap.tryGet(DcMotor.class, "right_intake_servo");
            if (leftMotor != null && rightMotor != null) {
                sideWheelType = SideWheelType.DCMOTOR;
                leftMotor.setDirection(DcMotor.Direction.FORWARD);
                rightMotor.setDirection(DcMotor.Direction.REVERSE);
                leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftMotor.setPower(0.0);
                rightMotor.setPower(0.0);
            } else {
                leftPosServo = hwMap.tryGet(Servo.class, "left_intake_servo");
                rightPosServo = hwMap.tryGet(Servo.class, "right_intake_servo");
                if (leftPosServo != null && rightPosServo != null) {
                    sideWheelType = SideWheelType.POS_SERVO;
                    leftPosServo.setPosition(0.5);
                    rightPosServo.setPosition(0.5);
                }
            }
        }

        // Main intake roller: allows motor, CRServo, or regular servo.
        intakeRollerMotor = hwMap.tryGet(DcMotor.class, "intake_roller_motor");
        intakeRollerCR = hwMap.tryGet(CRServo.class, "intake_roller_motor");
        intakeRollerServo = hwMap.tryGet(Servo.class, "intake_roller_motor");
        rollerConnected = intakeRollerMotor != null || intakeRollerCR != null || intakeRollerServo != null;

        if (intakeRollerMotor != null) {
            intakeRollerMotor.setDirection(DcMotor.Direction.FORWARD);
            intakeRollerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeRollerMotor.setPower(0.0);
        }
        if (intakeRollerCR != null) {
            intakeRollerCR.setDirection(CRServo.Direction.FORWARD);
            intakeRollerCR.setPower(0.0);
        }
        if (intakeRollerServo != null) {
            intakeRollerServo.setPosition(0.5);
        }
    }

    public boolean isRollerConnected() {
        return rollerConnected;
    }

    public SideWheelType getSideWheelType() {
        return sideWheelType;
    }

    public void setSideWheelPower(double leftPower, double rightPower) {
        switch (sideWheelType) {
            case CRSERVO:
                leftCR.setPower(leftPower);
                rightCR.setPower(rightPower);
                break;
            case DCMOTOR:
                leftMotor.setPower(leftPower);
                rightMotor.setPower(rightPower);
                break;
            case POS_SERVO:
                leftPosServo.setPosition(servoPowerToPosition(leftPower));
                rightPosServo.setPosition(servoPowerToPosition(rightPower));
                break;
            case NONE:
            default:
                break;
        }
    }

    public void spinSideWheels(double leftPower, double rightPower) {
        setSideWheelPower(leftPower, rightPower);
    }

    public void spinSideWheels(double power) {
        setSideWheelPower(power, power);
    }

    public void stopSideWheels() {
        setSideWheelPower(0.0, 0.0);
    }

    public void setRollerPower(double power) {
        if (intakeRollerMotor != null) {
            intakeRollerMotor.setPower(power);
            return;
        }
        if (intakeRollerCR != null) {
            intakeRollerCR.setPower(power);
            return;
        }
        if (intakeRollerServo != null) {
            intakeRollerServo.setPosition(servoPowerToPosition(power));
        }
    }

    public void intake() {
        setRollerPower(1.0);
        spinSideWheels(0.75, 0.75);
    }

    public void eject() {
        setRollerPower(-1.0);
        spinSideWheels(-0.75, -0.75);
    }

    public void stop() {
        stopSideWheels();
        setRollerPower(0.0);
    }

    private double servoPowerToPosition(double power) {
        double clamped = Math.max(-1.0, Math.min(1.0, power));
        return 0.5 + (clamped * 0.5);
    }
}
