package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Turret subsystem: two position servos driven together. Holding a turn button moves the turret
 * at a fixed speed, scaled by loop time so it turns the same regardless of loop rate.
 */
public class Turret {
    public static final double MIN_POSITION = 0.05;
    public static final double MAX_POSITION = 0.95;
    public static final double SPEED = 5.0;

    private Servo leftTurretServo;
    private Servo rightTurretServo;
    private final ElapsedTime loopTimer = new ElapsedTime();

    private double position = 0.5;

    public void init(HardwareMap hwMap) {
        leftTurretServo = hwMap.get(Servo.class, "turret_left_servo");
        rightTurretServo = hwMap.get(Servo.class, "turret_right_servo");

        leftTurretServo.setDirection(Servo.Direction.FORWARD);
        rightTurretServo.setDirection(Servo.Direction.FORWARD);
        setPosition(position);
    }

    /** Call right after waitForStart() so the first update doesn't count time spent in init. */
    public void start() {
        loopTimer.reset();
    }

    public void update(boolean turnLeft, boolean turnRight) {
        double elapsedSeconds = Math.min(loopTimer.seconds(), 0.05);
        loopTimer.reset();

        if (turnLeft && !turnRight) {
            position += SPEED * elapsedSeconds;
        } else if (turnRight && !turnLeft) {
            position -= SPEED * elapsedSeconds;
        }

        setPosition(position);
    }

    public void setPosition(double newPosition) {
        position = Math.max(MIN_POSITION, Math.min(MAX_POSITION, newPosition));
        leftTurretServo.setPosition(position);
        rightTurretServo.setPosition(position);
    }

    public double getPosition() {
        return position;
    }
}
