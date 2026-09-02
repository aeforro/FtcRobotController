package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcontroller.external.samples.externalhardware.RobotHardware;

public class MecanumDrive {
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private IMU imu;

    // Optional pinpoint
    private GoBildaPinpointDriver pinpoint = null;
    private boolean usePinpoint = false;

    public void init(HardwareMap hwMap) {

        frontLeftMotor = hwMap.get(DcMotor.class, "front_left_motor");
        backLeftMotor = hwMap.get(DcMotor.class, "back_left_motor");
        frontRightMotor = hwMap.get(DcMotor.class, "front_right_motor");
        backRightMotor = hwMap.get(DcMotor.class, "back_right_motor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        // Use encoder-based closed-loop speed control and BRAKE on zero power so the robot resists coasting
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);

        imu.initialize((new IMU.Parameters(RevOrientation)));
    }

    /**
     * Initialize using a RobotHardware object so subsystems can prefer its configured Pinpoint.
     */
    public void init(HardwareMap hwMap, RobotHardware robotHardware) {
        init(hwMap);
        if (robotHardware != null && robotHardware.hasPinpoint()) {
            pinpoint = robotHardware.getPinpoint();
            usePinpoint = (pinpoint != null);
        }
    }

    public void drive(double forward, double strafe, double rotate) {
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = Math.max(1.0, Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(backLeftPower),
                        Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)))));

        double maxSpeed = 1.0;
        //Change this if robot is too fast for kids to control

        double scale = maxSpeed / maxPower;
        frontLeftMotor.setPower(frontLeftPower * scale);
        backLeftMotor.setPower(backLeftPower * scale);
        frontRightMotor.setPower(frontRightPower * scale);
        backRightMotor.setPower(backRightPower * scale);
    }

    public void stop() {
        frontLeftMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
    }

    public void setBrakeMode(boolean brake) {
        DcMotor.ZeroPowerBehavior behavior = brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        frontLeftMotor.setZeroPowerBehavior(behavior);
        backLeftMotor.setZeroPowerBehavior(behavior);
        frontRightMotor.setZeroPowerBehavior(behavior);
        backRightMotor.setZeroPowerBehavior(behavior);
    }

    public void driveFieldRelative(double forward, double strafe, double rotate) {
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        double robotYawRad;
        if (usePinpoint && pinpoint != null) {
            pinpoint.update();
            robotYawRad = Math.toRadians(pinpoint.getPosition().getHeading(AngleUnit.DEGREES));
        } else {
            robotYawRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }

        theta = AngleUnit.normalizeRadians(theta - robotYawRad);

        double newforward = r * Math.sin(theta);
        double newstrafe = r * Math.cos(theta);

        this.drive(newforward, newstrafe, rotate);
    }
}

