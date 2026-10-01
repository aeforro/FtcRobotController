/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
j *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.OpModes;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot with Mecanum
 * Wheels for the 2026-2027 FIRST® Tech Challenge. On top of a mecanum wheel drivetrain, it uses
 * one motor driving an intake roller, two servos which pull elements out of corners, two high-speed
 * launcher motors, and separate pollen and nectar gate servos.
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
 */

@TeleOp(name = "Mec BioBuzz StarterBot Teleop", group = "StarterBot")
//@Disabled
public class BioBuzzStarterbotTeleopMecanum extends OpMode {

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx pollenLauncher = null;
    private DcMotorEx nectarLauncher = null;
    private DcMotor intake = null;

    private Servo pollenGate = null;
    private Servo nectarGate = null;


    /*
     * These two variables are used to control the velocity of the launcher motor.
     * They are both in encoder ticks per second. The motors we use in the FIRST Tech Challenge
     * have encoders with a resolution of 28 ticks per revolution. We can convert this to RPM
     * by dividing the value by 28, to get to revolutions per second, before multiplying by 60
     * to get revolutions per minute.
     * We pass the target velocity variable to our motor to set the goal. We use the min velocity
     * in the launch() function to only run the windmill servo when the motor is spinning fast
     * enough to make a successful throw.
     */
    public final int LAUNCHER_TARGET_VELOCITY = 1650;
    public final int LAUNCHER_MIN_VELOCITY = 1580;
    private static final double SHOOTER_READY_DWELL_SECONDS = 0.25;
    private static final double STRAFE_DEADBAND = 0.08;
    private static final double INITIAL_GATE_POSITION = 1.0;
    private static final double POLLEN_GATE_CLOSED_POSITION = 0.8;
    private static final double POLLEN_GATE_OPEN_POSITION = 0.2;
    private static final double NECTAR_GATE_CLOSED_POSITION = 1.0;
    private static final double NECTAR_GATE_OPEN_POSITION = 0.0;
    private final ElapsedTime shooterReadyTimer = new ElapsedTime();
    private boolean shooterReadyTiming;


    /*
     * These four variables store the power we need to apply to the motors. In other cases, we may
     * choose to declare these variables inside the mecanumDrive() function, instead we declare them
     * here so that we can access them in our main loop for telemetry.
     */
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    // Create a variable to set to the intake.
    double intakePower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        pollenLauncher = hardwareMap.get(DcMotorEx.class, "pollenLauncher");
        nectarLauncher = hardwareMap.get(DcMotorEx.class, "nectarLauncher");
        pollenGate = hardwareMap.get(Servo.class, "pollen_gate");
        nectarGate = hardwareMap.get(Servo.class, "nectar_gate");
        nectarGate.setDirection(Servo.Direction.REVERSE);

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */

        pollenLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        nectarLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        pollenLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        nectarLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients launcherPidf = new PIDFCoefficients(40, 0, 0, 12.5);
        pollenLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherPidf);
        nectarLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherPidf);


        pollenGate.setPosition(INITIAL_GATE_POSITION);
        nectarGate.setPosition(INITIAL_GATE_POSITION);

        /*
         * Much like our drivetrain motors, we set the right intake servo to reverse so that both
         * servos work to pull elements into the intake.
         */


        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        /*
         * Here we call a function called mecanumDrive. The mecanumDrive function takes the input from
         * the joysticks, and applies power to the drive motors to move the robot as requested
         * by the driver. Moving the left joystick forwards/back moves all motors forwards/back,
         * moving the right joystick left/right rotates the robot clockwise or counterclockwise,
         * and moving the left joystick left moves the motors in the right way to create a sideways
         * "strafe" movement. Combinations of these inputs can be used to create more complex maneuvers.
         * Note, moving the joystick forward on most gamepads results in a negative signal, so
         * we invert it before passing it to the function.
         */
        double rawStrafeInput = gamepad1.left_stick_x;
        double strafeInput = rawStrafeInput;
        if (Math.abs(strafeInput) < STRAFE_DEADBAND) {
            strafeInput = 0;
        }
        mecanumDrive(-gamepad1.left_stick_y, strafeInput, gamepad1.right_stick_x);

        if (gamepad1.left_trigger > 0.1) {
            intakePower = -1.0;
        } else if (gamepad1.right_trigger > 0.1) {
            intakePower = 1.0;
        } else {
            intakePower = 0.0;
        }

        launch();

        /*
         * Here we set our intake motor and servos to their intake power. The order of operations
         * here is important though. The gamepad triggers define the starting point for the intake
         * power variable in each loop of our code, but inside our launch function we also sometimes
         * change the intake power. So we need to give our launch function a chance to modify the
         * variable before we write it to our motor and servos.
         */

        intake.setPower(intakePower);


        /*
         * Show motor powers on the Driver Station via telemetry.
         */
        
        telemetry.addData("Drive powers", "LF %.2f, RF %.2f, LB %.2f, RB %.2f",
                leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        telemetry.addData("Strafe input", "raw %.3f, applied %.3f", rawStrafeInput, strafeInput);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        intake.setPower(0);
        pollenLauncher.setVelocity(0);
        nectarLauncher.setVelocity(0);
        pollenGate.setPosition(POLLEN_GATE_CLOSED_POSITION);
        nectarGate.setPosition(NECTAR_GATE_CLOSED_POSITION);
    }

    void mecanumDrive(double forward, double strafe, double rotate) {
        leftFrontPower = forward + strafe + rotate;
        rightFrontPower = forward - strafe - rotate;
        leftBackPower = forward - strafe + rotate;
        rightBackPower = forward + strafe - rotate;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        /*
         * Send calculated power to wheels
         */
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    void launch() {
        boolean pollenRequested = gamepad2.left_bumper || gamepad2.a;
        boolean nectarRequested = gamepad2.right_bumper;
        boolean shootingRequested = pollenRequested || nectarRequested;

        if (shootingRequested) {
            pollenLauncher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            nectarLauncher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else {
            pollenLauncher.setVelocity(0);
            nectarLauncher.setVelocity(0);
        }

        double pollenVelocity = Math.abs(pollenLauncher.getVelocity());
        double nectarVelocity = Math.abs(nectarLauncher.getVelocity());
        boolean shooterAtSpeed =
                pollenVelocity >= LAUNCHER_MIN_VELOCITY
                        && nectarVelocity >= LAUNCHER_MIN_VELOCITY;
        if (shootingRequested && shooterAtSpeed) {
            if (!shooterReadyTiming) {
                shooterReadyTimer.reset();
                shooterReadyTiming = true;
            }
        } else {
            shooterReadyTiming = false;
        }
        boolean shooterReady = shooterReadyTiming
                && shooterReadyTimer.seconds() >= SHOOTER_READY_DWELL_SECONDS;
        telemetry.addData("Shooter velocity (ticks/s)", "pollen %.0f, nectar %.0f",
                pollenVelocity, nectarVelocity);
        telemetry.addData("Shooter ready", shooterReady);

        pollenGate.setPosition(
                gamepad2.a || (gamepad2.left_bumper && shooterReady)
                        ? POLLEN_GATE_OPEN_POSITION
                        : POLLEN_GATE_CLOSED_POSITION);
        nectarGate.setPosition(
                nectarRequested && shooterReady
                        ? NECTAR_GATE_OPEN_POSITION
                        : NECTAR_GATE_CLOSED_POSITION);
    }
}