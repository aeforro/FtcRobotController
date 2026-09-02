package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;

@TeleOp(name = "Limelight: Follow Tag (A toggle)", group = "Concept")
public class LimelightFollowTeleOp extends OpMode {
    private MecanumDrive drive = new MecanumDrive();
    private Limelight3A limelight;

    // Tunable gains
    private final double KP_TURN = 0.015;      // deg -> rotate
    private final double KP_AREA = 0.03;       // area -> forward
    private final double MAX_FORWARD = 0.6;
    private final double MAX_TURN = 0.5;

    // Desired target area (tweak for your tag distance). Larger -> closer.
    private final double DESIRED_AREA = 5.0;

    // If limelight camera is mounted facing backwards on the robot set this to true
    private final boolean cameraFacingBack = true;

    private boolean followMode = false;
    private boolean prevA = false;

    @Override
    public void init() {
        drive.init(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        // Choose pipeline that detects AprilTags (adjust if your pipeline index differs)
        limelight.pipelineSwitch(8);
        limelight.start();
        telemetry.addLine("Limelight follow ready. Press A to toggle follow mode.");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean a = gamepad1.a;
        // toggle on button edge
        if (a && !prevA) {
            followMode = !followMode;
        }
        prevA = a;

        if (followMode) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx(); // horizontal offset degrees
                double ta = result.getTa(); // target area (percent)

                // Steering to center the target
                double turn = clamp(-tx * KP_TURN, -MAX_TURN, MAX_TURN);
                // Move forward/back based on difference between desired area and current area
                double forward = clamp((DESIRED_AREA - ta) * KP_AREA, -MAX_FORWARD, MAX_FORWARD);

                // If camera is facing backwards relative to robot, invert controls
                if (cameraFacingBack) {
                    forward = -forward;
                    turn = -turn;
                }

                // No strafing in follow mode; you can add strafe control if needed
                drive.drive(forward, 0.0, turn);

                telemetry.addData("Follow", "ON");
                telemetry.addData("tx", "%.2f", tx);
                telemetry.addData("ta", "%.2f", ta);
                telemetry.addData("forward", "%.2f", forward);
                telemetry.addData("turn", "%.2f", turn);
            } else {
                // No valid tag: stop and notify
                drive.stop();
                telemetry.addData("Follow", "ON (no tag)");
            }
        } else {
            // Manual drive when follow is off
            double forward = -gamepad1.left_stick_y; // forward/back
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            drive.drive(forward, strafe, rotate);
            telemetry.addData("Follow", "OFF");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
        if (limelight != null) limelight.stop();
    }

    private double clamp(double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }
}
