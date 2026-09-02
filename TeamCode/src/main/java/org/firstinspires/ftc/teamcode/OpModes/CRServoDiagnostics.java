package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "CRServo Diagnostics", group = "Team")
public class CRServoDiagnostics extends LinearOpMode {
    @Override
    public void runOpMode() {
        CRServo leftCR = hardwareMap.tryGet(CRServo.class, "left_intake_servo");
        CRServo rightCR = hardwareMap.tryGet(CRServo.class, "right_intake_servo");
        DcMotor leftMotor = hardwareMap.tryGet(DcMotor.class, "left_intake_servo");
        DcMotor rightMotor = hardwareMap.tryGet(DcMotor.class, "right_intake_servo");
        Servo leftPos = hardwareMap.tryGet(Servo.class, "left_intake_servo");
        Servo rightPos = hardwareMap.tryGet(Servo.class, "right_intake_servo");

        telemetry.addData("leftCR", leftCR != null);
        telemetry.addData("rightCR", rightCR != null);
        telemetry.addData("leftMotor", leftMotor != null);
        telemetry.addData("rightMotor", rightMotor != null);
        telemetry.addData("leftPosServo", leftPos != null);
        telemetry.addData("rightPosServo", rightPos != null);
        telemetry.addLine("Controls: A = spin in, B = spin out, X = stop, Y = toggle hold");
        telemetry.addLine("If `leftCR`/`rightCR` true but servo returns to center, likely hardware not true CRServo");
        telemetry.update();

        boolean hold = false;
        double lastA = 0.0;

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // Controls
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;
            boolean y = gamepad1.y;

            if (y && !hold) {
                hold = true;
            } else if (y && hold) {
                hold = false;
            }

            double desired = 0.0;
            if (a) desired = 1.0; // full in
            else if (b) desired = -1.0; // full out
            else if (x) desired = 0.0; // stop

            // If holding, keep last desired
            if (hold) {
                // do nothing, keep previous desired
            }

            // Prefer CRServo if present
            if (leftCR != null || rightCR != null) {
                if (leftCR != null) leftCR.setPower(desired);
                if (rightCR != null) rightCR.setPower(desired);
            } else if (leftMotor != null || rightMotor != null) {
                if (leftMotor != null) leftMotor.setPower(desired);
                if (rightMotor != null) rightMotor.setPower(desired);
            } else if (leftPos != null || rightPos != null) {
                // Map desired (-1..1) to position (0..1)
                double pos = (desired + 1.0) / 2.0;
                if (leftPos != null) leftPos.setPosition(pos);
                if (rightPos != null) rightPos.setPosition(pos);
            }

            telemetry.addData("desired", desired);
            telemetry.addData("hold", hold);
            telemetry.addData("leftCR.set?", leftCR != null);
            telemetry.addData("rightCR.set?", rightCR != null);
            telemetry.update();

            sleep(20);
        }
    }
}
