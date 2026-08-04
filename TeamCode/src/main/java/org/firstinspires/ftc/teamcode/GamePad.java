package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class GamePad extends OpMode {

    @Override
    public void init() {
        telemetry.addData("GamePad", "Initialized");
    }

    @Override
    public void loop() {
        telemetry.addData("Left Stick X", gamepad1.left_stick_x);
        telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
        telemetry.addData("Right Stick X", gamepad1.right_stick_x);
        telemetry.addData("Right Stick Y", gamepad1.right_stick_y);
        telemetry.addData("Left Trigger", gamepad1.left_trigger);
        telemetry.addData("Right Trigger", gamepad1.right_trigger);
        telemetry.addData("A Button", gamepad1.a);
        telemetry.addData("B Button", gamepad1.b);
        telemetry.addData("X Button", gamepad1.x);
        telemetry.addData("Y Button", gamepad1.y);
        telemetry.addData("D-Pad Up", gamepad1.dpad_up);
        telemetry.addData("D-Pad Down", gamepad1.dpad_down);
        telemetry.addData("D-Pad Left", gamepad1.dpad_left);
        telemetry.addData("D-Pad Right", gamepad1.dpad_right);
        telemetry.update();
    }
}