package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Autonomous
public class LimelightApril extends OpMode{
    
    private Limelight3A limelight3A;
    
    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(8);
    }
    
    @Override
    public void start() {
        limelight3A.start();
    }
    @Override


    @Override
    public void loop() {
        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose();
            telemetry.addData("Target X", botPose.getTx());
            telemetry.addData("Target Y", botPose.getTy());
            telemetry.addData("Target Z", botPose.getTa());
            telemetry.addData("BotPose", botPose.toString());
            telemetry.addData("Yaw", botPose.getOrientation().getYaw());
        }
    }
}
