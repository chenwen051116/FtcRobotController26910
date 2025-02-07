package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "Testllcolor")
public class TestLlColor extends LinearOpMode {

    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        imu = hardwareMap.get(IMU.class, "imu");
        limelight = hardwareMap.get(Limelight3A.class, "lm");

        limelight.setPollRateHz(100);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                )
        );

//// Initialize IMU using Parameters
//        imu.initialize(myIMUparameters);
        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                // Access general information
                Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);
                telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

                if (result.isValid()) {
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("txnc", result.getTxNC());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("tync", result.getTyNC());

//                    // Access fiducial results
//                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
//                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
//                        telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(),fr.getTargetXDegrees(), fr.getTargetYDegrees());
//                        Pose3D botposeb = fr.getRobotPoseTargetSpace();
//                        if (botposeb != null) {
//                            double x = botposeb.getPosition().x;
//                            double y = botposeb.getPosition().y;
//
//                            telemetry.addData("MT1 Location", "(" + x + ", " + y + ")");
//                        }
//                    }
                    //double robotYaw = imu.getAngularOrientation().firstAngle;
                    double robotYaw = imu.getRobotYawPitchRollAngles().getYaw();
                    limelight.updateRobotOrientation(robotYaw);
                    if (result != null && result.isValid()) {
                        Pose3D botpose_mt2 = result.getBotpose_MT2();
                        if (botpose_mt2 != null) {
                            double x = botpose_mt2.getPosition().x;
                            double y = botpose_mt2.getPosition().y;
                            telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
                        }
                    }


                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }

            telemetry.update();
        }
        limelight.stop();
    }
}