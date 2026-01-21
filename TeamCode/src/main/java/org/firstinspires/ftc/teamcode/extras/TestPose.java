package org.firstinspires.ftc.teamcode.extras;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

//@TeleOp
@Deprecated
@Disabled
public class TestPose extends LinearOpMode {

    private Limelight3A limelight;


    @Override
    public void runOpMode() throws InterruptedException
    {


        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(5);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {


            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());


            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                // Access general information
                Pose3D botpose = result.getBotpose();



                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);
                telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

                telemetry.addData("tx", result.getTx());
                telemetry.addData("txnc", result.getTxNC());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("tync", result.getTyNC());

                telemetry.addData("Botpose", botpose.toString());
                telemetry.addData("Botpose x", botpose.getPosition().x);
                telemetry.addData("Botpose y", botpose.getPosition().y);
                telemetry.addData("Botpose z", botpose.getPosition().z);
                telemetry.addData("AVERAGE DISTANCE", result.getBotposeAvgDist());

                telemetry.addData("Botpose adjusted pedro x", botpose.getPosition().x + 72);
                telemetry.addData("Botpose adjusted pedro y", botpose.getPosition().y + 72);
                telemetry.addData("Botpose adjusted pedro z", botpose.getPosition().z + 72);


                if (gamepad1.dpad_up) {

                    int[] validIDs = {3, 4};

                    //limelight.updateRobotOrientation(0.0);
                    telemetry.addData("MT2", result.getBotpose_MT2().toString());
                    telemetry.addData("Botpose mt2 x", result.getBotpose_MT2().getPosition().x);
                    telemetry.addData("Botpose mt2 y" , result.getBotpose_MT2().getPosition().y);
                    telemetry.addData("Botpose mt2 z", result.getBotpose_MT2().getPosition().z);
                }




                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }

            telemetry.update();
        }
        limelight.stop();
    }
}
