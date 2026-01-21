package org.firstinspires.ftc.teamcode.extras;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
//import org.firstinspires.ftc.teamcode.Utils.utilities.ServoMapper;

import java.util.ArrayList;
import java.util.List;

//@Config
//@TeleOp
@Disabled
@Deprecated
public class colorTest extends LinearOpMode {

    //TODO ---------------------------COLOR Sensor

    public NormalizedRGBA clrfrontIntakergba;
    public float[] clrfrontIntakehsv;
    public double clrfrontIntakedistance;

    public NormalizedRGBA clrbackIntakergba;
    public float[] clrbackIntakehsv;
    public double clrbackIntakedistance;

    //TODO ---------------------------Flags

    public static boolean frontIntakeON = false;
    public static boolean purple = false;
    public static boolean frontpurpleIN = false;

    public static boolean beamBack = false;

    public static List<Action> runningActions = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {


        //TODO ---------------------------COLOR Gain

//        robot.clrfrontIntake.setGain((float) 50);
//        robot.clrbackIntake.setGain((float) 50);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (opModeInInit()) {
            runningActions = updateAction();


            //TODO --------------------------- Color detection
//            clrfrontIntakergba = robot.clrfrontIntake.getNormalizedColors();
//            clrfrontIntakedistance = robot.clrfrontIntake.getDistance(DistanceUnit.MM);
            clrfrontIntakehsv = rgbToHsv(clrfrontIntakergba.red, clrfrontIntakergba.green, clrfrontIntakergba.blue);

//            clrbackIntakergba = robot.clrbackIntake.getNormalizedColors();
//            clrbackIntakedistance = robot.clrbackIntake.getDistance(DistanceUnit.MM);
            clrbackIntakehsv = rgbToHsv(clrbackIntakergba.red, clrbackIntakergba.green, clrbackIntakergba.blue);


            telemetry.addLine();
            printColorSensor();
            telemetry.update();
        }


        waitForStart();

        while (opModeIsActive()){
            runningActions = updateAction();

            //TODO --------------------------- Color detection
//            clrfrontIntakergba = robot.clrfrontIntake.getNormalizedColors();
//            clrfrontIntakedistance = robot.clrfrontIntake.getDistance(DistanceUnit.MM);
            clrfrontIntakehsv = rgbToHsv(clrfrontIntakergba.red, clrfrontIntakergba.green, clrfrontIntakergba.blue);

//            clrbackIntakergba = robot.clrbackIntake.getNormalizedColors();
//            clrbackIntakedistance = robot.clrbackIntake.getDistance(DistanceUnit.MM);
            clrbackIntakehsv = rgbToHsv(clrbackIntakergba.red, clrbackIntakergba.green, clrbackIntakergba.blue);

            // TODO HSV PURPLE
//            if (((hsv[0] < 26) && (hsv[0] > 18) && distance < 15)) {

            if (((clrfrontIntakehsv[0] < 230) && (clrfrontIntakehsv[0] > 180) && clrfrontIntakedistance < 30)) {
                purple = true;
            }
//
//            //TODO HSV YELLOW
////          else if (((hsv[0] < 80) && (hsv[0] > 55) && (hsv[2] > 0.95)  && distance < 15)) {
//
//            else if (((hsv[0] < 85) && (hsv[0] > 59.8) && (hsv[2] > 0.95)  && distance < 15)) {
//                Globals.intakeItem = 2;
//            }
//
//            // TODO HSV BLUE
////          else if (((hsv[0] < 235) && (hsv[0] > 210) && distance < 15)) {
//
//            else if (((hsv[0] < 225) && (hsv[0] > 185) && distance < 15)) {
//                Globals.intakeItem = 3;
//            }
//            else {
//                Globals.intakeItem = 0;
//            }


            //TODO USING COLOR SENSOR
//            if (purple) {
//                runningActions.add(INITSeq.InitAction(frontIntakeSubsystem, backIntakeSubsystem, shooterSubsystem));
//                purple= false;
//                frontpurpleIN=true;
//            }
//
//            if (frontpurpleIN) {
//                if (gamepad1.y) {
//                    runningActions.add(FrontShootSeq.FrontShoot3Action(frontIntakeSubsystem, backIntakeSubsystem, shooterSubsystem));
//                }
//            }
//
//            if (gamepad1.b){
//                frontIntakeON = false;
//                purple= false;
//                frontpurpleIN =false;
//            }

            printFlags();
            telemetry.addLine();

            telemetry.addLine();
            printColorSensor();
            telemetry.update();
        }
    }

    public void printFlags(){
        telemetry.addData("frontintakeON",frontIntakeON);
        telemetry.addData("Purple",purple);
        telemetry.addData("frontpurpleIN",frontpurpleIN);
        telemetry.addData("Beam Back",beamBack);
//        telemetry.addData("ballDetected at back",ballDetected);
    }

    public void printColorSensor(){
        telemetry.addData("Front CLR Distance",clrfrontIntakedistance );

        telemetry.addLine()
                .addData("HUE","%.3f",clrfrontIntakehsv[0])
                .addData("Saturation","%.3f", clrfrontIntakehsv[1])
                .addData("Value","%.3f", clrfrontIntakehsv[2]);
        telemetry.addLine()
                .addData("RED","%.3f",clrfrontIntakergba.red)
                .addData("GREEN","%.3f", clrfrontIntakergba.green)
                .addData("BLUE","%.3f", clrfrontIntakergba.blue);

        telemetry.addData("Back CLR Distance",clrbackIntakedistance );

        telemetry.addLine()
                .addData("HUE","%.3f",clrbackIntakehsv[0])
                .addData("Saturation","%.3f", clrbackIntakehsv[1])
                .addData("Value","%.3f", clrbackIntakehsv[2]);
        telemetry.addLine()
                .addData("RED","%.3f",clrbackIntakergba.red)
                .addData("GREEN","%.3f", clrbackIntakergba.green)
                .addData("BLUE","%.3f", clrbackIntakergba.blue);
    }
    public float[] rgbToHsv(float rNorm, float gNorm, float bNorm) {
        float[] hsv = new float[3];

        float max = Math.max(rNorm, Math.max(gNorm, bNorm));
        float min = Math.min(rNorm, Math.min(gNorm, bNorm));
        float delta = max - min;
        // Value
        hsv[2] = max;

        // Saturation
        hsv[1] = max == 0 ? 0 : delta / max;

        // Hue
        if (delta == 0) {
            hsv[0] = 0;
        } else {
            if (max == rNorm) {
                hsv[0] = (60 * ((gNorm - bNorm) / delta) + 360) % 360;
            } else if (max == gNorm) {
                hsv[0] = (60 * ((bNorm - rNorm) / delta) + 120) % 360;
            } else if (max == bNorm) {
                hsv[0] = (60 * ((rNorm - gNorm) / delta) + 240) % 360;
            }
        }
        return hsv;
    }

    public static List<Action> updateAction(){
        TelemetryPacket packet = new TelemetryPacket();
        List<Action> newActions = new ArrayList<>();
        List<Action> RemovableActions = new ArrayList<>();

        for (Action action : runningActions) {

            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        return newActions;
    }
}
