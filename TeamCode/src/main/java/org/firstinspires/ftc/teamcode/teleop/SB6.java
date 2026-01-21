package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.subsystem.MOTIF.zeroFlag;

import android.provider.Settings;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.dashboard.FtcDashboard;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
//import org.firstinspires.ftc.teamcode.Utils.utilities.ServoMapper;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
//import org.firstinspires.ftc.teamcode.sequences.teleOp.FrontIntakeSeq;
//import org.firstinspires.ftc.teamcode.sequences.teleOp.FrontShootSeq;
import org.firstinspires.ftc.teamcode.sequences.InitSeq;
import org.firstinspires.ftc.teamcode.sequences.IntakeSeq;
import org.firstinspires.ftc.teamcode.sequences.ShootSeq;
import org.firstinspires.ftc.teamcode.subsystem.Feeder;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.MOTIF;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.ArrayList;
import java.util.List;

@Config
//@TeleOp(name = "TELEOP SB6", group = "TELEOP")
public class SB6 extends LinearOpMode {
    private static RobotHardware robot=RobotHardware.getInstance();
    ElapsedTime intakeTimer;
    ElapsedTime motionTimer;
    ElapsedTime feedButtonTimer;
    private Intake intake;
    private Outtake outtake;
    private Feeder feeder;
    private MecanumDrive drive;
    double pos;
    private FtcDashboard dashboard;

    //TODO ---------------------------COLOR Sensor and Beam Breaks

    public NormalizedRGBA clrfrontIntakergba;
    public float[] clrfrontIntakehsv;
    public double clrfrontIntakedistance;

    public NormalizedRGBA clrbackIntakergba;
    public float[] clrbackIntakehsv;
    public double clrbackIntakedistance;


    //TODO -------------------Turret Constants--------

    public static double c;
    public static double target = 0;
    public static double error ;
    public static double integral;
    public static double previous_error;
    public double derivative;
    public static double pid;
    public double a;

    public static double kp = 0.02, ki = 0, kd = 0.02;

    public static double yaw = 0;
    private double totalAngle;
    private double angle;
    private double prevAngle;

    //TODO -------------------Shooter Constants--------

    public static double P = 30;
    public static double I = 0;
    public static double D = 0;
    public static double F = 12.5;
    double actualPos;
    //TODO -------------------Flags & Constants----------

    public static double buff= 100;
    public static double diffVel= 50;
    public static double OBeamcounter= 0;
    public static boolean OBeam;
    public static boolean IBeam;
    public static boolean FBeam;
    public static double rampTime= 200;
    public static double motionTime= 280;
    public static double thresh= 100;
    public static int counter= 0;
    public static int counterFeed= 0;
    public static int counterShootFeed= 0;
    public static int thirdFeed= 0;
    public static boolean currentOBeamstate= false;
    public static boolean lastOBeamstate= false;
    public static boolean feedToggle = false;
    public static boolean intakeToggle = false;
//    public static boolean Globals.Globals.intakeFlag = false;
    public static boolean oneFlag = false;
    public static boolean feedFlag = false;
    public static boolean zeroFlag = false;
    public static boolean thirdBallFlag = false;
    public static boolean motionFlag = false;
    public static boolean shooterFlag = false;
    public static boolean feedButtonFlag = false;
//    public static boolean Globals.Globals.intakeFlag2 = false;
    public static boolean shootFlagFar = false;
    public static boolean shootFlagNear = false;
    public static boolean shootStop = false;


    public static boolean Flag_x, Flag_b, Flag_y, Flag_a, Flag ;
    public double botHeading;


    public static List<Action> runningActions = new ArrayList<>();
    private boolean SFlag=false;
    private int counterB;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = RobotHardware.getInstance();
        robot.init(hardwareMap,telemetry);
        intakeTimer = new ElapsedTime();
        motionTimer = new ElapsedTime();
        feedButtonTimer = new ElapsedTime();


        intake = new Intake(robot);
        outtake = new Outtake(robot);
        feeder = new Feeder(robot);
        dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        robot.resetEncoder();

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        integral = 0;
        error = 0;
        previous_error = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (opModeInInit()) {
            drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));
            feedToggle = false;
//            Globals.Globals.intakeFlag = false;
            feedFlag = false;
            OBeamcounter = 0;
            counterFeed =0;
            counterB =0;
            intakeTimer.reset();
            motionTimer.reset();
            feedButtonTimer.reset();
            motionFlag = false;
zeroFlag();
            runningActions = updateAction();
            runningActions.add(InitSeq.InitAction(intake,outtake,feeder));
        }

        waitForStart();

        while (opModeIsActive()){
            robot.shooter.setVelocityPIDFCoefficients(P,I,D,F);
            runningActions = updateAction();
            botHeading = drive.localizer.getPose().heading.toDouble();

//            currentOBeamstate = robot.outtakeBeam.getState();
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x*0.5
            ));

//            drive.driveFieldCentric(-gamepad1.left_stick_x,
//                    -gamepad1.left_stick_y,
//                    gamepad1.right_stick_x,
//                    botHeading);
            drive.updatePoseEstimate();

//            drive.updatePoseEstimate();

//            if (lastOBeamstate != currentOBeamstate){
//                OBeamcounter += 1;
//            }

//            telemetry.update();

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            actualPos = robot.turretEncoder.getCurrentPosition();

            // Intake and Store Artifacts

            //TODO: BASIC

            if (gamepad1.touchpad)
            {
                drive.navxMicro.initialize();
            }


            if (currentGamepad2.x && !previousGamepad2.x){

                MOTIF.SORT_STATE(MOTIF.MOTIF_STATE.PGP, MOTIF.MOTIF_STATE.PPG);

//                motionTimer.reset();
//                motionFlag = false;
            }

//            if (Flag_x)
//            {
//
//            }
//            else
//            {
//                zeroFlag();
//            }




            if (currentGamepad2.b && !previousGamepad2.b){

//                Flag_b = !Flag_b;
                MOTIF.SORT_STATE(MOTIF.MOTIF_STATE.GPP, MOTIF.MOTIF_STATE.PPG);

//                motionTimer.reset();
//                motionFlag = false;
            }

//            if (Flag_b)
//            {
//
//            }
//            else {
//                zeroFlag();
//            }



            if (currentGamepad2.y && !previousGamepad2.y){

//                Flag_y = !Flag_y;
                MOTIF.SORT_STATE(MOTIF.MOTIF_STATE.GPP, MOTIF.MOTIF_STATE.PGP);

//                motionTimer.reset();
//                motionFlag = false;
            }

//            if (Flag_y)
//            {
//
//            }
//            else
//            {
//                zeroFlag();
//            }

            if (currentGamepad2.a && !previousGamepad2.a){

//                Flag_b = !Flag_b;
                MOTIF.SORT_STATE(MOTIF.MOTIF_STATE.PPG, MOTIF.MOTIF_STATE.GPP);

//                motionTimer.reset();
//                motionFlag = false;
            }

//            if (Flag_b)
//            {
//
//            }
//            else{
//                zeroFlag();
//            }


//            if (Flag_y || Flag_x || Flag_a || Flag_b)
//            {
//                zeroFlag();
//            }

            if (Globals.intakeFlag_PPGtoGPP && Globals.counterFeed_PPGtoGPP ==0)
            {
//                thirdBallFlag = false;
//                runningActions.add(IntakeSeq.IntakeStoreAction(intake,outtake,feeder));

//                Globals.counterFeed_GPPtoPGP =;
                runningActions.add(
                        new ParallelAction(
                                Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                                Outtake.TurretCommand(Outtake.TurretState.INIT)
                        )
                );
            }
//            if (!Globals.intakeFlag_GPPtoPGP)
//            {
//                runningActions.add(
//                        new ParallelAction(
//                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
//                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
//                                Intake.RollerCommand(Intake.IntakeRollerState.OFF)
//                        )
//                );
//            }

            if (!robot.intakeBeam.getState() && Globals.counterFeed_PPGtoGPP == 0 && Globals.intakeFlag_PPGtoGPP){
                Globals.counterFeed_PPGtoGPP += 1;
                runningActions.add(
                        new ParallelAction(
                                Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                                Outtake.TurretCommand(Outtake.TurretState.INIT)
                        )
                );

            }
            if (robot.intakeBeam.getState() && Globals.counterFeed_PPGtoGPP ==1 && Globals.intakeFlag_PPGtoGPP){
                Globals.counterFeed_PPGtoGPP += 1;
                runningActions.add(
                        new ParallelAction(
                                Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                Feeder.LFCommand(Feeder.LowerFeederState.RELEASE),
                                Outtake.TurretCommand(Outtake.TurretState.INIT)
                        )
                );
            }
            if (Globals.counterFeed_PPGtoGPP==2 && !robot.intakeBeam.getState()&& Globals.intakeFlag_PPGtoGPP){
                Globals.counterFeed_PPGtoGPP += 1;
                runningActions.add(
                        new ParallelAction(
                                Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                Feeder.LFCommand(Feeder.LowerFeederState.RELEASE),
                                Outtake.TurretCommand(Outtake.TurretState.INIT)
                        )
                );
//                zeroFlag = true;
            }

            if (Globals.counterFeed_PPGtoGPP==3 && robot.intakeBeam.getState()&& Globals.intakeFlag_PPGtoGPP){

                runningActions.add(
                        new ParallelAction(
                                Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                Feeder.LFCommand(Feeder.LowerFeederState.ON),
                                Outtake.TurretCommand(Outtake.TurretState.INIT)
                        )
                );
                Globals.counterFeed_PPGtoGPP += 1;

            }

            if (Globals.counterFeed_PPGtoGPP==4 && !robot.outtakeBeam.getState()&& Globals.intakeFlag_PPGtoGPP){
                Globals.counterFeed_PPGtoGPP += 1;

                runningActions.add(
                        new ParallelAction(
                                Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                Intake.RollerCommand(Intake.IntakeRollerState.OFF),
                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                                Outtake.TurretCommand(Outtake.TurretState.INIT)
                        )
                );
//                zeroFlag = false;

            }


            if (Globals.counterFeed_PPGtoGPP==5  && Globals.intakeFlag_PPGtoGPP){
                Globals.counterFeed_PPGtoGPP = 0;
                gamepad1.rumble(1,1,400);
                runningActions.add(
                        new ParallelAction(
                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                                Intake.RollerCommand(Intake.IntakeRollerState.OFF)
                        )
                );
                Globals.intakeFlag_PPGtoGPP = false;
//                zeroFlag = false;

            }
















            if (Globals.intakeFlag_GPPtoPGP && Globals.counterFeed_GPPtoPGP ==0)
            {
//                thirdBallFlag = false;
//                runningActions.add(IntakeSeq.IntakeStoreAction(intake,outtake,feeder));

//                Globals.counterFeed_GPPtoPGP =;
                runningActions.add(
                        new ParallelAction(
                                Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                                Outtake.TurretCommand(Outtake.TurretState.INIT)
                        )
                );
            }
//            if (!Globals.intakeFlag_GPPtoPGP)
//            {
//                runningActions.add(
//                        new ParallelAction(
//                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
//                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
//                                Intake.RollerCommand(Intake.IntakeRollerState.OFF)
//                        )
//                );
//            }

            if (!robot.intakeBeam.getState() && Globals.counterFeed_GPPtoPGP == 0 && Globals.intakeFlag_GPPtoPGP){
                Globals.counterFeed_GPPtoPGP += 1;
                runningActions.add(
                        new ParallelAction(
                                Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                                Outtake.TurretCommand(Outtake.TurretState.INIT)
                        )
                );

            }
            if (robot.intakeBeam.getState() && Globals.counterFeed_GPPtoPGP ==1 && Globals.intakeFlag_GPPtoPGP){
                Globals.counterFeed_GPPtoPGP += 1;
                runningActions.add(
                        new ParallelAction(
                                Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                Feeder.LFCommand(Feeder.LowerFeederState.ON),
                                Outtake.TurretCommand(Outtake.TurretState.INIT)
                        )
                );
            }
            if (Globals.counterFeed_GPPtoPGP==2 && !robot.outtakeBeam.getState()&& Globals.intakeFlag_GPPtoPGP){
                Globals.counterFeed_GPPtoPGP += 1;
                thirdBallFlag = true;
                motionTimer.reset();
                motionFlag = false;
//                zeroFlag = true;
            }

            if (Globals.counterFeed_GPPtoPGP==3 && !robot.intakeBeam.getState()&& Globals.intakeFlag_GPPtoPGP){

                runningActions.add(
                        new ParallelAction(
                                Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                Feeder.LFCommand(Feeder.LowerFeederState.ON),
                                Outtake.TurretCommand(Outtake.TurretState.INIT)
                        )
                );
                Globals.counterFeed_GPPtoPGP += 1;

            }

            if (Globals.counterFeed_GPPtoPGP==4 && robot.intakeBeam.getState()&& Globals.intakeFlag_GPPtoPGP){
                Globals.counterFeed_GPPtoPGP += 1;

                runningActions.add(
                        new ParallelAction(
                                Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                Feeder.LFCommand(Feeder.LowerFeederState.ON),
                                Outtake.TurretCommand(Outtake.TurretState.INIT)
                        )
                );
//                zeroFlag = false;

            }

            if (Globals.counterFeed_GPPtoPGP==5 && !robot.feederBeam.getState()&& Globals.intakeFlag_GPPtoPGP){
                Globals.counterFeed_GPPtoPGP += 1;

                runningActions.add(
                        new ParallelAction(
                                Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                Feeder.LFCommand(Feeder.LowerFeederState.ON),
                                Outtake.TurretCommand(Outtake.TurretState.INIT)
                        )
                );
//                zeroFlag = false;

            }



            if (Globals.counterFeed_GPPtoPGP==6 && !robot.intakeBeam.getState()&& Globals.intakeFlag_GPPtoPGP){
                Globals.counterFeed_GPPtoPGP += 1;

                runningActions.add(
                        new ParallelAction(
                                Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                Feeder.LFCommand(Feeder.LowerFeederState.ON),
                                Outtake.TurretCommand(Outtake.TurretState.INIT)
                        )
                );
//                zeroFlag = false;

            }
            if (Globals.counterFeed_GPPtoPGP==7 && robot.intakeBeam.getState() && Globals.intakeFlag_GPPtoPGP){
                Globals.counterFeed_GPPtoPGP = 0;
                gamepad1.rumble(1,1,400);
                runningActions.add(
                        new ParallelAction(
                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                                Intake.RollerCommand(Intake.IntakeRollerState.OFF)
                        )
                );
                Globals.intakeFlag_GPPtoPGP = false;
//                zeroFlag = false;

            }





            if (Globals.intakeFlag_PGPtoPPG  && Globals.counterFeed_PGPtoPPG == 0)
            {
//                thirdBallFlag = false;
                runningActions.add(IntakeSeq.IntakeStoreAction2(intake,outtake,feeder));
            }
//            if (!Globals.intakeFlag_PGPtoPPG )
//            {
//                runningActions.add(
//                        new ParallelAction(
//                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
//                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
//                                Intake.RollerCommand(Intake.IntakeRollerState.OFF)
//                        )
//                );
//            }

            if (!robot.feederBeam.getState() && Globals.counterFeed_PGPtoPPG==0 && Globals.intakeFlag_PGPtoPPG ){
                Globals.counterFeed_PGPtoPPG += 1;
                runningActions.add(
                        new ParallelAction(
                                Intake.RollerCommand(Intake.IntakeRollerState.ON)
                        )
                );

            }
            if (robot.feederBeam.getState() && Globals.counterFeed_PGPtoPPG ==1 && Globals.intakeFlag_PGPtoPPG ){
                Globals.counterFeed_PGPtoPPG += 1;
                runningActions.add(
                        new ParallelAction(
                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                                Intake.RollerCommand(Intake.IntakeRollerState.ON)
                        )
                );
            }
            if (Globals.counterFeed_PGPtoPPG==2 && !robot.intakeBeam.getState()&& Globals.intakeFlag_PGPtoPPG ){
                Globals.counterFeed_PGPtoPPG += 1;
//                zeroFlag = true;
            }
            if (Globals.counterFeed_PGPtoPPG==3 && robot.intakeBeam.getState()&& Globals.intakeFlag_PGPtoPPG ){
                Globals.counterFeed_PGPtoPPG += 1;
                runningActions.add(
                        new ParallelAction(
                                Feeder.LFCommand(Feeder.LowerFeederState.ON)
                        )
                );
            }

            if (Globals.counterFeed_PGPtoPPG==4 && !robot.intakeBeam.getState()&& Globals.intakeFlag_PGPtoPPG ){
                Globals.counterFeed_PGPtoPPG += 1;
//                zeroFlag = false;

            }
            if (Globals.counterFeed_PGPtoPPG==5 && robot.intakeBeam.getState()&& Globals.intakeFlag_PGPtoPPG ){
//                motionFlag = true;
//                motionTimer.reset();
                Globals.counterFeed_PGPtoPPG = 0;
                gamepad1.rumble(1,1,400);
                runningActions.add(
                        new ParallelAction(
                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                                Intake.RollerCommand(Intake.IntakeRollerState.OFF)
                        )
                );
                Globals.intakeFlag_PGPtoPPG  = false;
            }





            if (Globals.intakeFlag_GPPtoPPG && Globals.counterFeed_GPPtoPPG ==0)
            {
//                thirdBallFlag = false;
//                runningActions.add(IntakeSeq.IntakeStoreAction(intake,outtake,feeder));

//                Globals.counterFeed_GPPtoPGP =;
                runningActions.add(
                        new ParallelAction(
                                Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                                Outtake.TurretCommand(Outtake.TurretState.INIT)
                        )
                );
            }
//            if ()
//            {
//                runningActions.add(
//                        new ParallelAction(
//                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
//                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
//                                Intake.RollerCommand(Intake.IntakeRollerState.OFF)
//                        )
//                );
//            }

            if (!robot.intakeBeam.getState() && Globals.counterFeed_GPPtoPPG==0 && Globals.intakeFlag_GPPtoPPG){
                Globals.counterFeed_GPPtoPPG += 1;
                runningActions.add(
                        new ParallelAction(
                                Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                                Outtake.TurretCommand(Outtake.TurretState.INIT)
                        )
                );

            }
            if (robot.intakeBeam.getState() && Globals.counterFeed_GPPtoPPG ==1 && Globals.intakeFlag_GPPtoPPG){
                Globals.counterFeed_GPPtoPPG += 1;
                runningActions.add(
                        new ParallelAction(
                                Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                Feeder.LFCommand(Feeder.LowerFeederState.ON),
                                Outtake.TurretCommand(Outtake.TurretState.INIT)
                        )
                );
            }
            if (Globals.counterFeed_GPPtoPPG==2 && !robot.feederBeam.getState()&& Globals.intakeFlag_GPPtoPPG){
                Globals.counterFeed_GPPtoPPG += 1;
//                zeroFlag = true;
            }
            if (Globals.counterFeed_GPPtoPPG==3 && robot.feederBeam.getState()&& Globals.intakeFlag_GPPtoPPG){
                Globals.counterFeed_GPPtoPPG += 1;

            }

            if (Globals.counterFeed_GPPtoPPG==4 && !robot.intakeBeam.getState()&& Globals.intakeFlag_GPPtoPPG){
                Globals.counterFeed_GPPtoPPG += 1;
//                zeroFlag = false;

            }
            if (Globals.counterFeed_GPPtoPPG==5 && robot.intakeBeam.getState()&& Globals.intakeFlag_GPPtoPPG){
//                motionFlag = true;
//                motionTimer.reset();
                Globals.counterFeed_GPPtoPPG = 0;
                gamepad1.rumble(1,1,400);
                runningActions.add(
                        new ParallelAction(
                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                                Intake.RollerCommand(Intake.IntakeRollerState.OFF)
                        )
                );
                Globals.intakeFlag_GPPtoPPG = false;
            }




            if (currentGamepad1.a && !previousGamepad1.a){
                Globals.counterFeed = 0;
                Globals.intakeFlag = ! Globals.intakeFlag;
//                motionTimer.reset();
//                motionFlag = false;
            }

            if (Globals.intakeFlag && robot.outtakeBeam.getState())
            {
//                thirdBallFlag = false;
                runningActions.add(IntakeSeq.IntakeStoreAction(intake,outtake,feeder));
            }
            if (!Globals.intakeFlag && !Globals.intakeFlag_PGPtoPPG && !Globals.intakeFlag_GPPtoPGP  && !Globals.intakeFlag_GPPtoPPG && !Globals.intakeFlag_PPGtoGPP)
            {
                runningActions.add(
                        new ParallelAction(
                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                                Intake.RollerCommand(Intake.IntakeRollerState.OFF)
                        )
                );
            }

            if (!robot.feederBeam.getState() && Globals.counterFeed==0 && Globals.intakeFlag){
                Globals.counterFeed += 1;
            }
            if (robot.feederBeam.getState() && Globals.counterFeed ==1 && Globals.intakeFlag){
                Globals.counterFeed += 1;
            }
            if (Globals.counterFeed==2 && !robot.feederBeam.getState()&& Globals.intakeFlag){
                Globals.counterFeed += 1;
                zeroFlag = true;
            }
            if (Globals.counterFeed==3 && robot.feederBeam.getState()&& Globals.intakeFlag){
                Globals.counterFeed += 1;
                runningActions.add(
                        new ParallelAction(
                                Feeder.LFCommand(Feeder.LowerFeederState.OFF)
                        )
                );
            }

            if (Globals.counterFeed==4 && !robot.intakeBeam.getState()&& Globals.intakeFlag){
                Globals.counterFeed += 1;
                zeroFlag = false;

            }
            if (Globals.counterFeed==5 && robot.intakeBeam.getState()&& Globals.intakeFlag){
//                motionFlag = true;
//                motionTimer.reset();
                Globals.counterFeed = 0;
                gamepad1.rumble(1,1,400);
                runningActions.add(
                        new ParallelAction(
                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                                Intake.RollerCommand(Intake.IntakeRollerState.OFF)
                        )
                );
                Globals.intakeFlag = false;
            }




            if (currentGamepad1.b && !previousGamepad1.b){

                counterB +=1;
//                feedFlag;
                feedButtonTimer.reset();
                feedButtonFlag = false;
            }

            if(counterB == 1)
            {
                feedFlag = true;
            }
            if (feedFlag && previousGamepad1.b)
            {
                feedButtonFlag = true;
            }
            else if (!feedFlag && previousGamepad1.b)
            {
                feedButtonFlag = true;
            }
            if (feedButtonFlag && counterB ==1)
            {
                if (feedButtonTimer.milliseconds()>rampTime)
                {
                    runningActions.add(
                            new ParallelAction(
                                    Feeder.UFCommand(Feeder.UpperFeederState.OFF)
                            )
                    );
                    feedButtonFlag = false;
                    thirdBallFlag = true;
                    motionTimer.reset();
                    motionFlag = false;
                }
                else {

                    runningActions.add(
                            new ParallelAction(
                                    Feeder.UFCommand(Feeder.UpperFeederState.ON)
                            )
                    );
                }

            }

            if (feedButtonFlag && counterB ==2 )
            {
                if (feedButtonTimer.milliseconds()>rampTime)
                {
                    runningActions.add(
                            new ParallelAction(
                                    Feeder.UFCommand(Feeder.UpperFeederState.OFF)
                            )
                    );
                    feedButtonFlag = false;
                    Globals.intakeFlag = true;

                }
                else {

                    runningActions.add(
                            new ParallelAction(
                                    Feeder.UFCommand(Feeder.UpperFeederState.ON)
                            )
                    );
                }

            }

            if (feedButtonFlag && counterB ==3)
            {
                if (feedButtonTimer.milliseconds()>rampTime)
                {
                    runningActions.add(
                            new ParallelAction(
                                    Feeder.UFCommand(Feeder.UpperFeederState.OFF)
                            )
                    );
                    feedButtonFlag = false;
                    counterB = 0;
                    Globals.intakeFlag = false;

                }
                else {

                    runningActions.add(
                            new ParallelAction(
                                    Feeder.UFCommand(Feeder.UpperFeederState.ON)
                            )
                    );
                }

            }


            if (feedButtonFlag && Globals.counterFeed == 2 && previousGamepad1.b && counterB ==3)
            {
                Globals.counterFeed = 0;

            }



            if (gamepad1.dpad_up){
                Globals.counterFeed = 0;

                Globals.intakeFlag = false;
            }

            if (thirdBallFlag)
            {
                motionFlag = true;
            }
            else if (!thirdBallFlag)
            {
                motionFlag = false;
            }
            if (motionFlag) {
//                motionTimer.reset();
                if (motionTimer.milliseconds() > motionTime) {
                    runningActions.add(
                            new SequentialAction(
//                                Feeder.UFCommand(Feeder.UpperFeederState.ON),
                                    Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                                    Intake.RollerCommand(Intake.IntakeRollerState.OFF)
                            )

                    );
                    thirdBallFlag = false;
                    motionFlag = false;
                }
                else {

                    runningActions.add(
                            new SequentialAction(
//                                Feeder.UFCommand(Feeder.UpperFeederState.ON),
                                    Feeder.LFCommand(Feeder.LowerFeederState.ON),
                                    Intake.RollerCommand(Intake.IntakeRollerState.RELEASE)
                            )

                    );
                }
            }

            if (gamepad1.right_trigger>0)
            {
                runningActions.add(
                        new SequentialAction(
                                Feeder.UFCommand(Feeder.UpperFeederState.ON),
                                Feeder.LFCommand(Feeder.LowerFeederState.ON),
                                Intake.RollerCommand(Intake.IntakeRollerState.ON)
                        )

                );
            }
            if (gamepad1.left_trigger>0)
            {
                runningActions.add(
                        new SequentialAction(
                                Feeder.UFCommand(Feeder.UpperFeederState.RELEASE),
                                Feeder.LFCommand(Feeder.LowerFeederState.RELEASE),
                                Intake.RollerCommand(Intake.IntakeRollerState.RELEASE)
                        )

                );
            }
            

            if (gamepad1.y){
                Globals.shooterMode = true;
                runningActions.add(
                        new SequentialAction(
                                Outtake.HoodCommand(Outtake.HoodState.FAR),
                                Outtake.ShooterCommand(Outtake.ShooterState.FAR)
                        )
                );
            }

            if (gamepad1.x){
                Globals.shooterMode = true;
                runningActions.add(
                        new SequentialAction(
                                Outtake.HoodCommand(Outtake.HoodState.NEAR_END),
                                Outtake.ShooterCommand(Outtake.ShooterState.NEAR)
                        )
                );
            }

//            if (gamepad1.x){
//                Globals.shooterMode = true;
//                runningActions.add(
//                        new SequentialAction(
//                                Outtake.ShooterCommand(Outtake.ShooterState.NEAR)
//                        )
//                );
//            }
            if (gamepad1.right_bumper){
                Globals.shooterMode = true;
                runningActions.add(
                        new SequentialAction(
                                Outtake.ShooterCommand(Outtake.ShooterState.OFF)
                        )
                );
            }



//            else
//            {
//                runningActions.add(
//                        new ParallelAction(
//                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
//                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
//                                Intake.RollerCommand(Intake.IntakeRollerState.OFF)
//                        )
//                );
//            }



//            if (!robot.outtakeBeam.getState())
//            {
//                runningActions.add(
//                        new ParallelAction(
//                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
//                                Feeder.LFCommand(Feeder.LowerFeederState.OFF)
////                                new InstantAction(()-> SFlag=true)
//                        )
//                );
//            }
//
//            if ((!robot.intakeBeam.getState()) ) {
//
//                runningActions.add(
//                        new ParallelAction(
//                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
//                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
//                                Intake.RollerCommand(Intake.IntakeRollerState.OFF)
////                                new InstantAction(()-> SFlag=true)
//                        )
//                );
//            }


            angle = getContinuousIMU(Globals.currentTurretState);
            run_turret(angle, 0, 27845, actualPos, telemetry);
//            lastOBeamstate = currentOBeamstate;
            dashboard.sendTelemetryPacket(packet);

//            telemetry.update();
            telemetry.addLine();
            packet.put("Target Velocity", Globals.curretShooterStateVelMode);
            packet.put("Current Velocity", robot.shooter.getVelocity());
//            packet.put("", OBeamcounter);

//            telemetry.addData("con : ", feedButtonFlag && zeroFlag);
            telemetry.addData("pid : ", pid);
            telemetry.addData("diff : ", Globals.shooterFarVel-robot.shooter.getVelocity());
            telemetry.addData("Globals.counterFeed: ",  Globals.counterFeed);
            telemetry.addData("Target Velocity : ",  Globals.curretShooterStateVelMode);
            telemetry.addData("Current Velocity : ", robot.shooter.getVelocity());
            telemetry.addData("Counter : ", counter);
            telemetry.addData("Turret State : ", Globals.currentTurretState);
            telemetry.addData("Shooter State : ", Globals.curretShooterStateVelMode);
            telemetry.addData("Intake Flag : ", Globals.intakeFlag);
            telemetry.addData("Feed Toggle : ", feedToggle);
            telemetry.addData("Feed Flag : ", feedFlag);
            telemetry.addData("Shooter Mode : ", Globals.shooterMode);
            telemetry.addData("vel condition : ", robot.shooter.getVelocity()> Globals.shooterFarVel-buff);
            telemetry.addData("vel  : ", robot.shooter.getVelocity());
            telemetry.addData("ib  : ", robot.intakeBeam.getState());
            telemetry.addData("fb  : ", robot.feederBeam.getState());
            telemetry.addData("ob  : ", robot.outtakeBeam.getState());

            printDriveTelemetry();

            telemetry.addData("Intake Current : ", robot.intakeRoller.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("UF Current : ", robot.upperFeeder.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("LF Current : ", robot.lowerFeeder.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Shooter Current : ", robot.shooter.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }

    }

    public static void printCamera(){
    }
    public void printTelemetry(){
        YawPitchRollAngles orientation = drive.lazyImu.get().getRobotYawPitchRollAngles();
        telemetry.addData("Yaw / Pitch / Roll", "%.1f / %.1f / %.1f",
                orientation.getYaw(AngleUnit.DEGREES),
                orientation.getPitch(AngleUnit.DEGREES),
                orientation.getRoll(AngleUnit.DEGREES));
        telemetry.addData("Hood Pose",robot.hood.getPosition());
        telemetry.addLine();
    }
    public void printDriveTelemetry(){
        telemetry.addData("Right Back Current : ", drive.rightBack.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Left Back Current : ", drive.leftBack.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Right Front Current : ", drive.rightFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Left Front Current : ", drive.leftFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addLine();
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
        telemetry.addLine();

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

    public static double map(double x, double inMin, double inMax, double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    public double  getContinuousIMU(double currentAngle) {
        double delta = currentAngle - prevAngle;

        // Handle wrap-around
        if (delta > 180) {
            delta -= 360;
        } else if (delta < -180) {
            delta += 360;
        }

        totalAngle += delta;
        prevAngle = currentAngle;

        return totalAngle;
    }

    public static double run_turret(double imu, double min_in_pos, double max_in_pos, double pose, Telemetry telemetry){
        pid = 0;

        c = map(pose, min_in_pos, max_in_pos, 0, 360);

        if (imu < 0){
            c = -c;
        }

        double target = imu;

        double derivative = 0;
        if(imu < 0){
            telemetry.addLine("neagtive heading");
            error = c + target;
//            error = -error;
            integral = integral + error;
            derivative = error - previous_error;

            previous_error = error;
            pid = kp*error + kd*derivative + ki*integral;
            robot.turret1.setPower(-pid);
            robot.turret2.setPower(pid);
            return -pid;

        }
        else {
            telemetry.addLine("positive heading");

            error = target - c;
//            error = -error;
            integral = integral + error;
            derivative = error - previous_error;

            previous_error = error;
            pid = kp*error + kd*derivative + ki*integral;
            robot.turret1.setPower(-pid);
            robot.turret2.setPower(pid);
            return -pid;
        }
    }


}
