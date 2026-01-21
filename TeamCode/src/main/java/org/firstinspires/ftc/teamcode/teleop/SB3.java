package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import static org.firstinspires.ftc.teamcode.teleop.tag_final.preset;

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
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
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
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.ArrayList;
import java.util.List;

@Config
//@TeleOp(name = "TELEOP SB3", group = "TELEOP")
public class SB3 extends LinearOpMode {
    IMU imu;
    public  static double preset0 = 0;

    private static RobotHardware robot=RobotHardware.getInstance();
    ElapsedTime intakeTimer;
    ElapsedTime motionTimer;
    ElapsedTime secondBallTimer;
    ElapsedTime feedButtonTimer;
    ElapsedTime thirdIntake;
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

    public static double kp = 0.03, ki = 0, kd = 0;
    public static String WEBCAM_NAME = "Webcam 1";

    public static double yaw = 0;
    private double totalAngle;
    private double angle;
    private double prevAngle;
    public static double DESIRED_ANGLE = 0;
    public static double offset = 0;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            -90, 0, -90, 0);


    //TODO -------------------Shooter Constants--------

    public static double P = 130;
    public static double I = 0;
    public static double D = 0;
    public static double F = 12;
    double actualPos;
    //TODO -------------------Flags & Constants----------

    public static double buff= 100;
    public static double diffVel= 50;
    public static double OBeamcounter= 0;
    public static boolean OBeam;
    public static boolean IBeam;
    public static boolean FBeam;
    public static double rampTime= 200;
    public static double secondBallTime= 350;
    public static double motionTime=280;// 280;
    public static double thirdTime=200;// 280;
    public static double thresh= 100;
    public static int counter= 0;
    public static int counterFeed= 0;
    public static int counterShootFeed= 0;
    public static int thirdFeed= 0;
    public static boolean currentOBeamstate= false;
    public static boolean lastOBeamstate= false;
    public static boolean feedToggle = false;
    public static boolean intakeToggle = false;
    public static boolean intakeFlag = false;
    public static boolean oneFlag = false;
    public static boolean feedFlag = false;
    public static boolean zeroFlag = false;
    public static boolean thirdBallFlag = false;
    public static boolean motionFlag = false;
    public static boolean shooterFlag = false;
    public static boolean feedButtonFlag = false;
    public static boolean secondBallFlag = false;
    public static boolean intakeFlag2 = false;
    public static boolean shootFlagFar = false;
    public static boolean shootFlagNear = false;
    public static boolean shootStop = false;
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
        secondBallTimer = new ElapsedTime();
        feedButtonTimer = new ElapsedTime();
        thirdIntake = new ElapsedTime();
        imu = hardwareMap.get(IMU.class, "imu");
        initAprilTag();
        double xRotation = 0;
        double zRotation = 0;
        double yRotation = 0;

        Orientation hubRotation = xyzOrientation(xRotation, yRotation, zRotation);

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(hubRotation);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        integral = 0;
        error = 0;
        previous_error = 0;
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
            imu.resetYaw();

            drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));
            feedToggle = false;
            intakeFlag = false;
            feedFlag = false;
            OBeamcounter = 0;
            counterFeed =0;
            counterB =0;
            intakeTimer.reset();
            motionTimer.reset();
            secondBallTimer.reset();
            feedButtonTimer.reset();
            thirdIntake.reset();
            motionFlag = false;

            runningActions = updateAction();
            runningActions.add(InitSeq.InitAction(intake,outtake,feeder));
        }
        DESIRED_ANGLE = 0;

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
//                    botHeading*0.8);
//            drive.updatePoseEstimate();

            drive.updatePoseEstimate();

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
            if (currentGamepad1.a && !previousGamepad1.a){
                counterFeed = 0;
//                counterB = 0;
                intakeFlag = !intakeFlag;
//                motionTimer.reset();
//                motionFlag = false;
            }

            if (intakeFlag && robot.outtakeBeam.getState())
            {
//                thirdBallFlag = false;
                runningActions.add(IntakeSeq.IntakeStoreAction(intake,outtake,feeder));
            }
            if (!intakeFlag)
            {
                runningActions.add(
                        new ParallelAction(
                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                                Intake.RollerCommand(Intake.IntakeRollerState.OFF)
                        )
                );
            }

            if (!robot.feederBeam.getState() && counterFeed==0 && intakeFlag){
                counterFeed += 1;
            }
            if (robot.feederBeam.getState() && counterFeed ==1 && intakeFlag){
                counterFeed += 1;
            }
            if (counterFeed == 2 && !robot.intakeBeam.getState() && intakeFlag)
            {
                secondBallTimer.reset();
                secondBallFlag = true;
            }
            if(secondBallFlag)
            {
                if(secondBallTimer.milliseconds()>secondBallTime){
                    counterFeed = 4;
                    runningActions.add(
                            new ParallelAction(
                                    Feeder.LFCommand(Feeder.LowerFeederState.OFF)
                            )
                    );
                    secondBallFlag = false;

                }
//                else{
//                    runningActions.add(
//                            new ParallelAction(
//                                    Feeder.LFCommand(Feeder.LowerFeederState.ON)
//                            )
//                    );
//                }


            }

            if(!secondBallFlag && counterFeed >2)
            {
                runningActions.add(
                        new ParallelAction(
                                Feeder.LFCommand(Feeder.LowerFeederState.OFF)
                        )
                );
            }
            if (counterFeed==2 && !robot.feederBeam.getState()&& intakeFlag){
                counterFeed += 1;
                zeroFlag = true;
            }
            if (counterFeed==3 && robot.feederBeam.getState()&& intakeFlag){
                counterFeed += 1;
                runningActions.add(
                        new ParallelAction(
                                Feeder.LFCommand(Feeder.LowerFeederState.OFF)
                        )
                );
            }

            if (counterFeed==4 && !robot.intakeBeam.getState()&& intakeFlag){

                if (thirdIntake.milliseconds() > thirdTime) {


                    counterFeed = 0;
                    gamepad1.rumble(1, 1, 400);
                    runningActions.add(
                            new ParallelAction(
                                    Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                    Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                                    Intake.RollerCommand(Intake.IntakeRollerState.OFF)
                            )
                    );
                    intakeFlag = false;
                }

//                else {
//
//                    runningActions.add(
//                            new ParallelAction(
////                                Feeder.UFCommand(Feeder.UpperFeederState.ON),
//                                    Feeder.LFCommand(Feeder.LowerFeederState.ON),
//                                    Intake.RollerCommand(Intake.IntakeRollerState.ON)
//                            )
//
//                    );
//                }

            }
//            if (counterFeed==4 && robot.intakeBeam.getState()&& intakeFlag){
////                motionFlag = true;
////                motionTimer.reset();
//                counterFeed = 0;
//                gamepad1.rumble(1,1,400);
//                runningActions.add(
//                        new ParallelAction(
//                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
//                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
//                                Intake.RollerCommand(Intake.IntakeRollerState.OFF)
//                        )
//                );
//                intakeFlag = false;
//            }

//            if (motionFlag)
//            {
//
//                if (motionTimer.milliseconds()>thresh)
//                {
//                    runningActions.add(
//                            new ParallelAction(
//                                    Feeder.UFCommand(Feeder.UpperFeederState.OFF),
//                                    Feeder.LFCommand(Feeder.LowerFeederState.OFF),
//                                    Intake.RollerCommand(Intake.IntakeRollerState.OFF)
//                            )
//                    );
//                    motionFlag = false;
//                    shooterFlag = true;
//
//
//                }
//                else {
//
//                    runningActions.add(
//                            new ParallelAction(
//                                    Feeder.UFCommand(Feeder.UpperFeederState.RELEASE),
//                                    Feeder.LFCommand(Feeder.LowerFeederState.RELEASE),
//                                    Intake.RollerCommand(Intake.IntakeRollerState.ON)
//                            )
//                    );
//                }
//
//            }

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
                    intakeFlag = true;

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
                    intakeFlag = false;

                }
                else {

                    runningActions.add(
                            new ParallelAction(
                                    Feeder.UFCommand(Feeder.UpperFeederState.ON)
                            )
                    );
                }

            }


            if (feedButtonFlag && counterFeed == 2 && previousGamepad1.b && counterB ==3)
            {
                counterFeed = 0;

            }



//            if (currentGamepad1.b && !previousGamepad1.b){
//                feedFlag = !feedFlag;
//            }
//
//            if (feedFlag)
//            {
//                runningActions.add(
//                        new ParallelAction(
//                                Feeder.UFCommand(Feeder.UpperFeederState.ON)
//                        )
//                );            }
//            else if (!feedFlag)
//            {
//                runningActions.add(
//                        new ParallelAction(
//                                Feeder.UFCommand(Feeder.UpperFeederState.OFF)
//                        )
//                );
//            }

            if (gamepad1.dpad_up){
                counterFeed = 0;

                intakeFlag = false;
            }



//            if (oneFlag )
//            {
//                runningActions.add(
//                        new ParallelAction(
////                                Feeder.UFCommand(Feeder.UpperFeederState.ON),
//                                Feeder.LFCommand(Feeder.LowerFeederState.ON)
//                        )
//                );            }
//            else if (oneFlag && thirdBallFlag)
//            {
//                runningActions.add(
//                        new ParallelAction(
//                                Intake.RollerCommand(Intake.IntakeRollerState.ON),
//                                Feeder.LFCommand(Feeder.LowerFeederState.ON)
//
//                        )
//                );
//            }



//            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
//                thirdBallFlag = !thirdBallFlag;
//                motionTimer.reset();
//                motionFlag = false;
//            }
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
                            new ParallelAction(
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
                            new ParallelAction(
//                                Feeder.UFCommand(Feeder.UpperFeederState.ON),
                                    Feeder.LFCommand(Feeder.LowerFeederState.ON),
                                    Intake.RollerCommand(Intake.IntakeRollerState.ON)
                            )

                    );
                }
            }
//            if (!robot.intakeBeam.getState() && thirdFeed==0 && thirdBallFlag){
//                thirdFeed += 1;
//
//            }
//            if (robot.feederBeam.getState() && thirdFeed==1 && thirdBallFlag){
//
//                thirdFeed += 1;
//            }
//            if (!robot.feederBeam.getState() && thirdFeed==2 && thirdBallFlag){
//                runningActions.add(
//                        new SequentialAction(
////                                Feeder.UFCommand(Feeder.UpperFeederState.ON),
//                                Feeder.LFCommand(Feeder.LowerFeederState.OFF)
//                        )
//
//                );
//                thirdFeed = 0;
//                counterFeed = 0;
//                intakeFlag = true;
//                thirdBallFlag = false;
//
//            }

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


//            if (thirdFeed==1 && thirdBallFlag){
//                runningActions.add(
//                        new SequentialAction(
////                                Feeder.UFCommand(Feeder.UpperFeederState.ON),
//                                Feeder.LFCommand(Feeder.LowerFeederState.ON),
//                                Intake.RollerCommand(Intake.IntakeRollerState.ON)
//                        )
//
//                );
//                thirdFeed += 1;
////                thirdFeed = 0;
////                thirdBallFlag = false;
//            }
//            if (robot.intakeBeam.getState() && thirdFeed==2 && thirdBallFlag){
////                thirdFeed = 0;
////                thirdBallFlag=false;
//
//            }


//            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
//                thirdBallFlag = !thirdBallFlag;
//            }
//
//            if (thirdBallFlag && !intakeFlag)
//            {
//                runningActions.add(
//                        new SequentialAction(
////                                Feeder.UFCommand(Feeder.UpperFeederState.ON),
//                                Feeder.LFCommand(Feeder.LowerFeederState.RELEASE),
//                                Intake.RollerCommand(Intake.IntakeRollerState.RELEASE),
//                                new SleepAction(2000),
//                                Feeder.LFCommand(Feeder.LowerFeederState.ON),
//                                Intake.RollerCommand(Intake.IntakeRollerState.ON)
//                        )
//
//                );
//            }
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

//            if (currentGamepad1.b && !previousGamepad1.b){
//                feedFlag =! feedFlag;
//                intakeTimer.reset();
//            }
//            if (feedFlag && !intakeFlag)
//            {
//
//                if (intakeTimer.milliseconds()>rampTime) {
//                    runningActions.add(
//                            new ParallelAction(
//                                    Intake.IntakeCommand(Intake.IntakeServoState.IN),
//                                    Feeder.UFCommand(Feeder.UpperFeederState.OFF),
//                                    Feeder.LFCommand(Feeder.LowerFeederState.ON)
//
//                            )
//                    );
//                    intakeTimer.reset();
//                }
//                else {
//                    runningActions.add(
//                            new ParallelAction(
//                                    Feeder.UFCommand(Feeder.UpperFeederState.ON)
////                                    Intake.RollerCommand(Intake.IntakeRollerState.ON)
//                            )
//                    );
//                }
//            }
//            else   {
//                runningActions.add(
//                        new ParallelAction(
//                                Feeder.UFCommand(Feeder.UpperFeederState.OFF)
//                        )
//                );
//            }

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

            if(gamepad2.x)
            {
                preset0 = -55;
            }

            if(gamepad2.b)
            {
                preset0 = 60;
            }
            if (gamepad2.y)
            {
                preset0 = 0;
            }

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            List<AprilTagDetection> detections = aprilTag.getDetections();

//            double actualPos = extEncoder.getCurrentPosition();
            Vector2d currentPos = drive.localizer.getPose().position;
            double currentHeading = drive.localizer.getPose().heading.toDouble();

            boolean tagVisible = !detections.isEmpty();
            if (tagVisible) {
                telemetry.addData("Yaw", "%.2f",detections.get(0).ftcPose.yaw );
                telemetry.addData("robot yaw", "%.2f", detections.get(0).robotPose.getOrientation().getYaw());
                telemetry.addData("robot x", "%.2f", detections.get(0).robotPose.getPosition().x);
                telemetry.addData("robot y", "%.2f", detections.get(0).robotPose.getPosition().y);
//                tagYawDeg = detections.get(0).ftcPose.yaw;
                double tagYawDeg = detections.get(0).robotPose.getOrientation().getYaw();
                DESIRED_ANGLE = tagYawDeg;
                offset = -DESIRED_ANGLE;
//                run_turret(angle + DESIRED_ANGLE - offset, 0, 27845, actualPos, telemetry);

            }
            else{
//                DESIRED_ANGLE = 0;
//                tagYawDeg = 0;
//                angle = 0;
//                offset = 0;
            }


            if(Math.abs(error) < 2){
                error = 0;
            }
            run_turret(angle + offset + DESIRED_ANGLE + preset0, 0, 27845, actualPos, telemetry);

            target = Math.toDegrees(drive.localizer.getPose().heading.toDouble()) ;
            angle = getContinuousIMU(target);
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
            telemetry.addData("counterFeed: ",  counterFeed);
            telemetry.addData("Target Velocity : ",  Globals.curretShooterStateVelMode);
            telemetry.addData("Current Velocity : ", robot.shooter.getVelocity());
            telemetry.addData("Counter : ", counter);
            telemetry.addData("Turret State : ", Globals.currentTurretState);
            telemetry.addData("Shooter State : ", Globals.curretShooterStateVelMode);
            telemetry.addData("Intake Flag : ", intakeFlag);
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

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawTagOutline(false)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(DistanceUnit.MM,AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTag);


        visionPortal = builder.build();
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
