package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

//import androidx.core.math.MathUtils;


import android.renderscript.Matrix4f;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.data.Matrix;
import org.ejml.dense.row.MatrixFeatures_CDRM;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.sequences.InitSeq;
import org.firstinspires.ftc.teamcode.sequences.IntakeSeq;
import org.firstinspires.ftc.teamcode.subsystem.Feeder;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

//@TeleOp
@Config

// TODO: turret integrated
public class SB2 extends LinearOpMode {
    private static RobotHardware robot=RobotHardware.getInstance();

    private Limelight3A limelight;
    IMU imu;
    Encoder enc;
//    ServoImplEx hood;
    double pos;
    public static double hood_min = 15;
    public static double hood_max = 70;
//    private static CRServo crServo1, crServo2;
//    private DcMotorEx extEncoder;
    public double b;
    public static double c;
    //    private CRServo crServo2;
//    private AnalogInput axonFeedback;
    public static double target = 0;
    public static double error ;
    public static double integral;
    public static double previous_error;
    public double derivative;
    public static double pid;
    public double a;
    public static double DESIRED_ANGLE = 0;
    public static int ENCODER_TICKS_PER_REV = 8192;

    public static double kp = 0.02, ki = 0, kd = 0;
    public static String WEBCAM_NAME = "Webcam 1";
    public static double corr = 0, yaw = 0;
    private double totalAngle;
    private double angle;
    public static double preset = 0;
    private double presetFlag;
    public static double offset = 0;
    private double prevAngle;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            -90, 0, -90, 0);
    private double tagYawDeg = 0;
    public static double targetX = 66;
    public static double targetY = -66;

    //    private double integral = 0;
//    private double lastError = 0;
//    private double fieldTargetHeading = 0;
    private PIDFController headingController;




    ElapsedTime intakeTimer;
    ElapsedTime motionTimer;
    ElapsedTime secondBallTimer;
    ElapsedTime feedButtonTimer;
    ElapsedTime thirdIntake;
    private Intake intake;
    private Outtake outtake;
    private Feeder feeder;
    private MecanumDrive drive;
//    double pos;
    private FtcDashboard dashboard;

    //TODO ---------------------------COLOR Sensor and Beam Breaks

    public NormalizedRGBA clrfrontIntakergba;
    public float[] clrfrontIntakehsv;
    public double clrfrontIntakedistance;

    public NormalizedRGBA clrbackIntakergba;
    public float[] clrbackIntakehsv;
    public double clrbackIntakedistance;


    //TODO -------------------Turret Constants--------

    double total_angle;



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
    public static double thirdTime=150;// 280;
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

        intake = new Intake(robot);
        outtake = new Outtake(robot);
        feeder = new Feeder(robot);
        dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
//        robot.resetEncoder();
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        integral = 0;
        error = 0;
        previous_error = 0;
//        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
//        enc = hardwareMap.get(Encoder.class, "enc");
        imu = hardwareMap.get(IMU.class, "imu");
        initAprilTag();
        robot.turretEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.turretEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.turretEncoder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        double xRotation = 0;
        double zRotation = 0;
        double yRotation = 0;

        Orientation hubRotation = xyzOrientation(xRotation, yRotation, zRotation);

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(hubRotation);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        while (opModeInInit()){
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
            robot.turretEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            robot.turretEncoder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            telemetry.addData("angle ", angle);
            telemetry.addData("I",integral);
            telemetry.addData("D",derivative);
            telemetry.update();
        }
        DESIRED_ANGLE = 0;
        waitForStart();
        while (opModeIsActive()){

//            Globals.

            robot.shooter.setVelocityPIDFCoefficients(P,I,D,F);
            runningActions = updateAction();


            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x*0.5
            ));

            if     (gamepad1.b)
            {
                preset = 30;
            }

            else if     (gamepad1.x)
            {
                preset = -30;
            }
            else if (gamepad1.y){
                preset = 0;
            }
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

//            actualPos = robot.turretEncoder.getCurrentPosition();

//            drive.updatePoseEstimate();

//            Pose2d pose = drive.localizer.getPose();

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
                    counterFeed = 3;
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
                    gamepad1.rumble(1,1,400);
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




            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            List<AprilTagDetection> detections = aprilTag.getDetections();

            double actualPos = robot.turretEncoder.getCurrentPosition();
            Vector2d currentPos = drive.localizer.getPose().position;
            double currentHeading = drive.localizer.getPose().heading.toDouble();

            boolean tagVisible = !detections.isEmpty();
            if (tagVisible) {
                telemetry.addData("Yaw", "%.2f",detections.get(0).ftcPose.yaw );
                telemetry.addData("robot yaw", "%.2f", detections.get(0).robotPose.getOrientation().getYaw());
                telemetry.addData("robot x", "%.2f", detections.get(0).robotPose.getPosition().x);
                telemetry.addData("robot y", "%.2f", detections.get(0).robotPose.getPosition().y);
//                tagYawDeg = detections.get(0).ftcPose.yaw;
                double tagYawDeg = detections.get(0).ftcPose.z;
                DESIRED_ANGLE = tagYawDeg;
                pos = map (detections.get(0).ftcPose.range, hood_min,hood_max,0,1);
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

            total_angle = angle + offset +DESIRED_ANGLE + preset;
            if (total_angle > 90)
            {
                total_angle = 90;
            }
            if (total_angle < -90)
            {
                total_angle = -90;
            }

//            DESIRED_ANGLE = Math.toDegrees(Math.atan2(targetY - currentPos.y, targetX - currentPos.x));
            run_turret(total_angle, 0, 27845, actualPos, telemetry);

            target = Math.toDegrees(drive.localizer.getPose().heading.toDouble()) ;
            angle = getContinuousIMU(target);
            a = orientation.getYaw(AngleUnit.DEGREES);
            robot.hood.setPosition(pos);

//            target = a;
            drive.updatePoseEstimate();
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
            telemetry.addData("Actual Pos", actualPos);

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
//            error = c - target;
//            integral = integral + error;
//            derivative = error - previous_error;
//            previous_error = error;
//
//            pid = kp*error + kd*derivative + ki*integral;
//
//            b = pid;
//
//            error = target - c;
//            crServo1.setPower(b);
//            crServo2.setPower(b);

//            telemetry.addData("Feedback Voltage", voltage);

            telemetry.update();
        }
        visionPortal.close();
    }

    public void printDriveTelemetry(){
        telemetry.addData("Right Back Current : ", drive.rightBack.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Left Back Current : ", drive.leftBack.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Right Front Current : ", drive.rightFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Left Front Current : ", drive.leftFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addLine();
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
    private double computeHeadingCorrection(Pose2d pose) {
        Vector2d currentPos = pose.position;
        double currentHeading = pose.heading.toDouble();

        // Desired angle
        double angleToTarget = Math.atan2(targetY - currentPos.y, targetX - currentPos.x);

        // Wrap error to [-π, π]
        double error = angleWrap(angleToTarget - currentHeading);

        // Feed error into PIDF controller
        return headingController.calculate(0, error);
    }

    // Keep angle in range [-π, π]
    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
    public static void run_turret(double imu, double min_in_pos, double max_in_pos, double pose, Telemetry telemetry){
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
            robot.turret2.setPower(-pid);
            robot.turret1.setPower(pid);

        }
        else {
            telemetry.addLine("positive heading");

            error = target - c;
//            error = -error;
            integral = integral + error;
            derivative = error - previous_error;

            previous_error = error;
            pid = kp*error + kd*derivative + ki*integral;
            robot.turret2.setPower(-pid);
            robot.turret1.setPower(pid);
        }
    }

    public static double map(double x, double inMin, double inMax, double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
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

    private double getRobotHeading() {
        double y = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        if (Double.isNaN(y)) return 0;
        if (y < 0) y += 360.0;
        return y;
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

    private double ticksToAngle(double ticks) {
        return (ticks / (double) ENCODER_TICKS_PER_REV) * 360.0;
    }

    private double normalizeDeg(double angle) {
        return ((angle + 180) % 360 + 360) % 360 - 180;
    }

    private double normalizeFull(double angle) {
        return ((angle % 360) + 360) % 360;
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }


}

