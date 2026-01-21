package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.teleop.SB5.map;

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
import com.qualcomm.hardware.lynx.LynxModule;
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
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.FSM.IntakeFSM;
import org.firstinspires.ftc.teamcode.MecanumDrive;
//import org.firstinspires.ftc.teamcode.Utils.utilities.ServoMapper;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
//import org.firstinspires.ftc.teamcode.sequences.teleOp.FrontIntakeSeq;
//import org.firstinspires.ftc.teamcode.sequences.teleOp.FrontShootSeq;
import org.firstinspires.ftc.teamcode.instantCommands.LFCommand;
import org.firstinspires.ftc.teamcode.instantCommands.RollerCommand;
import org.firstinspires.ftc.teamcode.instantCommands.UFCommand;
import org.firstinspires.ftc.teamcode.sequences.InitSeq;
import org.firstinspires.ftc.teamcode.sequences.IntakeSeq;
import org.firstinspires.ftc.teamcode.sequences.ShootSeq;
import org.firstinspires.ftc.teamcode.subsystem.Feeder;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.utils.DualSenseLightbar;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "SB5 RAPID", group = "ATELEOP")
public class SB5_rapid extends LinearOpMode {
    private static RobotHardware robot=RobotHardware.getInstance();

    public static double pose_x,pose_y,pose_heading;
    public static double servo_low = 0.82,servo_high = 0.88;
    public List<LynxModule> allHubs;
    ElapsedTime intakeTimer,motionTimer,secondBallTimer,feedButtonTimer;
    private IntakeFSM intakeFSM;
    private Intake intake;
    private Outtake outtake;
    private Feeder feeder;
    private MecanumDrive drive;
    double pos;
    private FtcDashboard dashboard;

    //TODO ---------------------------TAG CONSTANTS

    public static double tagID_tag, x_tag, y_tag, z_tag, val_tag = 0,kP_tag = 0.025,kI_tag = 0,kD_tag = 0.25,Integral_tag = 0,LastError_tag = 0;
    public static int Target_tag = 0;
    public double Power_tag = 0;
    public static boolean April_tag = true;
    public static boolean bFlag_tag = false;
    public static boolean farFlag ;


    //TODO ---------------------------COLOR Sensor and Beam Breaks

    public NormalizedRGBA clrfrontIntakergba;
    public float[] clrfrontIntakehsv;
    public double clrfrontIntakedistance;

    public NormalizedRGBA clrbackIntakergba;
    public float[] clrbackIntakehsv;
    public double clrbackIntakedistance;


    //TODO -------------------Turret Constants--------
    public static boolean turretActuate =false;
    public static double required_target = 45;
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

    public static double P = 130;
    public static double I = 0;
    public static double D = 0;
    public static double F = 12.65;
    double actualPos;
    //TODO -------------------Flags & Constants----------
    public static double  strafe, turn,forward;
    public static double buff= 100;
    public static double diffVel= 50;
    public static double OBeamcounter= 0;
    public static boolean OBeam;
    public static boolean IBeam;
    public static boolean FBeam;
    public static double rampTime= 200;
    public static double secondBallTime= 350;
    public static double motionTime= 200;
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
    public static boolean singleShoot ;
    public static boolean sequenceFlag ;
    public static boolean oneFlag = false;
    public static boolean feedFlag = false;
    public static boolean shoot1 = false;
    public static boolean shoot2 = false;
    public static boolean shoot3 = false;
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
    //    public static boolean farFlag = false;
//    public static boolean nearFlag = false;
    public double botHeading;
    private ElapsedTime loopTimer = new ElapsedTime();
    public static double Yaw = 0;
    public static double YawNavx = 0;

    public static List<Action> runningActions = new ArrayList<>();
    private boolean SFlag=false;
    private int counterB;
    public int tagID = 24;
    Action trajectoryAction2;
    AprilTagDetection tag_tag;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = RobotHardware.getInstance();
        robot.init(hardwareMap,telemetry);
        intakeTimer = new ElapsedTime();
        motionTimer = new ElapsedTime();
        secondBallTimer = new ElapsedTime();
        feedButtonTimer = new ElapsedTime();
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        intake = new Intake(robot);
        outtake = new Outtake(robot);
        feeder = new Feeder(robot);
        intakeFSM = new IntakeFSM();
        dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        integral = 0;
        error = 0;
        previous_error = 0;

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                //all boolean
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                // x position of the camera from the centre of the robot  ==  -2.36
                // y position of the camera from the centre of the robot  ==  +2.44
                // z position of the camera from the centre of the robot  ==
                .setCameraPose(
                        new Position(DistanceUnit.INCH, -2.36, 2.44, 15, 0),
                        new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 90, 0)
                )
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // Webcam 1 name is always constant
                .setCameraResolution(new Size(640, 480))
                .build();


        // TODO: Thread Start 🎈

        Thread Campid = new Thread(()->{
            while (!Thread.currentThread().isInterrupted() && opModeIsActive()){
                try {
                    val_tag = Aprilpid(z_tag);

                    if (y_tag>140)
                    {
                        robot.hood.setPosition(pos);
                        farFlag = true;
                    }

//                    else if (y_tag > 50 && y_tag < 115)
//                    {
//                        robot.hood.setPosition(pos);
//                        farFlag = true;
//                    }

                    else {
                        farFlag = false;
                    }

                }
                catch (Exception e) {
                    Thread.currentThread().interrupt();
                    throw new RuntimeException(e);
                }
            }
            sleep(10);
        });

        // TODO: Turret Thread Start

        Thread TurretPID = new Thread(()->{
            while (!Thread.currentThread().isInterrupted() && opModeIsActive()){
                try {
                    angle = getContinuousIMU(Globals.currentTurretState);
                    run_turret(angle, 0, 27845, actualPos, telemetry);

                }
                catch (Exception e) {
                    Thread.currentThread().interrupt();
                    throw new RuntimeException(e);
                }
            }
            sleep(10);
        });



        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // TODO: INIT
        while (opModeInInit()) {
            drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,Math.toRadians(0)));
            feedToggle = false;
            intakeFlag = false;
            feedFlag = false;
            shoot1 = false;
            shoot2 = false;
            shoot3 = false;
            OBeamcounter = 0;
            counterFeed =0;
            counterB =0;
            intakeTimer.reset();
            motionTimer.reset();
            secondBallTimer.reset();
            feedButtonTimer.reset();
            turretActuate =false;

            motionFlag = false;
            drive.navxMicro.initialize();

//            drive.lazyImu.get().resetYaw();
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            runningActions = updateAction();
            runningActions.add(InitSeq.InitAction(intake,outtake,feeder));

            if (gamepad1.a)
            {
                tagID = 20;
                Target_tag = 5;
            }
            if (gamepad1.b)
            {
                tagID = 24;
                Target_tag = 0;
            }

            actualPos = robot.turretEncoder.getCurrentPosition();

            angle = getContinuousIMU(Globals.currentTurretState);
            run_turret(angle, 0, 27845, actualPos, telemetry);
        }
        robot.resetEncoder();

        waitForStart();
        loopTimer.reset();
        TurretPID.start();
        Campid.start();

        // TODO: OP-MODE START
        while (opModeIsActive()){
            loopTimer.reset();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            Orientation orientation = drive.navxMicro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            Yaw = Math.toDegrees(orientation.firstAngle);

            YawNavx= Math.toDegrees(drive.localizer.getPose().heading.toDouble());

            Globals.turret_imu_track= required_target -YawNavx;

            // Cap both values between -100 and +100
            Globals.turret_imu_track = clamp(Globals.turret_imu_track, -85, 85);
            Globals.currentTurretState = clamp(Globals.currentTurretState, -85, 85);


            pose_x = drive.localizer.getPose().position.x;
            pose_y = drive.localizer.getPose().position.y;
            pose_heading = drive.localizer.getPose().heading.toDouble();

            Globals.shooterMode = true;
            robot.shooter.setVelocityPIDFCoefficients(P,I,D,F);
            runningActions = updateAction();
            botHeading = drive.localizer.getPose().heading.toDouble();

            if (tagProcessor.getDetections().size() > 0) {
                tag_tag = tagProcessor.getDetections().get(0); // to detect first tag unless other present

                tagID_tag = tag_tag.id;
                x_tag = tag_tag.ftcPose.x;
                y_tag = tag_tag.ftcPose.y;
                z_tag = tag_tag.ftcPose.z;
                pos = map (y_tag, 142,177,servo_low,servo_high);


                April_tag = true;
                telemetry.addData(" ID ",tag_tag.id);
            }
            else {
                April_tag = false;
            }

            if (gamepad1.left_trigger>0)
            {
                forward = 0.5;
                strafe = 0.3;
                turn = 0.3;

            }

            else{
                forward = 1;
                strafe = 1;
                turn = 1;
            }


            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y*forward,
                            -gamepad1.left_stick_x*strafe
                    ),
                    -gamepad1.right_stick_x*turn
            ));

//            drive.driveFieldCentric(-gamepad1.left_stick_x,
//                    -gamepad1.left_stick_y,
//                    gamepad1.right_stick_x,
//                    botHeading*0.8);
//            drive.updatePoseEstimate();

//            telemetry.update();

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            actualPos = robot.turretEncoder.getCurrentPosition();

            // Intake and Store Artifacts

            //TODO: CAMERA ALIGNMENT

//            drive.updatePoseEstimate();
            if (currentGamepad1.right_trigger > 0)
            {
                bFlag_tag = true;
            } else {
                bFlag_tag = false;
            }

            if (bFlag_tag)
            {
                if(April_tag == true && tagID_tag == tagID){
//
////                    robot.turret1.setPower(val_tag);
//                    robot.turret1.setPower(-val_tag);
                    drive.leftFront.setPower(-val_tag/2);
                    drive.leftBack.setPower(-val_tag/2);
                    drive.rightBack.setPower(val_tag/2);
                    drive.rightFront.setPower(val_tag/2);

                }
            }

            // TODO: TURRET ALIGNMENT
            if (gamepad2.dpad_left)
            {
                required_target = 40;
            }

            if (gamepad2.dpad_right)
            {
                required_target =-45;
            }

            if (gamepad2.dpad_up)
            {
                required_target = Globals.turretInit;
            }

            if (gamepad1.start){
                turretActuate=true;
            }
            if(turretActuate){
                runningActions.add(
                        new SequentialAction(
                                Outtake.TurretCommand(Outtake.TurretState.IMU_TRACK)
                        )
                );
            }
            if(!turretActuate){
                runningActions.add(
                        new SequentialAction(
                                Outtake.TurretCommand(Outtake.TurretState.INIT)
                        )
                );
            }
            if (gamepad1.share){
                turretActuate=false;
            }


            if (gamepad1.touchpad)
            {
                drive.navxMicro.initialize();
            }


            // TODO: INTAKE START

            boolean intakeToggleEdge =
                    currentGamepad1.left_bumper && !previousGamepad1.left_bumper;

            intakeFSM.update(intakeToggleEdge, robot, intake, feeder);


            // TODO: SINGLE FEED + SHOOTING
            if (counterB > 3)
            {
                counterB =0;
            }

            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper && counterFeed <=2){

                counterB +=1;
                feedButtonTimer.reset();
                feedButtonFlag = false;
            }

            if(counterB == 1)
            {
                feedFlag = true;
            }
            if (feedFlag && previousGamepad1.right_bumper)
            {
                feedButtonFlag = true;
            }
            else if (!feedFlag && previousGamepad1.right_bumper)
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


                }

                if (feedButtonTimer.milliseconds()> rampTime+500)
                {
                    runningActions.add(
                            new ParallelAction(
                                    Feeder.UFCommand(Feeder.UpperFeederState.OFF),
//                                    Outtake.HoodCommand(Outtake.HoodState.NEAR_END),
                                    Outtake.ShooterCommand(Outtake.ShooterState.SLOW)
                            )
                    );

                    feedButtonFlag = false;
                    counterB = 0;
                    counterFeed =0;
                }
                else {

                    runningActions.add(
                            new ParallelAction(
                                    Feeder.UFCommand(Feeder.UpperFeederState.ON)
                            )
                    );
                }

            }


            if (feedButtonFlag && counterFeed == 2 && previousGamepad1.right_bumper && counterB ==3)
            {

                counterFeed = 0;
                runningActions.add(
                        new ParallelAction(
                                Feeder.UFCommand(Feeder.UpperFeederState.OFF)
                        )
                );

            }

            if (gamepad1.dpad_up){
                counterFeed = 0;
                intakeFlag = false;
            }


            // TODO: RAPID SHOOTING SEQUENCE

            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up){
                sequenceFlag = !sequenceFlag;
            }


            if (sequenceFlag && !farFlag)
            {
                Actions.runBlocking(
                        new SequentialAction(
                                new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.ON)),
                                new SleepAction(0.3), //0.3
                                new ParallelAction(
                                        new InstantAction(() -> new UFCommand(feeder,Feeder.UpperFeederState.OFF)),
                                        new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.ON))
                                ),
                                new SleepAction(0.3), //0.3
                                new ParallelAction(
                                        new InstantAction(() -> new UFCommand(feeder,Feeder.UpperFeederState.ON)),
                                        new InstantAction(()-> new RollerCommand(intake, Intake.IntakeRollerState.ON))
                                ),
                                new SleepAction(0.2), //0.3
                                new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.OFF)),
                                new SleepAction(0.3), //0.3
                                new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.ON)),
                                new SleepAction(0.5),
                                new InstantAction(()-> sequenceFlag = false)
                        )
                );
            }


            if (sequenceFlag && farFlag)
            {
                Actions.runBlocking(
                        new SequentialAction(
                                new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.ON)),
                                new SleepAction(0.3), //0.3
                                new ParallelAction(
                                        new InstantAction(() -> new UFCommand(feeder,Feeder.UpperFeederState.OFF)),
                                        new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.ON))
                                ),
                                new SleepAction(0.5), //0.3
                                new ParallelAction(
                                        new InstantAction(() -> new UFCommand(feeder,Feeder.UpperFeederState.ON)),
                                        new InstantAction(()-> new RollerCommand(intake, Intake.IntakeRollerState.ON))
                                ),
                                new SleepAction(0.1), //0.3
                                new ParallelAction(
                                        new InstantAction(() -> new UFCommand(feeder,Feeder.UpperFeederState.OFF)),
                                        new InstantAction(()-> new RollerCommand(intake, Intake.IntakeRollerState.ON))
                                ),
                                new SleepAction(0.8), //0.3
                                new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.ON)),
                                new SleepAction(0.5),
                                new InstantAction(()-> sequenceFlag = false)
                        )
                );
            }

            // TODO: SINGLE ARTIFACT SHOOT

            if(currentGamepad1.b && !previousGamepad1.b)
            {
                singleShoot = !singleShoot;
                counterFeed = 0;
            }
//
            if(singleShoot)
            {
                runningActions.add(
                        new SequentialAction(
                                Feeder.UFCommand(Feeder.UpperFeederState.ON),
                                new SleepAction(0.15),
                                new ParallelAction(
                                        Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                        new InstantAction(()-> singleShoot = false)
                                )
                        )
                );
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
                                    Feeder.LFCommand(Feeder.LowerFeederState.ON),
                                    Intake.RollerCommand(Intake.IntakeRollerState.RELEASE)
                            )
                    );
                }
            }


            // TODO: FAR SHOOTING

            if (gamepad1.y){
                Globals.shooterMode = true;
                runningActions.add(
                        new SequentialAction(
                                Outtake.HoodCommand(Outtake.HoodState.FAR),
                                Outtake.ShooterCommand(Outtake.ShooterState.TELE_FAR)
                        )
                );

            }

            // TODO: NEAR SHOOTING

            if (gamepad1.x){
                Globals.shooterMode = true;

                runningActions.add(
                        new SequentialAction(
                                Outtake.HoodCommand(Outtake.HoodState.INIT),
                                Outtake.ShooterCommand(Outtake.ShooterState.NEAR)
                        )
                );

            }

            if (farFlag)
            {
                runningActions.add(
                        new SequentialAction(
//                                Outtake.HoodCommand(Outtake.HoodState.FAR),
                                Outtake.ShooterCommand(Outtake.ShooterState.TELE_FAR)
                        )
                );
            }
            if (!farFlag)
            {
                runningActions.add(
                        new SequentialAction(
                                Outtake.HoodCommand(Outtake.HoodState.INIT),
                                Outtake.ShooterCommand(Outtake.ShooterState.NEAR)
                        )
                );
            }

            if (angle > 90)
            {
                angle = 90;
            }
            if (angle < -90)
            {
                angle = -90;
            }



//            lastOBeamstate = currentOBeamstate;


            dashboard.sendTelemetryPacket(packet);

            telemetry.addLine();
            packet.put("Target Velocity", Globals.curretShooterStateVelMode);
            packet.put("Current Velocity", robot.shooter.getVelocity());

            telemetry.addData("loop (ms)", "%.2f", loopTimer.milliseconds());
            telemetry.addData("tagID : ", tagID);

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

    public double Aprilpid(double x){
        double Error = Target_tag - x;
        Integral_tag += Error;
        double Derivative = Error - LastError_tag;
        LastError_tag = Error;

        Power_tag = kP_tag * Error + kI_tag * Integral_tag + kD_tag * Derivative;
        Power_tag = Range.clip(Power_tag, -1.0, 1.0);
        return Power_tag;
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

    public double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
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
//            telemetry.addLine("neagtive heading");
            error = c + target;
//            error = -error;
            integral = integral + error;
            derivative = error - previous_error;

            previous_error = error;
            pid = Globals.kp_turret*error + Globals.kd_turret*derivative + Globals.ki_turret*integral+0.05;
            robot.turret1.setPower(-pid);
            robot.turret2.setPower(pid);
            return -pid;

        }
        else {
//            telemetry.addLine("positive heading");

            error = target - c;
//            error = -error;
            integral = integral + error;
            derivative = error - previous_error;

            previous_error = error;
            pid = Globals.kp_turret*error + Globals.kd_turret*derivative + Globals.ki_turret*integral+0.05;
            robot.turret1.setPower(-pid);
            robot.turret2.setPower(pid);
            return -pid;
        }
    }

}