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
import org.firstinspires.ftc.teamcode.utils.DualSenseLightbar;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.ArrayList;
import java.util.List;

@Config
//@TeleOp(name = "SB5 TAG", group = "ATELEOP")
public class SB5_Tag_Turret extends LinearOpMode {
    private static RobotHardware robot=RobotHardware.getInstance();

    public static double pose_x;
    public static double pose_y;
    public static double pose_heading;

    public static double servo_low = 0.79;
    public static double servo_high = 0.83;

    ElapsedTime intakeTimer;
    ElapsedTime motionTimer;
    ElapsedTime secondBallTimer;
    ElapsedTime feedButtonTimer;
    private Intake intake;
    private Outtake outtake;
    private Feeder feeder;
    private MecanumDrive drive;
    double pos;
    private FtcDashboard dashboard;

    //TODO ---------------------------TAG CONSTANTS

    public static double tagID_tag, x_tag, y_tag, z_tag, val_tag = 0;
    public static double kP_tag = 0.025;
    public static double kI_tag = 0;
    public static double kD_tag = 0.25;
    public static double Integral_tag = 0;
    public static double LastError_tag = 0;
    public static int Target_tag = 0;
    public double Power_tag = 0;
    public static boolean allThrre_tag = false;
    public static boolean April_tag = true;
    public int flagin_tag = 0;
    public static boolean bFlag_tag = false;


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


        intake = new Intake(robot);
        outtake = new Outtake(robot);
        feeder = new Feeder(robot);
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


        Thread Campid = new Thread(()->{
            while (!Thread.currentThread().isInterrupted() && opModeIsActive()){
                try {
                    val_tag = Aprilpid(z_tag);
                }
                catch (Exception e) {
                    Thread.currentThread().interrupt();
                    throw new RuntimeException(e);
                }
            }
            sleep(10);
        });


//         trajectoryAction2 = drive.actionBuilder(
//
//                 new Pose2d(pose_x, pose_y,Math.toRadians(pose_heading)))
//
//                .strafeToLinearHeading(new Vector2d(60, -41), Math.toRadians(149))
//
//                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

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
            motionFlag = false;
            drive.lazyImu.get().resetYaw();

            runningActions = updateAction();
            runningActions.add(InitSeq.InitAction(intake,outtake,feeder));

            if (gamepad1.a)
            {
                tagID = 21;
            }
            if (gamepad1.b)
            {
                tagID = 24;
            }

            actualPos = robot.turretEncoder.getCurrentPosition();

            angle = getContinuousIMU(Globals.currentTurretState);
            run_turret(angle, 0, 27845, actualPos, telemetry);
        }
        robot.resetEncoder();
        Campid.start();

        waitForStart();

        while (opModeIsActive()){

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
//            currentOBeamstate = robot.outtakeBeam.getState();


//            drive.driveFieldCentric(-gamepad1.left_stick_x,
//                    -gamepad1.left_stick_y,
//                    gamepad1.right_stick_x,
//                    botHeading*0.8);
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
            if (gamepad1.a)
            {

                trajectoryAction2 = drive.actionBuilder(

                                new Pose2d(pose_x, pose_y,Math.toRadians(pose_heading)))

                        .strafeToLinearHeading(new Vector2d(-98, -98), Math.toRadians(55))

                        .build();
                Actions.runBlocking(
                        new SequentialAction(
                                trajectoryAction2
                        ));
            }

            drive.updatePoseEstimate();
            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper)
            {
                bFlag_tag = true;
            } else if (gamepad2.left_bumper) {
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
//                bFlag = false;
            }

//            if(!bFlag_tag)
//            {
//                robot.turret1.setPower(0);
//                robot.turret1.setPower(0);
//            }


            if (gamepad1.touchpad)
            {
                drive.navxMicro.initialize();
            }
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper){


//                counterFeed = 0;
//                counterB = 0;
                intakeFlag = !intakeFlag;
//                motionTimer.reset();
//                motionFlag = false;
            }

            if (intakeFlag && robot.outtakeBeam.getState())       //TODO : //////////////////BUGS/////////////////////////
            {
//                thirdBallFlag = false;
//                counterFeed = 0;

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
//                counterB = 0;
            }
            if (robot.feederBeam.getState() && counterFeed ==1 && intakeFlag){
                counterFeed += 1;
                shoot1 = true;
            }
            if (counterFeed == 2 && !robot.intakeBeam.getState() && intakeFlag)
            {
//                nearFlag =true;
                secondBallTimer.reset();
                secondBallFlag = true;

            }
            if(secondBallFlag)
            {
                if(secondBallTimer.milliseconds()>secondBallTime){
                    counterFeed = 4;
                    runningActions.add(
                            new ParallelAction(
                                    Feeder.LFCommand(Feeder.LowerFeederState.OFF),
//                                    Outtake.HoodCommand(Outtake.HoodState.NEAR_END),
                                    Outtake.ShooterCommand(Outtake.ShooterState.NEAR)

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
                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
//                                Outtake.HoodCommand(Outtake.HoodState.NEAR_END),
                                Outtake.ShooterCommand(Outtake.ShooterState.NEAR)
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
                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
//                                Outtake.HoodCommand(Outtake.HoodState.NEAR_END),
                                Outtake.ShooterCommand(Outtake.ShooterState.NEAR)
                        )
                );

                shoot1 = false;
                shoot2 = true;
            }

            if (counterFeed==4 && !robot.intakeBeam.getState()&& intakeFlag){
                counterFeed += 1;
                runningActions.add(
                        new ParallelAction(
                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
//                                Outtake.HoodCommand(Outtake.HoodState.NEAR_END),
                                Outtake.ShooterCommand(Outtake.ShooterState.NEAR)
                        )
                );
                zeroFlag = false;

            }
            if (counterFeed==5 && robot.intakeBeam.getState()&& intakeFlag){
//                motionFlag = true;
//                motionTimer.reset();
                counterFeed = 0;
                counterB = 0;
//                DualSenseLightbar.setColor(0, 255, 0); // 🟢 Green

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

            if (counterB > 3)
            {
                counterB =0;
            }

            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper && counterFeed <=2){

                counterB +=1;
//                feedFlag;
                feedButtonTimer.reset();
                feedButtonFlag = false;
            }

//            if (counterFeed != 0 && counterB>0)
//            {
//
//            }

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
//                intakeFlag = true;
//                nearFlag =false;

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
//                                new InstantAction(()-> intakeFlag = false),
                                        new InstantAction(()-> singleShoot = false)

                                )

                        )
                );
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
                                Outtake.ShooterCommand(Outtake.ShooterState.TELE_FAR)
                        )
                );
//                farFlag = true;

            }

            if (gamepad1.x){
                Globals.shooterMode = true;

                runningActions.add(
                        new SequentialAction(
                                Outtake.HoodCommand(Outtake.HoodState.NEAR_END),
                                Outtake.ShooterCommand(Outtake.ShooterState.NEAR)
                        )
                );
//                nearFlag =true;

            }



//            if (gamepad1.x){
//                Globals.shooterMode = true;
//                runningActions.add(
//                        new SequentialAction(
//                                Outtake.ShooterCommand(Outtake.ShooterState.NEAR)
//                        )
//                );
//            }
//            if (gamepad1.right_bumper){
//                Globals.shooterMode = true;
//
//
//            }


//            if (nearFlag)
//            {

//            }
//
//            if(farFlag)
//            {

//            }
//
//            if(!farFlag || !nearFlag)
//            {
//                runningActions.add(
//                        new SequentialAction(
//                                Outtake.ShooterCommand(Outtake.ShooterState.OFF)
//                        )
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


//            angle = getContinuousIMU(Globals.currentTurretState);
//            run_turret(angle, 0, 27845, actualPos, telemetry);
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
            telemetry.addData("counterB: ",  counterB);
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
