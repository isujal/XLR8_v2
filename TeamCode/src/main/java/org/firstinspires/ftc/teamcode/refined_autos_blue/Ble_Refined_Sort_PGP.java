package org.firstinspires.ftc.teamcode.refined_autos_blue;


import static org.firstinspires.ftc.teamcode.subsystem.Outtake.extendShooterUsingVelocity;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.instantCommands.HoodCommand;
import org.firstinspires.ftc.teamcode.instantCommands.LFCommand;
import org.firstinspires.ftc.teamcode.instantCommands.RollerCommand;
import org.firstinspires.ftc.teamcode.instantCommands.ServoCommand;
import org.firstinspires.ftc.teamcode.instantCommands.ShooterCommand;
import org.firstinspires.ftc.teamcode.instantCommands.TurretCommand;
import org.firstinspires.ftc.teamcode.instantCommands.UFCommand;
import org.firstinspires.ftc.teamcode.sequences.AutoInitSeq;
import org.firstinspires.ftc.teamcode.subsystem.Feeder;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

import java.util.Arrays;

@Config
//@Autonomous(name="SB AUTO NEAR SORT PGP 🔵")
//@Deprecated
public class Ble_Refined_Sort_PGP extends LinearOpMode {
    private RobotHardware robot = RobotHardware.getInstance();
    //Subsystems
    Outtake outtake ;

    Intake intake ;
    Feeder feeder ;
    double actualPos;
    public static double ramptime = 0.5;
    public static int counterFeed = 0;
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
    ElapsedTime intakeTimer;
    public static int intakeTime = 800;
    public static int intakeTimex = 1500;

    private Thread PIDThread;

    public static double P = 30;
    public static double I = 0;
    public static double D = 0;
    public static double F = 12.5;
    //Drive
    private MecanumDrive drive = null;
    private static int buffer  = 20;
    private static int state1  = -1;
    private static boolean readytoShootFar  = false;
    private static boolean readytoShootNear  = false;
    private ElapsedTime timer = new ElapsedTime();

    VelConstraint vel =new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(30)));
    AccelConstraint accel0 = new ProfileAccelConstraint(-45,30);
    AccelConstraint accel = new ProfileAccelConstraint(-45,10);
    AccelConstraint accel2 = new ProfileAccelConstraint(-45,5);


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        intakeTimer = new ElapsedTime();

        intake = new Intake(robot);
        outtake = new Outtake(robot);
        feeder = new Feeder(robot);

        Pose2d  startPose = new Pose2d(-38, -55, Math.toRadians(-90));
        drive = new MecanumDrive(hardwareMap, startPose);
        robot.shooter.setVelocityPIDFCoefficients(P,I,D,F);
        actualPos = robot.turretEncoder.getCurrentPosition();

        angle = getContinuousIMU(Globals.currentTurretState);
        run_turret(angle, 0, 27845, actualPos, telemetry);

        //TODO ===============================================TRAJECTORIES =============================================================

//        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.ON)),
//        new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.ON)),
//        new InstantAction(() -> new RollerCommand(intake, Intake.IntakeRollerState.ON)),

        Action trajectoryAction01 = drive.actionBuilder(new Pose2d(-38, -55,Math.toRadians(-90)))
                .afterTime(0.1, ()->Actions.runBlocking( new ParallelAction(

                        new InstantAction(()-> new TurretCommand(outtake, Outtake.TurretState.BUFF_RED)),
                        new InstantAction(()-> new ShooterCommand(outtake, Outtake.ShooterState.RELEASE)),
                        new InstantAction(()-> new HoodCommand(outtake, Outtake.HoodState.NEAR_END)),
                        new InstantAction(()-> new ServoCommand(intake, Intake.IntakeServoState.AUTO_IN))

                )))
                .strafeToLinearHeading(new Vector2d(-5, -16.01), Math.toRadians(-90))


                .build();

        Action trajectoryAction1 = drive.actionBuilder(new Pose2d(-5, -16.01,Math.toRadians(-90)))

                .strafeToLinearHeading(new Vector2d(-5,-16), Math.toRadians(-90))
                .waitSeconds(0.05)
                .stopAndAdd(()->Actions.runBlocking(new SequentialAction(
                        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.ON)),
                        new SleepAction(0.3),
                        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.OFF)),
                        new SleepAction(0.2),
                        new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.ON)),
                        new SleepAction(0.3),
                        new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.OFF)),
                        new SleepAction(0.3),
                        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.ON)),
                        new SleepAction(0.2),
                        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.OFF)),

                        new ParallelAction(
                                new InstantAction(() -> new RollerCommand(intake, Intake.IntakeRollerState.RELEASE)),
                                new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.ON))
                        ),
                        new SleepAction(0.3),
                        new ParallelAction(
                                new InstantAction(() -> new RollerCommand(intake, Intake.IntakeRollerState.ON)),
                                new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.ON))
                        ),
                        new SleepAction(0.4),
                        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.ON))
//
//
                )))
                .waitSeconds(0.2)


                .build();

        Action trajectoryAction21 = drive.actionBuilder(new Pose2d(-5, -16,Math.toRadians(-90)))

                .strafeToLinearHeading(new Vector2d(-5, -16.01), Math.toRadians(-90))

                .build();

        Action trajectoryAction31 = drive.actionBuilder(new Pose2d(-5, -16.01,Math.toRadians(-90)))
                .afterTime(0.01,() ->Actions.runBlocking(
                                new SequentialAction(
                                        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.OFF)),
                                        new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.ON))
                                )
                        )
                )

                .strafeToLinearHeading(new Vector2d(-5, -35), Math.toRadians(-90), vel, accel0)
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(-5, -55), Math.toRadians(-90), vel, accel2)


                .build();



        Action trajectoryAction41 = drive.actionBuilder(new Pose2d(-5, -55.01,Math.toRadians(-90)))

                .strafeToLinearHeading(new Vector2d(-5, -16), Math.toRadians(-90))
                .build();


        Action trajectoryAction4x1 = drive.actionBuilder(new Pose2d(-5, -55,Math.toRadians(-90)))
                .stopAndAdd(()->
                        new RollerCommand(intake,Intake.IntakeRollerState.ON))
                .strafeToConstantHeading(new Vector2d(-5, -55.01))
                .waitSeconds(0.05)
                .stopAndAdd(()->Actions.runBlocking(new ParallelAction(
                        new InstantAction(()-> new RollerCommand(intake,Intake.IntakeRollerState.ON)),
                        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.OFF)),
                        new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.OFF))
                )))
                .waitSeconds(0.1)
                .stopAndAdd(()->Actions.runBlocking(new ParallelAction(
                        new InstantAction(()-> new RollerCommand(intake,Intake.IntakeRollerState.OFF)),
                        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.OFF)),
                        new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.OFF))
                )))
                .waitSeconds(0.05)
                .build();

        Action trajectoryAction51 = drive.actionBuilder(new Pose2d(-5, -16,Math.toRadians(-90)))

                .waitSeconds(0.1)
                .stopAndAdd(()->Actions.runBlocking(new SequentialAction(
                        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.ON)),
                        new SleepAction(0.3),

                        new ParallelAction(
                                new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.OFF)),
                                new InstantAction(() -> new RollerCommand(intake, Intake.IntakeRollerState.RELEASE)),
                                new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.ON))
                        ),

                        new SleepAction(0.2),


                        new ParallelAction(
                                new InstantAction(() -> new RollerCommand(intake, Intake.IntakeRollerState.ON)),
                                new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.ON))
                        ),
                        new SleepAction(0.2),


                        new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.OFF)),
                        new SleepAction(0.4),
                        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.ON)),
                        new SleepAction(0.2),


                        new ParallelAction(
                                new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.OFF)),
                                new InstantAction(() -> new RollerCommand(intake, Intake.IntakeRollerState.ON)),
                                new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.ON))
                        ),


                        new SleepAction(0.5),
                        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.ON))

                )))
                .waitSeconds(0.2)








//                .afterTime(0.01,() ->Actions.runBlocking(
//                                new SequentialAction(
//                                        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.OFF)),
//                                        new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.ON))
//                                )
//                        )
//                )

//                .waitSeconds(0.1)
                .build();




        Action trajectoryAction61 = drive.actionBuilder(new Pose2d(-5, -16,Math.toRadians(-90)))
                .afterTime(0.01,() ->Actions.runBlocking(
                                new SequentialAction(
                                        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.OFF)),
                                        new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.ON))
                                )
                        )
                )
                .strafeToLinearHeading(new Vector2d(20, -32), Math.toRadians(-90))


                .build();


        Action trajectoryAction6x1 = drive.actionBuilder(new Pose2d(20, -32,Math.toRadians(-90)))

                .strafeToLinearHeading(new Vector2d(20, -60), Math.toRadians(-90), vel,accel0)


                .build();



        Action trajectoryAction71 = drive.actionBuilder(new Pose2d(20, -60.01,Math.toRadians(-90)))

                .strafeToLinearHeading(new Vector2d(-5, -16), Math.toRadians(-90))
                .build();


        Action trajectoryAction7x1 = drive.actionBuilder(new Pose2d(20, -60,Math.toRadians(-90)))
                .stopAndAdd(()->
                        new RollerCommand(intake,Intake.IntakeRollerState.ON))
                .strafeToConstantHeading(new Vector2d(20, -60.01))
                .waitSeconds(0.05)
                .stopAndAdd(()->Actions.runBlocking(new ParallelAction(
                        new InstantAction(()-> new RollerCommand(intake,Intake.IntakeRollerState.ON)),
                        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.OFF)),
                        new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.OFF))
                )))
                .waitSeconds(0.1)
                .stopAndAdd(()->Actions.runBlocking(new ParallelAction(
                        new InstantAction(()-> new RollerCommand(intake,Intake.IntakeRollerState.OFF)),
                        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.OFF)),
                        new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.OFF))
                )))
                .waitSeconds(0.05)
                .build();

        Action trajectoryAction81 = drive.actionBuilder(new Pose2d(-5, -16,Math.toRadians(-90)))

                .waitSeconds(0.1)
                .stopAndAdd(()->Actions.runBlocking(new SequentialAction(
                        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.ON)),
                        new SleepAction(0.3),

                        new ParallelAction(
                                new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.OFF)),
                                new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.ON))
                        ),

                        new SleepAction(0.2),
                        new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.OFF)),
                        new SleepAction(0.3),
                        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.ON)),
                        new SleepAction(0.2),
                        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.OFF)),

                        new ParallelAction(
                                new InstantAction(() -> new RollerCommand(intake, Intake.IntakeRollerState.RELEASE)),
                                new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.ON))
                        ),
                        new SleepAction(0.3),
                        new ParallelAction(
                                new InstantAction(() -> new RollerCommand(intake, Intake.IntakeRollerState.ON)),
                                new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.ON))
                        ),
                        new SleepAction(0.4),
                        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.ON))

                )))
                .waitSeconds(0.2)
//                .afterTime(0.01,() ->Actions.runBlocking(
//                                new SequentialAction(
//                                        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.OFF)),
//                                        new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.ON))
//                                )
//                        )
//                )

//                .waitSeconds(0.1)
                .build();





        Action trajectoryAction91 = drive.actionBuilder(new Pose2d(-5, -16,Math.toRadians(-90)))
                .afterTime(0.01,() ->Actions.runBlocking(
                                new SequentialAction(
                                        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.OFF)),
                                        new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.OFF))
                                )
                        )
                )
                .strafeToLinearHeading(new Vector2d(42.5, -32), Math.toRadians(-90))

                .build();


        Action trajectoryAction9x1 = drive.actionBuilder(new Pose2d(42.5, -32,Math.toRadians(-90)))

                .strafeToLinearHeading(new Vector2d(42.5, -40), Math.toRadians(-90), vel, accel2)
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(42.5, -58), Math.toRadians(-90), vel, accel2)


                .build();



        Action trajectoryAction101 = drive.actionBuilder(new Pose2d(42.5, -58.01,Math.toRadians(-90)))

                .strafeToLinearHeading(new Vector2d(-5, -16), Math.toRadians(-90))
                .build();


        Action trajectoryAction10x1 = drive.actionBuilder(new Pose2d(42.5, -58,Math.toRadians(-90)))
                .stopAndAdd(()->
                        new RollerCommand(intake,Intake.IntakeRollerState.ON))
                .strafeToConstantHeading(new Vector2d(42.5, -58.01))
                .waitSeconds(0.1)
                .stopAndAdd(()->Actions.runBlocking(new ParallelAction(
                        new InstantAction(()-> new RollerCommand(intake,Intake.IntakeRollerState.ON)),
                        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.OFF)),
                        new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.OFF))
                )))
                .waitSeconds(0.1)
                .stopAndAdd(()->Actions.runBlocking(new ParallelAction(
                        new InstantAction(()-> new RollerCommand(intake,Intake.IntakeRollerState.OFF)),
                        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.OFF)),
                        new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.OFF))
                )))
                .waitSeconds(0.1)
                .build();

        Action trajectoryAction111 = drive.actionBuilder(new Pose2d(-5, -16,Math.toRadians(-90)))

                .waitSeconds(0.1)
                .stopAndAdd(()->Actions.runBlocking(new SequentialAction(
                        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.ON)),
                        new SleepAction(0.3),

                        new ParallelAction(
                                new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.OFF)),
                                new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.ON))
                        ),

                        new SleepAction(0.2),
                        new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.OFF)),
                        new SleepAction(0.3),
                        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.ON)),
                        new SleepAction(0.2),
                        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.OFF)),

                        new ParallelAction(
                                new InstantAction(() -> new RollerCommand(intake, Intake.IntakeRollerState.RELEASE)),
                                new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.ON))
                        ),
                        new SleepAction(0.3),
                        new ParallelAction(
                                new InstantAction(() -> new RollerCommand(intake, Intake.IntakeRollerState.ON)),
                                new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.ON))
                        ),
                        new SleepAction(0.4),
                        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.ON))

                )))
                .waitSeconds(0.2)
//                .afterTime(0.01,() ->Actions.runBlocking(
//                                new SequentialAction(
//                                        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.OFF)),
//                                        new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.ON))
//                                )
//                        )
//                )

                .waitSeconds(0.1)
                .build();









        PIDThread = new Thread(() -> {
            while (!Thread.currentThread().isInterrupted() && opModeIsActive()) {
                try {

                    robot.shooter.setVelocityPIDFCoefficients(P,I,D,F);
                    extendShooterUsingVelocity(Globals.curretShooterStateVelMode);
                    actualPos = robot.turretEncoder.getCurrentPosition();

                    angle = getContinuousIMU(Globals.currentTurretState);
                    run_turret(angle, 0, 27845, actualPos, telemetry);



                    if (!robot.feederBeam.getState() && counterFeed == 0 && state1 == 9) {
                        counterFeed += 1;

                        Actions.runBlocking(
                                new ParallelAction(
                                        Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                        Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                        Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                        Feeder.LFCommand(Feeder.LowerFeederState.ON)
                                )
                        );

                    }
                    if (robot.feederBeam.getState() && counterFeed == 1 && state1 == 9   ) {
                        counterFeed += 1;
                        Actions.runBlocking(
                                new ParallelAction(
                                        Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                        Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                        Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                        Feeder.LFCommand(Feeder.LowerFeederState.ON)
                                )
                        );
                    }
                    if (counterFeed == 2 && !robot.feederBeam.getState() && state1 == 9) {
                        counterFeed += 1;
                        Actions.runBlocking(
                                new ParallelAction(
                                        Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                        Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                        Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                        Feeder.LFCommand(Feeder.LowerFeederState.ON)
                                )
                        );
                    }
                    if (counterFeed == 3 && robot.feederBeam.getState() && state1 == 9) {
                        counterFeed += 1;
                        Actions.runBlocking(
                                new ParallelAction(
                                        Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                        Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                        Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                        Feeder.LFCommand(Feeder.LowerFeederState.OFF)
                                )
                        );

                    }

                    if (counterFeed == 4 && !robot.intakeBeam.getState() && state1 == 9) {
                        counterFeed += 1;
                        Actions.runBlocking(
                                new ParallelAction(
                                        Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                        Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                        Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                        Feeder.LFCommand(Feeder.LowerFeederState.OFF)
                                )
                        );
                    }
                    if (counterFeed == 5 && robot.intakeBeam.getState() && state1 == 9) {
//                motionFlag = true;
//                motionTimer.reset();
                        counterFeed = 0;
//                    gamepad1.rumble(1, 1, 400);
                        Actions.runBlocking(
                                new ParallelAction(
                                        Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                        Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                                        Intake.RollerCommand(Intake.IntakeRollerState.OFF)
                                )
                        );
                        state1 = 10;

                    }







                    if (state1 == 3  && Globals.counterFeed_PGPtoPPG == 0)
                    {
//                thirdBallFlag = false;


//                        Actions.runBlocking(IntakeSeq.IntakeStoreAction2(intake,outtake,feeder));

                        Actions.runBlocking(
                                new ParallelAction(
                                        Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                        Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                        Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                        Feeder.LFCommand(Feeder.LowerFeederState.ON)
                                )
                        );



                    }


                    if (!robot.outtakeBeam.getState() && Globals.counterFeed_PGPtoPPG==0 && state1 == 3 ){
                        Globals.counterFeed_PGPtoPPG += 1;


                        Actions.runBlocking(
                                new ParallelAction(
                                        Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                        Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                        Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                        Feeder.LFCommand(Feeder.LowerFeederState.OFF)
                                )
                        );
//                        Actions.runBlocking(
//                                new ParallelAction(
//                                        Intake.RollerCommand(Intake.IntakeRollerState.ON)
//                                )
//                        );

                    }

                    if (Globals.counterFeed_PGPtoPPG==1 && !robot.intakeBeam.getState()&& state1 == 3 ){
                        Globals.counterFeed_PGPtoPPG += 1;


                        Actions.runBlocking(
                                new ParallelAction(
                                        Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                        Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                        Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                        Feeder.LFCommand(Feeder.LowerFeederState.OFF)
                                )
                        );

//                zeroFlag = true;
                    }
                    if (Globals.counterFeed_PGPtoPPG==2 && robot.intakeBeam.getState()&& state1 == 3 ){
                        Globals.counterFeed_PGPtoPPG += 1;


                        Actions.runBlocking(
                                new ParallelAction(
                                        Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                        Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                        Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                        Feeder.LFCommand(Feeder.LowerFeederState.ON)
                                )
                        );


                    }

                    if (Globals.counterFeed_PGPtoPPG==3 && !robot.intakeBeam.getState()&& state1 == 3 ){
                        Globals.counterFeed_PGPtoPPG += 1;

                        Actions.runBlocking(
                                new ParallelAction(
                                        Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                        Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                        Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                        Feeder.LFCommand(Feeder.LowerFeederState.ON)
                                )
                        );


                    }
                    if (Globals.counterFeed_PGPtoPPG==4 && !robot.feederBeam.getState()&& state1 == 15 ){
//                motionFlag = true;
//                motionTimer.reset();
                        Globals.counterFeed_PGPtoPPG = 0;
                        gamepad1.rumble(1,1,400);


                        Actions.runBlocking(
                                new ParallelAction(
                                        Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                        Intake.RollerCommand(Intake.IntakeRollerState.OFF),
                                        Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                        Feeder.LFCommand(Feeder.LowerFeederState.OFF)
                                )
                        );

                        state1 =4;
                    }





                    if (state1 ==15 && Globals.counterFeed_GPPtoPPG ==0)
                    {
//                thirdBallFlag = false;
//                Actions.runBlocking(IntakeSeq.IntakeStoreAction(intake,outtake,feeder));

//                Globals.counterFeed_GPPtoPGP =;
                        Actions.runBlocking(
                                new ParallelAction(
                                        Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                        Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                        Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                        Feeder.LFCommand(Feeder.LowerFeederState.OFF)
                                )
                        );
                    }
//            if ()
//            {
//                Actions.runBlocking(
//                        new ParallelAction(
//                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
//                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
//                                Intake.RollerCommand(Intake.IntakeRollerState.OFF)
//                        )
//                );
//            }

                    if (!robot.intakeBeam.getState() && Globals.counterFeed_GPPtoPPG==0 && state1 ==15){
                        Globals.counterFeed_GPPtoPPG += 1;
                        Actions.runBlocking(
                                new ParallelAction(
                                        Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                        Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                        Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                        Feeder.LFCommand(Feeder.LowerFeederState.OFF)
                                )
                        );

                    }
                    if (robot.intakeBeam.getState() && Globals.counterFeed_GPPtoPPG ==1 && state1 ==15){
                        Globals.counterFeed_GPPtoPPG += 1;
                        Actions.runBlocking(
                                new ParallelAction(
                                        Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                        Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                        Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                        Feeder.LFCommand(Feeder.LowerFeederState.ON)
                                )
                        );
                    }
                    if (Globals.counterFeed_GPPtoPPG==2 && !robot.feederBeam.getState()&& state1 ==15){
                        Globals.counterFeed_GPPtoPPG += 1;
                        Actions.runBlocking(
                                new ParallelAction(
                                        Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                        Intake.RollerCommand(Intake.IntakeRollerState.RELEASE),
                                        Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                        Feeder.LFCommand(Feeder.LowerFeederState.ON)
                                )
                        );
//                zeroFlag = true;
                    }

                    if (Globals.counterFeed_GPPtoPPG==3 && !robot.intakeBeam.getState()&& state1 ==15){
                        Globals.counterFeed_GPPtoPPG += 1;
                        Actions.runBlocking(
                                new ParallelAction(
                                        Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                        Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                        Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                        Feeder.LFCommand(Feeder.LowerFeederState.ON)
                                )
                        );
//                zeroFlag = true;
                    }


                    if (Globals.counterFeed_GPPtoPPG==4 && robot.intakeBeam.getState()&& state1 ==15){
                        Globals.counterFeed_GPPtoPPG += 1;
                        Actions.runBlocking(
                                new ParallelAction(
                                        Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                        Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                        Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                        Feeder.LFCommand(Feeder.LowerFeederState.ON)
                                )
                        );

                    }

                    if (Globals.counterFeed_GPPtoPPG==5 && !robot.intakeBeam.getState()&& state1 ==15){
//                        Globals.counterFeed_GPPtoPPG += 1;
                        Globals.counterFeed_GPPtoPPG = 0;
                        gamepad1.rumble(1,1,400);
                        Actions.runBlocking(
                                new ParallelAction(
                                        Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                        Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                                        Intake.RollerCommand(Intake.IntakeRollerState.OFF)
                                )
                        );
                        state1 =16;

                    }













                    try {
                        if (robot.shooter.getVelocity()>(Globals.shooterFarVel-buffer)){
                            readytoShootFar=true;
                        }
                        else if(robot.shooter.getVelocity()<(Globals.shooterFarVel-buffer)){
                            readytoShootFar=false;
                        }
                        if (robot.shooter.getVelocity()>(Globals.shooterNearVel-buffer)){
                            readytoShootNear=true;
                        }
                        else if(robot.shooter.getVelocity()<(Globals.shooterNearVel-buffer)){
                            readytoShootNear=false;
                        }
                        robot.outtakeBeamStore = robot.outtakeBeam.getState();
                        robot.intakeBeamStore = robot.intakeBeam.getState();
                        robot.feederBeamState = robot.feederBeam.getState();

                    }
                    catch (Exception e){
                        telemetry.addLine("May be thread error");
                    }
                    telemetry.addData("ob",robot.outtakeBeamStore);
                    telemetry.addData("ib", robot.intakeBeamStore);
                    telemetry.addData("fb",robot.feederBeamState);

                    telemetry.addData("state1",state1);
                    telemetry.addData("counterFeed",counterFeed);

                    telemetry.addData("ob",robot.outtakeBeam.getState());
                    telemetry.addData("fb",robot.feederBeam.getState());
                    telemetry.addData("ib",robot.intakeBeam.getState());
                    telemetry.addData("ready to shoot",readytoShootFar);
                    telemetry.addData("ready to shoot",Globals.curretShooterStateVelMode);

                    /// Your Call For Thread
                    telemetry.addData("Shooter current  ", robot.shooter.getCurrent(CurrentUnit.AMPS));
                    telemetry.addData("Shooter Velocity  ", robot.shooter.getVelocity());

                    telemetry.update();


                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        });


        // TODO INIT
        while (opModeInInit() )
        {
            intakeTimer.reset();
            Globals.shooterMode=true;
            actualPos = robot.turretEncoder.getCurrentPosition();

            angle = getContinuousIMU(Globals.currentTurretState);
            run_turret(angle, 0, 27845, actualPos, telemetry);
            telemetry.addLine("ROBOT INIT MODE");
            Actions.runBlocking(

                    AutoInitSeq.InitActionNearBlue(intake, outtake, feeder)

            );

            readytoShootFar=false;
            readytoShootNear=false;
            state1=-1;
            telemetry.addData("ready to shoot",readytoShootFar);
            telemetry.addData("ready to shoot",readytoShootNear);
            telemetry.addData("state1",state1);
            telemetry.addData("counterFeed",counterFeed);
            telemetry.addData("ob",robot.outtakeBeam.getState());
            telemetry.addData("fb",robot.feederBeam.getState());
            telemetry.addData("ib",robot.intakeBeam.getState());
            telemetry.addData("VEL",robot.shooter.getVelocity());
            telemetry.addData("DIFF",Globals.shooterFarVel-buffer);

            telemetry.addData("x", drive.localizer.getPose().position.x);// drive.pose.position.x);
            telemetry.addData("y", drive.localizer.getPose().position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));//  drive.pose.heading.toDouble()));
//            telemetry.addData("Navx heading (deg)", TwoDeadWheelLocalizer.robotHeading);
            telemetry.update();
            counterFeed=0;
        }

        waitForStart();
        //pidThread.start();
        PIDThread.start();


        while (opModeIsActive() && !isStopRequested()) {
//            actualPos = robot.turretEncoder.getCurrentPosition();
//            robot.shooter.setVelocityPIDFCoefficients(P,I,D,F);

            //TODO preload Shoot Pose
            Globals.shooterMode=true;

            if (state1 == -1) {
                Actions.runBlocking(
                        new SequentialAction(
                                trajectoryAction01
                        )
                );
                state1 = 0;
            }
            if (state1 == 0) {
                Actions.runBlocking(
                        new SequentialAction(
                                trajectoryAction1
                        )
                );
                state1 = 1;
            }


            //TODO preload Shoot 1
            if(state1 == 1){
                Actions.runBlocking(
                        new SequentialAction(
                                trajectoryAction21
                        )
                );
                state1 = 2;
            }


            //TODO preload Shoot 3
            if (state1==2) {

                Actions.runBlocking(
                        new SequentialAction(
                                new InstantAction(()-> state1 = 3),
                                trajectoryAction31
                        )
                );
                intakeTimer.reset();

            }

            //TODO preload Shoot 2

            if (state1 == 3 && intakeTimer.milliseconds()>intakeTime)
            {
                Globals.counterFeed_PGPtoPPG = 0;
                state1 = 4;
                intakeTimer.reset();
            }


            if (state1==4) {
                Actions.runBlocking(
                        new SequentialAction(
                                trajectoryAction4x1

                        )
                );
                state1 = 5;
            }

            if (state1==5) {
                Actions.runBlocking(
                        new SequentialAction(
                                trajectoryAction41
                        )
                );

                state1 = 6;
            }

            if (state1==6) {
                Actions.runBlocking(
                        new SequentialAction(
                                trajectoryAction51
                        )
                );

                state1 = 7;
            }



            if (state1==7) {

                Actions.runBlocking(
                        new SequentialAction(
                                trajectoryAction61
                        )
                );
                intakeTimer.reset();
                state1 = 8;

            }





            if (state1==8) {

                Actions.runBlocking(
                        new SequentialAction(
                                new InstantAction(()-> state1 =9),
                                trajectoryAction6x1
                        )
                );
                intakeTimer.reset();

            }

            //TODO preload Shoot 2

            if (state1 == 9 && intakeTimer.milliseconds()>intakeTime)
            {
                counterFeed = 0;
                state1 = 10;
                intakeTimer.reset();
            }


            if (state1==10) {
                Actions.runBlocking(
                        new SequentialAction(
                                trajectoryAction7x1

                        )
                );
                state1 = 11;
            }

            if (state1==11) {
                Actions.runBlocking(
                        new SequentialAction(
                                trajectoryAction71
                        )
                );

                state1 = 12;
            }

            if (state1==12) {
                Actions.runBlocking(
                        new SequentialAction(
                                trajectoryAction81
                        )
                );

                state1 = 13;
            }






            if (state1==13) {

                Actions.runBlocking(
                        new SequentialAction(
                                trajectoryAction91
                        )
                );
                intakeTimer.reset();
                state1 = 14;

            }

            if (state1==14) {

                Actions.runBlocking(
                        new SequentialAction(
                                new InstantAction(()-> state1 =15),
                                trajectoryAction9x1
                        )
                );
                intakeTimer.reset();

            }

            //TODO preload Shoot 2

            if (state1 == 15 && intakeTimer.milliseconds()>intakeTime)
            {
                Globals.counterFeed_PGPtoPPG = 0;
                state1 = 16;
                intakeTimer.reset();
            }


            if (state1==16) {
                Actions.runBlocking(
                        new SequentialAction(
                                trajectoryAction10x1

                        )
                );
                state1 = 17;
            }

            if (state1==17) {
                Actions.runBlocking(
                        new SequentialAction(
                                trajectoryAction101
                        )
                );

                state1 = 18;
            }

            if (state1==18) {
                Actions.runBlocking(
                        new SequentialAction(
                                trajectoryAction111
                        )
                );

                state1 = 19;
            }

















//            PIDThread.interrupt();
            telemetry.addData("ready to shoot Far",readytoShootFar);
            telemetry.addData("ready to shoot Near",readytoShootNear);
            telemetry.addData("x", drive.localizer.getPose().position.x);// drive.pose.position.x);
            telemetry.addData("y", drive.localizer.getPose().position.y);
            telemetry.addData("VEL",robot.shooter.getVelocity());
            telemetry.addData("VEL",robot.shooter.getVelocity());
            telemetry.addData("state1",state1);
            telemetry.addData("counterFeed",counterFeed);

            telemetry.addData("ob",robot.outtakeBeam.getState());
            telemetry.addData("fb",robot.feederBeam.getState());
            telemetry.addData("ib",robot.intakeBeam.getState());
            telemetry.addData("DIFF",Globals.shooterFarVel-buffer);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));//  drive.pose.heading.toDouble()));
//            telemetry.addData("Navx heading (deg)", TwoDeadWheelLocalizer.robotHeading);
            telemetry.update();

//            if (isStopRequested()) {
//                PIDThread.interrupt();
//            }
        }

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

    public double run_turret(double imu, double min_in_pos, double max_in_pos, double pose, Telemetry telemetry){
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
