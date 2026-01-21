package org.firstinspires.ftc.teamcode.autos;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
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
import org.firstinspires.ftc.teamcode.instantCommands.ShooterCommand;
import org.firstinspires.ftc.teamcode.instantCommands.TurretCommand;
import org.firstinspires.ftc.teamcode.instantCommands.UFCommand;
import org.firstinspires.ftc.teamcode.sequences.AutoInitSeq;
import org.firstinspires.ftc.teamcode.sequences.InitSeq;
import org.firstinspires.ftc.teamcode.sequences.IntakeSeq;
import org.firstinspires.ftc.teamcode.subsystem.Feeder;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

import java.util.Arrays;

@Config
//@Autonomous(name="sort auto 🔴")
//@Deprecated
public class sort_auto extends LinearOpMode {
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
    public static int intakeTime = 200;
    public static int intakeTimex = 1500;

    private Thread PIDThread;

    public static double P=30;//1.8 //300
    public static double I=0;//0.18
    public static double D=0;//0
    public static double F=12.5;//13;//18 //13
    //Drive
    private MecanumDrive drive = null;
    private static int buffer  = 20;
    private static int state  = 0;
    private static boolean readytoShootFar  = false;
    private static boolean readytoShootNear  = false;
    private ElapsedTime timer = new ElapsedTime();

    VelConstraint vel =new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(30)));
    AccelConstraint accel = new ProfileAccelConstraint(-45,15);


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        intakeTimer = new ElapsedTime();

        intake = new Intake(robot);
        outtake = new Outtake(robot);
        feeder = new Feeder(robot);

        Pose2d  startPose = new Pose2d(60, 15, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, startPose);
        robot.shooter.setVelocityPIDFCoefficients(P,I,D,F);
        actualPos = robot.turretEncoder.getCurrentPosition();

        angle = getContinuousIMU(Globals.currentTurretState);
        run_turret(angle, 0, 27845, actualPos, telemetry);

        //TODO ===============================================TRAJECTORIES =============================================================


//        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.ON)),
//        new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.ON)),
//        new InstantAction(() -> new RollerCommand(intake, Intake.IntakeRollerState.ON)),

        Action trajectoryAction = drive.actionBuilder(startPose)

                .stopAndAdd(()->Actions.runBlocking(new SequentialAction(
                                new InstantAction(()-> new ShooterCommand(outtake, Outtake.ShooterState.FAR)),
                                new InstantAction(()-> new HoodCommand(outtake, Outtake.HoodState.FAR))
                        ))
                )
//                .waitSeconds(0.05)
                .stopAndAdd(()->
                        new TurretCommand(outtake, Outtake.TurretState.SHOOT))
                .waitSeconds(0.05)
                .strafeToLinearHeading(new Vector2d(47,9), Math.toRadians(90))
                .waitSeconds(0.2)
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
//
                )))
                .waitSeconds(0.2)
                .afterTime(0.01,() ->Actions.runBlocking(
                                new SequentialAction(
                                        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.OFF)),
                                        new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.OFF))
                                )
                        )
                )

                .waitSeconds(0.05)

                .build();

        Action trajectoryAction2 = drive.actionBuilder(new Pose2d(47, 9,Math.toRadians(90)))

                .strafeToConstantHeading(new Vector2d(30, 30))
                .build();

        Action trajectoryAction3 = drive.actionBuilder(new Pose2d(30, 30,Math.toRadians(90)))

//                .afterTime(0.5,()->Actions.runBlocking(new ParallelAction(
//                                new InstantAction(()-> new RollerCommand(intake,Intake.IntakeRollerState.ON)),
//                                new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.ON))
//                        )))

                .strafeToConstantHeading(new Vector2d(30, 60), vel, accel)
                .build();



        Action trajectoryAction4 = drive.actionBuilder(new Pose2d(30, 60.01,Math.toRadians(90)))

                .strafeToConstantHeading(new Vector2d(47, 9))
                .stopAndAdd(()->
                        new TurretCommand(outtake, Outtake.TurretState.SHOOT))
                .build();


        Action trajectoryAction4x = drive.actionBuilder(new Pose2d(30, 60,Math.toRadians(90)))
                .stopAndAdd(()->
                        new RollerCommand(intake,Intake.IntakeRollerState.ON))
                .strafeToConstantHeading(new Vector2d(30, 60.01), vel, accel)
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

        Action trajectoryAction5 = drive.actionBuilder(new Pose2d(47, 9,Math.toRadians(90)))

                .waitSeconds(0.2)
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
                .afterTime(0.01,() ->Actions.runBlocking(
                                new SequentialAction(
                                        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.OFF)),
                                        new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.ON))
                                )
                        )
                )

                .waitSeconds(0.05)
                .build();

        Action trajectoryAction6 = drive.actionBuilder(new Pose2d(47, 9,Math.toRadians(90)))

                .strafeToConstantHeading(new Vector2d(8, 30))
                .build();

        Action trajectoryAction7 = drive.actionBuilder(new Pose2d(8, 30,Math.toRadians(90)))

                .strafeToConstantHeading(new Vector2d(8, 60))
                .build();

        Action trajectoryAction8 = drive.actionBuilder(new Pose2d(8, 60.01,Math.toRadians(90)))

                .strafeToConstantHeading(new Vector2d(47, 9))
                .stopAndAdd(()->
                        new TurretCommand(outtake, Outtake.TurretState.SHOOT))
                .build();


        Action trajectoryAction8x = drive.actionBuilder(new Pose2d(8, 60,Math.toRadians(90)))
                .stopAndAdd(()->
                        new RollerCommand(intake,Intake.IntakeRollerState.ON))
                .strafeToConstantHeading(new Vector2d(8, 60.01))
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

        Action trajectoryAction9 = drive.actionBuilder(new Pose2d(47, 9,Math.toRadians(90)))

                .waitSeconds(0.2)
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
                .afterTime(0.01,() ->Actions.runBlocking(
                                new SequentialAction(
                                        new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.OFF)),
                                        new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.ON))
                                )
                        )
                )

                .waitSeconds(0.05)
                .build();





        PIDThread = new Thread(() -> {
            while (!Thread.currentThread().isInterrupted() && opModeIsActive()) {
                try {

                    robot.shooter.setVelocityPIDFCoefficients(P,I,D,F);
                    extendShooterUsingVelocity(Globals.curretShooterStateVelMode);
                    actualPos = robot.turretEncoder.getCurrentPosition();

                    angle = getContinuousIMU(Globals.currentTurretState);
                    run_turret(angle, 0, 27845, actualPos, telemetry);


                    if (state == 3 && intakeTimer.milliseconds()>intakeTime)
                    {
                        Globals.counterFeed_GPPtoPPG = 0;
                        state = 4;
                        intakeTimer.reset();
                    }


                    if (state == 9 && intakeTimer.milliseconds()>intakeTime)
                    {
                        Globals.counterFeed_PGPtoPPG = 0;
                        state = 10;
                        intakeTimer.reset();
                    }
                    if (state ==3 && Globals.counterFeed_GPPtoPPG ==0)
                    {
//                thirdBallFlag = false;
//                Actions.runBlocking(IntakeSeq.IntakeStoreAction(intake,outtake,feeder));

//                Globals.counterFeed_GPPtoPGP =;
                        Actions.runBlocking(
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
//                Actions.runBlocking(
//                        new ParallelAction(
//                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
//                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
//                                Intake.RollerCommand(Intake.IntakeRollerState.OFF)
//                        )
//                );
//            }

                    if (!robot.intakeBeam.getState() && Globals.counterFeed_GPPtoPPG==0 && state ==3){
                        Globals.counterFeed_GPPtoPPG += 1;
                        Actions.runBlocking(
                                new ParallelAction(
                                        Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                        Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                        Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                        Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                                        Outtake.TurretCommand(Outtake.TurretState.INIT)
                                )
                        );

                    }
                    if (robot.intakeBeam.getState() && Globals.counterFeed_GPPtoPPG ==1 && state ==3){
                        Globals.counterFeed_GPPtoPPG += 1;
                        Actions.runBlocking(
                                new ParallelAction(
                                        Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                        Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                        Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                        Feeder.LFCommand(Feeder.LowerFeederState.ON),
                                        Outtake.TurretCommand(Outtake.TurretState.INIT)
                                )
                        );
                    }
                    if (Globals.counterFeed_GPPtoPPG==2 && !robot.feederBeam.getState()&& state ==3){
                        Globals.counterFeed_GPPtoPPG += 1;
                        Actions.runBlocking(
                                new ParallelAction(
                                        Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                        Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                        Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                        Feeder.LFCommand(Feeder.LowerFeederState.ON),
                                        Outtake.TurretCommand(Outtake.TurretState.INIT)
                                )
                        );
//                zeroFlag = true;
                    }
                    if (Globals.counterFeed_GPPtoPPG==3 && robot.feederBeam.getState()&& state ==3){
                        Globals.counterFeed_GPPtoPPG += 1;
                        Actions.runBlocking(
                                new ParallelAction(
                                        Intake.IntakeCommand(Intake.IntakeServoState.IN),
                                        Intake.RollerCommand(Intake.IntakeRollerState.ON),
                                        Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                        Feeder.LFCommand(Feeder.LowerFeederState.ON),
                                        Outtake.TurretCommand(Outtake.TurretState.INIT)
                                )
                        );

                    }

                    if (Globals.counterFeed_GPPtoPPG==4 && !robot.intakeBeam.getState()&& state ==3){
                        Globals.counterFeed_GPPtoPPG += 1;
                        Actions.runBlocking(
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
                    if (Globals.counterFeed_GPPtoPPG==5 && robot.intakeBeam.getState()&& state ==3){
//                motionFlag = true;
//                motionTimer.reset();
                        Globals.counterFeed_GPPtoPPG = 0;
                        gamepad1.rumble(1,1,400);
                        Actions.runBlocking(
                                new ParallelAction(
                                        Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                        Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                                        Intake.RollerCommand(Intake.IntakeRollerState.OFF)
                                )
                        );
                        state =4;
                    }






                    if (state == 9  && Globals.counterFeed_PGPtoPPG == 0)
                    {
//                thirdBallFlag = false;
                        Actions.runBlocking(IntakeSeq.IntakeStoreAction2(intake,outtake,feeder));
                    }
//            if (!state == 3 )
//            {
//                Actions.runBlocking(
//                        new ParallelAction(
//                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
//                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
//                                Intake.RollerCommand(Intake.IntakeRollerState.OFF)
//                        )
//                );
//            }

                    if (!robot.feederBeam.getState() && Globals.counterFeed_PGPtoPPG==0 && state == 9 ){
                        Globals.counterFeed_PGPtoPPG += 1;
                        Actions.runBlocking(
                                new ParallelAction(
                                        Intake.RollerCommand(Intake.IntakeRollerState.ON)
                                )
                        );

                    }
                    if (robot.feederBeam.getState() && Globals.counterFeed_PGPtoPPG ==1 && state == 9 ){
                        Globals.counterFeed_PGPtoPPG += 1;
                        Actions.runBlocking(
                                new ParallelAction(
                                        Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                        Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                                        Intake.RollerCommand(Intake.IntakeRollerState.ON)
                                )
                        );
                    }
                    if (Globals.counterFeed_PGPtoPPG==2 && !robot.intakeBeam.getState()&& state == 9 ){
                        Globals.counterFeed_PGPtoPPG += 1;

                        Actions.runBlocking(
                                new ParallelAction(
                                        Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                        Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                                        Intake.RollerCommand(Intake.IntakeRollerState.ON)
                                )
                        );
//                zeroFlag = true;
                    }
                    if (Globals.counterFeed_PGPtoPPG==3 && robot.intakeBeam.getState()&& state == 9 ){
                        Globals.counterFeed_PGPtoPPG += 1;
                        Actions.runBlocking(
                                new ParallelAction(
                                        Feeder.LFCommand(Feeder.LowerFeederState.ON)
                                )
                        );
                    }

                    if (Globals.counterFeed_PGPtoPPG==4 && !robot.intakeBeam.getState()&& state == 9 ){
                        Globals.counterFeed_PGPtoPPG += 1;
//                zeroFlag = false;
                        Actions.runBlocking(
                                new ParallelAction(
                                        Feeder.LFCommand(Feeder.LowerFeederState.ON)
                                )
                        );

                    }
                    if (Globals.counterFeed_PGPtoPPG==5 && robot.intakeBeam.getState()&& state == 9 ){
//                motionFlag = true;
//                motionTimer.reset();
                        Globals.counterFeed_PGPtoPPG = 0;
                        gamepad1.rumble(1,1,400);
                        Actions.runBlocking(
                                new ParallelAction(
                                        Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                                        Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                                        Intake.RollerCommand(Intake.IntakeRollerState.OFF)
                                )
                        );
                        state =10;
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

                    telemetry.addData("state",state);
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

                    AutoInitSeq.InitActionFar(intake, outtake, feeder)

            );

            readytoShootFar=false;
            readytoShootNear=false;
            state=0;
            telemetry.addData("ready to shoot",readytoShootFar);
            telemetry.addData("ready to shoot",readytoShootNear);
            telemetry.addData("state",state);
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
            if (state == 0) {
                Actions.runBlocking(
                        new SequentialAction(
                                trajectoryAction
                        )
                );
                state = 1;
            }


            //TODO preload Shoot 1
            if(state == 1){
                Actions.runBlocking(
                        new SequentialAction(
                                trajectoryAction2
                        )
                );
                state = 2;
            }


            //TODO preload Shoot 3
            if (state==2) {

                Actions.runBlocking(
                        new SequentialAction(
                                new InstantAction(()-> state=3),
                                trajectoryAction3
                        )
                );
//                state = 3;
                intakeTimer.reset();

            }

            //TODO preload Shoot 2


            if (state==4) {
                Actions.runBlocking(
                        new SequentialAction(
                                trajectoryAction4x

                        )
                );
                state = 5;
            }

            if (state==5) {
                Actions.runBlocking(
                        new SequentialAction(
                                trajectoryAction4
                        )
                );

                state = 6;
            }

            if (state==6) {
                Actions.runBlocking(
                        new SequentialAction(
                                trajectoryAction5
                        )
                );

                state = 7;
            }

            if (state==7) {
                Actions.runBlocking(
                        new SequentialAction(
                                trajectoryAction6
                        )
                );

                state = 8;
            }

            if (state==8) {
                Actions.runBlocking(
                        new SequentialAction(
                                trajectoryAction7
                        )
                );

                state = 9;
                intakeTimer.reset();
            }



            if (state==10) {
                Actions.runBlocking(
                        new SequentialAction(
                                trajectoryAction8x

                        )
                );
                state = 11;
            }

            if (state==11) {
                Actions.runBlocking(
                        new SequentialAction(
                                trajectoryAction8
                        )
                );

                state = 12;
            }

            if (state==12) {
                Actions.runBlocking(
                        new SequentialAction(
                                trajectoryAction9
                        )
                );

                state = 13;
            }



//            PIDThread.interrupt();
            telemetry.addData("ready to shoot Far",readytoShootFar);
            telemetry.addData("ready to shoot Near",readytoShootNear);
            telemetry.addData("x", drive.localizer.getPose().position.x);// drive.pose.position.x);
            telemetry.addData("y", drive.localizer.getPose().position.y);
            telemetry.addData("VEL",robot.shooter.getVelocity());
            telemetry.addData("VEL",robot.shooter.getVelocity());
            telemetry.addData("State",state);
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
            error = c - target;
//            error = -error;
            integral = integral + error;
            derivative = error - previous_error;

            previous_error = error;
            pid = kp*error + kd*derivative + ki*integral;
            robot.turret1.setPower(pid);
            robot.turret2.setPower(-pid);
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
