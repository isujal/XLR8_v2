package org.firstinspires.ftc.teamcode.autos;


import static org.firstinspires.ftc.teamcode.subsystem.Outtake.extendShooterUsingVelocity;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.instantCommands.LFCommand;
import org.firstinspires.ftc.teamcode.instantCommands.RollerCommand;
import org.firstinspires.ftc.teamcode.instantCommands.ShooterCommand;
import org.firstinspires.ftc.teamcode.instantCommands.TurretCommand;
import org.firstinspires.ftc.teamcode.instantCommands.UFCommand;
import org.firstinspires.ftc.teamcode.sequences.AutoInitSeq;
import org.firstinspires.ftc.teamcode.sequences.InitSeq;
import org.firstinspires.ftc.teamcode.subsystem.Feeder;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

@Config
//@Autonomous(name="SB AUTO 🔴")
@Deprecated
public class SB_AUTO extends LinearOpMode {
    private RobotHardware robot = RobotHardware.getInstance();
    //Subsystems
    Outtake outtake ;

    Intake intake ;
    Feeder feeder ;
    double actualPos;
    public static double ramptime = 0.5;
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


    private Thread PIDThread;


    public static double P=30;//1.8 //300
    public static double I=0;//0.18
    public static double D=0;//0
    public static double F=13;//13;//18 //13
    //Drive
    private MecanumDrive drive = null;
    private static int buffer  = 20;
    private static int state  = 0;
    private static boolean readytoShootFar  = false;
    private static boolean readytoShootNear  = false;
    private ElapsedTime timer = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);

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

        Action trajectoryAction = drive.actionBuilder(startPose)

                .stopAndAdd(()->
                        new ShooterCommand(outtake, Outtake.ShooterState.FAR))
                .waitSeconds(0.5)
                .stopAndAdd(()->
                        new TurretCommand(outtake, Outtake.TurretState.SHOOT))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(47,9), Math.toRadians(90))
                .build();

//        Action trajectoryAction1 = drive.actionBuilder(new Pose2d(60, 14, Math.toRadians(90)))
//                .waitSeconds(1)
//                .afterTime(0.5,() -> {
//                            Actions.runBlocking(
//                                    new SequentialAction(
//                                            new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.ON)),
//                                            new SleepAction(1000),
//                                            new InstantAction(() -> state = 2),
//                    new InstantAction(()-> new UFCommand(feeder,Feeder.UpperFeederState.OFF)),
//                                            new SleepAction(1000),
//                                            new InstantAction(()-> new LFCommand(feeder,Feeder.LowerFeederState.ON)),
//                                            new InstantAction(() -> new RollerCommand(intake, Intake.IntakeRollerState.ON)),
//                                            new InstantAction(() -> new RollerCommand(intake, Intake.IntakeRollerState.ON)),
//                                            new InstantAction(() -> state = 5)
//                                    )
//                            );
//                        }
//                )
//                .strafeToConstantHeading(new Vector2d(36, 30))  //4.34S
//
//                .build();
//        //TODO Slide Into Spike Marks
//
//        Action trajectoryAction2 = drive.actionBuilder(new Pose2d(36, 37,Math.toRadians(90)))
//
//                .strafeToLinearHeading(new Vector2d(36,48), Math.toRadians(90))
//                .build();
//
//        //TODO Shoot Artifacts Pose
//        Action trajectoryAction3 = drive.actionBuilder(new Pose2d(36, 48,Math.toRadians(90)))
//
//                .stopAndAdd(()->
//                        new ShooterCommand(outtake, Outtake.ShooterState.FAR))
//                .strafeToLinearHeading(new Vector2d(59,14), Math.toRadians(158))
//                .build();
//
//        Action trajectoryAction4 = drive.actionBuilder(new Pose2d(59, 14, Math.toRadians(158)))
//                .waitSeconds(1)
//                .afterTime(0.5,() -> {
//                            Actions.runBlocking(
//                                    new SequentialAction(
//                                            //TODO For low shooter speed during trajectory
////                                           new InstantAction(()-> new ShooterCommand(shooterSubsystem, ShooterSubsystem.ShooterState.NEAR)),
//
////                                            new InstantAction(() -> new FrontIntakeCommand(frontIntakeSubsystem, FrontIntakeSubsystem.FrontIntakeState.IN)),
////                                            new InstantAction(() -> new BackIntakeCommand(backIntakeSubsystem, BackIntakeSubsystem.BackIntakeState.OUT)),
////                                            new InstantAction(() -> new FrontFeedCommand(frontIntakeSubsystem, FrontIntakeSubsystem.FrontFeedState.PASS)),
////                                            new InstantAction(() -> new BackFeedCommand(backIntakeSubsystem, BackIntakeSubsystem.BackFeedState.PASS))
////                                            new InstantAction(() -> state = 5)
//                                    )
//
//                            );
//                        }
//                )
//                .splineToLinearHeading(new Pose2d(12, 37,Math.toRadians(90)), Math.toRadians(90))
//                .build();
//        //TODO Slide Into Spike Marks
//
//        Action trajectoryAction5 = drive.actionBuilder(new Pose2d(12, 37,Math.toRadians(90)))
//
//                .strafeToLinearHeading(new Vector2d(12,48), Math.toRadians(90))
//                .build();
//
//        //TODO Shoot Artifacts Pose
//        Action trajectoryAction6 = drive.actionBuilder(new Pose2d(12, 48,Math.toRadians(90)))
//
//                .stopAndAdd(()->
//                        new ShooterCommand(shooterSubsystem, ShooterSubsystem.ShooterState.NEAR))
//                .strafeToLinearHeading(new Vector2d(-10 ,16), Math.toRadians(136))
//                .build();
//
//        Action trajectoryAction7 = drive.actionBuilder(new Pose2d(-10, 16, Math.toRadians(135)))
//                .waitSeconds(1)
////                .afterDisp(0.5,)
//                .afterTime(0.5,() -> {
//                            Actions.runBlocking(
//                                    new SequentialAction(
//                                            //TODO For low shooter speed during trajectory
////                                           new InstantAction(()-> new ShooterCommand(shooterSubsystem, ShooterSubsystem.ShooterState.NEAR)),
//
//                                            new InstantAction(() -> new FrontIntakeCommand(frontIntakeSubsystem, FrontIntakeSubsystem.FrontIntakeState.IN)),
//                                            new InstantAction(() -> new BackIntakeCommand(backIntakeSubsystem, BackIntakeSubsystem.BackIntakeState.OUT)),
//                                            new InstantAction(() -> new FrontFeedCommand(frontIntakeSubsystem, FrontIntakeSubsystem.FrontFeedState.PASS)),
//                                            new InstantAction(() -> new BackFeedCommand(backIntakeSubsystem, BackIntakeSubsystem.BackFeedState.PASS))
////                                            new InstantAction(() -> state = 5)
//                                    )
//
//                            );
//                        }
//                )
//                .splineToLinearHeading(new Pose2d(-12, 37,Math.toRadians(90)), Math.toRadians(90))
//                .build();
//
//        //TODO Slide Into Spike Marks
//
//        Action trajectoryAction8 = drive.actionBuilder(new Pose2d(-12, 37,Math.toRadians(90)))
//
//                .strafeToLinearHeading(new Vector2d(-12,48), Math.toRadians(90))
//                .build();
//
//        //TODO Shoot Artifacts Pose
//        Action trajectoryAction9 = drive.actionBuilder(new Pose2d(-12, 48,Math.toRadians(90)))
//
//                .stopAndAdd(()->
//                        new ShooterCommand(outtake, Outtake.ShooterState.FAR))
////                .strafeToLinearHeading(new Vector2d(59,14), Math.toRadians(158))
//                .strafeToLinearHeading(new Vector2d(-10 ,16), Math.toRadians(136 ))
//
//                .build();


        PIDThread = new Thread(() -> {
            while (!Thread.currentThread().isInterrupted() && opModeIsActive()) {
                try {

                        robot.shooter.setVelocityPIDFCoefficients(P,I,D,F);
                        extendShooterUsingVelocity(Globals.curretShooterStateVelMode);
                        actualPos = robot.turretEncoder.getCurrentPosition();

                        angle = getContinuousIMU(Globals.currentTurretState);
                        run_turret(angle, 0, 27845, actualPos, telemetry);
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
            Globals.shooterMode=true;
            actualPos = robot.turretEncoder.getCurrentPosition();

            angle = getContinuousIMU(Globals.currentTurretState);
            run_turret(angle, 0, 27845, actualPos, telemetry);
            telemetry.addLine("ROBOT INIT MODE");
            Actions.runBlocking(

                    AutoInitSeq.InitAction(intake, outtake, feeder)

            );

            readytoShootFar=false;
            readytoShootNear=false;
            state=0;
            telemetry.addData("ready to shoot",readytoShootFar);
            telemetry.addData("ready to shoot",readytoShootNear);
            telemetry.addData("state",state);
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
                if(readytoShootFar) {
                    Actions.runBlocking(
                            new SequentialAction(
                                    new InstantAction(() -> new TurretCommand(outtake, Outtake.TurretState.SHOOT)),
                                    new SleepAction(0.5),
                                            new InstantAction(()->  new UFCommand(feeder, Feeder.UpperFeederState.ON)),
                                            new InstantAction(()->  new LFCommand(feeder, Feeder.LowerFeederState.OFF)),

                                    new InstantAction(() -> state = 2)
                            )

                    );
                }

                else if (!readytoShootFar){
                    Actions.runBlocking(new SequentialAction(
                            new InstantAction(()->  new ShooterCommand(outtake, Outtake.ShooterState.FAR))));
                }
            }

            //TODO preload Shoot 2
            if (state==2){
//                if (!robot.bShoot){
                if (robot.outtakeBeam.getState() && readytoShootFar){
                    Actions.runBlocking(
                            new SequentialAction(
                                    new InstantAction(() -> new TurretCommand(outtake, Outtake.TurretState.SHOOT)),
                                    new SleepAction(0.5),
                                    new InstantAction(() -> new UFCommand(feeder, Feeder.UpperFeederState.OFF)),
                                    new InstantAction(() -> new LFCommand(feeder, Feeder.LowerFeederState.ON))
                            ));
//                    state = 3;
                }
                else if (!robot.outtakeBeam.getState() && readytoShootFar){
                    Actions.runBlocking(
                            new SequentialAction(
                                    new InstantAction(() -> new TurretCommand(outtake, Outtake.TurretState.SHOOT)),
                                    new SleepAction(1),
                                    new InstantAction(() -> new UFCommand(feeder, Feeder.UpperFeederState.ON)),
                                    new InstantAction(() -> state = 3)
//                                    new InstantAction(() -> new LFCommand(feeder, Feeder.LowerFeederState.ON))
                            ));
//                    state = 3;
                }

//                else if (robot.bShoot) {
                else if (robot.outtakeBeam.getState() && !readytoShootFar) {
                    Actions.runBlocking(new SequentialAction(
                            new InstantAction(() -> new UFCommand(feeder, Feeder.UpperFeederState.OFF)),
                            new InstantAction(() -> new LFCommand(feeder, Feeder.LowerFeederState.OFF)),
                    new InstantAction(()->  new ShooterCommand(outtake, Outtake.ShooterState.FAR))));
                }
            }

            //TODO preload Shoot 3
            if (state==3) {
                if (robot.outtakeBeam.getState() && robot.intakeBeam.getState() && readytoShootFar) {
                    Actions.runBlocking(
                            new SequentialAction(
                                    new InstantAction(() -> new UFCommand(feeder, Feeder.UpperFeederState.OFF)),
                                    new ParallelAction(
                                            new InstantAction(() -> new RollerCommand(intake, Intake.IntakeRollerState.RELEASE)),
                                            new InstantAction(() -> new LFCommand(feeder, Feeder.LowerFeederState.ON))
                                    )

                                    )

                    );


                }
                else if (!robot.intakeBeam.getState() && readytoShootFar){
                    Actions.runBlocking(
                            new SequentialAction(
                                    new ParallelAction(
                                            new InstantAction(() -> new RollerCommand(intake, Intake.IntakeRollerState.ON)),
                                            new InstantAction(() -> new LFCommand(feeder, Feeder.LowerFeederState.ON))
                                    ),

//                                    new InstantAction(() -> new UFCommand(feeder, Feeder.UpperFeederState.OFF)),
                                    new InstantAction(() -> state = 4)

                            )

                    );

//                    state = 4;

                }
                else if (!readytoShootFar){
                    Actions.runBlocking(new SequentialAction(
                            new InstantAction(()->  new ShooterCommand(outtake, Outtake.ShooterState.FAR))));
                }
            }

            if (state==4) {
                if (robot.outtakeBeam.getState() && readytoShootFar) {
                    Actions.runBlocking(
                            new SequentialAction(

                                    new InstantAction(() -> new TurretCommand(outtake, Outtake.TurretState.SHOOT)),
                                    new SleepAction(1),
                                    new InstantAction(() -> new LFCommand(feeder, Feeder.LowerFeederState.ON))
//                                    new InstantAction(() -> new UFCommand(feeder, Feeder.UpperFeederState.ON))

                            )

                    );


                }
                else if (!robot.outtakeBeam.getState() && readytoShootFar)
                {
                    Actions.runBlocking(
                            new SequentialAction(

                                    new InstantAction(() -> new TurretCommand(outtake, Outtake.TurretState.SHOOT)),
                                    new SleepAction(1),
                                    new InstantAction(() -> new LFCommand(feeder, Feeder.LowerFeederState.OFF)),
                                    new InstantAction(() -> new UFCommand(feeder, Feeder.UpperFeederState.OFF)),
                                    new SleepAction(1),
                                    new InstantAction(() -> new UFCommand(feeder, Feeder.UpperFeederState.ON)),
                                    new InstantAction(() -> state = 5)
                            )

                    );

//                    state = 5;
                }
                else if (!readytoShootFar){
                    Actions.runBlocking(new SequentialAction(
                            new InstantAction(()->  new ShooterCommand(outtake, Outtake.ShooterState.FAR)),
                            new InstantAction(() -> new LFCommand(feeder, Feeder.LowerFeederState.OFF)),
                            new InstantAction(() -> new UFCommand(feeder, Feeder.UpperFeederState.OFF))
                            )
                    );
                }
            }


//            PIDThread.interrupt();
            telemetry.addData("ready to shoot Far",readytoShootFar);
            telemetry.addData("ready to shoot Near",readytoShootNear);
            telemetry.addData("x", drive.localizer.getPose().position.x);// drive.pose.position.x);
            telemetry.addData("y", drive.localizer.getPose().position.y);
            telemetry.addData("VEL",robot.shooter.getVelocity());
            telemetry.addData("VEL",robot.shooter.getVelocity());
            telemetry.addData("State",state);
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
