package org.firstinspires.ftc.teamcode.scrap_autos;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;


import java.util.Arrays;

@Config
//@Autonomous(name="TEST AUTO")
//@Deprecated
public class scrap1 extends LinearOpMode {

    VelConstraint vel =new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(30)));

//    AccelConstraint baseAccelConstraint3 = new ProfileAccelConstraint(-45, 30);
//    VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(30), new AngularVelConstraint(Math.PI / 2)));


    AccelConstraint accel = new ProfileAccelConstraint(-45,20);

    //Drive
    private MecanumDrive drive = null;
    VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(30.0),
            new AngularVelConstraint(Math.PI / 4)
    ));

    VelConstraint baseVelConstraint50 = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(50.0),
            new AngularVelConstraint(Math.PI / 3)
    ));



    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive = new MecanumDrive(hardwareMap, startPose);

        //TODO ===============================================TRAJECTORIES =============================================================


        TrajectoryActionBuilder trajectoryActionNear = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                //TODO : Preload Shoot
                .strafeToLinearHeading(new Vector2d(24, 0), Math.toRadians(0))
                .waitSeconds(1)

//                .strafeToConstantHeading(new Vector2d(-14, 16))
                //TODO : Pick First Motif from loading zone
                .strafeToLinearHeading(new Vector2d(48, 0), Math.toRadians(0), vel,accel);
//                .splineToConstantHeading(new Vector2d(-11.5, 42.5), Math.toRadians(70))
//                .strafeToLinearHeading(new Vector2d(11, -25), Math.toRadians(52))
//                //TODO : First Motif Shoot
//                .strafeToLinearHeading(new Vector2d(-8, -51), Math.toRadians(52))
////                .waitSeconds(0.25)
////                //TODO : Pick from 2nd SpikeMark and Shoot
//                .strafeToLinearHeading(new Vector2d(15, -58), Math.toRadians(52))
//                .strafeToLinearHeading(new Vector2d(35, -35), Math.toRadians(52))
//                .strafeToLinearHeading(new Vector2d(-8, -51), Math.toRadians(52))
////                .waitSeconds(0.25)
////                .strafeToLinearHeading(new Vector2d(11.5, 42.5), Math.toRadians(180))
////                //TODO : Second Motif Shoot
////                .strafeToLinearHeading(new Vector2d(-20, 32), Math.toRadians(180))
//                .waitSeconds(0.25);

        TrajectoryActionBuilder trajectoryActionFar = drive.actionBuilder(new Pose2d(60, 15, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(36, 30))  //4.34S
//                .splineToConstantHeading(new Vector2d(36, 32), Math.toRadians(60))
                .strafeToConstantHeading(new Vector2d(36, 42))  //4.34S
                //TODO : Shoot Second Motif
                .strafeToConstantHeading(new Vector2d(50, 13))
//                .strafeToLinearHeading(new Vector2d(50, 13), Math.toRadians(90))  //4.34S
                .waitSeconds(0.25)
                //TODO : Pick from Second Spike Mark
                .strafeToConstantHeading(new Vector2d(12, 30))
                .strafeToConstantHeading(new Vector2d(12, 42))

//                .splineToConstantHeading(new Vector2d(12, 42), Math.toRadians(100))
                .waitSeconds(0.25)
                //TODO : Gate Open
//                .splineToConstantHeading(new Vector2d(7, 55), Math.toRadians(180))
                //TODO : Shoot Third Motif
//                .strafeToConstantHeading(new Vector2d(-5, 12))  //4.34S
                .strafeToLinearHeading(new Vector2d(50, 13), Math.toRadians(90));  //4.34S

        if (opModeInInit()) {
            telemetry.addLine("ROBOT INIT MODE");

        }


        waitForStart();
        //pidThread.start();

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionNear.build()
                ));
        while (opModeIsActive()) {
        }


    }
}
