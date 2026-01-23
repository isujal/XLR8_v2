package org.firstinspires.ftc.teamcode.prototyping;



//import static IntakeSubsystem.getSort2State;

import static org.firstinspires.ftc.teamcode.teleop.SB5.map;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystem.Feeder;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp
public class EDUCO_teleop_v2 extends LinearOpMode {
    public static double c;
    public static double target = 0;
    public static double error ;
    public static double integral;
    public static double previous_error;
    public double derivative;
    public static double pid;
    public static double servo_low = 0.79;
    public static double servo_high = 0.83;
    public double a;
    double pos;
    public static double kp = 0.02, ki = 0, kd = 0.02;

    public static double yaw = 0;
    private double totalAngle;
    private double angle;
    private double prevAngle;

    public static double tagID_tag, x_tag, y_tag, z_tag, val_tag = 0;
    // ---- AprilTag turn bias (fixes left/right asymmetry) ----
    public static double turnBias = 0.0; // units SAME as z_tag

    public static double kP_tag = 0.025;
    public static double kI_tag = 0;
    public static double kD_tag = 0.25;
    public static double Integral_tag = 0;
    public static double LastError_tag = 0;
    public static int Target_tag = 0;
    public double Power_tag = 0;

    boolean lastRightBumper = false;   // Tracks previous state
    long lastPressTime = 0;            // For time-based debounce (optional)
    final long debounceDelay = 250;
    public static boolean allThrre_tag = false;
    public static boolean April_tag = true;
    public static int countout = 0;
    public static int countin = 0;
    public int flagshoot = 0;
    public int flagin_tag = 0;
    public static double botHeading_tag ;
    MultipleTelemetry m = new MultipleTelemetry();
//    IntakeSubsystem intake;

    public static boolean bFlag_tag = false;

    public static double var = 0;
    private Intake intake;
    private Outtake outtake;
    private Feeder feeder;
    public MecanumDrive drive;
    public static RobotHardware robot = RobotHardware.getInstance();
    public static List<Action> runningActions = new ArrayList<>();
    private boolean s1Flag=false;


    AprilTagDetection tag_tag;
    @Override
    public void runOpMode() throws InterruptedException {
//        RobotHardware robot = RobotHardware.getInstance();
        robot.init(hardwareMap,telemetry);
        intake = new Intake(robot);
        outtake = new Outtake(robot);
        feeder = new Feeder(robot);
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                //all boolean
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(445.035,445.035,333.909,231.625)


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
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)

                .build();


        Thread Campid = new Thread(()->{
            while (!Thread.currentThread().isInterrupted() && opModeIsActive()){
                try {
                    if (z_tag>0.2)
                    {
                        turnBias = -7;
                    }
                    if (z_tag<-0.2)
                    {
                        turnBias = 16;
                    }
                    val_tag = Aprilpid(z_tag - turnBias);

                }
                catch (Exception e) {
                    Thread.currentThread().interrupt();
                    throw new RuntimeException(e);
                }
            }
            sleep(10);
        });



        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard.getInstance().setTelemetryTransmissionInterval(25);
        ///  Drive
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        while (opModeInInit())
        {

            drive.navxMicro.initialize();
//            drive.localizer.getPose().heading.toDouble()

//            drive.lazyImu.get().resetYaw();
        }

        Campid.start();

        waitForStart();

        while (opModeIsActive()) {


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

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            robot.hood.setPosition(pos);


            ///  dive
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

//            botHeading = drive.localizer.getPose().heading.toDouble();
//            drive.driveFieldCentric(
//                            -gamepad1.left_stick_y,
//                            gamepad1.left_stick_x,
//                    gamepad1.right_stick_x,
//                    drive.localizer.getPose().heading.toDouble());


            drive.updatePoseEstimate();
            if (gamepad1.right_trigger>0)
            {
                bFlag_tag = true;
            } else {
                bFlag_tag = false;
            }

            if (bFlag_tag)
            {
                if(April_tag == true && tagID_tag == 24){


//                    robot.turret1.setPower(-val_tag);
//                    robot.turret2.setPower(val_tag);

                    drive.leftFront.setPower(-val_tag/2);
                    drive.leftBack.setPower(-val_tag/2);
                    drive.rightBack.setPower(val_tag/2);
                    drive.rightFront.setPower(val_tag/2);
                }
//                bFlag = false;
            }

//            if (!bFlag_tag)
//            {
//
//                robot.turret1.setPower(0);
//                robot.turret2.setPower(0);
//            }
            if (gamepad1.touchpad){
                drive.lazyImu.get().resetYaw();
            }

//            if(gamepad1.b){
//
//            }

            if(gamepad1.back){

//                drive.lazyImu.get().resetYaw();
                drive=new MecanumDrive(hardwareMap,new Pose2d(0,0,0));
                drive.updatePoseEstimate();

            }


            else {
                allThrre_tag = false;
            }


            ///  updating actions
            runningActions = updateAction();

            telemetry.addData("Z", z_tag);

            telemetry.addData("currentLF", drive.leftFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("currentLB", drive.leftBack.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("currentRF", drive.rightFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("currentRB", drive.rightBack.getCurrent(CurrentUnit.AMPS));


            telemetry.addData("April", April_tag);
            telemetry.addData("tagid", tagID_tag);
            telemetry.addData("val", val_tag);
            telemetry.addData("bFlag_tag", bFlag_tag);

            telemetry.addData("X", x_tag);
            telemetry.addData("Y", y_tag);
            telemetry.addData("Z", z_tag);

//            telemetry.addData("ROLL", tag.ftcPose.roll);
//            telemetry.addData("PTICH", tag.ftcPose.pitch);
//            telemetry.addData("YAW", tag.ftcPose.yaw);
            if(allThrre_tag){
                telemetry.addLine("3 balls present");
            }
            telemetry.update();

        }
    }





    /// List action
    public static List<Action> updateAction(){
        TelemetryPacket packet = new TelemetryPacket();
        List<Action> newActions = new ArrayList<>();
        List<Action> RemovableActions = new ArrayList<>();

        for (Action action : runningActions) {
//            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        return newActions;
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
//    public double Aprilpid(double x){
//        double Error = Target_tag - x;
//        Integral_tag += Error;
//        double Derivative = Error - LastError_tag;
//        LastError_tag = Error;
//
//        Power_tag = kP_tag * Error + kI_tag * Integral_tag + kD_tag * Derivative;
//        Power_tag = Range.clip(Power_tag, -1.0, 1.0);
//        return Power_tag;
//    }

    public double Aprilpid(double x){
        double Error = Target_tag - x;
        Integral_tag += Error;
        double Derivative = Error - LastError_tag;
        LastError_tag = Error;

        Power_tag = kP_tag * Error + kI_tag * Integral_tag + kD_tag * Derivative;
        Power_tag = Range.clip(Power_tag, -1.0, 1.0);
        return Power_tag;
    }
}
