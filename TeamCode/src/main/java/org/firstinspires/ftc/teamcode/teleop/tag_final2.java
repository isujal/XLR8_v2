package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

//@TeleOp
//@Config
public class tag_final2 extends LinearOpMode {

    private Limelight3A limelight;
    public static double tagID, x, y, z;
    public static boolean April = true;

    IMU imu;
    double pos;
    private static CRServo crServo1, crServo2;
    private DcMotorEx extEncoder;

    public static int ENCODER_TICKS_PER_REV = 8192;

    public static double kp = 0.05, ki = 0, kd = 0.2;
    public static String WEBCAM_NAME = "Webcam 1";
    public static double corr = 0, yaw = 0;
    private double totalAngle;
    private double angle;
    public static double offset = 0;
    private double prevAngle;
    //
//    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
//    YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, -90, 0, -90, 0);
    private double tagYawDeg = 0;
    public static double targetX = 66;
    public static double targetY = -66;

    private PIDFController headingController;

    AprilTagDetection tag;


    @Override
    public void runOpMode() throws InterruptedException {
        crServo1 = hardwareMap.get(CRServo.class, "turret2");
        crServo2 = hardwareMap.get(CRServo.class, "turret1");
        extEncoder = hardwareMap.get(DcMotorEx.class, "upperFeeder");

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        imu = hardwareMap.get(IMU.class, "imu");
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                //all boolean
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // Webcam 1 name is always constant
                .setCameraResolution(new Size(640, 480))
                .build();



        double xRotation = 0, yRotation = 0, zRotation = 0;
        Orientation hubRotation = xyzOrientation(xRotation, yRotation, zRotation);
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(hubRotation);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();


        while (opModeInInit()) {
            imu.resetYaw();
            telemetry.addData("Init Angle", angle);
            telemetry.update();
        }
        waitForStart();


        while (opModeIsActive()) {


            if (tagProcessor.getDetections().size() > 0) {
                tag = tagProcessor.getDetections().get(0); // to detect first tag unless other present

                tagID = tag.id;
                x = tag.ftcPose.x;
                y = tag.ftcPose.y;
                z = tag.ftcPose.z;


                April = true;
                telemetry.addData(" ID ",tag.id);
            }
            else {
                April = false;
            }


            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                    -gamepad1.right_stick_x
            ));



            drive.updatePoseEstimate();

            telemetry.addData("X", x);
            telemetry.addData("Y", y);
            telemetry.addData("Z", z);
            telemetry.addData("ROLL", tag.ftcPose.roll);
            telemetry.addData("PTICH", tag.ftcPose.pitch);
            telemetry.addData("YAW", tag.ftcPose.yaw);
            telemetry.update();

        }


    }
}
