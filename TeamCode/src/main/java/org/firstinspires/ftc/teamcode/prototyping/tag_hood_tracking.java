package org.firstinspires.ftc.teamcode.prototyping;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

//import androidx.core.math.MathUtils;


import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

//@TeleOp
//@Config
public class tag_hood_tracking extends LinearOpMode {

    IMU imu;
    ServoImplEx hood;
    double pos;
    public static double hood_min = 15;
    public static double hood_max = 70;
    public double b;
    public static double c;
    public static double DESIRED_ANGLE = 0;
    public static int ENCODER_TICKS_PER_REV = 8192;

    public static double kp = 0.05, ki = 0, kd = 0.2;
    public static String WEBCAM_NAME = "Webcam 1";
    public static double corr = 0, yaw = 0;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

//    private double integral = 0;
//    private double lastError = 0;
//    private double fieldTargetHeading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
//        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        hood = hardwareMap.get(ServoImplEx.class, "hood");
        imu = hardwareMap.get(IMU.class, "imu");
        initAprilTag();
        double xRotation = 0;
        double yRotation = 0;
        double zRotation = 0;

        Orientation hubRotation = xyzOrientation(xRotation, yRotation, zRotation);

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(hubRotation);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        while (opModeInInit()){
            imu.resetYaw();
            telemetry.update();
        }
        DESIRED_ANGLE = 0;
        waitForStart();
        while (opModeIsActive()){

//            Pose2d pose = drive.localizer.getPose();
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            List<AprilTagDetection> detections = aprilTag.getDetections();
            boolean tagVisible = !detections.isEmpty();
            if (tagVisible) {
                telemetry.addData("Yaw ", "%.2f",detections.get(0).ftcPose.yaw );
                telemetry.addData("pitch ", "%.2f",detections.get(0).ftcPose.pitch );
                telemetry.addData("roll ", "%.2f",detections.get(0).ftcPose.roll );
                telemetry.addData("bearing ", "%.2f",detections.get(0).ftcPose.bearing );
                telemetry.addData("elevation ", "%.2f",detections.get(0).ftcPose.elevation );
                telemetry.addData("range ", "%.2f",detections.get(0).ftcPose.range );
                telemetry.addData("x ", "%.2f",detections.get(0).ftcPose.x );
                telemetry.addData("y ", "%.2f",detections.get(0).ftcPose.y );
                telemetry.addData("z ", "%.2f",detections.get(0).ftcPose.z );
                telemetry.addData("robot yaw ", "%.2f", detections.get(0).robotPose.getOrientation().getYaw());
                telemetry.addData("robot x ", "%.2f", detections.get(0).robotPose.getPosition().x);
                telemetry.addData("robot y ", "%.2f", detections.get(0).robotPose.getPosition().y);
                double tagYawDeg = detections.get(0).ftcPose.yaw;
                DESIRED_ANGLE = detections.get(0).robotPose.getOrientation().getYaw();
                pos = map (detections.get(0).ftcPose.range, hood_min,hood_max,0,1);
            }


            hood.setPosition(pos);

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
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", c);
            telemetry.addData("CRServo Power", b);
            telemetry.addData("kp", kp);
            telemetry.addData("kd", kd);
            telemetry.addData("ki", ki);
            telemetry.update();
        }
        visionPortal.close();
    }

//    public double  getContinuousIMU(double currentAngle) {
//        double delta = currentAngle - prevAngle;
//
//        // Handle wrap-around
//        if (delta > 180) {
//            delta -= 360;
//        } else if (delta < -180) {
//            delta += 360;
//        }
//
//        totalAngle += delta;
//        prevAngle = currentAngle;
//
//        return totalAngle;
//    }
//
//    public static void run_turret(double imu, double min_in_pos, double max_in_pos, double pose, Telemetry telemetry){
//        pid = 0;
//
//        c = map(pose, min_in_pos, max_in_pos, 0, 360);
//
//        if (imu < 0){
//            c = -c;
//        }
//
//
//        double target = imu;
//
//        double derivative = 0;
//        if(imu < 0){
//            telemetry.addLine("neagtive heading");
//            error = c + target;
////            error = -error;
//            integral = integral + error;
//            derivative = error - previous_error;
//
//            previous_error = error;
//            pid = kp*error + kd*derivative + ki*integral;
//            crServo1.setPower(-pid);
//            crServo2.setPower(-pid);
//
//        }
//        else {
//            telemetry.addLine("positive heading");
//
//            error = target - c;
////            error = -error;
//            integral = integral + error;
//            derivative = error - previous_error;
//
//            previous_error = error;
//            pid = kp*error + kd*derivative + ki*integral;
//            crServo1.setPower(-pid);
//            crServo2.setPower(-pid);
//        }
//    }

    public static double map(double x, double inMin, double inMax, double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawTagOutline(false)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
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

