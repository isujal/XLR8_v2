package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

//import androidx.core.math.MathUtils;


import android.renderscript.Matrix4f;
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
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.ejml.data.Matrix;
import org.ejml.dense.row.MatrixFeatures_CDRM;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

//@TeleOp
@Config
public class tag_final3 extends LinearOpMode {

    private static RobotHardware robot=RobotHardware.getInstance();


    private Limelight3A limelight;
    IMU imu;
    Encoder enc;
//    ServoImplEx hood;
    double pos;
    double total_angle;
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

    public static double kp = 0.03, ki = 0, kd = 0;
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
    @Override
    public void runOpMode() throws InterruptedException {

        RobotHardware robot = RobotHardware.getInstance();
        robot.init(hardwareMap,telemetry);
//        crServo1 = hardwareMap.get(CRServo.class, "turret2");
//        crServo2 = hardwareMap.get(CRServo.class, "turret1");
//        crServo2 = hardwareMap.get(CRServo.class, "crservo2");
//        extEncoder = hardwareMap.get(DcMotorEx.class, "upperFeeder");
//        hood = hardwareMap.get(ServoImplEx.class, "hood");
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
//        enc = hardwareMap.get(Encoder.class, "enc");
        imu = hardwareMap.get(IMU.class, "imu");
        initAprilTag();
//        robot.turret1.setDirection(DcMotorSimple.Direction.REVERSE);
//        robot.turret2.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.turretEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.resetEncoder();
//        extEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        extEncoder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        double xRotation = 0;
        double zRotation = 0;
        double yRotation = 0;

        Orientation hubRotation = xyzOrientation(xRotation, yRotation, zRotation);

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(hubRotation);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        integral = 0;
        error = 0;
        previous_error = 0;
        while (opModeInInit()){
            imu.resetYaw();
            robot.resetEncoder();
//            extEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//            extEncoder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            telemetry.addData("angle ", angle);
            telemetry.addData("I",integral);
            telemetry.addData("D",derivative);
            telemetry.update();
        }
        DESIRED_ANGLE = 0;
        waitForStart();
        while (opModeIsActive()){

            if     (gamepad1.b)
            {
                preset = 60;
            }

            else if     (gamepad1.x)
            {
                preset = -60;
            }
            if (gamepad1.y){
                total_angle = -Math.toDegrees(drive.localizer.getPose().heading.toDouble());
            }
            else
            {
                total_angle = angle + offset +DESIRED_ANGLE + preset;

            }

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();

//            Pose2d pose = drive.localizer.getPose();
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


            if (total_angle > 140)
            {
                total_angle = 140;
            }
            if (total_angle < -120)
            {
                total_angle = -120;
            }

//            DESIRED_ANGLE = Math.toDegrees(Math.atan2(targetY - currentPos.y, targetX - currentPos.x));
            run_turret(total_angle, 0, 27845, actualPos, telemetry);

            target = Math.toDegrees(drive.localizer.getPose().heading.toDouble()) ;
            angle = getContinuousIMU(target);
            a = orientation.getYaw(AngleUnit.DEGREES);
//            hood.setPosition(pos);

//            target = a;

            telemetry.addData("total angle", total_angle);
            telemetry.addData("desired angle", DESIRED_ANGLE);
            telemetry.addData("angle", angle);
            telemetry.addData("angle", offset);
            telemetry.addData("angle", preset);
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
            telemetry.addData("Actual Pos", actualPos);
//            telemetry.addData("Actual Pos", actualPos);
//            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
//            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", drive.localizer.getPose().heading.toDouble());
//            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", c);
            telemetry.addData("CRServo Power", b);
            telemetry.addData("kp", kp);
            telemetry.addData("kd", kd);
            telemetry.addData("ki", ki);
            telemetry.addData("E", error);
            telemetry.addData("angle ", angle);
            telemetry.addData("I",integral);
            telemetry.addData("D",derivative);
            telemetry.addData("target", target);
            telemetry.addData("output power ", pid);
            telemetry.addData(" angle degree ", Math.toDegrees(Math.atan2(targetY - currentPos.y, targetX - currentPos.x)));
            telemetry.addData(" angle degree ", DESIRED_ANGLE);
            telemetry.update();
        }
        visionPortal.close();
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
