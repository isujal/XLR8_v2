package org.firstinspires.ftc.teamcode.prototyping;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;

//@TeleOp(name = "CRServoTurret_Manual2", group = "ProtoTypes")
//@Config
public class manual_turret_tracking2 extends LinearOpMode {

    private Limelight3A limelight;
    IMU imu;
    private static CRServo crServo1, crServo2;
    public double b;
    public static double c;
//    private CRServo crServo2;
//    private AnalogInput axonFeedback;
    private DcMotorEx extEncoder;
    public static double target = 0;
    public static double error ;
    public static double integral;
    public static double previous_error;
    public double derivative;
    public static double pid;
    public double a;

    public static double kp = 0.02, ki = 0, kd = 0.02;

    public static double corr = 0, yaw = 0;
    private double totalAngle;
    private double angle;
    private double prevAngle;
    private MecanumDrive drive;


    // Configurable PIDF constants (tuned from dashboard)
    public static double kP = 0.03;
    public static double kI = 0.0;
    public static double kD = 0.001;
    public static double kF = 0.0;

    // Target location and heading
    public static double targetX = -6;
    public static double targetY = 12;
    public static double targetHeading = Math.toRadians(90);

//
    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

    // FTC PIDF controller
    private PIDFController headingController;

    private boolean BFlag=false;


    @Override
    public void runOpMode() throws InterruptedException {
        crServo1 = hardwareMap.get(CRServo.class, "turret1");
        crServo2 = hardwareMap.get(CRServo.class, "turret2");
//        crServo2 = hardwareMap.get(CRServo.class, "crservo2");
        extEncoder = hardwareMap.get(DcMotorEx.class, "upperFeeder");
        imu = hardwareMap.get(IMU.class, "imu");
        drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));
//        extEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        extEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extEncoder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        double xRotation = 0;
        double yRotation = 0;
        double zRotation = 0;


        Orientation hubRotation = xyzOrientation(xRotation, yRotation, zRotation);

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(hubRotation);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        integral = 0;
        error = 0;
        previous_error = 0;
        imu.resetYaw();

        extEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extEncoder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        angle = 0;

        //todo heading follow
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Create PIDF controller for heading
        headingController = new PIDFController(kP, kI, kD, kF);
        while (opModeInInit()){
            drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));

        }

        waitForStart();
        while (opModeIsActive()){

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            double actualPos = extEncoder.getCurrentPosition();
            double correction = Math.toDegrees(computeHeadingCorrection(drive.localizer.getPose()));

            a = orientation.getYaw(AngleUnit.DEGREES);
//            a=correction;

            target = a;
            angle = getContinuousIMU(a);
            run_turret(angle + correction, 0, 27845, actualPos, telemetry);
            if(gamepad1.start){
                imu.resetYaw();
            }

            //todo heading follow
            // Compute correction

            // Gamepad translational input


            // Drive robot
//                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(driveX, driveY), correction));

            Vector2d currentPos = drive.localizer.getPose().position;
            double currentHeading = drive.localizer.getPose().heading.toDouble();
            double angleToTarget=Math.atan2(targetY - currentPos.y, targetX - currentPos.x);

            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(Math.pow(Range.clip(-gamepad1.left_stick_y,-1,1),3),
                                    Math.pow(Range.clip(-gamepad1.left_stick_x,-1,1),3)),
                            Math.pow(Range.clip(-gamepad1.right_stick_x,-1,1),3))
            );

            drive.updatePoseEstimate();

            telemetry.addData("Current Heading (deg)", Math.toDegrees(currentHeading));
            telemetry.addData("Correction", correction);
            telemetry.addData("Target Heading (deg)", Math.toDegrees(targetHeading));
            telemetry.addData("Current Pos", currentPos);
            telemetry.addData("Error ", Math.toDegrees(headingController.getPositionError()));
            telemetry.addData("error (C-AT)",angleWrap(currentHeading-angleToTarget));
            telemetry.addData("Angle To Target", Math.toDegrees(Math.atan2(targetY - currentPos.y, targetX - currentPos.x)));

            telemetry.addData("Actual Pos", actualPos);
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
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
            telemetry.update();
        }
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
            crServo1.setPower(-pid);
            crServo2.setPower(-pid);

        }
        else {
            telemetry.addLine("positive heading");

            error = target - c;
//            error = -error;
            integral = integral + error;
            derivative = error - previous_error;

            previous_error = error;
            pid = kp*error + kd*derivative + ki*integral;
            crServo1.setPower(-pid);
            crServo2.setPower(-pid);
        }
    }

    public static double map(double x, double inMin, double inMax, double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    private double computeHeadingCorrection(Pose2d pose) {
        Vector2d currentPos = pose.position;
        double currentHeading = pose.heading.toDouble();

        // Desired angle
        double angleToTarget = Math.atan2(targetY - currentPos.y, targetX - currentPos.x);

        // Wrap error to [-π, π]
        double error = angleWrap(angleToTarget - currentHeading);

        // Feed error into PIDF controller
//        return headingController.calculate(0, error);
        return error;
    }

    // Keep angle in range [-π, π]
    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }


}

