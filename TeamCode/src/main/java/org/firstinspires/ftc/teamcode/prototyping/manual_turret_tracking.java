package org.firstinspires.ftc.teamcode.prototyping;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

//import androidx.core.math.MathUtils;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;

//@TeleOp(name = "CRServoTurret_Manual", group = "ProtoTypes")
//@Config
public class manual_turret_tracking extends LinearOpMode {

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

        while (opModeInInit()){
            drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));

        }

        waitForStart();
        while (opModeIsActive()){
            drive.updatePoseEstimate();
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            double actualPos = extEncoder.getCurrentPosition();
            if (gamepad1.a)
            {
                a = 0;
            }
            if (gamepad1.b)
            {
                a = 90;
            }
            if (gamepad1.x)
            {
                a = 180;
            }

//            a = orientation.getYaw(AngleUnit.DEGREES);
//
            target = a;
            angle = getContinuousIMU(target);
            run_turret(angle, 0, 27845, actualPos, telemetry);
            if(gamepad1.start){
                imu.resetYaw();
            }

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


    //heading follow


}

