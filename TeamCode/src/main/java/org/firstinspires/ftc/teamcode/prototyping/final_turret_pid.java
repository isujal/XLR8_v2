package org.firstinspires.ftc.teamcode.prototyping;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

import java.util.List;

//@TeleOp(name = "final_turret_pid", group = "ProtoTypes")
//@Config
public class final_turret_pid extends LinearOpMode {

    public static String SERVO_NAME = "crservo";
    public static String SERVO_NAME2 = "crservo2";
    public static String ENCODER_NAME = "extEncoder";
    public static String WEBCAM_NAME = "Webcam 1";

    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0;
    public static double kFF = 0;

    public static int ENCODER_TICKS_PER_REV = 8192;

    public static double maxIntegral = 1e6;
    public static double maxPower = 0.8;

    private CRServo turretServo;
    private CRServo turretServo2;
    private DcMotorEx turretEncoder;
    private IMU imu;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private double integral = 0;
    private double lastError = 0;
    private double fieldTargetHeading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        turretServo = hardwareMap.get(CRServo.class, SERVO_NAME);
        turretServo2 = hardwareMap.get(CRServo.class, SERVO_NAME2);
        turretEncoder = hardwareMap.get(DcMotorEx.class, ENCODER_NAME);
        imu = hardwareMap.get(IMU.class, "imu");

        turretEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        initAprilTag();

        telemetry.addLine("Ready. Press start.");
        telemetry.update();
        waitForStart();

        double robotHeading = normalizeDeg(getRobotHeading());
        double currentTicks = turretEncoder.getCurrentPosition();
        double currentAngleDeg = ticksToAngle(currentTicks);
        fieldTargetHeading = normalizeDeg(robotHeading + currentAngleDeg);

        while (opModeIsActive()) {
            robotHeading = normalizeDeg(getRobotHeading());
            AngularVelocity yawRate = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            double chassisYawRateDegPerSec = yawRate != null ? yawRate.zRotationRate : 0.0;

            List<AprilTagDetection> detections = aprilTag.getDetections();
            boolean tagVisible = !detections.isEmpty();
            if (tagVisible) {
                telemetry.addData("Yaw", "%.2f",detections.get(0).ftcPose.yaw );
                double tagYawDeg = detections.get(0).ftcPose.yaw;
                fieldTargetHeading = normalizeFull(robotHeading + tagYawDeg);
            }

            double desiredTurretRelDeg = normalizeDeg(fieldTargetHeading - robotHeading);

            double actualTicks = turretEncoder.getCurrentPosition();
            double actualDeg = ticksToAngle(actualTicks);

            double error = desiredTurretRelDeg - actualDeg;

            // PID
            integral += error;
            if (Math.abs(integral) > maxIntegral) integral = Math.signum(integral) * maxIntegral;

            double derivative = error - lastError;
            double pidPower = kP * error + kI * integral + kD * derivative;

            double ffPower = -kFF * chassisYawRateDegPerSec;

            double power = pidPower + ffPower;
            power = clamp(power, -maxPower, maxPower);

            turretServo.setPower(power);
            turretServo2.setPower(power);

            lastError = error;

            telemetry.addData("TagVisible", tagVisible);
            telemetry.addData("RobotHeading", "%.2f", robotHeading);
//
            telemetry.addData("FieldTargetHeading", "%.2f", fieldTargetHeading);
            telemetry.addData("DesiredRelDeg", "%.2f", desiredTurretRelDeg);
            telemetry.addData("ActualDeg", "%.2f", actualDeg);
            telemetry.addData("Error", "%.2f", error);
            telemetry.addData("PID", "%.3f", pidPower);
            telemetry.addData("FF", "%.3f", ffPower);
            telemetry.addData("OutPower", "%.3f", power);
            telemetry.update();
        }

        visionPortal.close();
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
