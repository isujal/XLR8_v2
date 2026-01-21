package org.firstinspires.ftc.teamcode.extras;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

//@TeleOp(name = "Turret Lock with Vision+IMU", group = "ProtoTypes")
@Disabled
@Deprecated
public class tag_turret extends LinearOpMode {

    private DcMotor turretMotor;
    private IMU imu;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // PID coefficients
    private double kP = 0.01, kI = 0.0, kD = 0.0005;

    private double integral = 0, lastError = 0;

    private static final int TICKS_PER_REV = 112 * 40;
    private double turretZero = 0;
    private double desiredTurretAngle = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        imu = hardwareMap.get(IMU.class, "imu");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        initAprilTag();

        telemetry.addLine("Ready for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            double tagYaw = 0;
            List<AprilTagDetection> detections = aprilTag.getDetections();
            if (!detections.isEmpty()) {
                tagYaw = detections.get(0).ftcPose.yaw;
                desiredTurretAngle = robotHeading + tagYaw;
            }

            desiredTurretAngle = AngleUnit.normalizeDegrees(desiredTurretAngle);

            double targetTicks = angleToTicks(desiredTurretAngle);
            double currentTicks = turretMotor.getCurrentPosition();
            double error = targetTicks - currentTicks;

            // PID
            integral += error;
            double derivative = error - lastError;
            double output = (kP * error) + (kI * integral) + (kD * derivative);

            turretMotor.setPower(output);

            lastError = error;

            telemetry.addData("Robot Heading", robotHeading);
            telemetry.addData("Tag Yaw", tagYaw);
            telemetry.addData("Desired Angle", desiredTurretAngle);
            telemetry.addData("Turret Ticks", currentTicks);
            telemetry.addData("Target Ticks", targetTicks);
            telemetry.addData("Motor Power", output);
            telemetry.update();
        }
    }

    private double angleToTicks(double angleDeg) {

        return turretZero + (angleDeg / 360) * TICKS_PER_REV;
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }
}
