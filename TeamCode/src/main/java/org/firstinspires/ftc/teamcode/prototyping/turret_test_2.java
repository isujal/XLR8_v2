package org.firstinspires.ftc.teamcode.prototyping;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

//@TeleOp(name = "turret_test_2", group = "ProtoTypes")

//@Config
@Disabled
public class turret_test_2 extends LinearOpMode {

    private CRServo crServoLeft;
    private CRServo crServoRight;
    private IMU imu;

    // PID variables
    public static double kp = 0.01;
    public static double ki = 0.0;
    public static double kd = 0.002;

    private double integral = 0;
    private double lastError = 0;

    public static double targetHeading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        crServoLeft = hardwareMap.get(CRServo.class, "crservoLeft");
        crServoRight = hardwareMap.get(CRServo.class, "crservoRight");

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addData(">", "Ready. Press Play to start");
        telemetry.update();

        while (opModeInInit()) {
            imu.resetYaw();
        }

        waitForStart();

        while (opModeIsActive()) {

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double currentHeading = orientation.getYaw(AngleUnit.DEGREES);

            double pidOutput = 0;

            if (gamepad1.a) {

                double error = targetHeading - currentHeading;

                error = normalizeDegrees(error);

                integral += error;
                double derivative = error - lastError;
                pidOutput = (kp * error) + (ki * integral) + (kd * derivative);
                lastError = error;

                crServoLeft.setPower(pidOutput);
                crServoRight.setPower(-pidOutput);

            } else {

                targetHeading = currentHeading;

                crServoLeft.setPower(0);
                crServoRight.setPower(0);

                integral = 0;
                lastError = 0;
            }

            //
            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("PID Active", gamepad1.a);
            telemetry.addData("PID Output", pidOutput);
            telemetry.update();
        }
    }

    private double normalizeDegrees(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }
}
