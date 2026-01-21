package org.firstinspires.ftc.teamcode.extras;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

//@Config
@Deprecated
@Disabled
//@TeleOp(name = "Prototype Continuous PIDF", group = "Prototypes")
public class shootTest extends LinearOpMode {

    private DcMotorEx motor1;
    private DcMotorEx motor2;
    private DcMotorEx extEncoder;

    private Servo servo;
    public CRServo turretL;
    public CRServo turretR;

    // Tunable constants via FTC Dashboard
    public static double power = 0.4;
    public static double vel = 1500;

    public static double p = 0.1;
    public static double i = 0.0;
    public static double d = 0.0;
    public static double f = 0.0;

    private boolean velocityMode = false;   // whether PIDF velocity mode is active
    private double targetVelocity = 0;      // store target velocity

    @Override
    public void runOpMode() {
        motor1 = hardwareMap.get(DcMotorEx.class, "m1");
        motor2 = hardwareMap.get(DcMotorEx.class, "m2");
        servo = hardwareMap.get(Servo.class, "s1");
        turretL = hardwareMap.get(CRServo.class, "cr1");
        turretR = hardwareMap.get(CRServo.class, "cr2");
        extEncoder = hardwareMap.get(DcMotorEx.class, "extEncoder");

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeInInit()) {
            extEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            extEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad2.a) {
                velocityMode = true;
                targetVelocity = vel;
                motor1.setVelocityPIDFCoefficients(p, i, d, f);
                motor2.setVelocityPIDFCoefficients(p, i, d, f);
            }

            if (gamepad2.x) {
                velocityMode = true;
                targetVelocity = -vel;
                motor1.setVelocityPIDFCoefficients(p, i, d, f);
                motor2.setVelocityPIDFCoefficients(p, i, d, f);
            }

            if (gamepad2.b) {
                velocityMode = false;
                motor1.setPower(0);
                motor2.setPower(0);
            }

            if (velocityMode) {
                motor1.setVelocity(targetVelocity);
                motor2.setVelocity(targetVelocity);
            }

            if (!velocityMode) {
                if (gamepad1.a) {
                    motor1.setPower(power);
                    motor2.setPower(-power);
                }
                if (gamepad1.y) {
                    motor1.setPower(-power);
                    motor2.setPower(power);
                }
                if (gamepad1.b) {
                    motor1.setPower(0);
                    motor2.setPower(0);
                }
            }

            // Servo controls
            if (gamepad1.dpad_up) {
                servo.setPosition(1);
            }
            if (gamepad1.dpad_down) {
                servo.setPosition(0);
            }
            if (gamepad1.right_bumper) {
                servo.setPosition(servo.getPosition() + 0.01);
            }
            if (gamepad1.left_bumper) {
                servo.setPosition(servo.getPosition() - 0.01);
            }

            // Telemetry
            telemetry.addData("Velocity Mode", velocityMode);
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("Motor1 Velocity", motor1.getVelocity());
            telemetry.addData("Motor2 Velocity", motor2.getVelocity());
            telemetry.update();
        }
    }
}
