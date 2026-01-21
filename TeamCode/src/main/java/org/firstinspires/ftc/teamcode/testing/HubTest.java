package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


//@TeleOp
//@Config
//@Deprecated
//@Disabled
public class HubTest extends LinearOpMode {

    public Servo s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, s12;
    public DcMotorEx m1, m2, m3, m4, m5, m6, m7, m8;
    public RevColorSensorV3 cs1, cs2, cs3, cs4, cs5, cs6, cs7;
    public DigitalChannel d1, d2, d3, d4, d5, d6, d7, d8,  /* EXP*/
            d9, d10, d11, d12, d13, d14, d15, d16;
    public AnalogInput a1, a2, a3, a4,   /* EXP*/
            a5, a6, a7, a8;
    public IMU imu;

    //WEBCAM
//    private OpenCvCamera webcam;
//    pipelin pipeline = new pipelin();

    public static double servoInit = 0.5;
    public static double motorPow = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        //TODO SERVOS
        s1 = hardwareMap.get(Servo.class, "s1");
        s2 = hardwareMap.get(Servo.class, "s2");
        s3 = hardwareMap.get(Servo.class, "s3");
        s4 = hardwareMap.get(Servo.class, "s4");
        s5 = hardwareMap.get(Servo.class, "s5");
        s6 = hardwareMap.get(Servo.class, "s6");
        s7 = hardwareMap.get(Servo.class, "s7");
        s8 = hardwareMap.get(Servo.class, "s8");
        s9 = hardwareMap.get(Servo.class, "s9");
        s10 = hardwareMap.get(Servo.class, "s10");
        s11 = hardwareMap.get(Servo.class, "s11");
        s12 = hardwareMap.get(Servo.class, "s12");


        //TODO DC-MOTORS
        m1 = hardwareMap.get(DcMotorEx.class, "m1");
        m2 = hardwareMap.get(DcMotorEx.class, "m2");
        m3 = hardwareMap.get(DcMotorEx.class, "m3");
        m4 = hardwareMap.get(DcMotorEx.class, "m4");
        m5 = hardwareMap.get(DcMotorEx.class, "m5");
        m6 = hardwareMap.get(DcMotorEx.class, "m6");
        m7 = hardwareMap.get(DcMotorEx.class, "m7");
        m8 = hardwareMap.get(DcMotorEx.class, "m8");

        //I2C DEVICE
        cs1 = hardwareMap.get(RevColorSensorV3.class, "cs1");
        cs2 = hardwareMap.get(RevColorSensorV3.class, "cs2");
        cs3 = hardwareMap.get(RevColorSensorV3.class, "cs3");
        cs4 = hardwareMap.get(RevColorSensorV3.class, "cs4");
        cs5 = hardwareMap.get(RevColorSensorV3.class, "cs5");
        cs6 = hardwareMap.get(RevColorSensorV3.class, "cs6");
        cs7 = hardwareMap.get(RevColorSensorV3.class, "cs7");

        //DIGITAL DEVICE
        d1 = hardwareMap.get(DigitalChannel.class, "d1");
        d2 = hardwareMap.get(DigitalChannel.class, "d2");
        d3 = hardwareMap.get(DigitalChannel.class, "d3");
        d4 = hardwareMap.get(DigitalChannel.class, "d4");
        d5 = hardwareMap.get(DigitalChannel.class, "d5");
        d6 = hardwareMap.get(DigitalChannel.class, "d6");
        d7 = hardwareMap.get(DigitalChannel.class, "d7");
        d8 = hardwareMap.get(DigitalChannel.class, "d8");

        d9 = hardwareMap.get(DigitalChannel.class, "d9");
        d10 = hardwareMap.get(DigitalChannel.class, "d10");
        d11 = hardwareMap.get(DigitalChannel.class, "d11");
        d12 = hardwareMap.get(DigitalChannel.class, "d12");
        d13 = hardwareMap.get(DigitalChannel.class, "d13");
        d14 = hardwareMap.get(DigitalChannel.class, "d14");
        d15 = hardwareMap.get(DigitalChannel.class, "d15");
        d16 = hardwareMap.get(DigitalChannel.class, "d16");

        a1 = hardwareMap.get(AnalogInput.class, "a1");
        a2 = hardwareMap.get(AnalogInput.class, "a2");
        a3 = hardwareMap.get(AnalogInput.class, "a3");
        a4 = hardwareMap.get(AnalogInput.class, "a4");
        a5 = hardwareMap.get(AnalogInput.class, "a5");
        a6 = hardwareMap.get(AnalogInput.class, "a6");
        a7 = hardwareMap.get(AnalogInput.class, "a7");
        a8 = hardwareMap.get(AnalogInput.class, "a8");

        //IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);
        imu.resetYaw();

        servoInit(servoInit);

        while (opModeInInit()) {

            // Wait for the DS start button to be touched.
            telemetry.addLine("PRESS A FOR SERVO POS-- 0");
            telemetry.addLine("PRESS B FOR SERVO POS-- 1");
            telemetry.addLine("PRESS X FOR MOTOR FORWARD -- +1");
            telemetry.addLine("PRESS Y FOR MOTOR FORWARD -- -1");
            telemetry.addLine("PRESS BACK FOR MOTOR FORWARD -- 0");
            telemetry.update();
        }


        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.a) {
                servoInit(1);
            } else if (gamepad1.b) {
                servoInit(0);
            }

            telemetry.addLine("DIGITAL SENSOR");
            digitalRead(telemetry);
            telemetry.addLine("ANALOG SENSOR");
            analogRead(telemetry);
            telemetry.addLine("ENCODER-READ");
            encoderRead(telemetry);
            telemetry.addLine("MOTOR CURRENT");
            telemetry.addLine("COLOR SENSOR");
            colorI2Check(telemetry);

            telemetry.addData("IMU", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));


            if (gamepad1.x) {
                motorPow = -1;
            }
            if (gamepad1.y) {
                motorPow = 1;
            }
            if (gamepad1.back) {
                motorPow = 0;
            }

            motorCurrent(telemetry, motorPow);
            telemetry.update();


        }
    }

    public void analogRead(Telemetry telemetry) {
        telemetry.addData("a1", a1.getVoltage());
        telemetry.addData("a2", a2.getVoltage());
        telemetry.addData("a3", a3.getVoltage());
        telemetry.addData("a4", a4.getVoltage());
        telemetry.addData("a5", a5.getVoltage());
        telemetry.addData("a6", a6.getVoltage());
        telemetry.addData("a7", a7.getVoltage());
        telemetry.addData("a8", a8.getVoltage());
    }

    public void digitalRead(Telemetry telemetry) {
        telemetry.addData("d1", d1.getState());
        telemetry.addData("d2", d2.getState());
        telemetry.addData("d3", d3.getState());
        telemetry.addData("d4", d4.getState());
        telemetry.addData("d5", d5.getState());
        telemetry.addData("d6", d6.getState());
        telemetry.addData("d7", d7.getState());
        telemetry.addData("d8", d8.getState());
        telemetry.addData("d9", d9.getState());
        telemetry.addData("d10", d10.getState());
        telemetry.addData("d11", d11.getState());
        telemetry.addData("d12", d12.getState());
        telemetry.addData("d13", d13.getState());
        telemetry.addData("d14", d14.getState());
        telemetry.addData("d15", d15.getState());
        telemetry.addData("d16", d16.getState());
    }

    public void colorI2Check(Telemetry telemetry) {
        telemetry.addData("red", cs1.red());
        telemetry.addData("green", cs1.green());
        telemetry.addData("blue", cs1.blue());
        telemetry.addLine();
        telemetry.addData("red", cs2.red());
        telemetry.addData("green", cs2.green());
        telemetry.addData("blue", cs2.blue());
        telemetry.addLine();
        telemetry.addData("red", cs3.red());
        telemetry.addData("green", cs3.green());
        telemetry.addData("blue", cs3.blue());
        telemetry.addLine();
        telemetry.addData("red", cs4.red());
        telemetry.addData("green", cs4.green());
        telemetry.addData("blue", cs4.blue());
        telemetry.addLine();
        telemetry.addData("red", cs5.red());
        telemetry.addData("green", cs5.green());
        telemetry.addData("blue", cs5.blue());
        telemetry.addLine();
        telemetry.addData("red", cs6.red());
        telemetry.addData("green", cs6.green());
        telemetry.addData("blue", cs6.blue());
        telemetry.addLine();
        telemetry.addData("red", cs7.red());
        telemetry.addData("green", cs7.green());
        telemetry.addData("blue", cs7.blue());
    }

    public void setServo(Telemetry telemetry) {
        s1.setPosition(0.5);
        s2.setPosition(1);
        s3.setPosition(0.5);
        s4.setPosition(1);
        s5.setPosition(0.5);
        s6.setPosition(1);
        s7.setPosition(0.5);
        s8.setPosition(1);
        s9.setPosition(0.5);
        s10.setPosition(1);
        s11.setPosition(0.5);
        s12.setPosition(1);
    }

    public void servoInit(double pos) {
        s1.setPosition(pos);
        s2.setPosition(pos);
        s3.setPosition(pos);
        s4.setPosition(pos);
        s5.setPosition(pos);
        s6.setPosition(pos);
        s7.setPosition(pos);
        s8.setPosition(pos);
        s9.setPosition(pos);
        s10.setPosition(pos);
        s11.setPosition(pos);
        s12.setPosition(pos);
    }

    public void encoderRead(Telemetry telemetry) {
        telemetry.addData("m1 Position", m1.getCurrentPosition());
        telemetry.addData("m2 Position", m2.getCurrentPosition());
        telemetry.addData("m3 Position", m3.getCurrentPosition());
        telemetry.addData("m4 Position", m4.getCurrentPosition());
        telemetry.addData("m5 Position", m5.getCurrentPosition());
        telemetry.addData("m6 Position", m6.getCurrentPosition());
        telemetry.addData("m7 Position", m7.getCurrentPosition());
        telemetry.addData("m8 Position", m8.getCurrentPosition());
    }

    public void motorCurrent(Telemetry telemetry, double power) {
        m1.setPower(power);
        m2.setPower(power);
        m3.setPower(power);
        m4.setPower(power);
        m5.setPower(power);
        m6.setPower(power);
        m7.setPower(power);
        m8.setPower(power);
        telemetry.addData("m1 current ", m1.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("m2 current ", m2.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("m3 current ", m3.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("m4 current ", m4.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("m5 current ", m5.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("m6 current ", m6.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("m7 current ", m7.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("m8 current ", m8.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("m1 velo ", m1.getVelocity());
        telemetry.addData("m2 velo ", m2.getVelocity());
        telemetry.addData("m3 velo ", m3.getVelocity());
        telemetry.addData("m4 velo ", m4.getVelocity());
        telemetry.addData("m5 velo ", m5.getVelocity());
        telemetry.addData("m6 velo ", m6.getVelocity());
        telemetry.addData("m7 velo ", m7.getVelocity());
        telemetry.addData("m8 velo ", m8.getVelocity());
    }

    //CAMERA SETUP


}
