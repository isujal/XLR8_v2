package org.firstinspires.ftc.teamcode.prototyping;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
@Config
@TeleOp(name = "Prototype Test", group = "ProtoTypes")
public class prototypes_testing extends LinearOpMode {

    private DcMotorEx motor1;
    private DcMotorEx motor2;
    private DcMotorEx motor3;
    private DcMotorEx motor4;
//    private DcMotorEx extEncoder;

    private Servo servo;
    public CRServo turretL;
    public CRServo turretR;
    public static double power1 = 0.5;
    public static double power2 = 0.5;
    public static double power3 = 0.5;
    public static double power4 = 0.5;
    public static double turretRotate = 0.5;
    public static double vel = 1500;
    public static double pos = 0.5;
    public static double cr_power;

    public static double p = 0;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;

    public static double actualVelocity = 0;
    public static double velocity = 0;

    @Override
    public void runOpMode() {
        motor1 = hardwareMap.get(DcMotorEx.class, "m1");
        motor2 = hardwareMap.get(DcMotorEx.class, "m2");
        motor3 = hardwareMap.get(DcMotorEx.class, "m3");
        motor4 = hardwareMap.get(DcMotorEx.class, "m4");
        servo = hardwareMap.get(Servo.class, "s1");
        turretL = hardwareMap.get(CRServo.class, "cr1");
        turretR = hardwareMap.get(CRServo.class, "cr2");
//        extEncoder = hardwareMap.get(DcMotorEx.class, "extEncoder");
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeInInit()){
//            extEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//            extEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        waitForStart();

        while (opModeIsActive()) {
            actualVelocity = motor1.getVelocity();
            motor1.setVelocityPIDFCoefficients(p,i,d,f);

            if(gamepad1.a){
                motor1.setPower(power1);
            }
            if(gamepad1.y){
                motor2.setPower(power2);
            }
            if (gamepad1.b)
            {
                motor3.setPower(power3);
            }
            if (gamepad1.x)
            {
                motor4.setPower(power4);
            }
            if(gamepad1.dpad_up)
            {
                servo.setPosition(1);
            }
            if(gamepad1.dpad_down)
            {
                servo.setPosition(pos);
            }
            if(gamepad1.dpad_right)
            {
                servo.setPosition(-1);
            }

            if(gamepad1.right_bumper){
                servo.setPosition(servo.getPosition()+0.01);
            }  if(gamepad1.left_bumper){
                servo.setPosition(servo.getPosition()-0.01);
            }
            if(gamepad1.left_trigger>0.3){
                servo.setPosition(turretRotate);
            }

            double motor1Current = motor1.getCurrent(CurrentUnit.AMPS);
            double motor2Current = motor2.getCurrent(CurrentUnit.AMPS);
            double motor3Current = motor3.getCurrent(CurrentUnit.AMPS);
            double motor4Current = motor4.getCurrent(CurrentUnit.AMPS);

            telemetry.addData("Motor power1 ", power1);
            telemetry.addData("Motor 1 Current ", motor1Current);
            telemetry.addData("Motor 2 Current ", motor2Current);
            telemetry.addData("Motor 3 Current ", motor3Current);
            telemetry.addData("Motor 4 Current ", motor4Current);

            telemetry.addData("Motor 1 pos ", motor1.getCurrentPosition());
            telemetry.addData("Motor 2 pos ",motor2.getCurrentPosition());

            telemetry.addData("Motor 1 Velo ", motor1.getVelocity());
            telemetry.addData("Motor 2 Velo ", motor2.getVelocity());
//            telemetry.addData("Actual Pos", extEncoder.getCurrentPosition());
            telemetry.addData("ServoPos",servo.getPosition());
            telemetry.update();
        }
    }
}
