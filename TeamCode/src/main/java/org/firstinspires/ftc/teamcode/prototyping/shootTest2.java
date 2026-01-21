package org.firstinspires.ftc.teamcode.prototyping;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
//import com.bylazar.configurables.annotations.Configurable;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp(name = "shootTest_final", group = "ProtoTypes")

public class shootTest2 extends LinearOpMode {

    public FtcDashboard dashboard;
    public static int buffer = 100;
    public static double dt = 0;
    public static  boolean decider = false;

    public double timetaken = 0;
    public static double targetVelociry = 1900;
    public static boolean t = true;
    public static boolean t_out = true;
    ElapsedTime T = new ElapsedTime();
    public static double P = 1.24;
    public static double I = 0.124;
    public static double D = 0;
    public static double F = 12.4;
    DcMotorEx m1;
//    DcMotorEx m2;
    public static double power = 0.5;
    public static double velocity = 1900;
    public static int pose = 1000;


    public Servo hood ;
    @Override
    public void runOpMode() throws InterruptedException {


        dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        hood = hardwareMap.get(Servo.class, "hood");
        m1 = hardwareMap.get(DcMotorEx.class, "m1");
//        m2 = hardwareMap.get(DcMotorEx.class, "m2");
        m1.setDirection(DcMotorSimple.Direction.FORWARD);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        m2 = hardwareMap.get(DcMotorEx.class, "m2");
//        m2.setDirection(DcMotorSimple.Direction.REVERSE);
//        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        timetaken = 0;
        t = true;
        t_out = true;
        decider = false;
        dt = 0;

//        m1.setVelocityPIDFCoefficients(P,I,D,F);
//        m2.setVelocityPIDFCoefficients(P,I,D,F);


        while (opModeInInit()){
            hood.setPosition(0.5);
        }
        boolean velo = true;
        waitForStart();
        while (opModeIsActive()){


            packet.put("Target Velocity", velocity);
            packet.put("Current Velocity", m1.getVelocity());
//            packet.put("Current Velocity 2", m2.getVelocity());

            dashboard.sendTelemetryPacket(packet);

            if(gamepad1.a){
                hood.setPosition(hood.getPosition()+0.01);
            }
            if(gamepad1.b){
                hood.setPosition(hood.getPosition()-0.01);
            }
            if(t){
                T.startTime();
                T.reset();
                t = false;
            }

            m1.setVelocityPIDFCoefficients(P,I,D,F);
//            m2.setVelocityPIDFCoefficients(P,I,D,F);
            if(gamepad1.left_bumper){
                velo = true;
            }if(gamepad1.right_bumper){
                velo = false;
            }

            if(!velo){
                m1.setPower(power);
//                m2.setPower(power);
                telemetry.addLine("SetPower using");
            }
            else if(velo){
                m1.setVelocity(velocity);
//                m2.setVelocity(velocity);
                telemetry.addLine("SetVelocity using");
            }
            if(m1.getVelocity()> targetVelociry && t_out){
                timetaken = T.milliseconds()-dt;
                t_out = false;
                decider = true;
            }

            if((decider && m1.getVelocity()<targetVelociry-buffer)){
//                T.reset();
                dt = T.milliseconds();
                t_out = true;
            }




            telemetry.addData("ramp time ", timetaken);
            telemetry.addData("elapsed ",T.milliseconds());
            telemetry.addData("m1 curent", m1.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("m1 curent pose", m1.getCurrentPosition());
            telemetry.addData("m1 curent vel", m1.getVelocity());
//            telemetry.addLine();
//            telemetry.addData("m2 curent", m2.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("m2 curent pose", m2.getCurrentPosition());
//            telemetry.addData("m2 curent vel", m2.getVelocity());

            telemetry.addData("pose", pose);
            telemetry.addData("velocity", velocity);
            telemetry.update();




        }
    }

    public void runMotortoPose(DcMotorEx m, int pose){
        m.setTargetPosition(pose);
        m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m.setVelocity(power);
    }
    public void runMotortoVelo(DcMotorEx m, int pose){

        m.setTargetPosition(pose);
        m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m.setPower(velocity);
    }
}
