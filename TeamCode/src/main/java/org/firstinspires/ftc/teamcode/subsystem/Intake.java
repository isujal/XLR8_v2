package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Config

public class Intake {
    public static RobotHardware robot = RobotHardware.getInstance();
    public static IntakeServoState intakeServoState = IntakeServoState.INIT;
    public static IntakeRollerState rollerState = IntakeRollerState.OFF;

    public enum IntakeServoState {
        INIT, IN, AUTO_IN, RELEASE
    }public enum IntakeRollerState {
        INIT, ON, OFF, RELEASE
    }

    // Constructor
    public Intake(RobotHardware robot) {Intake.robot = robot;}

    // Update Enums
    public static void updateRollerState(@NonNull IntakeRollerState state){
        rollerState=state;
        double currentRollerState= Globals.intakeRollerOff;
        switch (state){
            case INIT:
                currentRollerState= Globals.intakeRollerOff;
                break;
            case OFF:
                currentRollerState= Globals.intakeRollerOff;
                break;
            case ON:
                currentRollerState= Globals.intakeRollerOn;
                break;
            case RELEASE:
                currentRollerState=Globals.intakeRollerReverse;
                break;
        }
        setIntakeRoller(currentRollerState);
    }

    public static void updateIntakeState(@NonNull IntakeServoState state){
        intakeServoState=state;
        double currentIntakeServoState=Globals.intakeServoInit;
        switch (state){
            case INIT:
                currentIntakeServoState=Globals.intakeServoInit;
                break;
            case IN:
                currentIntakeServoState=Globals.intakeServoPick;
                break;
            case AUTO_IN:
                currentIntakeServoState=Globals.intakeServoAutoPick;
                break;
            case RELEASE:
                currentIntakeServoState=Globals.intakeServoRelease;
                break;
        }
        setIntakeServo(currentIntakeServoState);
    }

    public static void setIntakeServo(double pos){
        robot.intake.setPosition(pos);
    }
    public static void setIntakeRoller(double power){
        robot.intakeRoller.setPower(power);
    }

    public static Action IntakeCommand(Intake.IntakeServoState state) {
        return new InstantAction(() -> updateIntakeState(state));
    }
    public static Action RollerCommand(Intake.IntakeRollerState state) {
        return new InstantAction(() -> updateRollerState(state));
    }

    public void setIntake(IntakeServoState state) {
        updateIntakeState(state);
    }
    public void setRoller(IntakeRollerState state) {
        updateRollerState(state);
    }


}