package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Config

public class Feeder {
    public static RobotHardware robot = RobotHardware.getInstance();
    public static UpperFeederState upperFeederState = UpperFeederState.OFF;
    public static LowerFeederState lowerFeederState = LowerFeederState.OFF;

    public enum UpperFeederState {
        ON, OFF, POW, RELEASE
    }public enum LowerFeederState {
        ON, OFF, POW, RELEASE
    }

    // Constructor
    public Feeder(RobotHardware robot) {Feeder.robot = robot;}

    // Update Enums
    public static void updateUFState(@NonNull UpperFeederState state){
        upperFeederState=state;
        double currentUFState= Globals.upperfeederOff;
        switch (state){
            case ON:
                currentUFState= Globals.upperfeederOn;
                break;
            case OFF:
                currentUFState= Globals.upperfeederOff;
                break;
            case POW:
                currentUFState= Globals.upperfeederPow;
                break;
            case RELEASE:
                currentUFState= Globals.upperfeederReverse;
                break;
        }
        setUpperFeederState(currentUFState);
    }

    public static void updateLFState(@NonNull LowerFeederState state){
        lowerFeederState=state;
        double currentLFState=Globals.lowerfeederOff;
        switch (state){
            case ON:
                currentLFState=Globals.lowerfeederOn;
                break;
            case OFF:
                currentLFState=Globals.lowerfeederOff;
                break;
            case POW:
                currentLFState=Globals.lowerfeederPow;
                break;
            case RELEASE:
                currentLFState=Globals.lowerfeederReverse;
                break;
        }
        setLowerFeederState(currentLFState);
    }
    public static void setUpperFeederState(double pow){
        robot.upperFeeder.setPower(pow);
    }
    public static void setLowerFeederState(double pow){
        robot.lowerFeeder.setPower(pow);
    }
    public static Action UFCommand(Feeder.UpperFeederState state) {
        return new InstantAction(() -> updateUFState(state));
    }
    public static Action LFCommand(Feeder.LowerFeederState state) {
        return new InstantAction(() -> updateLFState(state));
    }


    public void setLF(LowerFeederState state) {
        updateLFState(state);
    }
    public void setUF(UpperFeederState state) {
        updateUFState(state);
    }
}