package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Config

public class Endgame {
    public static RobotHardware robot = RobotHardware.getInstance();
    public static EndgameState endgameState = EndgameState.INIT;

    public enum EndgameState {
        INIT, POS, RELEASE
    }
    // Constructor
    public Endgame(RobotHardware robot) {Endgame.robot = robot;}

    // Update Enums
    public static void updateEGState(@NonNull EndgameState state){
        endgameState=state;
        double currentEGState= Globals.eg_init;
        switch (state){
            case INIT:
                currentEGState= Globals.eg_init;
                break;
            case POS:
                currentEGState= Globals.eg_pos;
                break;
            case RELEASE:
                currentEGState= Globals.eg_release;
                break;
        }
        setEndgameState(currentEGState);
    }

    public static void setEndgameState(double pos){
        robot.endgame.setPosition(pos);
    }


}