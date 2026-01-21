package org.firstinspires.ftc.teamcode.sequences;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystem.Feeder;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

public class FeederSeq {
    public static RobotHardware robot = RobotHardware.getInstance();

    public static Action UF_ON(Intake intake, Outtake outtake, Feeder feeder) {
        return new SequentialAction(
                feeder.UFCommand(Feeder.UpperFeederState.OFF)
        );
    }
    public static Action UF_OFF(Intake intake, Outtake outtake, Feeder feeder) {
        return new SequentialAction(
                feeder.UFCommand(Feeder.UpperFeederState.OFF)
        );
    }
    public static Action UF_RELEASE(Intake intake, Outtake outtake, Feeder feeder) {
        return new SequentialAction(
                feeder.UFCommand(Feeder.UpperFeederState.RELEASE)
        );
    }
}
