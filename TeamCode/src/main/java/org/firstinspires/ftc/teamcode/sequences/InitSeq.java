package org.firstinspires.ftc.teamcode.sequences;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;

import org.firstinspires.ftc.teamcode.subsystem.Feeder;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

public class InitSeq {
    public static Action InitAction(Intake intake, Outtake outtake, Feeder feeder) {
        return new ParallelAction(
                Intake.IntakeCommand(Intake.IntakeServoState.INIT),
                Intake.RollerCommand(Intake.IntakeRollerState.OFF),
                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                Outtake.TurretCommand(Outtake.TurretState.INIT),
                Outtake.ShooterCommand(Outtake.ShooterState.OFF)
//                Outtake.HoodCommand(Outtake.HoodState.INIT)

        );
    }
}