package org.firstinspires.ftc.teamcode.instantCommands;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.subsystem.Outtake;

public class TurretCommand {
    public TurretCommand(Outtake outtake, Outtake.TurretState state) {
        Actions.runBlocking(
                new SequentialAction(
                        new InstantAction( () ->outtake.updateTurretState(state))
                )
        );
    }
}
