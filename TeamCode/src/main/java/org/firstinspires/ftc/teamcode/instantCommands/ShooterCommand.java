package org.firstinspires.ftc.teamcode.instantCommands;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.subsystem.Outtake;

public class ShooterCommand {
    public ShooterCommand(Outtake outtake, Outtake.ShooterState state) {
        Actions.runBlocking(
                new SequentialAction(
                        new InstantAction( () -> outtake.updateShooterState(state))
                )
        );
    }
}
