package org.firstinspires.ftc.teamcode.instantCommands;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class RollerCommand {
    public RollerCommand(Intake intake, Intake.IntakeRollerState state) {
        Actions.runBlocking(
                new SequentialAction(
                        new InstantAction( () ->intake.updateRollerState(state))
                )
        );
    }
}
