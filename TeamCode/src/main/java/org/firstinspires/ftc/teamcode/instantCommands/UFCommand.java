package org.firstinspires.ftc.teamcode.instantCommands;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.subsystem.Feeder;

public class UFCommand {
    public UFCommand(Feeder feeder, Feeder.UpperFeederState state) {
        Actions.runBlocking(
                new SequentialAction(
                        new InstantAction( () -> feeder.updateUFState(state))
                )
        );
    }
}
