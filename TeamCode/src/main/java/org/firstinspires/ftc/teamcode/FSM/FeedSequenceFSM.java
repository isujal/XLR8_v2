package org.firstinspires.ftc.teamcode.FSM;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Feeder;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class FeedSequenceFSM {

    private SequenceState state = SequenceState.IDLE;
    private final ElapsedTime timer = new ElapsedTime();

    public void update(
            boolean startEdge,
            Feeder feeder,
            Intake intake
    ) {
        switch (state) {

            case IDLE:
                if (startEdge) {
                    feeder.setUF(Feeder.UpperFeederState.ON);
                    timer.reset();
                    state = SequenceState.UF_ON_1;
                }
                break;

            case UF_ON_1:
                if (timer.seconds() > 0.3) {
                    feeder.setUF(Feeder.UpperFeederState.OFF);
                    feeder.setLF(Feeder.LowerFeederState.ON);
                    timer.reset();
                    state = SequenceState.UF_OFF_LF_ON;
                }
                break;

            case UF_OFF_LF_ON:
                if (timer.seconds() > 0.3) {
                    feeder.setUF(Feeder.UpperFeederState.ON);
                    feeder.setLF(Feeder.LowerFeederState.ON);
                    intake.setRoller(Intake.IntakeRollerState.ON);
                    timer.reset();
                    state = SequenceState.UF_ON_ROLLER_ON;
                }
                break;

            case UF_ON_ROLLER_ON:
                if (timer.seconds() > 0.2) {
                    feeder.setUF(Feeder.UpperFeederState.OFF);
                    feeder.setLF(Feeder.LowerFeederState.ON);
                    intake.setRoller(Intake.IntakeRollerState.ON);

                    timer.reset();
                    state = SequenceState.UF_OFF;
                }
                break;

            case UF_OFF:
                if (timer.seconds() > 0.3) {
                    feeder.setUF(Feeder.UpperFeederState.ON);
                    feeder.setLF(Feeder.LowerFeederState.ON);
                    intake.setRoller(Intake.IntakeRollerState.ON);
                    timer.reset();
                    state = SequenceState.UF_ON_FINAL;
                }
                break;

            case UF_ON_FINAL:
                if (timer.seconds() > 0.5) {
                    feeder.setUF(Feeder.UpperFeederState.OFF);
                    intake.setRoller(Intake.IntakeRollerState.OFF);
                    feeder.setLF(Feeder.LowerFeederState.OFF);
                    state = SequenceState.DONE;
                }
                break;

            case DONE:
                state = SequenceState.IDLE;
                break;
        }
    }

    public boolean isBusy() {
        return state != SequenceState.IDLE;
    }
}
