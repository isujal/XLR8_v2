package org.firstinspires.ftc.teamcode.FSM;

public class BeamEdge {
    private boolean last;

    public void sync(boolean current) {
        last = current;
    }

    public boolean rising(boolean current) {
        boolean r = current && !last;
        last = current;
        return r;
    }
}

