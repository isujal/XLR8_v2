package org.firstinspires.ftc.teamcode.prototyping;

public class MathUtils {

    /**
     * Maps a value from one range to another.
     *
     * @param x       The input value to map
     * @param inMin   Minimum of the input range
     * @param inMax   Maximum of the input range
     * @param outMin  Minimum of the output range
     * @param outMax  Maximum of the output range
     * @return        The mapped value
     */
    public static double map(double x, double inMin, double inMax, double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }
}
