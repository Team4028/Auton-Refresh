package com.swervedrivespecialties.exampleswerve.util;

public class util {

    /**
     * Converts value in inches to value to metres
     * @param in Value in inches
     * @return Equivalent value in metres
     */
    public static double inToM(double in){
        return in / 39.37;
    }

    /**
     * Converts value in metres to inches
     * @param M Value in metres
     * @return Equivalent value in inches
     */
    public static double mToIn(double M){
        return M * 39.37;
    }
}