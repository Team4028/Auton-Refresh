package com.swervedrivespecialties.exampleswerve.util;

public class util {
    /**
     * @param minSS The minimum Speed Scale, toggled with ToggleSS
     * @param additionalSS Additional Speed Scale, added on to minSS
     * @return Double to use as a scalar for drive velocity
     */
    public static double getSpeedScale(double minSS, double additionalSS){
        return minSS + (1 - minSS) * Math.pow(additionalSS, 2);
    }
}