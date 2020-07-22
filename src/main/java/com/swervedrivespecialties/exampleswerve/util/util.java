package com.swervedrivespecialties.exampleswerve.util;

public class util {

    public static double getSpeedScale(double minSS, double additionalSS){
        return minSS + (1 - minSS) * Math.pow(additionalSS, 2);
    }

}