package com.swervedrivespecialties.autonrefresh.util;

public class AutonChooser {
    
    private SendableChooser<AUTONS> autonChooser = new SendableChooser<>();
    
    private static AutonChooser instance = new AutonChooser();
    private static AutonChooser getInstance() {
        return instance;
    }
    private enum AUTONS 
    {
        DO_NOTHING,
        STEAL_BALLS,
        OWN_TRENCH;
    }

    private AutonChooser() {
        autonChooser.setDefaultOption("Do Nothing", AUTONS.DO_NOTHING);
        autonChooser.addOption("Steal Balls", AUTONS.STEAL_BALLS);
        autonChooser.addOption("Own the Trench", AUTONS.OWN_TRENCH);
    }
}