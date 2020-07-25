package com.swervedrivespecialties.exampleswerve.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.swervedrivespecialties.exampleswerve.RobotMap;
import com.swervedrivespecialties.exampleswerve.constants.ChassisConstants;
import com.swervedrivespecialties.exampleswerve.util.util;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.drivers.Mk2SwerveModuleBuilder;
import org.frcteam2910.common.robot.drivers.NavX;

public class DrivetrainSubsystem extends SubsystemBase {
    private static final double TRACKWIDTH = 19.5;
    private static final double WHEELBASE = 23.5;

    private double minSpeedScale = ChassisConstants.INITIAL_SPEED_SCALE;

    private static DrivetrainSubsystem instance;

    CANSparkMax frontLeftDrive = new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax frontRightDrive = new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax backLeftDrive = new CANSparkMax(RobotMap.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax backRightDrive = new CANSparkMax(RobotMap.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax frontLeftSteer = new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax frontRightSteer = new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax backLeftSteer = new CANSparkMax(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax backRightSteer = new CANSparkMax(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final SwerveModule frontLeftModule = new Mk2SwerveModuleBuilder(
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER), ChassisConstants.FRONT_LEFT_ANGLE_OFFSET)
            .angleMotor(frontLeftSteer, ChassisConstants.ANGLE_CONSTANTS, ChassisConstants.ANGLE_REDUCTION)
            .driveMotor(frontLeftDrive, ChassisConstants.DRIVE_REDUCTION, ChassisConstants.WHEEL_DIAMETER)
            .build();
    private final SwerveModule frontRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER), ChassisConstants.FRONT_RIGHT_ANGLE_OFFSET)
            .angleMotor(frontRightSteer, ChassisConstants.ANGLE_CONSTANTS, ChassisConstants.ANGLE_REDUCTION)
            .driveMotor(frontRightDrive, ChassisConstants.DRIVE_REDUCTION, ChassisConstants.WHEEL_DIAMETER)
            .build();
    private final SwerveModule backLeftModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER), ChassisConstants.BACK_LEFT_ANGLE_OFFSET)
            .angleMotor(backLeftSteer, ChassisConstants.ANGLE_CONSTANTS, ChassisConstants.ANGLE_REDUCTION)
            .driveMotor(backLeftDrive, ChassisConstants.DRIVE_REDUCTION, ChassisConstants.WHEEL_DIAMETER)
            .build();
    private final SwerveModule backRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER), ChassisConstants.BACK_RIGHT_ANGLE_OFFSET)
            .angleMotor(backRightSteer, ChassisConstants.ANGLE_CONSTANTS, ChassisConstants.ANGLE_REDUCTION)
            .driveMotor(backRightDrive, ChassisConstants.DRIVE_REDUCTION, ChassisConstants.WHEEL_DIAMETER)
            .build();

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0)
    );

    private final Gyroscope gyroscope = new NavX(SPI.Port.kMXP);

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getGyroscope(), new Pose2d());

    public DrivetrainSubsystem() {
        gyroscope.calibrate();
        gyroscope.setInverted(true); // You might not need to invert the gyro

        frontLeftModule.setName("Front Left");
        frontRightModule.setName("Front Right");
        backLeftModule.setName("Back Left");
        backRightModule.setName("Back Right");
    }

    public static DrivetrainSubsystem getInstance() {
        if (instance == null) {
            instance = new DrivetrainSubsystem();
        }

        return instance;
    }

    /**
     * Update odometry from gyroscope and module states
     */
    public void updateOdometry(){
        odometry.update(getGyroscope(), getModuleStates());
    }

    @Override
    public void periodic() {
        frontLeftModule.updateSensors();
        frontRightModule.updateSensors();
        backLeftModule.updateSensors();
        backRightModule.updateSensors();

        SmartDashboard.putNumber("Front Left Module Angle", Math.toDegrees(frontLeftModule.getCurrentAngle()));
        SmartDashboard.putNumber("Front Right Module Angle", Math.toDegrees(frontRightModule.getCurrentAngle()));
        SmartDashboard.putNumber("Back Left Module Angle", Math.toDegrees(backLeftModule.getCurrentAngle()));
        SmartDashboard.putNumber("Back Right Module Angle", Math.toDegrees(backRightModule.getCurrentAngle()));

        SmartDashboard.putNumber("Gyroscope Angle", gyroscope.getAngle().toDegrees());

        updateOdometry();

        frontLeftModule.updateState(TimedRobot.kDefaultPeriod);
        frontRightModule.updateState(TimedRobot.kDefaultPeriod);
        backLeftModule.updateState(TimedRobot.kDefaultPeriod);
        backRightModule.updateState(TimedRobot.kDefaultPeriod);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldOriented) {
        rotation *= 2.0 / Math.hypot(WHEELBASE, TRACKWIDTH);
        ChassisSpeeds speeds;
        if (fieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
                    Rotation2d.fromDegrees(gyroscope.getAngle().toDegrees()));
        } else {
            speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        }

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        frontLeftModule.setTargetVelocity(states[0].speedMetersPerSecond, states[0].angle.getRadians());
        frontRightModule.setTargetVelocity(states[1].speedMetersPerSecond, states[1].angle.getRadians());
        backLeftModule.setTargetVelocity(states[2].speedMetersPerSecond, states[2].angle.getRadians());
        backRightModule.setTargetVelocity(states[3].speedMetersPerSecond, states[3].angle.getRadians());
    }

    public void resetGyroscope() {
        gyroscope.setAdjustmentAngle(gyroscope.getUnadjustedAngle());
    }

    public Rotation2d getGyroscope(){
        return Rotation2d.fromDegrees(gyroscope.getAngle().toDegrees());
    }

    private SwerveModuleState getCurrentState(SwerveModule mod){
        double velo = util.inToM(mod.getCurrentVelocity());
        Rotation2d rot = Rotation2d.fromDegrees(Math.toDegrees(mod.getCurrentAngle()));
        return new SwerveModuleState(velo, rot);
    }

    private SwerveModuleState[] getModuleStates(){
        return new SwerveModuleState[] {getCurrentState(frontLeftModule),
                                        getCurrentState(frontRightModule), 
                                        getCurrentState(backLeftModule), 
                                        getCurrentState(backRightModule)};
    }

    public Vector2 getKinematicVelocity(){
        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(getModuleStates());
        double xVelo = util.mToIn(chassisSpeeds.vxMetersPerSecond);
        double yVelo = util.mToIn(chassisSpeeds.vyMetersPerSecond);
        return new Vector2(xVelo, yVelo);
    }

    public Vector2 getKinematicPosition(){
        Pose2d odpos = odometry.getPoseMeters();
        double x = util.mToIn(odpos.getTranslation().getY()); // Need these if you decide to use a vector2
        double y = util.mToIn(odpos.getTranslation().getX());
        return new Vector2(x,y);
    }

    public void resetOdometry(){
        odometry.resetPosition(new Pose2d(), getGyroscope());
    }

    /**
    * Toggles Minimum Speed Scale Multiplier by Interval specified as a constant
    * in ChassisConstants class
    */
    public void toggleMinSpeedScale(){
        minSpeedScale = minSpeedScale < 1? minSpeedScale + ChassisConstants.SPEED_SCALE_INTERVAL: ChassisConstants.INITIAL_SPEED_SCALE;
    }

    /**
    * Resets Minimum Speed Scale Multiplier to Initial value Specified as a constant
    * in ChassisConstants class
    */
    public void resetMinSpeedScale(){
        minSpeedScale = ChassisConstants.INITIAL_SPEED_SCALE;
    }

    public double getMinSpeedScale(){
        return minSpeedScale;
    }

}
