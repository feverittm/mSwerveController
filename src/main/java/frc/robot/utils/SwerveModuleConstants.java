package frc.robot.utils;

public class SwerveModuleConstants {
    public final int moduleNumber;
    public final int driveMotorID;
    public final int angleMotorID;
    public final boolean driveMotorReversed;
    public final boolean angleMotorReversed;
    public final double angleEncoderOffsetDegrees;

    public SwerveModuleConstants(int moduleNumber, int driveMotorID, int angleMotorID, boolean driveMotorReversed,
            boolean angleMotorReversed, double angleEncoderOffsetDegrees) {
        this.moduleNumber = moduleNumber;
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.driveMotorReversed = driveMotorReversed;
        this.angleMotorReversed = angleMotorReversed;
        this.angleEncoderOffsetDegrees = angleEncoderOffsetDegrees;
    }
}
