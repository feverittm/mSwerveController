package frc.robot.utils;

public class SwerveModuleConstants {
    public final int moduleNumber;
    public final int driveMotorID;
    public final int angleMotorID;
    public final boolean driveMotorReversed;
    public final boolean angleMotorReversed;
    public final double angleEncoderOffset;

    public SwerveModuleConstants(int moduleNumber, int driveMotorID, int angleMotorID, boolean driveMotorReversed,
            boolean angleMotorReversed, double angleEncoderOffset) {
        this.moduleNumber = moduleNumber;
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.driveMotorReversed = driveMotorReversed;
        this.angleMotorReversed = angleMotorReversed;
        this.angleEncoderOffset = angleEncoderOffset;
    }
}
