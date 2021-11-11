// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class DriveTrain extends SubsystemBase {
    // Motor controllers.
    private final CANSparkMax rearLeft;
    private final CANSparkMax frontLeft;
    private final CANSparkMax rearRight;
    private final CANSparkMax frontRight;
    private final DifferentialDrive drive;
    //private final DifferentialDriveWheelSpeeds shootingDrive;

    private final CANSparkMax intakeMotor;
    private final CANSparkMax topLeftShootMotor;
    private final CANSparkMax topRightShootMotor;

    // Gyro.
    private final AHRS gyro;

    // Kinematics and Odometry.
    DifferentialDriveKinematics kinematics;
    DifferentialDriveOdometry odometry;
    // Constructor, initialize motor controllers and groups.

    public DriveTrain() {
        // Initialize motor controllers.
        rearLeft = new CANSparkMax(1, MotorType.kBrushless); 
        frontLeft = new CANSparkMax(3, MotorType.kBrushless); 
        rearRight = new CANSparkMax(2, MotorType.kBrushless); 
        frontRight = new CANSparkMax(4, MotorType.kBrushless); 

        intakeMotor = new CANSparkMax(5, MotorType.kBrushless);
        topLeftShootMotor = new CANSparkMax(8, MotorType.kBrushed);
        topRightShootMotor = new CANSparkMax(7, MotorType.kBrushed);

        

        // Synchronize left and right motors.
        frontLeft.follow(rearLeft, false); 
        frontRight.follow(rearRight, false);
        rearLeft.setInverted(false);
        rearRight.setInverted(false);

        //intakeMotor.follow(topLeftShootMotor);

        // Initialize drive configuration.
        drive = new DifferentialDrive(rearLeft, rearRight);
        //shootingDrive = new DifferentialDriveWheelSpeeds();
        

        gyro = new AHRS(SPI.Port.kMXP);
        odometry = new DifferentialDriveOdometry(gyro.getRotation2d()); // Track Robot Position

    }

    // Basic single joystick drive configuration.
    public void arcadeDrive(double speed, double rotation) {
        //System.out.println(speed);
        drive.arcadeDrive(speed, rotation);
    }

    public void shoot(double input)
    {
        topLeftShootMotor.setVoltage(-input * 8);
        topRightShootMotor.setVoltage(input * 8);
        intakeMotor.setVoltage(-input * 5);
    }

    // Drive forwards.
    public void driveForward(double speed) {
        drive.arcadeDrive(speed, 0.0);
    }

    // Tank drive control with voltage
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        rearLeft.setVoltage(leftVolts);
        rearRight.setVoltage(-rightVolts);
        drive.feed();
    }

    public void leftMotor(double voltage)
    {
        rearLeft.setVoltage(voltage);
        drive.feed();
    }

    // Stop the motors.
    public void stopMotors() {
        rearRight.set(0.0);
        rearLeft.set(0.0);
    }

    // Get the current robot angle.
    public Rotation2d getHeading() {
        // Gyros return positive values as the robot turns clockwise.
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }

    // Update odometry (robot position).
    @Override
    public void periodic() {
        odometry.update(getHeading(), rearLeft.getEncoder().getPosition(), rearRight.getEncoder().getPosition());
    }

    // Return the robot's current pose.
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }
    
        
    // RPM of Motor, converted to m/s. Divide by gear ratio. Use radius of wheels.
    // Returns wheel speeds.
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(rearLeft.getEncoder().getVelocity() / 7.29 * 2 * Math.PI * Units.inchesToMeters(3.0) / 60, 
        rearRight.getEncoder().getVelocity() / 7.29 * 2 * Math.PI * Units.inchesToMeters(3.0) / 60);
    }

    // Resets odometry to specific pose.
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, gyro.getRotation2d());
    }

    // Reset encoders.
    public void resetEncoders() {
        double oldRightPose = rearRight.getEncoder().getPosition();
        double newRightPose = rearRight.getEncoder().getPosition();
        rearRight.getEncoder().setPosition(newRightPose - oldRightPose);

        double oldLeftPose = rearLeft.getEncoder().getPosition();
        double newLeftPose = rearLeft.getEncoder().getPosition();
        rearLeft.getEncoder().setPosition(newLeftPose - oldLeftPose);
    }

    // Get average distance of two encoders.
    public double getAverageEncoderDistance() {
        return (rearLeft.getEncoder().getPosition() + rearRight.getEncoder().getPosition()) / 2.0;
    }

    // For setting max motor output. Useful for scaling speed.
    public void setMaxOutput(double maxOutput) {
        drive.setMaxOutput(maxOutput);
    }

    // Zeroes robot heading (angle).
    public void zeroHeading() {
        gyro.reset();
    }

    // Get the turn rate of the robot in degrees per second.
    public double getTurnRate() {
        return -gyro.getRate();
    }
}
