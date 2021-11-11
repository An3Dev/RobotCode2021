// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

/** Add your docs here. */
public class Constants {
    
    // PLACEHOLDERS: Use FRC Characterization Tool for values.
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    // Example value only - as above, this must be tuned for your drive.
    public static final double kPDriveVel = 8.5;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds.
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    // Track Robot Velocity, pass in horizontal distance between wheels.
    public static final double kTrackwidthMeters = Units.inchesToMeters(28);
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
}
