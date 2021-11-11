// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveCommand extends CommandBase {

  public DriveTrain driveTrain;
  public DoubleSupplier speed;
  public DoubleSupplier rotation;
  public DoubleSupplier modifier;
  public DoubleSupplier shootBall;

  public DoubleSupplier leftStick;
  public DoubleSupplier rightStick;
  public BooleanSupplier shoot;
  

  // /** Creates a new DriveCommand. */
  // public DriveCommand(DriveTrain subsystem, DoubleSupplier speed, DoubleSupplier rotation, DoubleSupplier rightTrigger) {
  //   // Use addRequirements() here to declare subsystem dependencies.
  //   driveTrain = subsystem;
  //   this.speed = speed;
  //   this.rotation = rotation;
  //   //this.modifier = modifier;
  //   this.shootBall = rightTrigger;
  //   addRequirements(driveTrain);
  // }

  /** Creates a new DriveCommand. */
  public DriveCommand(DriveTrain subsystem, DoubleSupplier leftStick, DoubleSupplier rightStick, BooleanSupplier shoot) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = subsystem;
    this.leftStick = leftStick;
    this.rightStick = rightStick;
    //this.modifier = modifier;
    this.shoot = shoot;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.arcadeDrive(0.0, 0.0);
    //driveTrain.tankDriveVolts(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    //System.out.println("L: " + leftStick.getAsDouble() + " R: " + rightStick.getAsDouble());
    System.out.println(shoot.getAsBoolean());
    //driveTrain.tankDriveVolts(speed.getAsDouble(), rotation.getAsDouble());
    driveTrain.tankDriveVolts(leftStick.getAsDouble() * (69 / 10.0), rightStick.getAsDouble() * (69 / 10.0));
    if (shoot.getAsBoolean())
    {
      driveTrain.shoot(-1);
    } 
    else
    {
      driveTrain.shoot(0);
    } 

    //driveTrain.arcadeDrive(-speed.getAsDouble(), rotation.getAsDouble() * 0.8);
    //driveTrain.shoot(shootBall.getAsDouble());

    //System.out.println("Right trigger: " + shootBall.getAsDouble());
    //driveTrain.leftMotor(speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
