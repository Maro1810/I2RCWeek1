// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class PIDTurnCCW extends CommandBase {
  DriveTrain dt;
  double setPointAngle;
  PIDController pid = new PIDController(0.3/90, 0, 0);
  int motorSign;

  
  /** Creates a new PIDTurnCCW. */
  public PIDTurnCCW(DriveTrain dt, double setPointAngle) {
    this.dt = dt;
    this.setPointAngle = setPointAngle;
    addRequirements(dt);
    pid.setTolerance(5.0);
    if(setPointAngle > 0 ) { //counterclockwise turn
      motorSign = 1;
    }
    else {
      motorSign = -1; //Clockwise turn

    }
    }
    // Use addRequirements() here to declare subsystem dependencies.
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     dt.resetNavx();
     dt.tankDrive(0, 0);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = pid.calculate(dt.getAngle(), setPointAngle);
    dt.tankDrive(-output*motorSign, output*motorSign);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.tankDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
