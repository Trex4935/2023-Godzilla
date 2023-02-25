// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoArmAction;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;

public class cm_GripperOpen extends CommandBase {

  private final Gripper gripper;
  private final Timer timer;

  /** Creates a new cm_GripperClose. */
  public cm_GripperOpen(Gripper grip) {
    gripper = grip;
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(grip);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gripper.gripOpen();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 1.0;
  }
}
