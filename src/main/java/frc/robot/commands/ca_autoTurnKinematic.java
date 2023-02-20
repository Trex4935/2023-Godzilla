// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoMovementConstraints;
import frc.robot.subsystems.Drivetrain;

public class ca_autoTurnKinematic extends CommandBase {
  /** Creates a new ca_autoTurnKinematic. */
  Timer timer;
  Drivetrain dt;
  Double eAngle;

  public ca_autoTurnKinematic(Drivetrain drivetrain, Double endAngle) {
    timer = new Timer();
    dt = drivetrain;
    eAngle = endAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();

    // System.out.println("Time: "+ timer.get() + " Velocity: " + 0 + " Omega: " + 0 +     " Angle: " +  Math.toDegrees(dt.zSimAngle) + " AngleTarget: " +  eAngle + " LeftSpeed: " + 0 + " RightSpeed: " + 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Double chassisSpeed = 0.0;
    Double comega = 0.0;
    
    // Gets which direction we are turning
    comega = dt.getOmega( Math.toDegrees(dt.zSimAngle),eAngle);
     // Constant for now

    Double leftSpeed = dt.getLeftSpeedKin(chassisSpeed, comega);
    Double rightSpeed = dt.getRightpeedKin(chassisSpeed, comega);

    dt.driveWithController(leftSpeed, rightSpeed);
    dt.simulateGyro(leftSpeed, rightSpeed, timer);
    Double error = eAngle.doubleValue() - Math.toDegrees(dt.zSimAngle) ;
    // System.out.println("Time: "+ timer.get() + " Velocity: " + chassisSpeed + " Omega: " + comega +     " Angle: " +  Math.toDegrees(dt.zSimAngle) + " AngleTarget: " +  eAngle + " LeftSpeed: " + leftSpeed + " RightSpeed: " + rightSpeed + " Error: " + error );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return eAngle.doubleValue() - Math.toDegrees(dt.zSimAngle) <= 2.0
        && eAngle.doubleValue() - Math.toDegrees(dt.zSimAngle) >= -2.0 || timer.get() > 10;
  }
}
