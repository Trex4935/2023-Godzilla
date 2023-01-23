// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ca_driveAutoSquare extends SequentialCommandGroup {
  /** Creates a new cm_driveAutoSquare. */
  public ca_driveAutoSquare(Drivetrain drivetrain, Trajectory trajectory) {
    double squareAngle = 95.0;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ca_autoTrajectory(drivetrain,trajectory),
    new ca_autoTurnKinematic(drivetrain, 0.0, squareAngle),
    new ca_autoTrajectory(drivetrain,trajectory),
    new ca_autoTurnKinematic(drivetrain, squareAngle, squareAngle*2),
    new ca_autoTrajectory(drivetrain,trajectory),
    new ca_autoTurnKinematic(drivetrain, squareAngle*2, squareAngle*3),
    new ca_autoTrajectory(drivetrain,trajectory),
    new ca_autoTurnKinematic(drivetrain, squareAngle*3, squareAngle*4)
    );
  }
}
