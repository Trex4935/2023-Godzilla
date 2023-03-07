// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoDriveActions;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ca_driveAutoSquare extends SequentialCommandGroup {
  /** Creates a new cm_driveAutoSquare. */
  public ca_driveAutoSquare(Drivetrain drivetrain) {
    double squareAngle = 45.0;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    addCommands(
    // Replace new ca_autoTrajectory(drivetrain,trajectory, end),
    new ca_autoTurnKinematicGyro(drivetrain, 0.0, squareAngle),
    // Replace new ca_autoTrajectory(drivetrain,trajectory, end),
    new ca_autoTurnKinematicGyro(drivetrain, squareAngle, squareAngle*2),
    // Replace new ca_autoTrajectory(drivetrain,trajectory, end),
    new ca_autoTurnKinematicGyro(drivetrain, squareAngle*2, squareAngle*3),
    // Replace new ca_autoTrajectory(drivetrain,trajectory, end),
    new ca_autoTurnKinematicGyro(drivetrain, squareAngle*3, squareAngle*4)
    );
  }
}
