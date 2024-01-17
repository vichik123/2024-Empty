// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private final Shooter shooter = new Shooter();
    private final CommandXboxController controller = new CommandXboxController(0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        SmartDashboard.putNumber("Top Velocity", 0);
        SmartDashboard.putNumber("Bottom Velocity", 0);

        // Configure the trigger bindings
        configureBindings();
    }

    private void configureBindings() {
        controller
                .rightTrigger(0.1)
                .whileTrue(shooter.runShooter(
                        () -> SmartDashboard.getNumber("Top Velocity", 0),
                        () -> SmartDashboard.getNumber("Bottom Velocity", 0)
                ))
                .onFalse(shooter.runShooter(() -> 0));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
