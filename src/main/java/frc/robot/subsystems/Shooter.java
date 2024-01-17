// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {

    private final CANSparkMax topRoller = new CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax bottomRoller = new CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final SimpleMotorFeedforward topFeedForward = new SimpleMotorFeedforward(0.02, 0, 0);
    private final PIDController topController = new PIDController(0, 0, 0);
    private final SimpleMotorFeedforward bottomFeedForward = new SimpleMotorFeedforward(0.02, 0, 0);
    private final PIDController bottomController = new PIDController(0, 0, 0);

    /**
     * Creates a new ExampleSubsystem.
     */
    public Shooter() {
        topRoller.restoreFactoryDefaults();
        topRoller.setSmartCurrentLimit(50);
        topRoller.burnFlash();

        bottomRoller.restoreFactoryDefaults();
        bottomRoller.setSmartCurrentLimit(50);
        bottomRoller.burnFlash();
    }

    public void set(double topVelocity, double bottomVelocity) {
        topRoller.set(
                topFeedForward.calculate(topVelocity) +
                        topController.calculate(
                                topRoller.getEncoder().getVelocity(), topVelocity)
        );
        bottomRoller.set(
                bottomFeedForward.calculate(bottomVelocity) +
                        bottomController.calculate(
                                bottomRoller.getEncoder().getVelocity(), bottomVelocity)
        );
    }

    public Command runShooter(DoubleSupplier topVelocity, DoubleSupplier bottomVelocity) {
        return run(
                () -> set(topVelocity.getAsDouble(), bottomVelocity.getAsDouble()));
    }

    public Command runShooter(DoubleSupplier velocity) {
        return runShooter(velocity, () -> -velocity.getAsDouble());
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
