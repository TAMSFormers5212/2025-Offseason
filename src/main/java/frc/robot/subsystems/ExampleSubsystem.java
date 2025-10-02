// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ExampleSubsystem extends SubsystemBase {
    public SparkMax exampleMotor = new SparkMax(1, SparkLowLevel.MotorType.kBrushless);


    /**
     * Creates a new ExampleSubsystem.
     */
    public ExampleSubsystem() {
        exampleMotor.configure(new SparkMaxConfig()
                        .smartCurrentLimit(50)
                        .idleMode(SparkBaseConfig.IdleMode.kBrake),
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
    }

    public Command startMotor() {
        return runOnce(() -> {
            exampleMotor.set(0.5);
        });
    }

    public Command stopMotor() {
        return runOnce(() -> {
            exampleMotor.set(0);
        });
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
