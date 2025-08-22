// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.IntakeSubsystem.intakeState;
import frc.robot.subsystems.LedSubsystem.LedState;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class IntakeCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final IntakeSubsystem intake;
    private final LedSubsystem led;
    private DoubleSupplier moveback;
    private double speed;
    private boolean intakeSensorStatus;
    private intakeState startState;
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public IntakeCommand(IntakeSubsystem subsystem, 
                        LedSubsystem m_ledSubSystem, 
                        DoubleSupplier moveback) {
        this.intake = subsystem;
        this.led = m_ledSubSystem;
        this.speed = 0;
        this.moveback = moveback;
        this.intakeSensorStatus = false;
        this.startState = intakeState.Empty;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.intakeSensorStatus = intake.detectCoral();
        this.startState = intakeState.Loading;
        this.led.setState(LedState.RunningIntake);
        if (moveback.getAsDouble() < -0.5){
            speed = -0.3;
        } else if (moveback.getAsDouble() > 0.5) {
            speed = 0.8;
        } else {
            speed = 0.0;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intake.startIntake(speed);
    }

    // Called once when the command isFinished is true.
    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
        if (intake.getIntatkeState() == intakeState.HasCoral){
            led.setState(LedState.HasCoral);
        } else {
            led.setState(LedState.Idle);
        }
    }

    // Returns true when the sensor status changes.
    // Goal is to prevent premature coral ejection.
    @Override
    public boolean isFinished() {
            
        return (intake.detectCoral() != this.intakeSensorStatus);
    }
}
