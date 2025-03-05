// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LedSubsystem.LedState;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class IntakeCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final IntakeSubsystem intake;
    private final LedSubsystem led;
    private boolean stopOnSensor;
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public IntakeCommand(IntakeSubsystem subsystem, LedSubsystem m_ledSubSystem, boolean stopOnSensor) {
        this.intake = subsystem;
        this.led = m_ledSubSystem;
        this.stopOnSensor = stopOnSensor;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intake.startIntake();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
        LedState ledState = led.getState();
        if (ledState == LedState.HasCoral){
            led.setState(ledState.Idle);
        } else {
            led.setState(ledState.HasCoral);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (intake.detectCoral() && stopOnSensor);
    }
}
