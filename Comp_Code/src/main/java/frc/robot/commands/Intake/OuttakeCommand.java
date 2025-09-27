// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.IntakeSubsystem.intakeState;
import frc.robot.subsystems.LedSubsystem.LedState;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class OuttakeCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final IntakeSubsystem intake;
    private final LedSubsystem led;
    private double speed;
    private intakeState startState;
    private boolean backSensorChanged = false;
    private boolean frontSensorStartState;
    private boolean backSensorStartState;
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public OuttakeCommand(IntakeSubsystem subsystem, 
                        LedSubsystem m_ledSubSystem) {
                            
        this.intake = subsystem;
        this.led = m_ledSubSystem;
        this.speed = 0.8;
        this.startState = intakeState.Empty;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.led.setState(LedState.RunningIntake);
        frontSensorStartState = intake.frontDetectCoral();
        backSensorStartState = intake.backDetectCoral();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intake.startIntake(speed);
        if (intake.backDetectCoral() != backSensorStartState){
            backSensorChanged = true;
        }
        if(!backSensorChanged && intake.frontDetectCoral() != frontSensorStartState){
            intake.setIntakeState(intakeState.LoadingBrokenBackSensor);
        }
    }

    // Called once when the command isFinished is true.
    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
        led.setState(LedState.Idle);
    }

    // Returns true when the sensor status changes.
    // Goal is to prevent premature coral ejection.
    @Override
    public boolean isFinished() {
            return intake.frontDetectCoral() != frontSensorStartState;
    }
}
