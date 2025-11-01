// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import frc.robot.Constants;
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
    private intakeState startState;
    private boolean backSensorChanged = false;
    private boolean frontSensorStartState;
    private boolean backSensorStartState;
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public IntakeCommand(IntakeSubsystem subsystem, 
                        LedSubsystem m_ledSubSystem) {
                            
        this.intake = subsystem;
        this.led = m_ledSubSystem;
        this.speed = Constants.Intake.speed;
        this.startState = intakeState.Empty;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.backSensorStartState = intake.backDetectCoral();
        this.frontSensorStartState = intake.frontDetectCoral();
        this.led.setState(LedState.RunningIntake);


    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //Starts the intake moving forward
        intake.startIntake(speed);

        // Watching the change in the back sensor. Should change once a coral enters the intake.
        if (intake.backDetectCoral() != backSensorStartState){
            backSensorChanged = true;
        }
        // If the back sensor has NOT changed and the front sensor does -> back sensor is broken.
        if(!backSensorChanged && intake.frontDetectCoral() != frontSensorStartState){
            intake.setIntakeState(intakeState.LoadingBrokenBackSensor);
            this.led.setState(LedState.LoadingBrokenBackSensor);
        }

        

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

    @Override
    public boolean isFinished() {
        if(intake.getIntatkeState() == intakeState.LoadingBrokenBackSensor){
            return (intake.frontDetectCoral());
        }
        else{
            return intake.frontDetectCoral() && !intake.backDetectCoral();
        }
        
    }
}
