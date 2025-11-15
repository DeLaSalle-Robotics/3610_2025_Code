package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class AdjustCoral extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private IntakeSubsystem intake;
    private double speed;
    /**
     * This command is run after the intake command to slowly pull the coral
     * back into the intake. 
     */
    public AdjustCoral(IntakeSubsystem subsystem) {
        this.intake = subsystem;
        this.speed = Constants.Intake.speed;
        addRequirements(subsystem);
                        }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (intake.backSensorTrust()){
            // When the front sensor is positive and the back is negative, back up the coral at half speed
            if (intake.frontDetectCoral() && !intake.backDetectCoral()){
                intake.startIntake(-speed/4);
            }
            // When both sensors detect coral stop moving. Intake subsystem will also detect and change state.
            if (intake.frontDetectCoral() && intake.backDetectCoral()){
                intake.stopIntake();
            }

        } else {intake.stopIntake();}
    }

    // Called once when the command isFinished is true.
    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
    }

    @Override
    public boolean isFinished() {
            
        if(intake.backSensorTrust()){
            return (intake.frontDetectCoral() && intake.backDetectCoral());
        } else {
            return (intake.frontDetectCoral());
        }
    }   
}
