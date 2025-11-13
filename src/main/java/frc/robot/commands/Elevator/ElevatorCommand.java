// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import frc.robot.subsystems.ElevatorSubsystem;
import java.util.function.DoubleSupplier;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ElevatorCommand extends Command {
    private final ElevatorSubsystem elevator;
    private DoubleSupplier speed;
    private BooleanPublisher goingUp;
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("elevator");
  
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ElevatorCommand(ElevatorSubsystem subsystem,DoubleSupplier speed) {
        this.elevator = subsystem;
        this.speed = speed;
        goingUp = table.getBooleanTopic("elevator").publish();

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
        if(speed.getAsDouble() > 0){goingUp.set(true);}
        else{goingUp.set(false);}
        elevator.runElevator(speed.getAsDouble());

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        elevator.stopElevator();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
