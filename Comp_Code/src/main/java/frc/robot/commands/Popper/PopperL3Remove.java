// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Popper;

import frc.robot.Constants;
import frc.robot.subsystems.Popper;
import frc.robot.subsystems.Popper.popperState;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class PopperL3Remove extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Popper m_popper;
    /**
     * Goal of this Command is to control the Popper arm movement 
     * down through the Algea position to a defined arc. Meant to
     * be part of a sequential command group that first moves the 
     * Popper armm just above the Algea
     *
     * @param subsystem The subsystem used by this command.
     */
    public PopperL3Remove(Popper m_popper) {
        this.m_popper = m_popper;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_popper);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (Constants.Verbose) {SmartDashboard.putString("L3 Popper Remove","Started");}
        m_popper.setPopperState(popperState.L3Plus);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Constants.Verbose) {SmartDashboard.putNumber("PoperTargetPostiton", m_popper.getGoalPosition());
        SmartDashboard.putString("L3 Popper Remove","Running");}
        m_popper.PopperSpinL3();
        m_popper.updatePosition();
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //m_popper.setPopperState(popperState.Start);
        m_popper.PopperSpinStop();
        if (Constants.Verbose) {SmartDashboard.putString("L3 Popper Remove","Ended");}
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
