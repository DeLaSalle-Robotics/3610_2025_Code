// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LedSubsystem extends SubsystemBase {
    public enum LedState {
        Idle,
        RunningIntake,
        HasCoral,
        PreparingToShoot,
        ReadyToShoot,
        Pride,
    }

    // Must be a PWM header, not MXP or DIO
    private final AddressableLED led;
    private final AddressableLEDSim ledSim;
    private final AddressableLEDBuffer ledBuffer;

    private LedState currentState;
    private double ledRainbowOffset = 0;



    /** Creates a new ExampleSubsystem. */
    public LedSubsystem() {
        super();

        setState(LedState.Pride);

        led = new AddressableLED(Constants.Led.ledPwm);
        led.setLength(Constants.Led.numLeds);
        ledSim = new AddressableLEDSim(led);
        ledSim.setLength(Constants.Led.numLeds);
        ledBuffer = new AddressableLEDBuffer(Constants.Led.numLeds);

        led.setData(ledBuffer);
        led.start();
    }

    public String ledStateToString(LedState state) {
        String out = null;
        switch(state) {
            case Pride -> out = "Rainbow";
            default -> {}
        }
        if(out == null) {
            out = state.toString();
        }
        return out;
    }

    void setAllLeds(Color color) {
        setLedRange(0, Constants.Led.numLeds, color);
    }
    /**
     * Sets all LEDs with indexes in a given range
     * @param start first LED index
     * @param end exclusive, last LED index + 1
     */
    void setLedRange(int start, int end, Color color) {
        for(int i = start;i < end;i++) {
            ledBuffer.setLED(i, color);
        }
    }
    void rainbow() {
        for(int i = 0;i < Constants.Led.numLeds;i++) {
            int hue = (int)((double)(i << 8) / (double)Constants.Led.numLeds + ledRainbowOffset) % 256;
            ledBuffer.setHSV(i, hue, 255, 200);
        }
        ledRainbowOffset = (ledRainbowOffset + Constants.Led.rainbowShiftSpeed) % 256.0;
    }

    public void updateLeds(LedState state) {
        switch(state) {
            case Pride -> {
                rainbow();
            }
            case HasCoral -> {
                this.setAllLeds(Color.kGreen);
            }
            case Idle -> {
                rainbow();
            }
            default -> {
                setAllLeds(Color.kBlack);
            }
        }
        led.setData(ledBuffer);
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    /* one-time action goes here */
                });
    }

    public LedState getState(){
        return currentState;
    }

    public void setState(LedState state) {
        currentState = state;
        SmartDashboard.putString("Led State", ledStateToString(state));
    }

    @Override
    public void periodic() {
        updateLeds(currentState);
        SmartDashboard.putString("LED State", ledStateToString(this.getState()));
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
