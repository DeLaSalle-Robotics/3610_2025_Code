import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.SimDevice;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.intakeState;

public class TestIntake {
    IntakeSubsystem m_intake;
    SimDevice simSensor;
    SimDevice simBackSensor;

    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0);
        m_intake = new IntakeSubsystem();

    }

    @SuppressWarnings("PMD.SignatureDeclareThrowsException")
    @AfterEach
    void shutdown() throws Exception {
        m_intake.close();
    }

    @Test
    void hasCoralBroken() {
        m_intake.setSimSensors(true,true);
        m_intake.setIntakeState(intakeState.LoadingBrokenBackSensor);
        m_intake.periodic();
        assertTrue(intakeState.HasCoral == m_intake.getIntatkeState());
    }
 
    @Test
    void coralEnterBroken() {
        m_intake.setSimSensors(false,true);
        m_intake.setIntakeState(intakeState.LoadingBrokenBackSensor);
        m_intake.periodic();
        assertTrue(intakeState.Empty == m_intake.getIntatkeState());
    }

    @Test
    void coralForwardBroken() {
        m_intake.setSimSensors(true,false);
        m_intake.setIntakeState(intakeState.LoadingBrokenBackSensor);
        m_intake.periodic();
        assertTrue(intakeState.HasCoral == m_intake.getIntatkeState());
    }

    @Test
    void TestEmptyBroken(){
        m_intake.setSimSensors(false,false);
        m_intake.setIntakeState(intakeState.LoadingBrokenBackSensor);
        m_intake.periodic();
        assertTrue(intakeState.Empty == m_intake.getIntatkeState());
    }

    @Test
    void hasCoral() {
        m_intake.setSimSensors(true,true);
        m_intake.periodic();
        assertTrue(intakeState.HasCoral == m_intake.getIntatkeState());
    }
 
    @Test
    void coralEnter() {
        m_intake.setSimSensors(false,true);
        m_intake.periodic();
        assertTrue(intakeState.LoadingEntry == m_intake.getIntatkeState());
    }

    @Test
    void coralForward() {
        m_intake.setSimSensors(true,false);
        m_intake.periodic();
        assertTrue(intakeState.LoadingForward == m_intake.getIntatkeState());
    }

    @Test
    void TestEmpty(){
        m_intake.setSimSensors(false,false);
        m_intake.periodic();
        assertTrue(intakeState.Empty == m_intake.getIntatkeState());
    }
}
