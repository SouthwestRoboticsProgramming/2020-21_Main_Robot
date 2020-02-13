package frc.robot.subsystems;

import static org.junit.Assert.assertEquals;
import org.junit.Test;
import static org.mockito.Mockito.*;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.BallSubsystem;


public class BallSubsystemTests {

    @Test
    public void ballCounterTest() {
        //Arrange
        DigitalInput lowerBallSensor = mock(DigitalInput.class);

        //Act
        // BallSubsystem ballSubsystem = new BallSubsystem();

        //Assert
        assertEquals("Error", 2, 1+1);
    }
}