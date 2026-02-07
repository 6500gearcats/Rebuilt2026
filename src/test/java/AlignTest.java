import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;
import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;

public class AlignTest {
    @Test
    public void getAlignRate() {
        double result = RobotContainer.AlignRateMath(4.007866, 4.3769 - 1, 7.4584, 2.3373, new Rotation2d(Math.toRadians(9.6951)));
        System.out.println(result);
        assertEquals(153.5375, result/ 0.01, 0.1);
    }
    @Test
    public void getAlignRate180() {
        double result = RobotContainer.AlignRateMath(4.007866, 4.3769, 7.4584, 4.3769, new Rotation2d(Math.toRadians(180)));
        System.out.println(result);
        assertEquals(0, result/ 0.01, 0.1);
    }

    @Test
    public void getAlignRate0() {
        double result = RobotContainer.AlignRateMath(4.007866, 4.3769, 2.4584, 4.3769, new Rotation2d(Math.toRadians(0)));
        System.out.println(result);
        assertEquals(0, result/ 0.01, 0.1);
    }

    @Test
    public void getAlignRateN180() {
        double result = RobotContainer.AlignRateMath(4.007866, 4.3769, 7.4584, 4.3769, new Rotation2d(Math.toRadians(-180)));
        System.out.println(result);
        assertEquals(0, result/ 0.01, 0.1);
    }
    
    
}
