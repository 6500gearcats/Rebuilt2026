import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;
import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;

public class AlignTest {
    @Test
    public void getAlignRate180() {
        double result = RobotContainer.getAlignRate(new Pose2d(4.007866, 4.3769, new Rotation2d()) , new Pose2d( 7.4584, 4.3769, new Rotation2d()) ,180);
        System.out.println(result);
        assertEquals(0, result/ 0.01, 0.1);
    }

    @Test
    public void getAlignRateTest180() {
        double result = RobotContainer.getAlignRate(new Pose2d(0, 1, new Rotation2d()) , new Pose2d( 0 , 0, new Rotation2d()) ,0);
        System.out.println(result);
        assertEquals(1.8, result, 0.1);
    }

    @Test
    public void getAlignRateTest0() {
        double result = RobotContainer.getAlignRate(new Pose2d(0, 0, new Rotation2d()) , new Pose2d( 0 , 1, new Rotation2d()) ,0);
        System.out.println(result);
        assertEquals(0.0, result/ 0.01, 0.1);
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
