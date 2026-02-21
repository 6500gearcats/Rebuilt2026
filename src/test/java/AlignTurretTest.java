import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.commands.AlignTurretToHub;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;

public class AlignTurretTest {
    @Test
    public void getAlignTurnStraight() {
        double result = AlignTurretToHub.calculateAlignmentConvertedDegrees(new Translation2d(0, 0),
                new Translation2d(1, 0), new Rotation2d(0), 0);
        assertEquals(0, result, 0.01);
    }

    @Test
    public void getAlignTurnBack90() {
        double result = AlignTurretToHub.calculateAlignmentConvertedDegrees(new Translation2d(0, 0),
                new Translation2d(0, 1), new Rotation2d(0), 0);
        assertEquals(-90, result, 0.01);
    }

    @Test
    public void getAlignTurretFront90() {
        double result = AlignTurretToHub.calculateAlignmentConvertedDegrees(new Translation2d(0, 0),
                new Translation2d(0, -1), new Rotation2d(0), 0);
        assertEquals(90, result, 0.01);
    }

    @Test
    public void getAlignTurreBack45() {
        double result = AlignTurretToHub.calculateAlignmentConvertedDegrees(new Translation2d(0, 0),
                new Translation2d(1, 1), new Rotation2d(0), 0);
        assertEquals(-45, result, 0.01);
    }
     @Test
    public void getAlignTurret() {
        double result = AlignTurretToHub.calculateAlignmentConvertedDegrees(new Translation2d(0, 0),
                new Translation2d(0, 1), new Rotation2d(100), 50);
        assertEquals(40, result, 0.01);
    }
}
