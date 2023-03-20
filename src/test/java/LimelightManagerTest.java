import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.*;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.LimelightManager;
import frc.robot.Constants.LimelightConstants;
import frc.utility.OrangeMath;

public class LimelightManagerTest {

  @BeforeEach
  public void setup() {
    return;
  }

  @AfterEach
  public void shutdown() {
    return;
  }

  @Test
  public void testGetTargetHeight_MiddleTapeHeight() {
    assertEquals(LimelightManager.getTargetHeight(0, -10),
        LimelightConstants.middleTapeHeight);
  }

  @Test
  public void testGetTargetHeight_HighTapeHeight() {
    assertEquals(LimelightManager.getTargetHeight(0, 10),
        LimelightConstants.highTapeHeight);
  }

  @Test
  public void testGetTargetHeight_GridHeight() {
    assertEquals(LimelightManager.getTargetHeight(1, 0),
        LimelightConstants.gridAprilTagHeight);
  }

  @Test
  public void testGetTargetHeight_SubstationHeight() {
    assertEquals(LimelightManager.getTargetHeight(2, 0),
        LimelightConstants.singleSubstationAprilTagHeight);
  }

  @Test
  public void testGetTargetHeight_Invalid() {
    assertEquals(LimelightManager.getTargetHeight(-1, 0), -1);
  }

  @Test
  public void testCalcTargetPos_RealAnswer() {
    Translation2d expectedAnswer = new Translation2d(2.322815251, 6.381882452);
    Translation2d calcAnswer = LimelightManager.calcTargetPos(OrangeMath.inchesToMeters(10), -10, 20);
    assertTrue(OrangeMath.equalToTwoDecimal(expectedAnswer.getX(), calcAnswer.getX()));
    assertTrue(OrangeMath.equalToTwoDecimal(expectedAnswer.getY(), calcAnswer.getY()));
  }

  @Test
  public void testCalcTargetPos_RealAnswerNegative() {
    Translation2d expectedAnswer = new Translation2d(2.322815251, -6.381882452);
    Translation2d calcAnswer = LimelightManager.calcTargetPos(OrangeMath.inchesToMeters(10), -10, -20);
    assertTrue(OrangeMath.equalToTwoDecimal(expectedAnswer.getX(), calcAnswer.getX()));
    assertTrue(OrangeMath.equalToTwoDecimal(expectedAnswer.getY(), calcAnswer.getY()));
  }

  @Test
  public void testCalcTargetPos_Invalid() {
    Translation2d expectedAnswer = new Translation2d(0, 0);
    Translation2d calcAnswer = LimelightManager.calcTargetPos(OrangeMath.inchesToMeters(10), 10, -20);
    assertTrue(OrangeMath.equalToTwoDecimal(expectedAnswer.getX(), calcAnswer.getX()));
    assertTrue(OrangeMath.equalToTwoDecimal(expectedAnswer.getY(), calcAnswer.getY()));
  }

}
