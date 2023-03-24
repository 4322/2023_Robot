import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.*;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Limelight;
import frc.utility.OrangeMath;

public class LimelightTest {

  Limelight testLime;

  @BeforeEach
  public void setup() {
    testLime = new Limelight("Test Limelight", 0, OrangeMath.inchesToMeters(26.125), 0, 0, 0, false, true);
    return;
  }

  @AfterEach
  public void shutdown() {
    testLime = null;
    return;
  }

  @Test
  public void testGetTargetHeight_MiddleTapeHeight() {
    assertEquals(testLime.getTargetHeight(0, -10),
        LimelightConstants.middleTapeHeight);
  }

  @Test
  public void testGetTargetHeight_HighTapeHeight() {
    assertEquals(testLime.getTargetHeight(0, 10),
        LimelightConstants.highTapeHeight);
  }

  @Test
  public void testGetTargetHeight_GridHeight() {
    assertEquals(testLime.getTargetHeight(1, 0),
        LimelightConstants.gridAprilTagHeight);
  }

  @Test
  public void testGetTargetHeight_SubstationHeight() {
    assertEquals(testLime.getTargetHeight(2, 0),
        LimelightConstants.singleSubstationAprilTagHeight);
  }

  @Test
  public void testGetTargetHeight_Invalid() {
    assertEquals(testLime.getTargetHeight(-1, 0), -1);
  }

  @Test
  public void testCalcTargetPos_RealAnswer() {
    Translation2d expectedAnswer = new Translation2d(2.322815251, 0.8454356111);
    Translation2d calcAnswer = testLime.calcTargetPos(OrangeMath.inchesToMeters(10), -10, 20);
    assertTrue(OrangeMath.equalToTwoDecimal(expectedAnswer.getX(), calcAnswer.getX()));
    assertTrue(OrangeMath.equalToTwoDecimal(expectedAnswer.getY(), calcAnswer.getY()));
  }

  @Test
  public void testCalcTargetPos_RealAnswerNegative() {
    Translation2d expectedAnswer = new Translation2d(2.322815251, -0.8454356111);
    Translation2d calcAnswer = testLime.calcTargetPos(OrangeMath.inchesToMeters(10), -10, -20);
    assertTrue(OrangeMath.equalToTwoDecimal(expectedAnswer.getX(), calcAnswer.getX()));
    assertTrue(OrangeMath.equalToTwoDecimal(expectedAnswer.getY(), calcAnswer.getY()));
  }

  @Test
  public void testCalcTargetPos_Invalid() {
    Translation2d expectedAnswer = new Translation2d(0, 0.8454356111);
    Translation2d calcAnswer = testLime.calcTargetPos(OrangeMath.inchesToMeters(10), 10, -20);
    assertTrue(OrangeMath.equalToTwoDecimal(expectedAnswer.getX(), calcAnswer.getX()));
    assertTrue(OrangeMath.equalToTwoDecimal(expectedAnswer.getY(), calcAnswer.getY()));
  }

  @Test
  public void testCalcTargetPosWithHeight_RealAnswer() {
    testLime = new Limelight("Test Limelight", 0, 0.5, 0, 0, 0, false, true);
    Translation2d expectedAnswer = new Translation2d(1.395135328, 0.5077877322);
    Translation2d calcAnswer = testLime.calcTargetPos(OrangeMath.inchesToMeters(10), -10, 20);
    assertTrue(OrangeMath.equalToTwoDecimal(expectedAnswer.getX(), calcAnswer.getX()));
    assertTrue(OrangeMath.equalToTwoDecimal(expectedAnswer.getY(), calcAnswer.getY()));
  }

  @Test
  public void testCalcTargetPosWithAngle_RealAnswer() {
    testLime = new Limelight("Test Limelight", 0, OrangeMath.inchesToMeters(26.125), 20, 0, 0, false, true);
    Translation2d expectedAnswer = new Translation2d(2.322815251, 0.8454356111);
    Translation2d calcAnswer = testLime.calcTargetPos(OrangeMath.inchesToMeters(10), -30, 20);
    assertTrue(OrangeMath.equalToTwoDecimal(expectedAnswer.getX(), calcAnswer.getX()));
    assertTrue(OrangeMath.equalToTwoDecimal(expectedAnswer.getY(), calcAnswer.getY()));
  }

  @Test
  public void testCalcTargetPosWithOffset_RealAnswer() {
    testLime = new Limelight("Test Limelight", 0, OrangeMath.inchesToMeters(26.125), 0, -1, -1, false, true);
    Translation2d expectedAnswer = new Translation2d(1.322815251, 0.8454356111 - 1);
    Translation2d calcAnswer = testLime.calcTargetPos(OrangeMath.inchesToMeters(10), -10, 20);
    assertTrue(OrangeMath.equalToTwoDecimal(expectedAnswer.getX(), calcAnswer.getX()));
    assertTrue(OrangeMath.equalToTwoDecimal(expectedAnswer.getY(), calcAnswer.getY()));
  }

  @Test
  public void testCalcTargetPosFacingBackward_RealAnswer() {
    testLime = new Limelight("Test Limelight", 0, OrangeMath.inchesToMeters(26.125), 0, 0, 0, true, true);
    Translation2d expectedAnswer = new Translation2d(-2.322815251, -0.8454356111);
    Translation2d calcAnswer = testLime.calcTargetPos(OrangeMath.inchesToMeters(10), -10, 20);
    assertTrue(OrangeMath.equalToTwoDecimal(expectedAnswer.getX(), calcAnswer.getX()));
    assertTrue(OrangeMath.equalToTwoDecimal(expectedAnswer.getY(), calcAnswer.getY()));
  }

  @Test
  public void testCalcTargetPosFacingBackwardWithOffset_RealAnswer() {
    testLime = new Limelight("Test Limelight", 0, OrangeMath.inchesToMeters(26.125), 0, -1, 0, true, true);
    Translation2d expectedAnswer = new Translation2d(-1.322815251, -0.8454356111);
    Translation2d calcAnswer = testLime.calcTargetPos(OrangeMath.inchesToMeters(10), -10, 20);
    assertTrue(OrangeMath.equalToTwoDecimal(expectedAnswer.getX(), calcAnswer.getX()));
    assertTrue(OrangeMath.equalToTwoDecimal(expectedAnswer.getY(), calcAnswer.getY()));
  }

}
