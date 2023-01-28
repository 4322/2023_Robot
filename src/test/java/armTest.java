import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.*;
import frc.robot.subsystems.Arm;
public class armTest {
  private Arm arm;
  @BeforeEach
  public void setup()
  {
    arm = new Arm();
  }
  @Test
  public void testMoveTo1500EqualsFalse()
  {
    assertFalse(arm.rotateToPosition(1500));
  }
  public void testMoveTo900EqualsTrue()
  {
    assertTrue(arm.rotateToPosition(900));
  }
}
