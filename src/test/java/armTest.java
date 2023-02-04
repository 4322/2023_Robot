import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.*;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.subsystems.Arm;
public class armTest {
  private Arm arm;
  public armTest()
  {
    arm = new Arm();
  }
  @Test
  public void testMoveTo1500EqualsFalse()
  {
    assertFalse(arm.rotateToPosition(1500));
  }
  @Test
  public void testMoveTo1100EqualsFalse()
  {
    assertFalse(arm.rotateToPosition(1100));
  }
}
