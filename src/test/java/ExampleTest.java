import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.*;

// unit test naming convention: test<MethodToTest>_desiredState

public class ExampleTest {
  
  // this code will run before each test
  @BeforeEach
  public void setup() {
    return;
  }

  // this code will run after each test 
  @AfterEach
  public void shutdown() {
    return;
  }

  // this test will pass, as 1 + 1 = 2
  @Test
  public void testOnePlusOne_Two() {
    assertEquals(1+1, 2);
  }

  // this test would fail if run (which would cause the build to fail), so it is skipped
  @Disabled
    @Test
    public void testOnePlusOne_Three() {
      assertEquals(1+1, 3);
    }

}