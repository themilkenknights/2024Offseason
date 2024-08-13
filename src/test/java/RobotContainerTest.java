// based of 6238's 2024 code

import static org.junit.jupiter.api.Assertions.fail;

import frc.robot.RobotContainer;
import org.junit.jupiter.api.Test;

public class RobotContainerTest {
  @Test
  public void Test() {
    try {
      new RobotContainer();
    } catch (Exception e) {
      e.printStackTrace();
      fail("Failed to instantiate RobotContainer, see stack trace above.");
    }
  }
}
