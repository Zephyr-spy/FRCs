package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

class DriverInputTest {
  @Test void smallMultipliersRemainEffectiveOnEveryAxis() {
    var speeds=DriverInput.speeds(-1,-1,-1);
    assertEquals(.0205,speeds.vxMetersPerSecond,1e-12);
    assertEquals(.0205,speeds.vyMetersPerSecond,1e-12);
    assertEquals(.005*2*Math.PI,speeds.omegaRadiansPerSecond,1e-12);
    assertTrue(DriverInput.speeds(-.051,0,0).vxMetersPerSecond>0);
    assertEquals(0,DriverInput.speeds(0,.049,0).vyMetersPerSecond);
    assertEquals(0,DriverInput.speeds(0,0,Double.NaN).omegaRadiansPerSecond);
  }
}