package frc.robot.config;

import static edu.wpi.first.units.Units.Celsius;

import edu.wpi.first.units.measure.Temperature;

public final class MotorConfig {

    /*
    NOTES:
     - TalonFX controllers (seem to) have a built-in thermal shutdown.
         - Seemingly triggers at 110 C and releases at 90 C for Falcon 500s
             - https://www.chiefdelphi.com/uploads/short-url/eVYO5tVOYZecwq6Tl2kURlFZFgq.pdf
         - I couldn't find any information on the threshold for Kraken X60s or X44
     - Different controllers have their temperature sensors placed differently. This means that some controllers are
       more vulnerable than others to the reading lagging behind the motor temperature
         - Vortexs have their sensors directly attached to the coils, giving them near zero lag. NEO v1.1s and NEO 550s
           lag significantly
             - https://www.chiefdelphi.com/t/rev-robotics-spark-flex-and-neo-vortex/442595/343
     - Motor failure is not a function of solely temperature, but rather many complex factors which are beyond our
       ability to reasonably measure and account for. Using temperature to predict failure will work better for some
       motors than others.
    */

    // NEO Vortex fails at ~140 C
    // I gave the Vortex an extra large margin mostly because I don't entirely trust the 140 C rating I found
    public static Temperature neoVortexTempWarnThreshold = Celsius.of(110);
    public static Temperature neoVortexThermalShutdownThreshold = Celsius.of(120);

    // NEO fails at ~105 C when running with a 60 A current limit
    // https://www.revrobotics.com/neo-brushless-motor-locked-rotor-testing/
    public static Temperature neoTempWarnThreshold = Celsius.of(85);
    public static Temperature neoThermalShutdownThreshold = Celsius.of(95);

    // NEO 550 fails at ~45 C when running with a 40 A current limit
    // NOTE: temperature seems to be a particular poor indication of failure for NEO 550s. Maybe don't have a thermal
    // shutdown on 550s
    // https://www.revrobotics.com/neo-550-brushless-motor-locked-rotor-testing/
    public static Temperature neo550TempWarnThreshold = Celsius.of(35);
    public static Temperature neo550ThermalShutdownThreshold = Celsius.of(40);

    // Falcon 500 fails at 110 C
    public static Temperature falcon500TempWarnThreshold = Celsius.of(90);
    public static Temperature falcon500ThermalShutdownThreshold = Celsius.of(100);

    // Kraken X60 fails at ~100 C
    // https://www.chiefdelphi.com/t/kraken-x60-hits-130-c-266-f/456354/14
    public static Temperature krakenX60TempWarnThreshold = Celsius.of(80);
    public static Temperature krakenX60ThermalShutdownThreshold = Celsius.of(90);

    private MotorConfig() {}
}
