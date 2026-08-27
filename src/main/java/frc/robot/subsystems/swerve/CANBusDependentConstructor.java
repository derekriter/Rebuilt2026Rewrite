package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.CANBus;

@FunctionalInterface
public interface CANBusDependentConstructor<T> {

    T construct(CANBus bus);
}
