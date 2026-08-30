package frc.robot.util;

@FunctionalInterface
public interface OneToOneFunction<IN, OUT> {

    OUT accept(IN a);
}
