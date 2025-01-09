package org.firstinspires.ftc.teamcode.teleop;

public abstract class LambdaInterfaces {
    public interface ShortFunction{
       void call();
    }
    public interface Condition{
        boolean call();
    }
    public interface DoubleFunction{
        double call();
    }
}