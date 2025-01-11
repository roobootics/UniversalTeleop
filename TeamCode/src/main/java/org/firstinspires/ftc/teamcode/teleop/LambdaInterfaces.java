package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

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
    public interface RoadrunnerFunction {}
    public interface StrafeToLinearHeading extends RoadrunnerFunction {
        TrajectoryActionBuilder call(Vector2d vector, double heading);
    }
    public interface WaitSeconds extends RoadrunnerFunction {
        TrajectoryActionBuilder call(double time);
    }
    public interface TurnTo extends RoadrunnerFunction {
        TrajectoryActionBuilder call(double heading);
    }
}