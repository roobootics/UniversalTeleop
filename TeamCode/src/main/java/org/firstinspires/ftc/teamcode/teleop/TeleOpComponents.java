package org.firstinspires.ftc.teamcode.teleop;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.LambdaInterfaces.DoubleFunction;
import org.firstinspires.ftc.teamcode.teleop.LambdaInterfaces.Condition;
import org.firstinspires.ftc.teamcode.teleop.TeleOpActions.TeleOpAction;
import org.firstinspires.ftc.teamcode.teleop.TeleOpActions.PressTrigger;
import org.firstinspires.ftc.teamcode.teleop.TeleOpActions.ConditionalAction;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;


public abstract class TeleOpComponents {
    static HardwareMap hardwareMap;
    static Telemetry telemetry;
    public static ArrayList<BotMotor> motors = new ArrayList<>();
    public static ArrayList<BotServo> servos = new ArrayList<>();

    //create mechanism variables here
    public static BotMotor ryanNemesis;
    public static BotServo tetsyWetsy;
    public static class BotMotor extends DcMotorImplEx {
        public double kP; public double kI; public double kD;
        public HashMap<String,Double> KEY_POSITIONS;
        public RunMode RUN_MODE;
        public double MAX_POSITION; public double MIN_POSITION;
        public double MAX_ACCELERATION; public double MAX_VELOCITY;
        public double currentMaxAcceleration = 0; public double currentMaxDeceleration = 0; public double currentMaxVelocity = 0;

        public double target = 0;

        public double accelDT = 0; public double decelDT = 0; public double cruiseDT = 0;
        public double accelDistance = 0; public double decelDistance = 0; public double cruiseDistance = 0;
        public double profileStartPos = 0;
        public double startVelocity = 0;
        final ElapsedTime MOVEMENT_TIMER = new ElapsedTime(); final ElapsedTime LOOP_TIMER = new ElapsedTime();
        double integralSum = 0;
        double previousError = 0;
        double previousVoltage = 0;
        boolean isStallResetting = false;
        String MOVEMENT_MODE;

        public class UpwardFSMAction implements TeleOpAction{
            private final double maxAcceleration;
            private final double maxVelocity;
            private final double[] positions;
            boolean isStart=true;

            public UpwardFSMAction(double maxAcceleration, double maxVelocity, double...positions) {
                this.maxAcceleration = maxAcceleration;
                this.maxVelocity = maxVelocity;
                this.positions = positions;
                Arrays.sort(this.positions);
            }
            public UpwardFSMAction(double...positions) {
                this(MAX_ACCELERATION,MAX_VELOCITY,positions);
            }
            @Override
            public boolean repeatFromStart(@NonNull TelemetryPacket packet) {
                isStart=true;
                return run(packet);
            }

            @Override
            public void stop() {
                setMotorTarget(getCurrentPosition());
                setPower(0);
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (isStart) {
                    double pos=target;
                    for (double position : positions){
                        if (position>target){
                            pos=position;
                            break;
                        }
                    }
                    setMotorTarget(pos,maxAcceleration,maxVelocity);
                    isStart=false;
                }
                runMotionProfileOnce();
                return Math.abs(target-getCurrentPosition())!=0;
            }
        }
        public class DownwardFSMAction implements TeleOpAction{
            private final double maxAcceleration;
            private final double maxVelocity;
            private final List<Double> positions;
            boolean isStart=true;

            public DownwardFSMAction(double maxAcceleration, double maxVelocity, double...positions) {
                this.maxAcceleration = maxAcceleration;
                this.maxVelocity = maxVelocity;
                Arrays.sort(positions);
                Double[] newPositions = new Double[positions.length];
                for (int i=0;i<=positions.length;i++){
                    newPositions[i]= positions[i];
                }
                this.positions=Arrays.asList(newPositions);
                Collections.reverse(this.positions);
            }
            public DownwardFSMAction(double...positions) {
                this(MAX_ACCELERATION,MAX_VELOCITY,positions);
            }
            @Override
            public boolean repeatFromStart(@NonNull TelemetryPacket packet) {
                isStart=true;
                return run(packet);
            }

            @Override
            public void stop() {
                setMotorTarget(getCurrentPosition());
                setPower(0);
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (isStart) {
                    double pos=target;
                    for (double position : positions){
                        if (position<target){
                            pos=position;
                            break;
                        }
                    }
                    setMotorTarget(pos,maxAcceleration,maxVelocity);
                    isStart=false;
                }
                runMotionProfileOnce();
                return Math.abs(target-getCurrentPosition())!=0;
            }
        }

        public PressTrigger triggeredFSMAction(Condition upCondition, Condition downCondition, double maxAcceleration, double maxVelocity, double...positions){
            return new PressTrigger(new Condition[]{upCondition,downCondition}, new TeleOpAction[]{new UpwardFSMAction(maxAcceleration,maxVelocity,positions),new DownwardFSMAction(maxAcceleration,maxVelocity,positions)});
        }
        public PressTrigger triggeredFSMAction(Condition upCondition, Condition downCondition,double...positions){
            return new PressTrigger(new Condition[]{upCondition,downCondition}, new TeleOpAction[]{new UpwardFSMAction(positions),new DownwardFSMAction(positions)});
        }
        public class SetPowerAction implements TeleOpAction{
            DoubleFunction powerFun;
            public SetPowerAction(double power){
                this.powerFun=() -> (Math.max(-1,Math.min(1,power)));
            }
            public SetPowerAction(DoubleFunction powerFun){
                this.powerFun=powerFun;
            }
            @Override
            public boolean repeatFromStart(@NonNull TelemetryPacket packet) {
                return run(packet);
            }

            @Override
            public void stop() {
                setPower(0);
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                setPower(powerFun.call());
                return false;
            }
        }
        public SetPowerAction setPowerAction(double power){
            return new SetPowerAction(power);
        }
        public SetPowerAction setPowerAction(DoubleFunction powerFun){
            return new SetPowerAction(powerFun);
        }
        public class SetTargetAction implements TeleOpAction{
            DoubleFunction targetFun;
            double maxAcceleration;
            double maxVelocity;
            boolean isStart=true;
            public SetTargetAction(double target, double maxAcceleration, double maxVelocity){
                this.targetFun = () -> (target);
                this.maxAcceleration=maxAcceleration;
                this.maxVelocity=maxVelocity;
            }
            public SetTargetAction(DoubleFunction targetFun, double maxAcceleration, double maxVelocity){
                this.targetFun = targetFun;
                this.maxAcceleration=maxAcceleration;
                this.maxVelocity=maxVelocity;
            }
            public SetTargetAction(double target){
                this(target,MAX_ACCELERATION,MAX_VELOCITY);
            }
            public SetTargetAction(DoubleFunction targetFun){
                this(targetFun,MAX_ACCELERATION,MAX_VELOCITY);
            }
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (isStart) {
                    setMotorTarget(targetFun.call(),maxAcceleration,maxVelocity);
                    isStart=false;
                }
                return Math.abs(target-getCurrentPosition())!=0;
            }

            @Override
            public boolean repeatFromStart(@NonNull TelemetryPacket packet) {
                isStart=true;
                return run(packet);
            }

            @Override
            public void stop() {
                setMotorTarget(getCurrentPosition());
            }

        }
        public SetTargetAction moveToPositionAction(double target, double maxAcceleration, double maxVelocity){
            return new SetTargetAction(target,maxAcceleration,maxVelocity);
        }
        public SetTargetAction moveToPositionAction(DoubleFunction targetFun, double maxAcceleration, double maxVelocity){
            return new SetTargetAction(targetFun,maxAcceleration,maxVelocity);
        }
        public SetTargetAction moveToPositionAction(double target){
            return new SetTargetAction(target,MAX_ACCELERATION,MAX_VELOCITY);
        }
        public SetTargetAction moveToPositionAction(DoubleFunction targetFun){
            return new SetTargetAction(targetFun,MAX_ACCELERATION,MAX_VELOCITY);
        }
        public class StallResetAction implements TeleOpAction{
            public boolean isStart;
            public boolean run(@NonNull TelemetryPacket packet){
                if (isStart) {
                    initiateStallReset();
                    isStart=false;
                }
                checkStallResetOnce();
                return isStallResetting;
            }

            @Override
            public boolean repeatFromStart(@NonNull TelemetryPacket packet) {
                isStart=true;
                return run(packet);
            }

            @Override
            public void stop() {
                isStallResetting=false;
                setPower(0);
            }
        }
        public StallResetAction stallResetAction(){
            return new StallResetAction();
        }
        public ConditionalAction triggeredDynamicAction(Condition upCondition, Condition downCondition, double change,double maxAcceleration, double maxVelocity){
            return new ConditionalAction(new Condition[]{upCondition,downCondition}, new TeleOpAction[]{new SetTargetAction(target+change,maxAcceleration,maxVelocity),new SetTargetAction(target-change,maxAcceleration,maxVelocity)});
        }
        public ConditionalAction triggeredDynamicAction(Condition upCondition, Condition downCondition, double change){
            return new ConditionalAction(new Condition[]{upCondition,downCondition}, new TeleOpAction[]{new SetTargetAction(target+change),new SetTargetAction(target-change)});
        }
        public PressTrigger triggeredMoveToTargetAction(Condition condition, double target, double maxAcceleration, double maxVelocity){
            return new PressTrigger(new Condition[]{condition},new TeleOpAction[]{new SetTargetAction(target,maxAcceleration,maxVelocity)});
        }
        public PressTrigger triggeredMoveToTargetAction(Condition condition, double target){
            return new PressTrigger(new Condition[]{condition},new TeleOpAction[]{new SetTargetAction(target)});
        }
        public PressTrigger triggeredToggleAction(Condition condition, double target1, double target2, double maxAcceleration, double maxVelocity){
            return new PressTrigger(new Condition[]{condition},new TeleOpAction[]{
                    new ConditionalAction(new Condition[]{()->(target==target1),()->(target==target2)},new TeleOpAction[]{
                            new SetTargetAction(target2,maxAcceleration,maxVelocity),
                            new SetTargetAction(target1,maxAcceleration,maxVelocity)
                    })
                    
            });
        }
        public BotMotor(String deviceName,
                        DcMotorController controller,
                        int portNumber,
                        @NonNull MotorConfigurationType motorType,

                        double kP,double kI,double kD,
                        HashMap<String,Double> keyPositions,
                        double maxPosition, double minPosition,
                        double maxAcceleration, double maxVelocity,
                        RunMode runMode, Direction direction, ZeroPowerBehavior zeroPowerBehaviour,
                        String movementMode)
        {
            super(controller, portNumber, DcMotor.Direction.FORWARD,motorType);

            this.kP=kP; this.kI=kI; this.kD=kD;
            this.KEY_POSITIONS=keyPositions;
            this.MAX_POSITION=maxPosition; this.MIN_POSITION = minPosition;
            this.MAX_ACCELERATION=maxAcceleration; this.MAX_VELOCITY=maxVelocity;
            this.RUN_MODE = runMode;
            this.MOVEMENT_MODE=movementMode;

            setMode(RunMode.STOP_AND_RESET_ENCODER);
            setMode(runMode);
            setDirection(direction);
            setZeroPowerBehavior(zeroPowerBehaviour);

            hardwareMap.put(deviceName,this);
            motors.add(this);
        }
        private void createMotionProfile(double max_velocity, double max_acceleration) {
            double distance=target-profileStartPos;
            startVelocity = getVelocity();
            currentMaxVelocity = max_velocity*Math.signum(distance);
            currentMaxAcceleration = max_acceleration*Math.signum(currentMaxVelocity - startVelocity);
            currentMaxDeceleration = -max_acceleration*Math.signum(distance);

            accelDT = (currentMaxVelocity - startVelocity) / currentMaxAcceleration;
            decelDT = (0-currentMaxVelocity) / currentMaxDeceleration;
            accelDistance = startVelocity*accelDT + 0.5 * currentMaxAcceleration * Math.pow(accelDT, 2);
            decelDistance = currentMaxVelocity * decelDT + 0.5 * currentMaxDeceleration * Math.pow(decelDT, 2);

            if (Math.abs(accelDistance+decelDistance) > Math.abs(distance)){
                double halfExceededDistance = (distance-accelDistance-decelDistance)/2;
                accelDistance = accelDistance+halfExceededDistance;
                accelDT = Math.max(
                        (-startVelocity + Math.sqrt(Math.abs(Math.pow(startVelocity,2) + 2*currentMaxAcceleration*accelDistance)))/(currentMaxAcceleration),
                        (-startVelocity - Math.sqrt(Math.abs(Math.pow(startVelocity,2) + 2*currentMaxAcceleration*accelDistance)))/(currentMaxAcceleration)
                );
                currentMaxVelocity = currentMaxAcceleration * accelDT + startVelocity;
                decelDistance = decelDistance+halfExceededDistance;
                decelDT = Math.max(
                        (-currentMaxVelocity + Math.sqrt(Math.abs(Math.pow(currentMaxVelocity,2) + 2*currentMaxDeceleration*decelDistance)))/(currentMaxDeceleration),
                        (-currentMaxVelocity - Math.sqrt(Math.abs(Math.pow(currentMaxVelocity,2) + 2*currentMaxDeceleration*decelDistance)))/(currentMaxDeceleration)
                );
            }
            cruiseDistance = distance - accelDistance - decelDistance;
            cruiseDT = cruiseDistance / currentMaxVelocity;
        }
        public void runMotionProfileOnce(){
            double instantTargetPosition;
            double elapsedTime = MOVEMENT_TIMER.time();
            if (elapsedTime > accelDT+decelDT+cruiseDT){
                instantTargetPosition=target;
            }

            if (elapsedTime < accelDT){
                instantTargetPosition=profileStartPos + startVelocity * elapsedTime + 0.5 * currentMaxAcceleration * Math.pow(elapsedTime, 2);


            }
            else if (elapsedTime < accelDT+cruiseDT){
                double cruiseCurrentDT = elapsedTime - accelDT;
                instantTargetPosition=profileStartPos + accelDistance + currentMaxVelocity * cruiseCurrentDT;
            }

            else {
                double decelCurrentDT = elapsedTime - accelDT - cruiseDT;
                instantTargetPosition = profileStartPos + accelDistance + cruiseDistance + currentMaxVelocity * decelCurrentDT + 0.5 * currentMaxDeceleration * Math.pow(decelCurrentDT, 2);
            }

            double error=instantTargetPosition-getCurrentPosition();
            double kpPower = kP*error;
            integralSum += LOOP_TIMER.time()*error;
            double kiPower = kI*integralSum;
            double kdPower = kD*(error-previousError)/ LOOP_TIMER.time();
            LOOP_TIMER.reset();
            previousError=error;
            setPower(Math.min(1,Math.max(-1,kpPower+kiPower+kdPower)));
        }
        public void setMotorTarget(double target, double maxVelocity, double maxAcceleration){
            if (target!=this.target || maxVelocity != currentMaxVelocity || maxAcceleration != currentMaxAcceleration) {
                this.target = Math.min(MAX_POSITION, Math.max(MIN_POSITION, target));
                integralSum = 0;
                previousError = 0;
                MOVEMENT_TIMER.reset();
                LOOP_TIMER.reset();
                createMotionProfile(maxVelocity, maxAcceleration);
            }
        }
        public void setMotorTarget(double target){
            this.setMotorTarget(target, MAX_VELOCITY, MAX_ACCELERATION);
        }
        public void initiateStallReset(){
            isStallResetting=true;
            setPower(-1);
            previousVoltage = getCurrent(CurrentUnit.AMPS);
        }
        public void checkStallResetOnce(){
            double voltage = getCurrent(CurrentUnit.AMPS);
            if (voltage-previousVoltage>2){
                setPower(0);
                setMode(RunMode.STOP_AND_RESET_ENCODER);
                setMode(RUN_MODE);
                isStallResetting=false;
            }
            else {
                previousVoltage=voltage;
            }
        }
    }

    public static class BotServo extends ServoImpl {

        public HashMap<String,Double> KEY_POSITIONS;
        public double MAXIMUM_POSITION; public double MINIMUM_POSITION;
        public double RANGE;
        public double SERVO_SPEED;
        public BotServo(String deviceName,
                ServoController controller,
                int portNumber,
                HashMap<String,Double> keyPositions,
                double maxPosition,
                double minPosition,
                double range,
                double servoSpeed,
                Direction direction)
        {
            super(controller, portNumber);

            this.KEY_POSITIONS = keyPositions;
            this.MAXIMUM_POSITION = maxPosition; this.MINIMUM_POSITION =minPosition;
            this.RANGE = range;
            this.SERVO_SPEED = servoSpeed;

            setDirection(direction);

            hardwareMap.put(deviceName,this);
            servos.add(this);
        }
        @Override
        public void setPosition(double position){
            super.setPosition(Math.max(MINIMUM_POSITION,Math.min(MAXIMUM_POSITION,position)) / RANGE);
        }
        @Override
        public double getPosition(){
            return super.getPosition() * RANGE;
        }
        public class SetPositionAction implements TeleOpAction {
            boolean isStart;
            DoubleFunction posFun;
            ArrayList<SleepAction> sleepActions = new ArrayList<>();
            public SetPositionAction(double pos){
                this.posFun = () -> (pos);
                sleepActions.add(new SleepAction(Math.abs(pos-getPosition())*SERVO_SPEED));
            }
            public SetPositionAction(DoubleFunction posFun) {
                this.posFun = posFun;
                sleepActions.add(new SleepAction(Math.abs(posFun.call()-getPosition())*SERVO_SPEED));
            }
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (isStart) {
                    setPosition(posFun.call());
                    isStart=false;
                }
                boolean result = sleepActions.get(0).run(telemetryPacket);
                if (!result){
                    sleepActions.remove(0);
                }
                return result;
            }

            @Override
            public boolean repeatFromStart(@NonNull TelemetryPacket packet) {
                isStart=true;
                sleepActions.add(new SleepAction(Math.abs(posFun.call()-getPosition())*SERVO_SPEED+0.07));
                return false;
            }

            @Override
            public void stop() {
                sleepActions.clear();
            }
        }
        public SetPositionAction setPositionAction(double pos){
            return new SetPositionAction(pos);
        }
        public SetPositionAction setPositionAction(DoubleFunction posFun){
            return new SetPositionAction(posFun.call());
        }
        public class UpwardFSMAction implements TeleOpAction{
            private final double[] positions;
            boolean isStart=true;
            ArrayList<SleepAction> sleepActions = new ArrayList<>();

            public UpwardFSMAction(double...positions) {
                this.positions = positions;
                Arrays.sort(this.positions);
            }
            @Override
            public boolean repeatFromStart(@NonNull TelemetryPacket packet) {
                isStart=true;
                return run(packet);
            }

            @Override
            public void stop() {
                sleepActions.clear();
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (isStart) {
                    double pos=getPosition();
                    for (double position : positions){
                        if (position>pos){
                            pos=position;
                            break;
                        }
                    }
                    setPosition(pos);
                    isStart=false;
                }
                boolean result = sleepActions.get(0).run(telemetryPacket);
                if (!result){
                    sleepActions.remove(0);
                }
                return result;
            }
        }
        public class DownwardFSMAction implements TeleOpAction{
            private final List<Double> positions;
            ArrayList<SleepAction> sleepActions = new ArrayList<>();
            boolean isStart=true;

            public DownwardFSMAction(double...positions) {
                Arrays.sort(positions);
                Double[] newPositions = new Double[positions.length];
                for (int i=0;i<=positions.length;i++){
                    newPositions[i]= positions[i];
                }
                this.positions=Arrays.asList(newPositions);
                Collections.reverse(this.positions);
            }
            @Override
            public boolean repeatFromStart(@NonNull TelemetryPacket packet) {
                isStart=true;
                return run(packet);
            }

            @Override
            public void stop() {
                sleepActions.clear();
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (isStart) {
                    double pos=getPosition();
                    for (double position : positions){
                        if (position<pos){
                            pos=position;
                            break;
                        }
                    }
                    setPosition(pos);
                    sleepActions.add(new SleepAction(Math.abs(pos-getPosition())*SERVO_SPEED));
                    isStart=false;
                }
                boolean result = sleepActions.get(0).run(telemetryPacket);
                if (!result){
                    sleepActions.remove(0);
                }
                return result;
            }
        }
        public PressTrigger triggeredFSMAction(Condition upCondition, Condition downCondition,double...positions){
            return new PressTrigger(new Condition[]{upCondition,downCondition}, new TeleOpAction[]{new UpwardFSMAction(positions),new DownwardFSMAction(positions)});
        }
        public ConditionalAction triggeredDynamicAction(Condition upCondition, Condition downCondition, double change){
            return new ConditionalAction(new Condition[]{upCondition,downCondition}, new TeleOpAction[]{new SetPositionAction(getPosition()+change),new SetPositionAction(getPosition()-change)});
        }
        public PressTrigger triggeredMoveToTargetAction(Condition condition, double target){
            return new PressTrigger(new Condition[]{condition},new TeleOpAction[]{new SetPositionAction(target)});
        }
        public PressTrigger triggeredToggleAction(Condition condition, double target1, double target2){
            return new PressTrigger(new Condition[]{condition},new TeleOpAction[]{
                    new ConditionalAction(new Condition[]{()->(getPosition()==target1),()->(getPosition()==target2)},new TeleOpAction[]{
                            new SetPositionAction(target2),
                            new SetPositionAction(target1)
                    })

            });
        }
    }
    public static void initializeMechanisms(HardwareMap hardwareMap, Telemetry telemetry){
        TeleOpComponents.hardwareMap=hardwareMap;
        TeleOpComponents.telemetry=telemetry;
        //initialize mechanism variables here
        ryanNemesis =new BotMotor(
                hardwareMap.get(DcMotorEx.class, "motor-1").getDeviceName(),
                hardwareMap.get(DcMotorEx.class, "motor-1").getController(),
                hardwareMap.get(DcMotorEx.class, "motor-1").getPortNumber(),
                hardwareMap.get(DcMotorEx.class, "motor-1").getMotorType(),
                0.0427,0,0.000325,
                new HashMap<>(),
                Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY,
                277408.169,1500,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                DcMotor.Direction.FORWARD,
                DcMotor.ZeroPowerBehavior.BRAKE,
                "MOTION_PROFILE"
        );
        tetsyWetsy=new BotServo(
                hardwareMap.get(Servo.class, "tets").getDeviceName(),
                hardwareMap.get(Servo.class, "tets").getController(),
                hardwareMap.get(Servo.class, "tets").getPortNumber(),
                new HashMap<>(),
                0,
                270,
                270,
                422,
                Servo.Direction.FORWARD
        );
    }
}
