package org.firstinspires.ftc.teamcode.teleop;

import android.provider.Settings;

import com.qualcomm.robotcore.util.ElapsedTime;

public class tets {
    private double target=2000;
    private double profileStartPos=0;
    private double velocity=-10000;
    private double currentMaxVelocity;
    private double currentMaxAcceleration;
    private double currentMaxDeceleration;
    private double accelDT;
    private double decelDT;
    private double accelDistance;
    private double decelDistance;
    private double cruiseDistance;
    private double cruiseDT;
    private ElapsedTime MOVEMENT_TIMER = new ElapsedTime();


    private void createMotionProfile(double max_velocity, double max_acceleration) {

        double distance=target-profileStartPos;
        double startVelocity = velocity;
        currentMaxVelocity = max_velocity*Math.signum(distance);
        currentMaxAcceleration = max_acceleration*Math.signum(currentMaxVelocity - startVelocity);
        currentMaxDeceleration = -max_acceleration*Math.signum(distance);

        accelDT = (currentMaxVelocity - startVelocity) / currentMaxAcceleration;
        decelDT = (0-currentMaxVelocity) / currentMaxDeceleration;
        accelDistance = startVelocity*accelDT + 0.5 * currentMaxAcceleration * Math.pow(accelDT, 2);
        decelDistance = currentMaxVelocity * decelDT + 0.5 * currentMaxDeceleration * Math.pow(decelDT, 2);
        System.out.println(accelDT);
        System.out.println(decelDT);
        System.out.println(accelDistance);
        System.out.println(decelDistance);
        if (Math.abs(accelDistance+decelDistance) > Math.abs(distance)){
            System.out.println("e");
            double halfExceededDistance = (distance-accelDistance-decelDistance)/2;
            accelDistance = accelDistance+halfExceededDistance;
            accelDT = Math.max(
                    (-startVelocity + Math.sqrt(Math.abs(Math.pow(startVelocity,2) + 2*currentMaxAcceleration*accelDistance)))/(currentMaxAcceleration),
                    (-startVelocity - Math.sqrt(Math.abs(Math.pow(startVelocity,2) + 2*currentMaxAcceleration*accelDistance)))/(currentMaxAcceleration)
            );
            currentMaxVelocity = currentMaxAcceleration * accelDT + velocity;
            decelDistance = decelDistance+halfExceededDistance;
            decelDT = Math.max(
                    (-currentMaxVelocity + Math.sqrt(Math.abs(Math.pow(currentMaxVelocity,2) + 2*currentMaxDeceleration*decelDistance)))/(currentMaxDeceleration),
                    (-currentMaxVelocity - Math.sqrt(Math.abs(Math.pow(currentMaxVelocity,2) + 2*currentMaxDeceleration*decelDistance)))/(currentMaxDeceleration)
            );
        }
        System.out.println(accelDT);
        System.out.println(decelDT);
        System.out.println(currentMaxVelocity);
        System.out.println(accelDistance);
        System.out.println(decelDistance);
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
            instantTargetPosition=profileStartPos + velocity*elapsedTime + 0.5 * currentMaxAcceleration * Math.pow(elapsedTime, 2);
            System.out.print(instantTargetPosition);
            System.out.print("     ");
            System.out.println("accel");

        }
        else if (elapsedTime < accelDT+cruiseDT){
            double cruiseCurrentDT = elapsedTime - accelDT;
            instantTargetPosition=profileStartPos + accelDistance + currentMaxVelocity * cruiseCurrentDT;
            System.out.print(instantTargetPosition);
            System.out.print("     ");
            System.out.println("cruise");
        }

        else {
            double decelCurrentDT = elapsedTime - accelDT - cruiseDT;
            instantTargetPosition = profileStartPos + accelDistance + cruiseDistance + currentMaxVelocity * decelCurrentDT + 0.5 * currentMaxDeceleration * Math.pow(decelCurrentDT, 2);
            System.out.print(instantTargetPosition);
            System.out.print("     ");
            System.out.println("decel");
        }
    }
    public static void main(String[] args){
        tets tets=new tets();
        tets.createMotionProfile(30000,200000);
        double counter=0;
        while (counter<10000){
            tets.runMotionProfileOnce();
            counter++;
        }
    }

}
