package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareChooChoo;

/**
 * Methods used to drive the robot in Autonomous.
 */

public class driveMoves {
    LinearOpMode opMode; // Allows us to use stuff from the calling linear opMode objec.
    HardwareChooChoo robot; // Allow us to use the robot hardware as an object.

    double desiredHeading = 0; // A globabl property
    private ElapsedTime accelTimer = new ElapsedTime(); // For accelerations.

    // This is the contructor, called when the object is created.
    // You need to pass it the opMode (that is, AutoChooChoo) as well as the configuration.
    public driveMoves(LinearOpMode opMode, HardwareChooChoo arobot ) throws InterruptedException{
        this.opMode = opMode;
        this.robot = arobot;
    }

    /////////////////////////////   STRAIGHT MOVES /////////////////////////////////////

    // Drive forward at the desired nominal power, at the desiredheading.
    // You should start any such move pointed in the right direction.
    // This will make small adjustments in the motor power to maintain the heading.
    void AtHeading(double nominalPower ) throws InterruptedException {
        double leftPower = nominalPower;
        double gain = 0.10;
        double angleError = robot.heading - desiredHeading;
        angleError = Range.clip( angleError, -5, 5);
        double rightPower = nominalPower *( 1 + gain * angleError);
        rightPower = Range.clip( rightPower, -1, 1 );

        robot.leftMotor.setPower( leftPower );
        robot.rightMotor.setPower(rightPower);
    }

    // Drive Forward or backward a given distance, in feet.
    // Will accelerate and decelerate to ge there quickly.
    // Set finalStop if you want it to stop at the end.  Otherwise it will be moving slowly.
    void Distance(double distanceToGo ) throws InterruptedException {
        robot.updateSensors();
        double target = robot.total_distance_feet + distanceToGo;

        // To keep it simple, all backward moves are at low power.
        if (distanceToGo < 0) {
            ToTargetDistance(target, 0.2);
            stopDrive();
            return;
        }


       double shiftPoint = robot.total_distance_feet + 0.5;
        double decelPoint1 = target - 1.5;
        double decelPoint2 = target - 0.5;

        if (distanceToGo < 1.0) {
            // Short move (less than a foot) at low power
            ToTargetDistance(target, 0.2);
            stopDrive();
            return;
        } else if (distanceToGo < 3.0) {
            // between on and two feet, do the first foot at higher power.
            ToTargetDistance(shiftPoint, 0.4);
            ToTargetDistance(target, 0.2);
            stopDrive();
            return;
        } else {
            // Mover of greater than 3 feet:
            // First half foot at 0.4
            ToTargetDistance(shiftPoint, 0.3);

            // Now run at 0.5 until 1.5 feet out
            ToTargetDistance(decelPoint1, 0.5);

            // Now run at 0.3 until 0.5 feet away
            ToTargetDistance(decelPoint2, 0.3);

            // And come in slow for 0.5
            ToTargetDistance(target, 0.2);
            stopDrive();
            return;
        }
    }


    // Drive at the specified distance to the target distance, given as the
    // total from the start of the run.
    void ToTargetDistance(double target, double power ) throws InterruptedException {
        robot.updateSensors();
        boolean forward = target > robot.total_distance_feet;

        if ( forward ) {
            while (true) {
                AtHeading( power );
                if (robot.total_distance_feet >= target) break;
                robot.updateSensors();
            }
        } else {
            while (true) { // Drive backwards if the target is behind us.
                // Drive at heading does not work right with negative value, so just
                // drive.
                robot.leftMotor.setPower( - power);
                robot.rightMotor.setPower( - power);
                if (robot.total_distance_feet <= target) break;
                robot.updateSensors();
            }
        }
    }

    // Drive for a fixed amount of time, at a given power.
    // Useful for pressing beacon buttons, because it will slow down if it has resisitance.
    // Does NOT try to maintain a heading.
    void ForTime(double targetMillisconds, double targetPower )throws InterruptedException {
        robot.updateSensors();
        accelTimer.reset();
        while (accelTimer.milliseconds() < targetMillisconds) {
            robot.leftMotor.setPower(targetPower);
            robot.rightMotor.setPower(targetPower);
            robot.updateSensors();
        }
        stopDrive();
    }


    // Shut everything down and do nothing.
    public void stopAndWait() throws InterruptedException {
        while (true) {
            stopDrive();
        }
    }

    void stopDrive() throws InterruptedException {
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        robot.updateSensors();
    }


    /////////////////////   SENSOR BASED MOVES ////////////////////////////////////////////
    // Drive forward until the light sensor detects the line, and then stop.
    // When you test this, do not do this first because you need to have
    // taken a bunch of light readings before the average is accurate.
    public void driveToLine()throws InterruptedException {
        robot.updateSensors();
        accelTimer.reset();
        double motorPower = 0.2;  // Nice and slow.
        while (true) {
            robot.leftMotor.setPower(motorPower);
            robot.rightMotor.setPower(motorPower);
            robot.updateSensors();
            // Look for an increase in light above the average.
            if (robot.light > robot.avgLight + 0.07){
                break;
            }
            // If this takes more than five seconds some thing is horribly wrong.  Stop.
            if (accelTimer.milliseconds() > 5000) {
                stopAndWait();
            }
        }
        stopDrive();
    }

    // Drive forward until the ultrasonic sensor detects that we are within the
    // passed range of the obstacle, and then stop.
    public void driveToRange(double targetRange, boolean finalStop ) throws InterruptedException {
        robot.updateSensors();
        accelTimer.reset();
        double motorPower = 0.2;  // Nice and slow.
        // Drive as long as we are are outside a range.
        // The sensor will sometimes say a range of zero, which is not valid.
        // So also keep driving if we do not have a valid range reading.
        while ((robot.range > targetRange) || (robot.range < 1.0)) {
            AtHeading( motorPower );
            robot.updateSensors();
            // If this takes more than five seconds some thing is horribly wrong.  Stop.
            if (accelTimer.milliseconds() > 5000) {
                stopAndWait();
            }
        }
        stopDrive();
    }

    // Drive backwards until the range is at least the target parameter.
    // Use for backing away from a target a predictable distance.
    public void driveFromRange(double targetRange, double desiredHeading ) throws InterruptedException {
        robot.updateSensors();
        accelTimer.reset();
        double motorPower = 0.2;  // Nice and slow.
        while (robot.range < targetRange) {
            // Drive at heading does not work right with negative value, so just
            // drive.
            robot.leftMotor.setPower( - motorPower);
            robot.rightMotor.setPower( - motorPower);
            robot.updateSensors();
            // If this takes more than five seconds some thing is horribly wrong.  Stop.
            if (accelTimer.milliseconds() > 5000) {
                stopAndWait();
            }
        }
        stopDrive();
    }

    // Drive until the color sensor has a valid reading.
    public boolean driveToColor( )throws InterruptedException {
        robot.updateSensors();
        accelTimer.reset();
        double motorPower = 0.05;  // Really slow!
           while (true) {
            AtHeading( motorPower );
            robot.updateSensors();
            // Requires one color to be at least two more than the other:
            if ((robot.right_color.red() + robot.right_color.blue() > 3) &&
                (Math.abs(robot.right_color.red() - robot.right_color.blue()) >=2))
                break;
            // If this takes more than a short time we missed it. Stop.
           // This is critical.  If this is too long we claim a random beacon.
            if (accelTimer.milliseconds() > 1200) {
                stopAndWait();
            }
        }
        stopDrive();
        return true;
    }


    //////////////////////   TURNS ///////////////////////////////////////////////////////////////////
    // Pivot to a given angle, referenced to the starting angle.
     public void pivotToAngle(double finalAngle) throws InterruptedException {
        double degreesToPivot = Math.abs(robot.heading -finalAngle);
        desiredHeading = finalAngle;
        boolean posTurn;
        double turnSign = 1.00;
        double gotGoingFast = 0;
        double breakAngle = 0;
        if (finalAngle > robot.heading) {
            posTurn = true;
            turnSign = 1.00;
        } else {
            posTurn = false;
            turnSign = -1.00;
        }

        double AngleToGo = Math.abs( finalAngle - robot.heading);

         // No point ina tiny angle don't waste time.
         if ( AngleToGo < 2) return;

             // Handle small pivots differently.
        if ( AngleToGo < 15) {
            // Kick start for 200 mS
            pivotForTime( 400,turnSign, 0.2);
          
            // Now pivot to two degrees shy of final angle at low power.
            breakAngle = finalAngle - turnSign * 2;
            Pivot( breakAngle, 0.05);
            stopDrive();
            return;
       }


        // if we mave more than 35 until final angle, go fast until we are 30 degrees from final angle
        if ( AngleToGo > 40) {
            breakAngle = finalAngle - (turnSign * 35);
            Pivot( breakAngle, 0.2);
            gotGoingFast = 1.00;
        }

        // if we have more than 20 degrees to go, medium until 15 degrees to final angle
        if ( AngleToGo > 20) {
            breakAngle = finalAngle - (turnSign * 15);
            Pivot( breakAngle, 0.08);
            gotGoingFast = 1.00;
        }

        // We must now be faily close.
        // Go slow, but set the breakAngle two decrees from the final angle,
         // or and extra two degrees if we got going fast.
        breakAngle = finalAngle - (turnSign * (2 + 2 * gotGoingFast));
        Pivot( breakAngle, 0.05);
        stopDrive();
        // pivotIfNeeded( finalAngle);
    }

    // Use this to fix it if is too far off.  If it's within a degree do nothing.
    // Use for if it needs to be really precise.
    public void pivotIfNeeded( double finalAngle )throws InterruptedException {
        // Wait for it to stop.
        while( robot.spinning ) {
            robot.updateSensors();
            stopDrive();
        }
        // Now pivot if needed.
        if (Math.abs(robot.heading - finalAngle) > 1) {
            Pivot(finalAngle, .05);
            stopDrive();
        }

    }

    // Pivot, used by other routines.
    // Note that it does not stop the motors at the end!
    public void Pivot(double finalAngle, double pivotPower ) throws InterruptedException {
        desiredHeading = finalAngle;
        robot.updateSensors();

        if (finalAngle > robot.heading) {
            while (true) {
                robot.leftMotor.setPower(pivotPower);
                robot.rightMotor.setPower(-pivotPower);
                if (robot.heading >= finalAngle) break;
                robot.updateSensors();
            }
        } else {
            while (true) {
                robot.leftMotor.setPower(-pivotPower);
                robot.rightMotor.setPower(pivotPower);
                if (robot.heading <= finalAngle) break;
                robot.updateSensors();
            }
        }
   }

    // Pivot a given number of degrees, from whatever it was when it started.
    public void pivotAngle(double degreesToPivot) throws InterruptedException {
        double finalAngle = (double) robot.heading + degreesToPivot;
        pivotToAngle( finalAngle );
    }



    public void pivotForTime(double targetMillisconds, double turnSign, double targetPower )throws InterruptedException {
        robot.updateSensors();
        accelTimer.reset();
        while (accelTimer.milliseconds() < targetMillisconds) {
            robot.leftMotor.setPower(turnSign * targetPower);
            robot.rightMotor.setPower(-turnSign * targetPower);
            robot.updateSensors();
        }
        stopDrive();
    }


}
