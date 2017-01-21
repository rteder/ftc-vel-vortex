package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Methods used to drive the robot in Autonomous.
 */

public class driveMoves2 {
    LinearOpMode opMode; // Allows us to use stuff from the calling linear opMode objec.
    HardwareChooChoo robot; // Allow us to use the robot hardware as an object.

    double desiredHeading = 0; // A globabl property
    double motorPower = 0.5;  // Positive means forward.
    private ElapsedTime accelTimer = new ElapsedTime(); // For accelerations.

    // This is the contructor, called when the object is created.
    // You need to pass it the opMode (that is, AutoChooChoo) as well as the configuration.
    public driveMoves2(LinearOpMode opMode, HardwareChooChoo arobot ) throws InterruptedException{
        this.opMode = opMode;
        this.robot = arobot;
    }

    /////////////////////////////   STRAIGHT MOVES /////////////////////////////////////
    // Power Profile.
    // This adjusts the power of the robot over the course of a move, gradually
    // accelerating and then decellerating.
    // When far from the target, gradually increase motorPower.
    // Decrease power as we get close.
      private void powerProfile( double distFromtarget, boolean forward ){
        double limitPower;
        // Make sure these are all positive numbers coming ing.
        distFromtarget = Math.abs( distFromtarget);
        // Accelerate!  Increase the motor power by 0.1 every 100 mS
        if (accelTimer.milliseconds() > 100) {
            if ( forward) motorPower += 0.1;
            else motorPower -= 0.1;
            accelTimer.reset();
        }
        // Limit power is 0.1 + 0.2 for every foot from target.
        // But no more than 0.9.
        limitPower = 0.1 + distFromtarget * 0.2;
        if (limitPower > 0.9) limitPower = 0.9;

        if (motorPower > limitPower) motorPower = limitPower;
        if (motorPower > limitPower) motorPower = limitPower;
        motorPower = Range.clip( motorPower, -limitPower, limitPower);
    }

    // Drive forward at the desired nominal power, at the desiredheading.
    // You should start any such move pointed in the right direction.
    // This will make small adjustments in the motor power to maintain the heading.
    void driveAtHeading(double nominalPower ) throws InterruptedException {
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
    void driveDistance(double distanceToGo, boolean finalStop ) throws InterruptedException {
        robot.updateSensors();
        double target = robot.total_distance_feet + distanceToGo;
        boolean forward;
        if (distanceToGo > 0) {
            motorPower = 0.2;
            forward = true;
        } else {
            motorPower = -0.2;
            forward = false;
        }
        accelTimer.reset();

             while (true) {
                powerProfile( target - robot.total_distance_feet, forward  );
                driveAtHeading( motorPower);
                if (forward) {
                    if (robot.total_distance_feet >= target) break;
                } else {
                    if (robot.total_distance_feet <= target) break;
                }
                robot.updateSensors();
            }
        if (finalStop ) stopDrive();
    }

    // Drive a short distance Forward or backward a given distance, in feet.
    // This does not have the decel profile because it does not get up to speed in
    // that short time.  So for less than one foot, this will be faster.
    void driveShortDistance(double distanceToGo, boolean finalStop ) throws InterruptedException {
        robot.updateSensors();
        double target = robot.total_distance_feet + distanceToGo;
        motorPower = 0.2;
        accelTimer.reset();

        if ( distanceToGo >=0 ) {
            while (true) {
                driveAtHeading( motorPower );
                if (robot.total_distance_feet >= target) break;
                robot.updateSensors();
            }
        } else {
            while (true) { // Drive backwards if the target is behind us.
                // Drive at heading does not work right with negative value, so just
                // drive.
                robot.leftMotor.setPower( - motorPower);
                robot.rightMotor.setPower( - motorPower);
                if (robot.total_distance_feet <= target) break;
                robot.updateSensors();
                // Stop the motors afterwards:
            }
        }
        if (finalStop ) stopDrive();
    }

    // Drive for a fixed amount of time, at a given power.
    // Useful for pressing beacon buttons, because it will slow down if it has resisitance.
    // Does NOT try to maintain a heading.
    void driveForTime( double targetMillisconds, double targetPower )throws InterruptedException {
        robot.updateSensors();
        accelTimer.reset();
        motorPower = targetPower;
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
        motorPower = 0.2;  // Nice and slow.
        while (true) {
            robot.leftMotor.setPower(motorPower);
            robot.rightMotor.setPower(motorPower);
            robot.updateSensors();
            // Look for an increase in light above the average.
            if (robot.light > robot.avgLight + 0.1){
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
        motorPower = 0.2;  // Nice and slow.
        // Drive as long as we are are outside a range.
        // The sensor will sometimes say a range of zero, which is not valid.
        // So also keep driving if we do not have a valid range reading.
        while ((robot.range > targetRange) || (robot.range < 1.0)) {
            driveAtHeading( motorPower );
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
        motorPower = 0.2;  // Nice and slow.
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
    public boolean driveToColor( boolean finalStop )throws InterruptedException {
        robot.updateSensors();
        accelTimer.reset();
        motorPower = 0.1;  // Really slow!
        // Drive as long as we are are outside a range.
        // The sensor will sometimes say a range of zero, which is not valid.
        // So also keep driving if we do not have a valid range reading.
        while (true) {
            driveAtHeading( motorPower );
            robot.updateSensors();
            if (robot.right_color.red() + robot.right_color.blue() > 3) break;
            // If this takes more than five seconds some thing is horribly wrong.  Stop.
            if (accelTimer.milliseconds() > 5000) {
                stopAndWait();
            }
        }
        if (finalStop ) stopDrive();
        return true;
    }


    //////////////////////   TURNS ///////////////////////////////////////////////////////////////////
    // Pivot to a given angle, referenced to the starting angle.
    // We make pivots less fast than straight line moves, for precision.
    // But if you want real precision, execute this twice in a row.


    public void pivotToAngle(double finalAngle) throws InterruptedException {
        desiredHeading = finalAngle;
        double compensatedFinalAngle = 0;
        // First get within 5 degrees of final.
        if (finalAngle > robot.heading) {
                compensatedFinalAngle = finalAngle - 10;
        } else {
            compensatedFinalAngle = finalAngle + 10;
        }
        profilePivot( compensatedFinalAngle );
        slowPivot( finalAngle );
        slowPivot( finalAngle );
        stopDrive();
    }


    // profile Pivot privots to the finalAngle, not desiredHeading
    // This is so you can use it to get close with this.
    public void profilePivot( double finalAngle) throws InterruptedException {
        robot.updateSensors();
        accelTimer.reset();
        motorPower = 0.1;

        if (finalAngle > robot.heading) {
            while (true) {
                pivotProfile( robot.heading - finalAngle );
                robot.leftMotor.setPower(motorPower);
                robot.rightMotor.setPower(-motorPower);
                if (robot.heading >= finalAngle) break;
                robot.updateSensors();
            }
        } else {
            while (true) {
                pivotProfile( robot.heading - finalAngle );
                robot.leftMotor.setPower(-motorPower);
                robot.rightMotor.setPower(motorPower);
                if (robot.heading <= finalAngle) break;
                robot.updateSensors();
            }
        }
        // Stop the motors afterwards:
        stopDrive();
    }

    // Slow pivot has no profile and is very slow, for a precise last bit of turning.
    public void slowPivot(double finalAngle) throws InterruptedException {
        desiredHeading = finalAngle;
        robot.updateSensors();
        motorPower = 0.05;

        if (finalAngle > robot.heading) {
            while (true) {
                robot.leftMotor.setPower(motorPower);
                robot.rightMotor.setPower(-motorPower);
                if (robot.heading >= finalAngle) break;
                robot.updateSensors();
            }
        } else {
            while (true) {
                robot.leftMotor.setPower(-motorPower);
                robot.rightMotor.setPower(motorPower);
                if (robot.heading <= finalAngle) break;
                robot.updateSensors();
            }
        }
        // Stop the motors afterwards:
        stopDrive();
    }



    // Pivot profile
    // Increase the speed gradually to do a pivot, but slow down near the end for accuracy.
    private void pivotProfile( double distFromtarget ) {
        // Make sure these are all positive numbers coming ing.
        distFromtarget = Math.abs( distFromtarget);
        double limitPower = 0.9;

        // Accelerate!  Increase the motor power by 0.05 every 100 mS
        if (accelTimer.milliseconds() > 100) {
            motorPower += 0.05;
            accelTimer.reset();
        }

        // Limit power is 0.1 + 0.1 for every twenty degrees from target.
        limitPower = 0.05 + distFromtarget / 200;

        // But no more than 0.4.
        if (limitPower > 0.4) limitPower = 0.4;

        if (motorPower > limitPower) motorPower = limitPower;
    }


    // Pivot a given number of degrees, from whatever it was when it started.
    public void pivotAngle(double degreesToPivot) throws InterruptedException {
        double finalAngle = (double) robot.heading + degreesToPivot;
        pivotToAngle( finalAngle );
    }

    // Drive forward in an arc until we reach the heading.
    // Smooth turns for speed!
    public void arcToAngle(double finalAngle, boolean finalStop ) throws InterruptedException {
        desiredHeading = finalAngle;
        double degreeToPivot = finalAngle - robot.heading;
        // but we only want to pivot a fraction:
        double compDegreeToPivot = degreeToPivot * 0.88;
        double compFinalAngle = robot.heading + compDegreeToPivot;
        robot.updateSensors();
        motorPower = 0.5;

        // Arc right
        if (finalAngle > robot.heading) {
            while (true) {
                robot.leftMotor.setPower(motorPower);
                robot.rightMotor.setPower(motorPower * 0.1);
                if (robot.heading >= compFinalAngle) break;
                opMode.telemetry.addData("compFinalAngle", compFinalAngle );
                opMode.telemetry.update();
                robot.updateSensors();
            }
        } else {
            // Arc left
            while (true) {
                robot.leftMotor.setPower(motorPower*0.1);
                robot.rightMotor.setPower(motorPower);
                if (robot.heading <= compFinalAngle) break;
                robot.updateSensors();
            }
        }

        if (finalStop ) stopDrive();
    }

}
