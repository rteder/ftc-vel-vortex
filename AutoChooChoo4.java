package org.firstinspires.ftc.teamcode;
// The softare is now split into multiple files.
// For the harware configuration, see Hardware.ChooChoo
//

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutoChooChoo4", group="TekC")
//@Disabled

public class AutoChooChoo4 extends LinearOpMode {
    HardwareChooChoo robot;
    driveMoves2 drive;

    // These are global to the class because they are the state of the robot.
    double setHeading; // desired heading.
    boolean teamColorBlue = false;
    boolean moveBall = false; // Set this if the mission is move ball, not claim beacons.
    boolean enabableStops = true; // Set to true to stop between steps for debugging.


    ////////////////////////////////  HELPER FUNCTIONS ////////////////////////////////////////////
    // Debugging aid-- wait for press of green button (a).
    //  Add these as needed so you can setp through the critcial parts.
    // If enableStops is not set this just returns.
    private void waitForGreen() throws InterruptedException {
        if (!enabableStops) return;
        while( true ) {
            drive.stopDrive();
            if (gamepad1.a) break;
            robot.updateSensors();
        }
        while( true ) {
            drive.stopDrive();
            if (!gamepad1.a) break;
            robot.updateSensors();
        }
    }


    ////////////////////////////   CLAIM BEACON ///////////////////////////////////
    // Sense which side of the beacon is our color and then claim it.
    // Complete this routine in the same orientation in which we started.
    public void senseBeaconAndClaim() throws InterruptedException {
        double claimAngle = 7;

        // First pivot so we are trying to press the right button.
        switch (isRightBlue() ){

            case 0:     // No color detected, nothing we can do here.
                return;
            case 1:    // Blue is on the right
                if (teamColorBlue) { // and we are blue, so pivot right;
                    drive.pivotAngle( -claimAngle );
                } else {
                    drive.pivotAngle( claimAngle );
                } break;
            case 2:
            default:   // Red is on the right
                if (teamColorBlue) {
                    drive.pivotAngle( claimAngle);
                } else {
                    drive.pivotAngle( -claimAngle);
                }
        }

        // Drive forward to claim the beacon, and then back off.
        drive.driveForTime( 600, 0.3);
        drive.driveForTime( 300, -0.3);
    }

    // Color sorting utility, for beacon claiming.
    // Return:
    // 0 if we detect no color
    // 1 if we detect blue
    // 2 if we detect red.
    int isRightBlue() {
        // Only try if the there is enough color to say we might know.
        if (robot.right_color.red() + robot.right_color.blue() > 3) {
            if  (robot.right_color.red() > robot.right_color.blue()){
                return 2;
            } else {
                return 1;
            }

        } else {
            return 0;
        }

    }

    ////////////////////////////////////  EXECUTION BEGINS HERE ///////////////////////////////////
    @Override
    public void runOpMode() throws InterruptedException {

        robot = new HardwareChooChoo( this );
        drive = new driveMoves2( this, robot );
        robot.init( this.hardwareMap, true );
        robot.showSensors = true;

        double mirror = 1.0;  // If on the Blue side, many moves are mirrored, and this is set to -1.0

        // Calibrate the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        robot.gyro.calibrate();

        // Wait for calibration to finish.
        while (robot.gyro.isCalibrating())  {
            Thread.sleep(50);
            idle();
        }

        // Select a team by pressing either the red or the blue buttons.
        // Also, you can make the mission to be moving the ball instead
        // of blaming the beacons.
        // Will stay in this loop until the game starts, so you
        // can pick a different team if you want.
        //telemetry.addData(">", "Pick Team: Red or Blue");
        //telemetry.addData(">", "GRN (default) Beaons YEL: Ball");
        //telemetry.update();
        while(! isStarted() ) {
            if (gamepad1.x) {
                teamColorBlue = true;
                mirror = -1.0;
            }
            if (gamepad1.b) {
                teamColorBlue = false;
                mirror = 1.0;
            }

            if (gamepad1.y) moveBall = true;
            if (gamepad1.a) moveBall = false;

            if (teamColorBlue)telemetry.addData("Team Color>", "BLUE BLUE BLUE");
            else  telemetry.addData("Team Color>", "RED RED RED");
            if (moveBall)telemetry.addData( "Mission", "MOVE THE BALL" );
            else telemetry.addData( "Mission", "CLAIM BEACONS" );
            telemetry.update();
            idle();
        }



        /////////////////  TEST CODE GOES HERE //////////////////////////
        // Delete or comment this out for competition
        // Use this to estimate overshoot of 90 deg pivot.

        /*
        drive.pivotToAngle( 90 );
        waitForGreen();
        drive.pivotToAngle( 0 );
        waitForGreen();

        /*
        drive.arcToAngle( 90, true );
        waitForGreen();
        drive.arcToAngle( 0, true );
        waitForGreen();
        drive.arcToAngle( -90, true );
        waitForGreen();
        drive.arcToAngle( 0, true );
        waitForGreen();


        //drive.profilePivot( 80 );
        waitForGreen();
        drive.pivotToAngle( 90 );
        waitForGreen();
        //sleep( 5000);
        drive.pivotToAngle( 0 );
        waitForGreen();
        //sleep( 5000);
        drive.pivotToAngle( -90 );
        waitForGreen();
        //drive.stopAndWait();
        drive.pivotToAngle( 0 );
        waitForGreen();
        */

        /////////////////////  AUTONOMOUS CODE TO MOVE THE BALL ////////////////
        // Mission: Move ball
        // Be able to execute this simple mission if needed.
        // Drive backwards because our foam pad makes it hard to push the ball
        if (moveBall){
            sleep( 10000 );
            drive.desiredHeading = 0;
            drive.driveDistance(-2.5, true );  // Should end up near center goal.
            setHeading = mirror * -45;       // turn to center goal.
            drive.pivotToAngle( setHeading );
            drive.driveDistance( -3, true );   // And drive up onto wood.
            drive.stopAndWait();                      // And we're done.
        }


        /////////////////////  THE ACTUAL AUTONOMOUS MOVEMENTS ///////////////////////////
        // Startng location is square with back wall,
        // Frame lined up with inside seam of first tile with no corner goal.
        // tile seam edge closest to the goal.
        // Mostly the two sides are a mirror of each other, but the robots are a little different.

        // Get to Near the first beacon.
        drive.driveShortDistance(0.4, false);
        if (teamColorBlue){
           setHeading = 44; // Point just a bit away from parallel to goal entrance.
        } else {
           setHeading = -48; //
        }
        /*
        drive.arcToAngle( setHeading,false );
        drive.driveDistance(4.0, false );  // Should endd near the beacon  an angle.
        drive.arcToAngle( 0, false );   // Get to straight ahead again.
        drive.stopDrive();
        waitForGreen();
        */
        drive.pivotToAngle( setHeading );
        drive.driveDistance(4.2, true);  // Should ned up near the beaon, line, at an angle.
        drive.pivotToAngle( 0 );    // Pointed toward perpendicular to line
        //waitForGreen();
        //driveShortDistance( 0.3); // get closer to line to save time.
        // waitForGreen();



        // Find the line, claim the first beacon.
        drive.driveToLine();
        if (teamColorBlue){
            drive.driveShortDistance( 0.1, true);  // Pivots are different for blue.
        }else {
            drive.driveShortDistance( 0.1, true);  // Light sensor ahead of pivot point; therefore, drive forward.
        }
        setHeading = -90 * mirror;
        drive.pivotToAngle( setHeading );  // Point to beacon
        // waitForGreen();
        drive.driveToRange( 15, false);   // Get as close as ultrasonic sensor will sense reliably
        drive.driveToColor( true);
        // waitForGreen();
        senseBeaconAndClaim(); // claim that beacon
        drive.driveShortDistance( - 0.1, true );
        drive.pivotToAngle(setHeading);              // straight away from the beacon.
        // drive.pivotToAngle( setHeading );      // straight away from the beacon.
        // waitForGreen();
        drive.driveFromRange( 20, setHeading );    // Now back away from beacon.
        // waitForGreen();
        // Here where we would shoot into the goal if we have time.

        // Second set of beacon claiming steps, just like the above.
        drive.pivotToAngle( 0 );    // Pointed toward perpendicular to line, slightly toward beacon
        if (teamColorBlue ) {
            drive.driveDistance( 3.4, false );           // End up close to the  line, beacon.
        } else {
            drive.driveDistance( 3.7, false );           // End up close to the  line, beacon.
        }
        // waitForGreen();
        drive.driveToLine();
        if (teamColorBlue){
            drive.driveShortDistance( 0.1, true);  // Pivots are different for blue.
        }else {
            drive.driveShortDistance( 0.1, true);  // Light sensor ahead of pivot point; therefore, drive forward.
        }
        setHeading = -90 * mirror;
        drive.pivotToAngle( setHeading );  // Point to beacon
        // waitForGreen();
        drive.driveToRange( 15, false);   // Get as close as ultrasonic sensor will sense reliably
        drive.driveToColor( true);
        // waitForGreen();
        senseBeaconAndClaim(); // claim that beacon
        drive.driveShortDistance( - 0.1, true );

        // Now would be a great time to dash back to a ramp for 5 pts.
        drive.stopAndWait();

    }
}