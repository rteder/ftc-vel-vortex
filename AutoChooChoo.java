package org.firstinspires.ftc.teamcode;
// The softare is now split into multiple files.
// For the harware configuration, see Hardware.ChooChoo
//

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoChooChoo", group="TekC")
//@Disabled

public class AutoChooChoo extends LinearOpMode {
    HardwareChooChoo robot;
    driveMoves drive;

    // These are global to the class because they are the state of the robot.
    double setHeading; // desired heading.
    boolean teamColorBlue = false;
    boolean moveBall = false; // Set this if the mission is move ball, not claim beacons.
    boolean enabableStops = true; // Set to true to stop between steps for debugging.
    private ElapsedTime matchTimer = new ElapsedTime(); // Since start of match.



    ////////////////////////////////  HELPER FUNCTIONS ////////////////////////////////////////////
    // Debugging aid-- wait for press of green button (a).
    // Add these as needed so you can step through the critical parts.
    // If enableStops is not set this just returns.
    private void waitForGreen() throws InterruptedException {
        if (!enabableStops) return;
         while( !gamepad1.a  ) {   // Stay here until we press wait for green
            drive.stopDrive();
            robot.updateSensors();
        }
        while( gamepad1.a ) {   // Stay here until we release wait for green
            drive.stopDrive();
            robot.updateSensors();
        }
    }


    ////////////////////////////   BEACON CLAIMING ///////////////////////////////////
    // These are constant that make the logic more understandable:
    static final boolean LEFT_SIDE = false;
    static final boolean RIGHT_SIDE = true;
    static final int NO_COLOR = 0;
    static final int COLOR_BLUE = 1;
    static final int COLOR_RED = 2;

    // Sense which side of the beacon is our color and then claim it.
    // Return true if this is successful.  Return false it it is not.
    public boolean senseBeaconAndClaim() throws InterruptedException {
        if (teamColorBlue) {
            if(getBeaconColor( RIGHT_SIDE ) == COLOR_BLUE){
                claimBeacon( RIGHT_SIDE );
                return true;
            }
            if(getBeaconColor( LEFT_SIDE ) == COLOR_BLUE){
                claimBeacon( LEFT_SIDE);
                return true;
            }
            if(getBeaconColor( RIGHT_SIDE ) == COLOR_RED){
                claimBeacon( LEFT_SIDE );
                return true;
            }
            if(getBeaconColor( LEFT_SIDE ) == COLOR_RED){
                claimBeacon( RIGHT_SIDE );
                return true;
            }
            return false;
        }else{
            if(getBeaconColor( RIGHT_SIDE ) == COLOR_BLUE){
                claimBeacon( LEFT_SIDE );
                return true;
            }
            if(getBeaconColor( LEFT_SIDE ) == COLOR_BLUE){
                claimBeacon( RIGHT_SIDE);
                return true;
            }
            if(getBeaconColor( RIGHT_SIDE ) == COLOR_RED){
                claimBeacon( RIGHT_SIDE );
                return true;
            }
            if(getBeaconColor( LEFT_SIDE ) == COLOR_RED){
                claimBeacon( LEFT_SIDE );
                return true;
            }
            return false;
        }

    }

    // If we claimed the wrong color, claim the beacon again.
    public void reclaimBeaconIfNeeded() throws InterruptedException{
        if ( verifyBeacon() == true ) return;
        sleep( 5000 );
        senseBeaconAndClaim();
    }

    // Color sorting utility, for beacon claiming.
    // Return:
    // 0 if we detect no color
    // 1 if we detect blue
    // 2 if we detect red.
    // Pass it true for right side, false for left side.
    int getBeaconColor( boolean whichSide) {
        if ( whichSide == RIGHT_SIDE) {
            // Only try if the there is enough color to say we might know, and
            // it is not a tie.
            if ((robot.right_color.red() + robot.right_color.blue() > 3) &&
                    (Math.abs(robot.right_color.red() - robot.right_color.blue()) >= 1))   {
                if (robot.right_color.red() > robot.right_color.blue()) {
                    return 2;
                } else {
                    return 1;
                }

            } else {
                return 0;
            }
        } else {
            if ((robot.left_color.red() + robot.left_color.blue() > 3) &&
                (Math.abs(robot.left_color.red() - robot.left_color.blue()) >= 1)) {
                if (robot.left_color.red() > robot.left_color.blue()) {
                    return 2;
                } else {
                    return 1;
                }

            } else {
                return 0;
            }
        }
    }

    // Make the moves to claim the beacon.
    // Pass this true to claim the right beacon, false to claim the left.
    // It pivots a small amount in the proper direction, moves forward, and
    // then backwards.
    public void claimBeacon( boolean whichSide) throws InterruptedException {
        double claimAngle = 7;
        if (whichSide ) {
            drive.pivotAngle(-claimAngle);
        } else {
            drive.pivotAngle(claimAngle);
        }
        // Drive forward to claim the beacon, and then back off.
        drive.ForTime( 700, 0.2);
        drive.ForTime( 300, -0.3);
        drive.pivotToAngle(setHeading); // Leave pointing straight at beacon.
    }

    // Verify that beacon is the right color after we claimed it.
    // If either color sensor shows wrong color, return false. Otherwise, return true.
    public boolean verifyBeacon() throws InterruptedException {
        if (teamColorBlue){
            if ((getBeaconColor( RIGHT_SIDE) == COLOR_RED ) || (getBeaconColor( LEFT_SIDE ) == COLOR_RED )){
            return false;
            }
        }else{
            if ((getBeaconColor( RIGHT_SIDE) == COLOR_BLUE ) || (getBeaconColor( LEFT_SIDE ) == COLOR_BLUE )){
                return false;
            }
        }
        return true;
    }

    ///////////////////////  SPECIAL MOVES ///////////////////////////////////
    // This is called after we claimed first beacon, and before we try to claim
    // the second.  If there is an obstable, go park on the ramp, because 5
    // points is better than just raming the other bot.
    public void retreatIfBeaconTwoBlocked() throws InterruptedException {
        if ((robot.range < 1.0 ) || (robot.range > 70.0 )){
            return;
        }


        if (teamColorBlue){
            drive.pivotToAngle( 5.0 );
            drive.Distance( -3.0 );
            drive.stopAndWait();
        }else{
            drive.pivotToAngle( -5.0 );
            drive.Distance( -3.0 );
            drive.stopAndWait();
        }


    }
    ///////////////////////  SHOOT BALL /////////////////////////////////////
    public void shootTheBall() throws InterruptedException {
        // Rotate shooter enough to fire ball.
        while(robot.shooterAngle <= 250){
            robot.updateSensors();
            robot.shooterMotor.setPower(0.5);
        }

        // Be sure we have gone at full revolution.
        // Lower power because it really wants to overshoot.
        while(robot.shooterAngle <= 340){
            robot.updateSensors();
            robot.shooterMotor.setPower(0.1);
        }

        // Back up until you are at one complete revolution
        while(robot.shooterAngle > 340 ){
            robot.updateSensors();
            robot.shooterMotor.setPower( -0.3 );
        }

        // Now run forward until just shy of a revolution.
        // It overshoots on average 10 degrees so we compensate.
        while(robot.shooterAngle < 357 ) {
            robot.updateSensors();
            robot.shooterMotor.setPower(0.05);
        }
        // Stop!
        robot.shooterMotor.setPower( 0.0 );
    }

    ////////////////////////////////////  EXECUTION BEGINS HERE ///////////////////////////////////
    @Override
    public void runOpMode() throws InterruptedException {

        robot = new HardwareChooChoo( this );
        drive = new driveMoves( this, robot );
        robot.init( this.hardwareMap, true );

        double mirror = 1.0;  // If on the Blue side, many moves are mirrored, and this is set to -1.0

        // Calibrate the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();

        //String calmsg = "Gyro status: " + robot.imu.getCalibrationStatus().toString();
        //telemetry.addLine( calmsg );
        //telemetry.update();
        sleep( 2000 );


        // Select a team by pressing either the red or the blue buttons.
        // Also, you can make the mission to be moving the ball instead
        // of blaming the beacons.
        // Will stay in this loop until the game starts, so you
        // can pick a different team if you want.
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

        //waitForGreen();
        // shootTheBall();
        /*
        waitForGreen();
        drive.Distance( 0.6);
        waitForGreen();

        drive.Distance( 2 );
        waitForGreen();
        drive.Distance( 4 );
        waitForGreen();
        drive.Distance( -4 );
        waitForGreen();
        */

        /*
        waitForGreen();
        drive.pivotToAngle( 10 );
        waitForGreen();
        drive.pivotToAngle( 0 );
        waitForGreen();
        drive.pivotToAngle( 10 );
        waitForGreen();
        drive.pivotToAngle( 0 );
        waitForGreen();
        drive.pivotToAngle( 90 );
        waitForGreen();
        drive.pivotToAngle( 0 );
        waitForGreen();
        drive.pivotToAngle( 90 );
        waitForGreen();
        drive.pivotToAngle( 0 );
        waitForGreen();
        drive.pivotToAngle( 45 );
        waitForGreen();
        drive.pivotToAngle( 0 );
        waitForGreen();
        drive.pivotToAngle( 45 );
        waitForGreen();
        drive.pivotToAngle( 0 );
        waitForGreen();
        */

        // drive.driveToColor();
        // waitForGreen();

        /////////////////////  AUTONOMOUS CODE TO MOVE THE BALL ////////////////
        // Mission: Move ball
        // Be able to execute this simple mission if needed.
        // Drive backwards because our foam pad makes it hard to push the ball
        // starts robot wheel on second seam in from corner, so robot is mostly
        // on second tile in.
        if (moveBall){
            sleep( 9000 );
            drive.desiredHeading = 0;
            drive.Distance(-1.0 );  // Away from wall.
            setHeading = mirror * -45;       // turn to center goal.
            drive.pivotToAngle( setHeading );
            drive.Distance(-0.8 );  //Closer to goal.
            shootTheBall();
            drive.Distance( -4 );   // And drive up onto wood.
            drive.stopAndWait();                      // And we're done.
        }


        /////////////////////  THE ACTUAL AUTONOMOUS MOVEMENTS ///////////////////////////
        // Starting location is square with back wall,
        // Frame lined up with inside seam of first tile with no corner goal.
        // tile seam edge closest to the goal.
        // Mostly the two sides are a mirror of each other, but the robots are a little different.

        // Get to Near the first beacon.
        drive.Distance( 0.4 );
        if (teamColorBlue){
           setHeading = 44;             // Point just a bit away from parallel to goal entrance.
        } else {
           setHeading = -44;
        }
        drive.pivotToAngle( setHeading );
        drive.Distance(3.7);            // Should end up near the line at about a 45 degree angle.
        //drive.pivotToAngle( 0 );        // Pointed toward perpendicular to line
        //waitForGreen();

                                        ////// FIRST BEACON ///////
        drive.driveToLine();            // Find Line
        /* Compensating for pivot point not needed
        if (teamColorBlue){             // compensate for sensor not being over pivot point.
            drive.Distance( 0.1);
        }else {
            drive.Distance( 0.1 );      //
        }
        */
        setHeading = -90 * mirror;
        drive.pivotToAngle( setHeading );  // Point to beacon
        //waitForGreen();
        drive.driveToRange( 13, false);   // Get as close as ultrasonic sensor will sense reliably
        //waitForGreen();
        //drive.Distance( 0.2 );            // Plus a little closer.  Should be good enough to claim.
        //drive.driveToColor( )
        if (senseBeaconAndClaim() == false ){ // claim that beacon!
            drive.Distance( 0.05 );          // If we missed first time, get closer,
            if (senseBeaconAndClaim() == false ){  // Try second time.
                drive.Distance( 0.05 );     // If we missed second time, forward
                senseBeaconAndClaim();      // Third time is a charm.
            }
        }
        reclaimBeaconIfNeeded();            // if we messed up try again!

                                                ////////////// Shoot Ball in Center Vortex //////////////
        drive.driveFromRange( 55.0, setHeading );  // Get to right distance to shoot the ball.
        drive.pivotIfNeeded(setHeading);
        shootTheBall();
        drive.driveToRange( 30.0, true );

                                                //// SECOND BEACON ////
        //waitForGreen();
        drive.pivotToAngle( 0 );                // Pointed toward perpendicular to line, slightly toward beacon

        retreatIfBeaconTwoBlocked();
        if (teamColorBlue ) {
            //waitForGreen();
            drive.Distance( 3.2 );      //   End up close to the  line, beacon.
        } else {
            //waitForGreen();
            drive.Distance( 3.2 );           // End up close to the  line, beacon.
        }
        // waitForGreen();
        drive.driveToLine();

        //if (teamColorBlue){
        //    drive.Distance( 0.1);
        //}else {
        //drive.Distance( 0.1);
        //}
        setHeading = -90 * mirror;
        drive.pivotToAngle( setHeading );  // Point to beacon
        // waitForGreen();
        drive.driveToRange( 13, false);   // Get as close as ultrasonic sensor will sense reliably
        //drive.Distance(0.2);
        //waitForGreen();
        // drive.driveToColor( );
        // waitForGreen();
        if (senseBeaconAndClaim() == false ){ // claim that beacon!
            drive.Distance( 0.05 );          // If we missed first time, get closer,
            //waitForGreen();
            if (senseBeaconAndClaim() == false ){  // Try second time.
                drive.Distance( 0.05 );     // If we missed second time, forward
                senseBeaconAndClaim();      // Third time is a charm.
            }
        }
        reclaimBeaconIfNeeded();
        drive.driveFromRange( 25, setHeading );    // Now back away from beacon.

        ///////////////////// PARK ON RAMP IF WE HAVE TIME
        // If we run out of time to park on the ramp, just stop.
        // if (matchTimer.seconds() > 26) drive.stopAndWait();
        if (teamColorBlue){
            drive.pivotToAngle( 181.0 );
            drive.Distance( 7.0 );
            drive.stopAndWait();
        }else{
            drive.pivotToAngle( -181.0 );
            drive.Distance( 7.0 );
            drive.stopAndWait();
        }
        drive.stopAndWait();

    }
}
