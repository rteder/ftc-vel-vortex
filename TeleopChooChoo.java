/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

@TeleOp(name="ChooChoo", group="Tekceratops")
//@Disabled
public class TeleopChooChoo extends OpMode{
    // Objects that correspond to hardware devices.
    DcMotor  leftMotor   = null;
    DcMotor  rightMotor  = null;
    DcMotor harvesterMotor = null;
    DcMotor shooterMotor = null;
    DcMotor liftMotor = null;
    //ModernRoboticsI2cGyro gyro = null;
    //ColorSensor left_color = null;
    //UltrasonicSensor ultrasonicSensor = null;
    //LightSensor lightSensor = null;
    Servo manipulator = null;

    // Global variables.
    private ElapsedTime shooterTimer = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();
    int shooterEncoder = 0;
    int shooterCountsPerRev = 1680;     // Using Neverest 60 motor.
    int harvesterEncoder = 0;
    int harvesterCountsPerHalfRev = 720;
    double upPosition = 0.75;
    double downPosition = 0.15;
    double shooterAngle = 0;
    double harvesterAngle = 0;
    double shooterPower = 0.0 ;
    boolean tryToCock = false;
    boolean harvesterRunning = false;


    // Code to run ONCE when the driver hits INIT.
    // This initializes all of the hardware.
    @Override
    public void init() {
        // Get the hardware map:
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor  = hardwareMap.dcMotor.get("right_drive");
        rightMotor.setDirection( DcMotor.Direction.REVERSE );


        harvesterMotor = hardwareMap.dcMotor.get("harvester");
        harvesterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        harvesterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        harvesterMotor.setZeroPowerBehavior( BRAKE );
        harvesterMotor.setDirection( DcMotor.Direction.REVERSE);
        manipulator = hardwareMap.servo.get("manipulator");
        liftMotor = hardwareMap.dcMotor.get("lift");
        shooterMotor = hardwareMap.dcMotor.get( "shooter");
        shooterMotor.setDirection( DcMotor.Direction.REVERSE );
        shooterMotor.setZeroPowerBehavior( BRAKE );
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Positive means we are winding the spring.
        shooterMotor.setDirection(  DcMotor.Direction.REVERSE);


        //left_color = hardwareMap.colorSensor.get("left_color");
        //gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        //ultrasonicSensor = hardwareMap.ultrasonicSensor.get("sonar");
        //lightSensor = hardwareMap.lightSensor.get("light");


        // Set motor powers to zero
        leftMotor.setPower( 0 );
        rightMotor.setPower( 0 );

        // This is giving trouble
         shooterMotor.setPower( 0 );


        // Set the LEDs off in the beginning
        // left_color.enableLed( false );
        //lightSensor.enableLed(true);


        // A deadzone for the joysticks is needed because they don't always
        // go to zero at rest.
        gamepad1.setJoystickDeadzone( (float) 0.05 );
        gamepad2.setJoystickDeadzone( (float) 0.05 );

        manipulator.setPosition(upPosition);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Bot", "ChooChoo");
        updateTelemetry(telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */

    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    ////////////////////////////  MAIN CONTROL LOOP //////////////////////////////////////
    @Override
    public void loop() {
        double left = 0;
        double right = 0;
        double harvesterPower = 0;
        int heading = 0;
        double range = 0;
        double light = 0;
        boolean cocked = false;
        String cockedStatus = "WINDING";
        String colorStatus = "";
        double harvesterPosition = 0;

        /////////////////////////////////  Drive //////////////////////////
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = (double) Range.clip(-gamepad1.left_stick_y, -1, 1);
        right = (double) Range.clip(-gamepad1.right_stick_y, -1, 1);
        leftMotor.setPower(left);
        rightMotor.setPower(right);


        ///////////////////////// Harvester Control ///////////////////////////////////
        // Left joystick is variable speed in caase you want to ever go slower than max.
        // Left trigger is harvest full speed
        // Left Bumper is un-harvest, full speed.
        // Yellow is harvest slowly to keep balls from coming out.
        // harvesterPower = (double) gamepad2.left_stick_y * -1.0;
        harvesterEncoder = harvesterMotor.getCurrentPosition();
        int harvesterEncMod = harvesterEncoder % harvesterCountsPerHalfRev;
        harvesterAngle = 180 * (double) harvesterEncMod / (double) harvesterCountsPerHalfRev;
        // harvesterAngle = Math.abs(harvesterAngle);

        if (gamepad2.left_trigger > 0.5) {
            harvesterPower = 1.00;
            harvesterRunning = true;

        } else if (gamepad2.left_bumper) {
            harvesterPower = -1.00;
            harvesterRunning = true;
        } else {
            harvesterRunning = false;
            harvesterPower = 0.00;
        }
        // Makes sure harvester is zeroed.
        if (harvesterRunning == false){
            if ((harvesterAngle > 165|| (harvesterAngle < 30))){
                // Right where we want it!
                harvesterPower = 0.0;
            } else if (harvesterAngle > 120){
                // Almost there, go really slow
                harvesterPower = 0.05;
            } else {
                // Run until we are closer.
                harvesterPower = 0.1;
            }
        }

        harvesterMotor.setPower(harvesterPower);
        ////////////////////////////////// Manipulator Control /////////////////////////////////////////
        if (gamepad1.y) {
            manipulator.setPosition(downPosition);
        }

        if (gamepad1.a) {
            manipulator.setPosition(upPosition);
        }

        ////////////////////////////////////// Lift Control /////////////////////////////////////////////

        if( gamepad2.y) {
            liftMotor.setPower( 1.0 );

        } else if (gamepad2.a) {
            liftMotor.setPower( -1.0 );
        } else  {
            liftMotor.setPower( 0 );
        }


        ////////////////// Shooter Control ////////////////////////////////////
        // Right trigger shoots, right bumper cocks.
        // Use encoder to determine it it is in cocked position.

        // This angle sets where the shooter should nominally be cocked.
        // If it cannot quite get to cock, this is too low.
        // if it just keeps running, this is too high:
        float cockedAngle = 290;

        shooterEncoder = shooterMotor.getCurrentPosition();
        int encMod = shooterEncoder % shooterCountsPerRev;
        shooterAngle = 360 * (double) encMod / (double) shooterCountsPerRev;

        if (gamepad2.right_trigger > 0.5) {
            shooterPower = 1.000;
            shooterTimer.reset();
            tryToCock = false;
        } else {
            if (gamepad2.right_bumper) {
                tryToCock = true;
                shooterTimer.reset();
            }
            // If we are trying to cock, run the motor if it is not there.
            if (tryToCock) {
                // If we are less than the desired cocked angle, run shooter at full power.
                if (shooterAngle < cockedAngle - 100 ) {
                    shooterPower = 0.20;   // 0 - 210
                    // telemetry.addData("cocking", 0);
                    // To prevent motor burn out, limit to this much time.
                    if (shooterTimer.milliseconds() > 5000) {
                        shooterPower = 0;
                        tryToCock = false;
                    }

                } else if (shooterAngle < cockedAngle - 55) {
                    shooterPower = 0.1;   // 210- 250
                } else if (shooterAngle < cockedAngle - 30) {
                    shooterPower = 0.05; // 250 - 280
                } else if (shooterAngle < cockedAngle - 10) {
                    shooterPower = 0.01; // 280 - 320
                } else if (shooterAngle < cockedAngle + 15) {
                    shooterPower = 0.0;
                }else if (shooterAngle < cockedAngle + 30) {
                    shooterPower = -0.01; // 300 - 350  too far, see if we can creep back
                } else {
                    // Past where we can cock, run around to next.
                    shooterPower = 0.20; //350 - 360
                    //telemetry.addData("past", 0);
                }
            }
            else {
                shooterPower = 0;
            }
        }

        // Emergency override: if you press he red button the shooter stops.
        if (gamepad2.b) shooterPower = 0;
        // Comment out the following if you ever need to disable the shooter:
        shooterMotor.setPower( shooterPower );

        //////////////////////  Telemetry /////////////////////////////////////////
        // Send useful data about the robot status:
        /*
        heading = gyro.getHeading();
        range = ultrasonicSensor.getUltrasonicLevel();
        light = lightSensor.getLightDetected();
        left_color.enableLed( false );
        */

        //telemetry.addData("left", String.format("%.2f", left) + "  right: " + String.format("%.2f", right)
        //        + " harv" + String.format("%.2f", harvesterPower));

        telemetry.addData("shooterPower", String.format("%.2f", shooterPower));
        telemetry.addData("shooter Angle", String.format("%.1f", shooterAngle));
        //telemetry.addData("Harvester Angle", String.format("%.2f", harvesterAngle));
        //telemetry.addData("Harvester Power", String.format("%.2f", harvesterPower));
        // colorStatus = String.format("Left R %d B %d ", left_color.red(), left_color.blue() );
        // telemetry.addData("Colors ", colorStatus + "Range: " + String.format("%.2f", range )
        // + " Light: " + String.format("%.2f", light));
        // updateTelemetry(telemetry);

        telemetry.addData("loop time mS", loopTimer.milliseconds());
        loopTimer.reset();
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
