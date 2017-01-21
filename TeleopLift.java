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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static android.R.attr.left;
import static android.R.attr.right;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Lift", group="Tekceratops")
//@Disabled
public class TeleopLift extends OpMode{
    public DcMotor  leftMotor   = null;
    public DcMotor  rightMotor  = null;
    public  DcMotor liftMotor = null;
    int phase = 0;
    private ElapsedTime phaseTimer = new ElapsedTime();


    /* Declare OpMode members. */
    //HardwarePushbot robot       = new HardwarePushbot(); // use the class created to define a Pushbot's hardware
                                                         // could also use HardwarePushbotMatrix class.

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor  = hardwareMap.dcMotor.get("right_drive");
        rightMotor.setDirection( DcMotor.Direction.REVERSE );
        liftMotor = hardwareMap.dcMotor.get("lift");
        //harvooterMotor.setDirection( DcMotor.Direction.REVERSE);

        // A deadzone for the joysticks is needed because they don't always
        // go to zero at rest.
        gamepad1.setJoystickDeadzone( (float) 0.05 );
        gamepad2.setJoystickDeadzone( (float) 0.05 );

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Bot:", "Lift");    //
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

    // Nomo
    void crab( boolean directionRight  ){
        double PivotPower = 0.2;
        double MovePower = 0.2;
        switch ( phase) {
            case 0:
                if (directionRight) {
                    // Pivot left
                    leftMotor.setPower(-PivotPower);
                    rightMotor.setPower(+PivotPower);
                } else {
                    // Pivot right
                    leftMotor.setPower(+PivotPower);
                    rightMotor.setPower(-PivotPower);
                }
                break;
            case 1:
                // Back up
                leftMotor.setPower(-MovePower);
                rightMotor.setPower(-MovePower);
                break;
            case 2:
                if (directionRight) {
                    // Pivot right
                    leftMotor.setPower(+PivotPower);
                    rightMotor.setPower(-PivotPower);
                } else {
                    // Pivot left
                    leftMotor.setPower(-PivotPower);
                    rightMotor.setPower(+PivotPower);

                }
                break;
            case 3:
                // Go forward
                leftMotor.setPower(MovePower);
                rightMotor.setPower(MovePower);
                break;
        }
    }

    @Override
    public void loop() {
        double left;
        double right;

        // Phase should increment from 0 to 3 repeatedly,
        // incrementing every 300 mS
        if (phaseTimer.milliseconds() > 300) {
            phaseTimer.reset();
            phase++;
            if (phase > 3) phase = 0;
        }

        // If gamepad x or b is pressed, crab left or right.
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        if(gamepad1.x || gamepad1.b) {
             if (gamepad1.x) crab( false );
            if (gamepad1.b) crab( true );
        }
        else {
            left = (double) Range.clip(-gamepad1.left_stick_y, -1, 1);
            right = (double) Range.clip(-gamepad1.right_stick_y, -1, 1);
            leftMotor.setPower(left);
            rightMotor.setPower(right);
        }

        // Control the lift
        if( gamepad1.y) {
            liftMotor.setPower( 1.0 );

        } else if (gamepad1.a) {
            liftMotor.setPower( -1.0 );
        } else  {
            liftMotor.setPower( 0 );
        }



        // Send useful data about the robot status:
        //telemetry.addData("left", String.format("%.2f", left) + "  right: " + String.format("%.2f", right));
        telemetry.addData("phase", phase);
       updateTelemetry(telemetry);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
