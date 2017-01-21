package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

@Autonomous(name="EasyAuto", group="TekC")
//@Disabled

public class EasyAuto extends LinearOpMode {
    // Objects that correspond to hardware devices.
    DcMotor  leftMotor   = null;
    DcMotor  rightMotor  = null;
    UltrasonicSensor ultrasonicSensor = null;
    private ElapsedTime driveTimer = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();
    double range = 0;

    public void stopDrive()throws InterruptedException{
        leftMotor.setPower( 0.0 );
        rightMotor.setPower( 0.0 );
    }

    public void driveToRange( double targetRange )throws InterruptedException{
        while ((range > targetRange)||(range < 1)) {
            range = ultrasonicSensor.getUltrasonicLevel();
            leftMotor.setPower(0.5);
            rightMotor.setPower(0.5);
            idle();
        }
            stopDrive();

    }

    public void driveForTime( long driveTime )throws InterruptedException {
        driveTimer.reset();
        while (driveTimer.milliseconds() < driveTime){
            leftMotor.setPower(0.5);
            rightMotor.setPower(0.5);
            idle();
        }
        stopDrive();
    }
    public void stopAndWait()throws InterruptedException {
        rightMotor.setPower( 0.0 );
        leftMotor.setPower( 0.0 );
    }


    ////////////////////////////////////  EXECUTION BEGINS HERE ///////////////////////////////////
    // This is the program that actually runs.
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the motor objects so they can communicate with the hardware.
        // The names like 'left_drive" need to be identical to those you set up
        // in the phone config.
        rightMotor = hardwareMap.dcMotor.get("left_drive");
        leftMotor  = hardwareMap.dcMotor.get("right_drive");
        rightMotor.setDirection( DcMotor.Direction.REVERSE );
        ultrasonicSensor = hardwareMap.ultrasonicSensor.get("sonar");




        // Stay here until the play button is pressed:
        waitForStart();
        range = ultrasonicSensor.getUltrasonicLevel();
        // Drive until there is an obstacle

        sleep( 10000 );
        driveToRange( 30 );
        driveForTime( 1000 );


        //driveForTime( 3255 );
        //pivotRightForTime( 3000 );
        //driveForTime( 1300 );

    }
}