package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static android.R.attr.left;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

/**
 * This is NOT an opmode.  A class to define the hardware.
 * See below for what must be defined on the robot for this to work.
 */
public class HardwareChooChoo
{
    LinearOpMode opMode; // Allows us to use stuff from the calling linear opMode objec.

    // Objects that correspond to hardware devices.
    public DcMotor  leftMotor   = null;
    public DcMotor  rightMotor  = null;
    public DcMotor harvesterMotor = null;
    public DcMotor shooterMotor = null;
    public TouchSensor cockedTouchSensor = null;
    public ColorSensor right_color = null;
    public ColorSensor left_color = null;
    public UltrasonicSensor ultrasonicSensor = null;
    public LightSensor lightSensor = null;
    private I2cAddr right_color_address = I2cAddr.create8bit(0x70);
    Servo manipulator = null;

    // The IMU sensor object  (Adafruit Gyro)  and variables.
    BNO055IMU imu;
    public Orientation angles;
    public Acceleration gravity;

    // Properties that correspond to values of the hardware.
    public double harvesterPower = 0;
    public double shooterPower = 0 ;
    int shooterEncoder = 0;
    double shooterAngle = 0;
    int shooterCountsPerRev = 1680;     // Using Neverest 60 motor.


    double upPosition = 0.75;
    public int total_distance_counts = 0;
    public double total_distance_feet = 0;
    int heading_angle_counts = 0;
    public double gyro = 0; // Raw reading
    public boolean headingNegative = false;
    public double heading = 0;
    public double prevHeading = 0;
    public double angularVel = 0;
    public double angularSpeed = 0;
    public boolean spinning = false;
    public double range = 0;
    public double light = 0;
    double[] lightReadings = { 0 , 0, 0, 0, 0, 0, 0};
    public double avgLight = 0;

    private ElapsedTime sampleTimer = new ElapsedTime();
    private ElapsedTime sampleTimer2 = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();



    // Scaling for distance conversions.
    // Distance per rev of 4" wheels is pi * 4", or 1.047 ft.   1120 counts per rev from encoder.
    // So nominal counts per foot should be 1120 / 1.047 = 1070.
    // But times two because we add both encoders, so 2140 Adjust as needed.
    public static final int COUNTS_PER_FOOT = 2140;


    /* local OpMode members. */
    HardwareMap hardwareMap =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareChooChoo(LinearOpMode opMode ) throws InterruptedException{
        this.opMode = opMode;
    }

    /* Initialize standard Hardware interfaces */
    public void init( HardwareMap ahwMap, boolean reversed) throws InterruptedException {
        ///////////////////// INITIALIZATION /////////////////////////////////////////
        // Get the hardware map:
        //  *** IMPORTANT: LEFT AND RIGHT REVERSED FROM PHONE CONFIG! ***
        // We use the same hardware configuration that we use for telop. Left and right in that
        // configuration are labeled assuming the harvester end is the front of the robot.
        // But for autonomous, we consider the sensor end to be the front of the robot.
        //  So, we reverse Left and Right from the hardware map.
        // Save reference to Hardware map.
        hardwareMap = ahwMap;

        if (reversed) {
            rightMotor = hardwareMap.dcMotor.get("left_drive");
            leftMotor = hardwareMap.dcMotor.get("right_drive");
            rightMotor.setDirection(DcMotor.Direction.REVERSE);
        } else {
            rightMotor = hardwareMap.dcMotor.get("right_drive");
            leftMotor = hardwareMap.dcMotor.get("left_drive");
            rightMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        rightMotor.setZeroPowerBehavior( BRAKE );
        leftMotor.setZeroPowerBehavior( BRAKE );
        harvesterMotor = hardwareMap.dcMotor.get("harvester");
        shooterMotor = hardwareMap.dcMotor.get( "shooter");
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setDirection(  DcMotor.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(BRAKE);
        manipulator = hardwareMap.servo.get("manipulator");

        // Reset the encoders, set power to zero.
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setPower( 0 );
        rightMotor.setPower( 0 );
        shooterMotor.setPower( 0 );

        // Ultrasonic and light sensors:
        ultrasonicSensor = hardwareMap.ultrasonicSensor.get("sonar");
        lightSensor = hardwareMap.lightSensor.get("light");
        lightSensor.enableLed(true);



        left_color = hardwareMap.colorSensor.get("left_color");
        right_color = hardwareMap.colorSensor.get("right_color");
        right_color.setI2cAddress( right_color_address);

        //gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        // ADAFRUIT IMU
        // Set up the parameters with which we will use our IMU.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        manipulator.setPosition(upPosition);


    }
    // Update the sensors and display telemetry.
    void updateSensors()throws InterruptedException {
        String colorStatus = "";

        // Add the two encoder values to get the current position in counts.
        // Use the constant to convert this to feet.
        total_distance_counts = leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition();
        total_distance_feet = (double) total_distance_counts / COUNTS_PER_FOOT;

        // HEADING
        // Output from the gyro is 0 to 360, but we want
        // positive and negative outputs, so we can go to, say, -10 degrees.
        // And, no discontinuity about 0.
        // We use a flag to detect that we are just positive or negative about zero.
        // This will work as long as you don't go close to all the way around.
        // Read the Adafruit Gyro  (IMU = Inertial Measurement Unit)
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        gravity  = imu.getGravity();
        // Sign is reversed compared to the MR Gyro we used to use.
        gyro = -angles.firstAngle;
        if (gyro > 350) headingNegative = true;
        if (gyro < 10) headingNegative = false;
        if (headingNegative) {
            heading = gyro - 360.0;
        }
        else {
            heading = gyro;
        }
        getAngularSpeed();

        range = ultrasonicSensor.getUltrasonicLevel();

        readLight();

        // Read the shooter angle, but not modulus calculations.
        shooterEncoder = shooterMotor.getCurrentPosition();
        int encMod = shooterEncoder; //  % shooterCountsPerRev;
        shooterAngle = 360 * (double) encMod / (double) shooterCountsPerRev;


        // If any of the flags are set, show data on telemetry.
        opMode.telemetry.addLine("feet:" + String.format("%.1f ", total_distance_feet) +
                    "Heading: " + String.format("%.2f", heading ));

          colorStatus = String.format("Color Left R: %d B:%d  Right R: %d B: %d   ", left_color.red(), left_color.blue(),
                  right_color.red(), right_color.blue());
        opMode.telemetry.addLine( colorStatus );

        //opMode.telemetry.addData( "AngSpeed", angularSpeed);
        //opMode.telemetry.addData("spin", spinning);
        // opMode.telemetry.addData("heading", heading );
        // opMode.telemetry.update();
        opMode.telemetry.addData("Shooter Angle", shooterAngle);

        // opMode.telemetry.addData("loop time mS", loopTimer.milliseconds());
        loopTimer.reset();
        opMode.telemetry.update();

        opMode.idle();
    }


    // Process the light.  Because we are looking for the transition of a line, we
    // maintain an average of readings 50 mS apart.
    void readLight() throws InterruptedException {
        double Summation = 0;

        light = lightSensor.getLightDetected();

        if (sampleTimer.milliseconds() > 50.0 ) {
            sampleTimer.reset();
            lightReadings[5] = lightReadings [ 4 ];
            lightReadings[4] = lightReadings [ 3 ];
            lightReadings[3] = lightReadings [ 2 ];
            lightReadings[2] = lightReadings [ 1 ];
            lightReadings[1] = lightReadings [ 0 ];
            lightReadings[ 0 ] = light;
            Summation = lightReadings[1] + lightReadings[2] + lightReadings[3] + lightReadings[4] + lightReadings[5];
            avgLight = Summation / 5;
        }
    }


    void getAngularSpeed( )throws InterruptedException  {
        if (sampleTimer2.milliseconds() > 200 ) {
            sampleTimer2.reset();
            angularVel = heading - prevHeading;
            angularSpeed = Math.abs(angularVel);
            spinning = angularSpeed >= 1;
            prevHeading = heading;
        }
    }


    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

