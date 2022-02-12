/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import java.io.BufferedWriter;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 */
public class JohnMap
{
    /* Public OpMode members. */

    public BNO055IMU imu;
    public DcMotor rearMotor, rightMotor, leftMotor, arm;
    public DcMotorSimple duck, turntable, intake;
    public RevBlinkinLedDriver blinkin;

    public boolean robotCentric = true;
    public boolean lowGear = false;

    public final double duckySpeed = 1.0;

    /* local OpMode members. */
    HardwareMap hwMap;

    public JohnMap() {}

    public void init( HardwareMap ahwMap ) {

        // Saving a reference to the Hardware map...
        hwMap = ahwMap;


        /*
         *  Motors
         */

        // Instantiating the Motors...
        rearMotor = hwMap.get( DcMotor.class, "Motor3" );
        rightMotor = hwMap.get( DcMotor.class, "Motor2" );
        leftMotor = hwMap.get( DcMotor.class, "Motor1" );
        arm = hwMap.get( DcMotor.class, "Arm" );

        // Inverting the Motors...
        rearMotor.setDirection( DcMotorSimple.Direction.REVERSE );
        rightMotor.setDirection( DcMotorSimple.Direction.REVERSE );
        leftMotor.setDirection( DcMotorSimple.Direction.REVERSE );
        arm.setDirection( DcMotorSimple.Direction.FORWARD );

        // Setting to run using Encoders...
        rearMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODERS );
        rightMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODERS );
        leftMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODERS );
        arm.setMode( DcMotor.RunMode.RUN_USING_ENCODERS );

        // Setting 0 Power Behavior...
        rearMotor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
        rightMotor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
        leftMotor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
        arm.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );


        /*
         *  Servos
         */

        // Defining and Instantiating the Servos...
        duck = hwMap.get( DcMotorSimple.class, "Duck" );
        turntable = hwMap.get( DcMotorSimple.class, "Turntable" );
        intake = hwMap.get( DcMotorSimple.class, "Intake" );

        // Setting the Directions of the Servos...
        duck.setDirection( DcMotorSimple.Direction.FORWARD );
        turntable.setDirection( DcMotorSimple.Direction.FORWARD );
        intake.setDirection( DcMotorSimple.Direction.REVERSE );


        /*
         *  Blinkin
         */

        // Instantiating Blinkin...
        blinkin = hwMap.get( RevBlinkinLedDriver.class, "Blinkin" );


        /*
         *  IMU
         */

        // Instantiating IMU Parameters, setting angleUnit...
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        // Instantiating IMU... Initializing it with the above Parameters...
        imu = hwMap.get( BNO055IMU.class, "IMU" );
        imu.initialize( parameters );
    }


    /*
     *  MOTORS
     */

    /**
     * Moves the robot. No spinning, no saucing. Just linear-like, across-the-floor
     *  movement.
     *
     * @param speed the speed the robot will move at
     * @param angle the angle at which the robot will move
     */
    public void move( double speed, double angle ) {

        rearMotor.setPower( speed * Math.cos( Math.toRadians( angle + 180 ) ) );
        rightMotor.setPower( speed *  Math.cos( Math.toRadians( angle + 60 ) ) );
        leftMotor.setPower( speed * Math.cos( Math.toRadians( angle + 300 ) ) );
    }

    /**
     * Spins the robot. No linear movement, so no saucin'. Just spinning in place.
     *
     * @param speed the speed the robot will spin at
     */
    public void spin( double speed ) {

        if( robotCentric )
        {
            rearMotor.setPower( -speed );
            rightMotor.setPower( -speed );
            leftMotor.setPower( -speed );
        }
        else
        {
            rightMotor.setPower( -speed );
            leftMotor.setPower( -speed );
        }
    }

    /**
     * Combines spinning and linear-like movement to sauce. The robot will move across
     *  the floor while spinning. (Moves on a line, spinning all the while...)
     *
     * @param speed the speed the robot will move across the floor (on a line) at
     * @param spinSpeed the speed the robot will spin at
     * @param angle the angle at which the robot will move across the floor (on a line) at
     * @param deltaHeading how far, in degrees, the robot has spun on the elevated-floor-like
     *  axis. The change in Heading. Is updated with every loop
     */
    public void sauce( double speed, double spinSpeed, double angle, double deltaHeading )
    {
        double sum = speed + spinSpeed;
        
        if( lowGear ) 
        {
            speed = ( speed / sum ) * 0.5;
            spinSpeed = ( spinSpeed / sum ) * 0.5;
        }
        else 
        {            
            speed /= sum;
            spinSpeed /= sum;
        }
        
        if( robotCentric )
        {
            rearMotor.setPower( -spinSpeed + ( speed * Math.cos( Math.toRadians( angle + 180 - deltaHeading ) ) ) );
            rightMotor.setPower( -spinSpeed + ( speed * Math.cos( Math.toRadians( angle + 60 - deltaHeading ) ) ) );
            leftMotor.setPower( -spinSpeed + ( speed * Math.cos( Math.toRadians( angle + 300 - deltaHeading ) ) ) );
        }
        else
        {
            rearMotor.setPower( speed * Math.cos( Math.toRadians( angle + 180 - deltaHeading ) ) );
            rightMotor.setPower( -spinSpeed + ( speed * Math.cos( Math.toRadians( angle + 60 - deltaHeading ) ) ) );
            leftMotor.setPower( -spinSpeed + ( speed * Math.cos( Math.toRadians( angle + 300 - deltaHeading ) ) ) );
        }
    }

    public void resetEncoders()
    {
        rearMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        rightMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        leftMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        arm.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

        rearMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODERS );
        rightMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODERS );
        leftMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODERS );
        arm.setMode( DcMotor.RunMode.RUN_USING_ENCODERS );
    }

    public void liftArm( double speed )
    {
        arm.setPower( speed );
    }
    
    public int getArmEncoderPosition()
    {
        return arm.getCurrentPosition();
    }
    
    
    /*
     *  RECORDING
     */
    
   // public void record( double rearPower, double rightPower, double leftPower, String dirName, String fileName )
   // {
   //     BufferedWriter writer = new BufferedWriter( "org.firstinspires.ftc.teamcode." + dirName + "." + fileName );
   //     writer.write( rearPower + rightPower + leftPower + "\n" );
        
  //  }

    /*
     *  SERVOS
     */

    public void spinDuckyRight()
    {
        duck.setPower( duckySpeed );
    }

    public void spinDuckyLeft()
    {
        duck.setPower( -duckySpeed );
    }

    public void turnTurntable( double speed )
    {
        turntable.setPower( speed );
    }

    public void intake( double speed )
    {
        intake.setPower( speed );
    }


    /*
     *  Blinkin
     */

    public void patternBlinkin( RevBlinkinLedDriver.BlinkinPattern pattern )
    {
        blinkin.setPattern( pattern );
    }


    /*
     *  IMU
     */

    /**
     * Gets the robot's current Heading, ( the robot's current angular orientation
     *  on an elevated-floor-like axis ).
     *
     * @return the robot's current heading
     */
    public double getHeading()
    {
        return (double) imu.getAngularOrientation().firstAngle;
    }


    /*
     *  MISC.
     */

    /** 
     * Gets the time for the robot.
     * 
     * @return the time, in millis
     */
    public double getTime()
    {
        return System.currentTimeMillis() / 1000;
    }

    /**
     * Stops the robot.
     */
    public void stopRobot()
    {
        // Stopping Wheels...
        stopWheels();

        // Stopping Arm...
        stopArm();

        // Stopping Ducky...
        stopDucky();

        // Stopping Turntable...
        stopTurntable();

        // Stopping Intake...
        stopIntake();
    }

    /**
     * Stops the robot's 3 wheels.
     */
    public void stopWheels()
    {
        // Stopping Wheels...
        rearMotor.setPower( 0 );
        rightMotor.setPower( 0 );
        leftMotor.setPower( 0 );
    }

    /**
     * Stops the robot's arm.
     */
    public void stopArm()
    {
        // Stopping Arm...
        arm.setPower( 0 );
    }

    /**
     * Stops the robot's ducky.
     */
    public void stopDucky()
    {
        // Stopping Ducky...
        duck.setPower( 0 );
    }

    /**
     * Stops the robot's turntable.
     */
    public void stopTurntable()
    {
        // Stopping Turntable...
        turntable.setPower( 0 );
    }

    /**
     * Stops the robot's intake.
     */
    public void stopIntake()
    {
        // Stopping Intake...
        intake.setPower( 0 );
    }

    /**
     * Resets the wheels' encoders.
     */
    public void resetWheelEncoders()
    {
        rearMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        rightMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        leftMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

        rearMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
        rightMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
        leftMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
    }
}