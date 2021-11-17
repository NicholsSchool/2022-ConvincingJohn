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
public class KiwiMap
{
    /* Public OpMode members. */
    public BNO055IMU imu;
    
    public DcMotor frontMotor, rightMotor, leftMotor;
    public DcMotorSimple duck, turntable, intake;
    public RevBlinkinLedDriver blinkin;
    
    public boolean robotCentric = true;

    /* local OpMode members. */
    HardwareMap hwMap;

    public KiwiMap() {}

    public void init( HardwareMap ahwMap ) {
        
        // Saving a reference to the Hardware map...
        hwMap = ahwMap;
        
        
        /*
         *  IMU
         */
        
        // Instantiating IMU Parameters, setting angleUnit...
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        
        // Defining and Initializing IMU... Initializing it with the above Parameters...
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        
        /*
         *  MOTORS
         */
         
        // Defining and Initializing the Motors...
        frontMotor = hwMap.get( DcMotor.class, "Motor1" );
        rightMotor = hwMap.get( DcMotor.class, "Motor2" );
        leftMotor = hwMap.get( DcMotor.class, "Motor3" );
        
        // Inverting the Motors...
        frontMotor.setDirection( DcMotorSimple.Direction.REVERSE );
        rightMotor.setDirection( DcMotorSimple.Direction.REVERSE );
        leftMotor.setDirection( DcMotorSimple.Direction.REVERSE );
        
        // Setting to run using Encoders...
        frontMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODERS );
        rightMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODERS );
        leftMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODERS );
        
        // Setting Brake as 0 Power Behavior...
        frontMotor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
        rightMotor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
        leftMotor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );


        /*
         *  SERVOS
         */

        // Defining and Initializing the Servos...
        duck = hwMap.get( DcMotorSimple.class, "Duck" );
        turntable = hwMap.get( DcMotorSimple.class, "Turntable" );
        intake = hwMap.get( DcMotorSimple.class, "Intake" );

        // Setting the Directions of the Servos...
        duck.setDirection( DcMotorSimple.Direction.FORWARD );
        turntable.setDirection( DcMotorSimple.Direction.FORWARD );
        intake.setDirection( DcMotorSimple.Direction.FORWARD );

        
        /*
         *  BLINKIN
         */

        // Defining and Initializing Blinkin...
        blinkin = hwMap.get( RevBlinkinLedDriver.class, "Blinkin" );
    }
    
    
    /*
     *  IMU
     */
    
    /**
     * Gets the robot's current Heading, ( the robot's current angular orientation 
     *  on an elevated-floor-like axis ). 
     * 
     * @return the robot's current Heading
     */
    public double getHeading() {
        
        return (double) imu.getAngularOrientation().firstAngle;
    }

    
    /**
     * Moves the robot. No spinning, no saucing. Just linear-like, across-the-floor
     *  movement.
     * 
     * @param speed the speed the robot will move at
     * @param angle the angle at which the robot will move
     */
    public void move( double speed, double angle ) {
        
        frontMotor.setPower( speed * Math.cos( Math.toRadians( angle ) ) );
        rightMotor.setPower( speed * Math.cos( Math.toRadians( angle + 120 ) ) );
        leftMotor.setPower( speed * Math.cos( Math.toRadians( angle + 240 ) ) );
    }
    
    /**
     * Spins the robot. No linear movement, so no saucin'. Just spinning in place.
     * 
     * @param speed the speed the robot will spin at
     */
    public void spin( double speed ) {
        
        if( robotCentric ) 
        {
            frontMotor.setPower( speed );
            rightMotor.setPower( speed );
            leftMotor.setPower( speed );
        }
        else 
        {
            rightMotor.setPower( speed );
            leftMotor.setPower( speed );
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
    public void sauce( double speed, double spinSpeed, double angle, double deltaHeading ) {
        
        if( robotCentric ) 
        {
            frontMotor.setPower( spinSpeed + ( speed * Math.cos( ( Math.PI / 180 ) * ( angle + ( 0 - deltaHeading ) ) ) ) );
            rightMotor.setPower( spinSpeed + ( speed * Math.cos( ( Math.PI / 180 ) * ( angle + ( 120 - deltaHeading ) ) ) ) );
            leftMotor.setPower( spinSpeed + ( speed * Math.cos( ( Math.PI / 180 ) * ( angle + ( 240 - deltaHeading ) ) ) ) );
        } 
        else 
        {        
        frontMotor.setPower( speed * Math.cos( Math.toRadians( angle + ( 0 - deltaHeading ) ) ) );
        rightMotor.setPower( spinSpeed + ( speed * Math.cos( Math.toRadians( angle + ( 120 - deltaHeading ) ) ) ) );
        leftMotor.setPower( spinSpeed + ( speed * Math.cos( Math.toRadians( angle + ( 240 - deltaHeading ) ) ) ) );
        }
    }

    public void sauce2( double r, double theta, double deltaHeading) {
        frontMotor.setPower( ( 1 - r ) + ( r * Math.cos( Math.toRadians( theta + ( 0 - deltaHeading ) ) ) ) );
        rightMotor.setPower( ( 1 - r ) + ( r * Math.cos( Math.toRadians( theta + ( 120 - deltaHeading ) ) ) ) );
        leftMotor.setPower( ( 1 - r ) + ( r * Math.cos( Math.toRadians( theta + ( 240 - deltaHeading ) ) ) ) );
    }
    
    
    /*
     *  MISC.
     */
    
    /**
     * Stops the robot. More accurately, stops all 3 of the robot's motors. 
     */
    public void stop() 
    {    
        // Stopping Motors...
        frontMotor.setPower( 0 );
        rightMotor.setPower( 0 );
        leftMotor.setPower( 0 );

        // Stopping Servos...
        duck.setPower( 0 );
        turntable.setPower( 0 );
        intake.setPower( 0 );
    }
}
