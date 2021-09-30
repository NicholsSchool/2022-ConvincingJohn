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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *  This code requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 *  This is performed when the INIT button is pressed on the Driver Station.
 *  This code assumes that the robot is stationary when the INIT button is pressed.
 *  If this is not the case, then the INIT should be performed again.
 *
 *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
 *  the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous( name="Apteryx: Auto", group="Apteryx" )
public class ApteryxAuto extends LinearOpMode {
    
    ApteryxMap robot = new ApteryxMap(); 
    
    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double CIRCUMFERENCE = 96 * Math.PI; // in mm
    static final double MM_PER_CM = 10;
    
    static final double COUNTS_PER_CM = ( COUNTS_PER_MOTOR_REV / CIRCUMFERENCE ) * MM_PER_CM;
    
    @Override
    public void runOpMode() {
        
        robot.init( hardwareMap );
    
        // Stopping and Resetting Encoders...
        telemetry.addData( "STATUS", "RESETTING ENCODERS" );
        telemetry.update();

        robot.frontMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        robot.rightMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        robot.leftMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

        // Setting to run using Encoders...
        robot.frontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Waiting for the start...
        telemetry.addData( "STATUS", "WAITING FOR THE START" );
        telemetry.update();

        waitForStart();
        
        while( opModeIsActive() ) {
            
            // Going...
            telemetry.addData( "STATUS", "GOING" );
            telemetry.update();
            
            spinNicely( 5 );
            
            robot.stop();
        }
        
        robot.stop();
    }
    
    public void spinNicely( int times ) {
        
        double distance = times * ( 5 * COUNTS_PER_MOTOR_REV );
        
        while( robot.frontMotor.getCurrentPosition() < distance && 
               opModeIsActive() ) 
        {
            double gone = robot.frontMotor.getCurrentPosition();
            double distanceLeft = distance - gone;
            
            double speed = Math.min( distanceLeft / distance , -1 * ( distance / distanceLeft ) + 1 );
            
            telemetry.addData( "1", distanceLeft / distance );
            telemetry.addData( "2", ( distance / distanceLeft ) - 1 );
            telemetry.addData( "speed", ( 3 / 2 ) * speed );
            telemetry.update();
            
            robot.spin( ( 3 / 2 ) * speed );
        }
        
        robot.stop();
    }
    
    public void go( double x, double y, double speed ) {
        
        double distance = Math.sqrt( Math.pow( x, 2 ) + Math.pow( y, 2) );
        
        double xLeft = x;
        double yLeft = y;
        
        while( ( Math.abs( xLeft ) > 1.5 || Math.abs( yLeft ) > 1.5 ) &&
               opModeIsActive() ) {
            
            xLeft = x - ( ( robot.frontMotor.getCurrentPosition() / COUNTS_PER_CM ) +
                        ( ( -1 ) * ( 1 / 2 ) * ( robot.rightMotor.getCurrentPosition() / COUNTS_PER_CM ) ) + 
                        ( ( -1 ) * ( 1 / 2 ) * ( robot.leftMotor.getCurrentPosition() / COUNTS_PER_CM ) ) );
            yLeft = y - ( ( ( -1 ) * ( Math.sqrt( 3 ) / 2 ) * ( robot.rightMotor.getCurrentPosition() / COUNTS_PER_CM ) ) + 
                        ( ( Math.sqrt( 3 ) / 2 ) * ( robot.leftMotor.getCurrentPosition() / COUNTS_PER_CM ) ) );
            
            telemetry.addData( "xLeft", "%.2f", xLeft );
            telemetry.addData( "yLeft", "%.2f", yLeft );
            
            double angle = Math.toDegrees(Math.atan2( yLeft, xLeft ));
            if( angle < 0 )
                angle += 360;
            
            telemetry.addData( "angle", angle );
            telemetry.update();
            
            double newSpeed;
            
            double distanceLeft = Math.sqrt( Math.pow( xLeft, 2 ) + Math.pow( yLeft, 2) );
            newSpeed = speed * ( distanceLeft / distance );
            
            robot.move( newSpeed, angle );
        }
        
        robot.stop();
        return;
    }
}
