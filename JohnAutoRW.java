package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;

@Autonomous( name = "John: Auto", group = "John" )
public class JohnAuto extends LinearOpMode
{
    public DcMotor rearMotor, rightMotor, leftMotor, arm;
    public DcMotorSimple duck, intake;
    public OpenCvCamera webcam;

                        // Robot constants
    public final double ROBOT_CIRC = 13.7 * Math.PI,
                        TICKS_PER_ROBOT_CM = 15,
                        CM_PER_INCH = 2.54,
                        TICKS_PER_ROBOT_INCH = TICKS_PER_ROBOT_CM * CM_PER_INCH,
                        
                        // Wheel constants
                        WHEEL_CIRC = 5.5 * Math.PI,
                        TICKS_PER_WHEEL_REV = 20.0 * 28.0,
                        
                        // Oddballs
                        WHEEL_REVS_PER_ROBOT_REV = ROBOT_CIRC / WHEEL_CIRC,
                        TICKS_PER_ROBOT_REV = TICKS_PER_WHEEL_REV * WHEEL_REVS_PER_ROBOT_REV,
                        
                        // vs
                        SPEED = 0.30,
                        APPLY_SPEED = 0.02,
                        SPIN_SPEED = 0.30,
                        ARM_SPEED = 0.30,
                        DUCK_SPEED = 0.60,
                        INTAKE_SPEED = 0.50,
                        OUTTAKE_SPEED = -0.50;
                    
                        // hs
                        public final int LEVEL_1 = 300,
                                         LEVEL_2 = 500,
                                         LEVEL_3 = 700,
                                         ZENITH = 1000;
                    
    public final Scalar CAP_UPPER_BOUND = new Scalar( 255, 255, 125, 255 ),
                        CAP_LOWER_BOUND = new Scalar( 175, 175, 0, 255 ),
                        BLUE_UPPER_BOUND = new Scalar( 30, 45, 80, 255 ),
                        BLUE_LOWER_BOUND = new Scalar( 10, 15, 40, 255 ),
                        RED_UPPER_BOUND = new Scalar( 200, 80, 80, 255 ),
                        RED_LOWER_BOUND = new Scalar( 150, 20, 20, 255 );

    public int level = -1;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Motors
        rearMotor = hardwareMap.get( DcMotor.class, "Motor3" );
        rightMotor = hardwareMap.get( DcMotor.class, "Motor2" );
        leftMotor = hardwareMap.get( DcMotor.class, "Motor1" );
        
        arm = hardwareMap.get( DcMotor.class, "Arm" );
        
        rearMotor.setDirection( DcMotor.Direction.REVERSE );
        rightMotor.setDirection( DcMotor.Direction.REVERSE );
        leftMotor.setDirection( DcMotor.Direction.REVERSE );
        arm.setDirection( DcMotor.Direction.FORWARD );
        
        rearMotor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
        rightMotor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
        leftMotor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
        
        arm.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
        
        // Servos
        duck = hardwareMap.get( DcMotorSimple.class, "Duck" );
        intake = hardwareMap.get( DcMotorSimple.class, "Intake" );

        duck.setDirection( DcMotorSimple.Direction.REVERSE );
        intake.setDirection( DcMotorSimple.Direction.FORWARD );

        // Webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier( "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName() );

        webcam = OpenCvCameraFactory.getInstance().createWebcam( hardwareMap.get( WebcamName.class, "cam" ), cameraMonitorViewId );

        webcam.openCameraDeviceAsync( new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming( 1280, 720, OpenCvCameraRotation.UPRIGHT );
            }
            @Override
            public void onError(int errorCode) {}
        } );

        webcam.setPipeline( new CapPipeline() );

        telemetry.add( "3..." );
        telemetry.update();
        
        sleep( 1000 );

        telemetry.add( "2..." );
        telemetry.update();
        
        sleep( 1000 );

        telemetry.add( "1..." );
        telemetry.update();
        
        sleep( 1000 );
                
        telemetry.addData( "level", level );

        telemetry.update();

        waitForStart();

        if( opModeIsActive() )
        {
            switch( level ) 
            {
                case 1:
                    doSync( new Go( 34, 135 ), new LiftArmTo( LEVEL_1) );
                    doNow( new OuttakeFor( 2 ) );
                    doNow( new Go( 31, -45 ) );
                    doNow( new Go( 50, 0 ) );
                    break;
                case 3:
                    doNow( new Turn( 30 ) );
                    doSync( new Go( 33, 90 ), new LiftArmTo( LEVEL_3 ) );
                    doNow( new OuttakeFor( 2 ) );
                    doNow( new Go( 31, -90 ) );
                    doNow( new Go( 50, -30 ) );
                    break;
                default:
                    doNow( new Turn( 30 ) );
                    doSync( new Go( 32, 90 ), new LiftArmTo( LEVEL_2 ) );
                    doNow( new OuttakeFor( 2 ) );
                    doNow( new Go( 30, -90 ) );
                    doNow( new Go( 50, -30 ) );
            }
        }
    }

    public void doNow( Thread toDo ) throws InterruptedException
    {
        toDo.start();

        toDo.join();
    }

    public void doIn( Thread toDo, int t ) throws InterruptedException
    {
        long startT = System.currentTimeMillis();

        while( opModeIsActive() && System.currentTimeMillis() - startT < t ) {}

        toDo.start();

        toDo.join();
    }

    public void doSync( Thread ... toDos ) throws InterruptedException
    {
        for( Thread toDo : toDos )
            toDo.start();

        for( Thread toDo : toDos )
            toDo.join();
    }

    public void resetWheelEncoders()
    {
        rearMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        rightMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        leftMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

        rearMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODERS );
        rightMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODERS );
        leftMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODERS );
    }

    public void resetArmEncoder()
    {
        arm.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

        arm.setMode( DcMotor.RunMode.RUN_USING_ENCODERS );
    }

    class Go extends Thread
    {
        int dist, angle;

        public Go( int d, int a )
        {
            dist = d;
            angle = a;
        }

        public void run()
        {
            resetWheelEncoders();
            
            int rearTarget = ( int )( ( TICKS_PER_ROBOT_INCH * dist ) * Math.cos( Math.toRadians( 180 + angle ) ) );
            int rightTarget = ( int )( ( TICKS_PER_ROBOT_INCH * dist ) * Math.cos( Math.toRadians( 60 + angle ) ) );
            int leftTarget = ( int )( ( TICKS_PER_ROBOT_INCH * dist ) * Math.cos( Math.toRadians( 300 + angle ) ) );
            
            rearMotor.setTargetPosition( rearTarget );
            rightMotor.setTargetPosition( rightTarget );
            leftMotor.setTargetPosition( leftTarget );
            
            rearMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );
            rightMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );
            leftMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );
            
            rearMotor.setPower( SPEED * Math.cos( Math.toRadians( 180 + angle ) ) );
            rightMotor.setPower( SPEED * Math.cos( Math.toRadians( 60 + angle ) ) );
            leftMotor.setPower( SPEED * Math.cos( Math.toRadians( 300 + angle ) ) );
            
            while( opModeIsActive() && ( rightMotor.isBusy() && leftMotor.isBusy() ) ) {}
            
            rearMotor.setPower( 0 );
            rightMotor.setPower( 0 );
            leftMotor.setPower( 0 );
        }
    }
    
    class Apply extends Thread 
    {
        int angle;
        int time;
        
        public Apply( int a, int t ) 
        {
            angle = a;
            time = t;
        }
        
        public void run() 
        {
            rearMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
            rightMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
            leftMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
            
            long startTime = System.currentTimeMillis() / 1000;
            
            rearMotor.setPower( APPLY_SPEED * Math.cos( Math.toRadians( 180 + angle ) ) );
            rightMotor.setPower( APPLY_SPEED * Math.cos( Math.toRadians( 60 + angle ) ) );
            leftMotor.setPower( APPLY_SPEED * Math.cos( Math.toRadians( 300 + angle ) ) );
                
            while( opModeIsActive() && ( System.currentTimeMillis() / 1000 ) - startTime < time ) {}
            
            rearMotor.setPower( 0 );
            rightMotor.setPower( 0 );
            leftMotor.setPower( 0 );
        }
    }

    class Turn extends Thread
    {
        int degrees;

        public Turn( int d )
        {
            degrees = d;
        }

        public void run()
        {
            resetWheelEncoders();

            int target = ( -1 ) * ( int )( TICKS_PER_ROBOT_REV * degrees / 360 );

            rearMotor.setTargetPosition( target );
            rightMotor.setTargetPosition( target );
            leftMotor.setTargetPosition( target );

            rearMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );
            rightMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );
            leftMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );

            rearMotor.setPower( SPIN_SPEED );
            rightMotor.setPower( SPIN_SPEED );
            leftMotor.setPower( SPIN_SPEED );

            while( opModeIsActive() && ( rearMotor.isBusy() || rightMotor.isBusy() || leftMotor.isBusy() ) ) {}

            rearMotor.setPower( 0 );
            rightMotor.setPower( 0 );
            leftMotor.setPower( 0 );
        }
    }

    class LiftArmTo extends Thread
    {
        int pos;

        public LiftArmTo( int p )
        {
            pos = p;
        }

        public void run()
        {
            arm.setTargetPosition( pos );

            arm.setMode( DcMotor.RunMode.RUN_TO_POSITION );

            arm.setPower( ARM_SPEED );

            while( opModeIsActive() && arm.isBusy() ) {}

            arm.setPower( 0 );
        }
    }

    class BringArmIn extends Thread
    {
        public void run()
        {
            new LiftArmTo( 0 ).run();
            resetArmEncoder();
        }
    }

    class DuckFor extends Thread
    {
        int time;

        public DuckFor( int t )
        {
            time = t;
        }

        public void run()
        {
            int startTime = ( int )( System.currentTimeMillis() / 1000.0 );

            duck.setPower( DUCK_SPEED );

            while( opModeIsActive() && ( int )( System.currentTimeMillis() / 1000.0 ) - startTime < time  ) {}

            duck.setPower( 0 );
        }
    }

    class IntakeFor extends Thread
    {
        int time;

        public IntakeFor( int t )
        {
            time = t;
        }

        public void run()
        {
            int startTime = ( int )( System.currentTimeMillis() / 1000.0 );

            intake.setPower( INTAKE_SPEED );

            while( opModeIsActive() && ( int )( System.currentTimeMillis() / 1000.0 ) - startTime < time  ) {}

            intake.setPower( 0 );
        }
    }

    class OuttakeFor extends Thread
    {
        int time;

        public OuttakeFor( int t )
        {
            time = t;
        }

        public void run()
        {
            int startTime = ( int )( System.currentTimeMillis() / 1000.0 );

            intake.setPower( OUTTAKE_SPEED );

            while( opModeIsActive() && ( int )( System.currentTimeMillis() / 1000.0 ) - startTime < time  ) {}

            intake.setPower( 0 );
        }
    }

    class CapPipeline extends OpenCvPipeline
    {
        Mat yellows = new Mat();

        @Override
        public Mat processFrame(Mat rgba)
        {
            Core.inRange( rgba, CAP_LOWER_BOUND, CAP_UPPER_BOUND, yellows );

            ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();

            Imgproc.findContours( yellows, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE );

            if( contours.isEmpty() )
                return yellows;

            MatOfPoint cap = contours.get( 0 );
            for( MatOfPoint contour : contours )
                if( Imgproc.contourArea( contour ) > Imgproc.contourArea( cap ) )
                    cap = contour;

            Moments m = Imgproc.moments( cap );

            double x = m.m10 / m.m00;
            double y = m.m01 / m.m00;

            if( 800 < x )
                level = 3;
            else if( x < 500 )
                level = 1;
            else
                level = 2;

            return yellows;
        }
    }
}