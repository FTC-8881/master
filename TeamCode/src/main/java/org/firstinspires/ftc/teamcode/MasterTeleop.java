package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import android.os.Handler;
import android.os.SystemClock;
import android.util.Log;
import android.view.ViewParent;
 
@TeleOp(name="Final: Teleop 1 Controller", group="Final")
public class MasterTeleop extends OpMode {
    DcMotor frontleft, frontright, backleft, backright; 

    public float x, y, z, w, pwr;
    public static double deadzone = 0.2;
 
    DcMotor lift;
    ModernRoboticsI2cGyro gyro;
    ModernRoboticsI2cRangeSensor range2;
    DistanceSensor range1;
    public DigitalChannel digitalBottomLow  = null;
    public DigitalChannel digitalBottomHigh = null;
    public DigitalChannel digitalTopLow     = null;
    public DigitalChannel digitalTopHigh    = null;
    
    static int maxEncoder = -25000;
    static int clicksPerPress = 500;
    static int minEncoder = 25000;
    static double MAX_PWR = 1.0;
    int currentEncoder;
    int targetEncoder=0;
    static double liftPower=0.7;
    static double stopPower=0.0;
    double desiredPower;
    //lift encoders
    @Override
    public void init() {
        
        lift = hardwareMap.dcMotor.get("lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setDirection(DcMotor.Direction.REVERSE);
        

        frontleft = hardwareMap.dcMotor.get("Front_Left");
        frontright = hardwareMap.dcMotor.get("Front_Right");
        backleft = hardwareMap.dcMotor.get("Rear_Left");
        backright = hardwareMap.dcMotor.get("Rear_Right");

        frontright.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotor.Direction.REVERSE);
        //reverses right side, so collector is front
 
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        gyro.resetZAxisIntegrator();

        
                // get a reference to our digitalTouch object.
        digitalBottomLow = hardwareMap.get(DigitalChannel.class, "bl");
        digitalBottomHigh = hardwareMap.get(DigitalChannel.class, "bh");
        digitalTopLow = hardwareMap.get(DigitalChannel.class, "tl");
        digitalTopHigh = hardwareMap.get(DigitalChannel.class, "th");

        // set the digital channel to input.
        digitalBottomLow.setMode(DigitalChannel.Mode.INPUT);
        digitalBottomHigh.setMode(DigitalChannel.Mode.INPUT);
        digitalTopLow.setMode(DigitalChannel.Mode.INPUT);
        digitalTopHigh.setMode(DigitalChannel.Mode.INPUT);
        
        targetEncoder = lift.getCurrentPosition();
    }
 
    @Override
    public void loop() {
        getJoyVals();
        currentEncoder = lift.getCurrentPosition();
        //updates joyvalues with deadzones, xyzw
 
        pwr = y; //this can be tweaked for exponential power increase
 
        backright.setPower(Range.clip(pwr + x+z, -1, 1)*MAX_PWR);
        backleft.setPower(Range.clip(pwr + x-z, -1, 1)*MAX_PWR);
        frontleft.setPower(Range.clip(pwr - x-z, -1, 1)*MAX_PWR);
        frontright.setPower(Range.clip(pwr - x+z, -1, 1)*MAX_PWR);
        
        if(gamepad1.dpad_up){
            targetEncoder -= clicksPerPress;
            desiredPower = liftPower;
            
        } else if(gamepad1.dpad_down){
            targetEncoder += clicksPerPress;
            desiredPower = liftPower;
        }else {
            targetEncoder = currentEncoder;
            desiredPower = stopPower;
        }
        lift.setTargetPosition(targetEncoder);
        // set both motors to 25% power. Movement will start.

        lift.setPower(desiredPower);
        if (digitalBottomLow.getState() == true) {
            telemetry.addData("Bottom Low", "Is Not Pressed");
        } else {
            telemetry.addData("Bottom Low", "Is Pressed");
        }

        if (digitalBottomHigh.getState() == true) {
            telemetry.addData("Bottom High", "Is Not Pressed");
        } else {
            telemetry.addData("Bottom High", "Is Pressed");
        }
        if (digitalTopLow.getState() == true) {
            telemetry.addData("Top Low", "Is Not Pressed");
        } else {
            telemetry.addData("Top Low", "Is Pressed");
        }
        if (digitalTopHigh.getState() == true) {
            telemetry.addData("Top High", "Is Not Pressed");
        } else {
            telemetry.addData("Top High", "Is Pressed");
        }
        telemetry.addData("Lift Current",
            String.format("%d",currentEncoder));
        telemetry.addData("Heading", 
            String.format("%3d",-gyro.getIntegratedZValue()));

        telemetry.update();
    }
 
    public void getJoyVals()
    {
        y = -gamepad1.left_stick_y;
        x = -gamepad1.left_stick_x;
        z = -gamepad1.right_stick_x;
        w = -gamepad1.right_stick_y;
        //updates joystick values
 
        if(Math.abs(x)<deadzone) x = 0;
        if(Math.abs(y)<deadzone) y = 0;
        if(Math.abs(z)<deadzone) z = 0;
        if(Math.abs(w)<0.9) w = 0;
        //checks deadzones
    }
    
    public double max (double one, double two){
        double retVal = one;
        if (two > one) {
            retVal = two;
        }
        return retVal;
    }
 
    @Override
    public void stop() {
        //nothing here? probably gotta call garbage collection at some point
    }
}


