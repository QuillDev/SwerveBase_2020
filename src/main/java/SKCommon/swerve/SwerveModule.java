package SKCommon.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import SKCommon.Utils.qMath;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpiutil.math.MathUtil;

public class SwerveModule {
    // create the parts of the swerve that we can actually control [Motors,
    // Encoders, etc]
    private TalonSRX driveMotor;
    private VictorSPX steerMotor;

    // encoder input for the STEERING GEAR
    private AnalogInput steerEncoder;

    // add steeroffset int
    private double steerOffset;

    // Add a placeholder for the name of the module
    private String moduleName = "Default";


    // Constants for PID controller
    private final double Kp = 1;
    private final double Ki = 0;
    private final double Kd = 0;

    // Constraints for Profiled pid controller
    Constraints steerConstraints = new TrapezoidProfile.Constraints(qMath.twoPI, qMath.twoPI);

    //Create a new profiled PIDController
    private final ProfiledPIDController steerPID = new ProfiledPIDController(Kp, Ki, Kd, steerConstraints);

    //boolean as to whether the robot is inverse
    boolean isInverse = false;

    /**
     * 
     * @param driveMotor int port for drive motor 
     * @param steerMotor int port for angle motor
     * @param steerEncoder int port for steer encoder
     * @param steerOffset double offset for adjusting steering inconsistencies
     */
    public SwerveModule(int driveMotor, int steerMotor, int steerEncoder, double steerOffset) {
        //create our local instances of the pieces within our modules

        //setup angle motors
        this.driveMotor = new TalonSRX(driveMotor);
        this.steerMotor = new VictorSPX(steerMotor);

        //default controllers
        this.driveMotor.configFactoryDefault();
        this.steerMotor.configFactoryDefault();

        //setup steer encoder as an AnalogInput
        this.steerEncoder = new AnalogInput(steerEncoder);

        //1/40*Map.wheelCircumference

        //setup steeroffset and convert it from degrees to radians
        this.steerOffset = Math.toRadians(steerOffset);

        //enable continuous input from 0 to 2 pi radians
        steerPID.enableContinuousInput(0, qMath.twoPI);
    }


    //read angle from the encoder and translate it to an angle
    //NOTE: In Radians, maybe convert to degrees?
    public double readAngle() {
        //get the angle by reading the output voltage of the encoder and turn it to a radian output
        double angle = (1.0 - steerEncoder.getVoltage() / RobotController.getVoltage5V()) * qMath.twoPI + steerOffset;
        
        //to prevent overadding [Angles over 360 degrees / 2pi radians] we modulo the result by 2
        angle %= qMath.twoPI;
        
        //if the angle is less than 0 [AKA negative] add 2pi radians in order to get the compliment
        if(angle < 0.0) {

            //add 2pi radians to the angle
            angle += qMath.twoPI;
        }
        return angle;
    }
    /**
     * calculates whether the angle used or the inverse of it is the best steering option, also flips the drive motor acoordingly
     * @return the optimal value to turn to
     */
    public double calculateBestGoal(double goal){
        //find the inverse of the goal given
        double inverseGoal = (goal >= Math.PI) ? (goal - Math.PI) : (Math.PI + goal);
        
        //calculate optimal goal lengths
        double optimalGoalLength = toPositive(calculateOptimalTurn(goal));
        double optimalInverseGoalLength = toPositive(calculateOptimalTurn(inverseGoal));

        //invert the drive motor if the inverse angle was favored
        driveMotor.setInverted( (optimalInverseGoalLength <= optimalGoalLength) ) ;

        return Math.min(optimalInverseGoalLength, optimalGoalLength);
    }

    /**
     * @param angle to convert to positive value
     * @return returns inverse angles as positive values
     */
    public double toPositive(double angle){
        return (angle < 0) ? ( angle += qMath.twoPI ) : angle;
    }
    /**
     * @return the lowest amount of degrees it will have to travel from the goal given the current angle
     */
    public double calculateOptimalTurn(double goal) {
        double thetaProvisional = goal - readAngle();
        double optimalOutput;

        if(-Math.PI < thetaProvisional && thetaProvisional <= Math.PI){
            optimalOutput = thetaProvisional;
        }
        else if(thetaProvisional > Math.PI){
            optimalOutput = thetaProvisional - qMath.twoPI;
        } else {
            optimalOutput = thetaProvisional + qMath.twoPI;
        }

        return optimalOutput;
    }

    /**
     * Simple Drive Command that simply sets the angle and speed
     * @param speed speed to set the drive motor to
     * @param angle angle to set the steering gears to
     */
    public void driveSimple(double speed, double goalAngle){

        //calculates the speed to set the motor to using the PID formula taking into the account the current angle position and the setpoint [goal]
        steerMotor.set(ControlMode.PercentOutput, MathUtil.clamp(steerPID.calculate(readAngle(), calculateBestGoal(goalAngle)), -1, 1));

        //Solve the wonderful 180 issue in the cleanest possible way @Ether
        //set the speed, if the motor controller is inverse set it to inverse
        driveMotor.set(ControlMode.PercentOutput, (isInverse ? -1 : 1) * speed);
    }

    /**
     * @param name name to change the module to
     * 
     * this changes the name of the module
     */
    public void setName(String name){ 
        this.moduleName = name;
    }

    /**
     * this returns the name of the module
     */
    public String getName(){
        return moduleName;
    }

    /**
     * Print data of the current module to the dashboard
     */

    /**
     * @param inverse - boolean saying whether the robot is to be inverse or not
     * sets the isinverse value to inverse
     */
    public void setInverse(boolean inverse){
        this.isInverse = inverse;
    }
}