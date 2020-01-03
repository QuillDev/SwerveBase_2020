package SKCommon.swerve;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Map;

public class SwerveModule {
    Timer test = new Timer();

    // create the parts of the swerve that we can actually control [Motors,
    // Encoders, etc]
    private VictorSP driveMotor;
    private VictorSP angleMotor;

    // encoder input for the STEERING GEAR
    private AnalogInput steerEncoder;

    private Encoder driveEncoder;

    // add steeroffset int
    private double steerOffset;

    // Add a placeholder for the name of the module
    private String moduleName = "Default";

    //Static value for radians in a circle
    private final double unitRads = 2.0 * Math.PI;

    // Constants for PID controller
    private final double Kp = 1;
    private final double Ki = 0;
    private final double Kd = 0;

    // Constraints for Profiled pid controller
    Constraints steerConstraints = new TrapezoidProfile.Constraints(unitRads, unitRads);

    //Create a new profiled PIDController
    private final ProfiledPIDController steerPID = new ProfiledPIDController(Kp, Ki, Kd, steerConstraints);

    //boolean as to whether the robot is inverse
    boolean isInverse = false;

    /**
     * 
     * @param driveMotor int port for drive motor 
     * @param angleMotor int port for angle motor
     * @param steerEncoder int port for steer encoder
     * @param steerOffset double offset for adjusting steering inconsistencies
     */
    public SwerveModule(int driveMotor, int angleMotor, int steerEncoder, double steerOffset) {
        //create our local instances of the pieces within our modules

        //setup angle motors
        this.driveMotor = new VictorSP(driveMotor);
        this.angleMotor = new VictorSP(angleMotor);

        //setup steer encoder as an AnalogInput
        this.steerEncoder = new AnalogInput(steerEncoder);

        //1/40*Map.wheelCircumference

        //setup steeroffset and convert it from degrees to radians
        this.steerOffset = Math.toRadians(steerOffset);

        //enable continuous input from 0 to 2 pi radians
        steerPID.enableContinuousInput(0, 2*Math.PI);
    }


    //read angle from the encoder and translate it to an angle
    //NOTE: In Radians, maybe convert to degrees?
    public double readAngle() {
        //get the angle by reading the output voltage of the encoder and turn it to a radian output
        double angle = (1.0 - steerEncoder.getVoltage() / RobotController.getVoltage5V()) * 2.0 * Math.PI + steerOffset;
        
        //to prevent overadding [Angles over 360 degrees / 2pi radians] we modulo the result by 2
        angle %= 2.0 * Math.PI;
        
        //if the angle is less than 0 [AKA negative] add 2pi radians in order to get the compliment
        if(angle < 0.0) {

            //add 2pi radians to the angle
            angle += 2.0 * Math.PI;
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
        return (angle < 0) ? ( angle += 2.0 * Math.PI ) : angle;
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
            optimalOutput = thetaProvisional - 2.0 * Math.PI;
        } else {
            optimalOutput = thetaProvisional + 2.0 * Math.PI;
        }

        return optimalOutput;
    }
    //Returns in inches
    public double readDistanceInches(){
        return driveEncoder.getDistance() * Map.wheelCircumference * Map.gearRatio;
    }

    //Return distance in feet
    public double readDistanceFeet(){
        return readDistanceInches()/12;
    }

    //read position
    public int readPosition() {
        return driveEncoder.get();
    }

    /**
     * @param speed speed to set the drive motor to
     */
    public void setDrive(double speed){
        //set the drive motor to the percentage speed specified
        driveMotor.set(speed);
    }

    /**
     * Simple Drive Command that simply sets the angle and speed
     * @param speed speed to set the drive motor to
     * @param angle angle to set the steering gears to
     */
    public void driveSimple(double speed, double goalAngle){

        //calculates the speed to set the motor to using the PID formula taking into the account the current angle position and the setpoint [goal]
        angleMotor.set(MathUtil.clamp(steerPID.calculate(readAngle(), calculateBestGoal(goalAngle)), -1, 1));

        //Solve the wonderful 180 issue in the cleanest possible way @Ether
        //set the speed, if the motor controller is inverse set it to inverse
        setDrive( isInverse ? -speed : speed );
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