package frc.robot;

import SKCommon.Utils.qMath;
import edu.wpi.first.wpilibj.drive.Vector2d;

public class Map{

    //TODO ADJUST THIS FOR THE LOVE OF GOD
    public final static double gearRatio = 6.67;

    //dimensions of the robot stored globally
    //needed in the kinematic equations from swerve
    public final static double TRACKWIDTH = 30; //distance from the centers of the wheels width wise 
    public final static double WHEELBASE = 30; //distance between the centers of the wheels length wise 

    //create vector for the chassis
    public final static Vector2d chassisVector = new Vector2d(TRACKWIDTH, WHEELBASE);

    //save vector length (magnitude of the vector of the chassis) to a double value
    public final static double chassisMagnitude = chassisVector.magnitude();

    //Wheel values [Wheel Diameter and Circumference] 
    public final static double wheelDiameter = 4;
    public final static double wheelDiameterMetres = qMath.feetToMetres(4/12);
    public final static double wheelCircumference = wheelDiameter * Math.PI;

    //angle offsets [ INPUT VALUES IN DEGREES!!! ]
    public final static double kflAngleOffset = Math.toRadians(0.0);
    public final static double kfrAngleOffset = Math.toRadians(0.0);
    public final static double kblAngleOffset = Math.toRadians(0.0);
    public final static double kbrAngleOffset = Math.toRadians(0.0);

    //joystick stuff
    public final static double deadzone = .18; //deadzone for determining wether a controller input should matter or not

    //Motor Controller Ports for Drive Motors
    public final static int flDrive = 0;
    public final static int frDrive = 2;
    public final static int blDrive = 4;
    public final static int brDrive = 6;

    //Motor Control Ports for Angle Motors
    public final static int flAngle = 1;
    public final static int frAngle = 3;
    public final static int blAngle = 5;
    public final static int brAngle = 7;

    //Encoder Ports on AnalogIn Points 
    public final static int flEncoder = 0;
    public final static int frEncoder = 1;
    public final static int blEncoder = 2;
    public final static int brEncoder = 3;

}