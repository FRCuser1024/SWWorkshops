import java.util.Scanner;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants;

class Drive {
    //private stuff;

    private TalonFX L1 = new TalonFX(Constants.id1);
    private TalonFX L2 = new TalonFX(Constants.id2);
    private TalonFX R1 = new TalonFX(Constants.id3);
    private TalonFX R2 = new TalonFX(Constants.id4);
    //create four talons, two on each side

    public Drive () {
        L2.set(ControlMode.Follower, Constants.id1);
        R2.set(ControlMode.Follower, Constants.id3);
        L1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,  0, 1000);
        R1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,  0, 1000);
        //second talon follows first, configure master (first) talons
    }
    
    public void setOpenLoop(double throttle, double turn) {
        if (throttle > 0) {
            L1.set(ControlMode.PercentOutput, throttle + turn);
            R1.set(ControlMode.PercentOutput, throttle - turn);
        }
        else {
            L1.set(ControlMode.PercentOutput, throttle - turn);
            R1.set(ControlMode.PercentOutput, throttle + turn);
        }
        //throttle = how fast it's moving, turn = how fast it's turning
    }
}

/*public class InsertFileName {
    public static void main (String[] args) {
        Scanner in = new Scanner(System.in);

        System.out.println("What color is the sky at day?");
        String string1 = in.nextLine();

        System.out.println(string1);
    }
}*/