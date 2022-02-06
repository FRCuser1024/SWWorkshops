package frc.robot.subsystems;
import frc.robot.Constants;
import java.lang.Math;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Drive extends Subsystem {
    private TalonFX LM = new TalonFX(Constants.kLMId);
    private TalonFX LF = new TalonFX(Constants.kLFId);
    private TalonFX RM = new TalonFX(Constants.kRMId);
    private TalonFX RF = new TalonFX(Constants.kLFId);
    public Drive(){
        LF.set(ControlMode.Follower, 1);
        LM.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 1000);
        RF.set(ControlMode.Follower, 3);
        RM.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 1000);
    }
    public void setOpenLoop(double throttle, double turn){
        double LT;
        double RT;
        if (Math.signum(throttle) != 0) {
            LT = Math.signum(throttle) * -turn;
            RT = Math.signum(throttle) * turn;
        } else {
            LT = -turn;
            RT = turn;
        }
        LM.set(ControlMode.PercentOutput, (throttle+LT)/2);
        RM.set(ControlMode.PercentOutput, (throttle+RT)/2);
    }
    public void stop(){
        LM.set(ControlMode.PercentOutput, 0.0);
        RM.set(ControlMode.PercentOutput, 0.0);
    }
    /*public class PIDF{
        private double Kp;
        private double Ki;
        private double Kd;
        private double Kf;
        private double setpoint;
        public PIDF(double Kp, double Ki, double Kd, double Kf){
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
            this.Kf = Kf;
        }
        public double get(int i){
            switch(i){
                case 0:
                    return Kp;
                case 1:
                    return Ki;
                case 2:
                    return Kd;
                case 3:
                    return Kf;
                case 4:
                    return setpoint;
                default:
                    return 0.0;
            }
        }
        public void set(int inp, double val){
            switch(inp){
                case 0:
                    Kp = val;
                    break;
                case 1:
                    Ki = val;
                    break;
                case 2:
                    Kd = val;
                    break;
                case 3:
                    Kf = val;
                    break;
                case 4:
                    setpoint = val;
                
            }
        }
        public double getError(){
            return 1;
        }
        public double update(double pos, double val, double allowable_error, int dt){
            double prev_error = 0;
            double integral = 0;
            double output = 0;
            double error = allowable_error + 1;
            while(error > allowable_error){
                error = setpoint - val;
                integral = integral + error * dt;
                double derivative = (error - prev_error) / dt;
                output = Kp*error + Ki*integral + Kd*derivative;
                prev_error = error;
                try {
                    Thread.sleep(dt);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
            return output;
        }
    }*/
    public class PIDF {
        private double kP;
        private double kD;
        private double kI;
        private double F;
        private double setpoint;
        public PIDF (double kP, double kD, double kI, double F) {
            this.kP = kP;
            this.kD = kD;
            this.kI = kI;
            this.F = F;
        }
    
        public double get (int id) {
            switch (id){
                case 1: 
                    return kP;
                case 2: 
                    return kD;
                case 3:
                    return kI;
                case 4:
                    return F;
                case 5: 
                    return setpoint;
                default: 
                    return -1;
            }
        }
    
        public void set (int id, double value) {
            switch (id){
                case 1: 
                    kP = value;
                    break;
                case 2: 
                    kD = value;
                    break;
                case 3:
                    kI = value;
                    break;
                case 4:
                    F = value;
                    break;
                case 5:
                    setpoint = value;
                    break;
                default: 
                    break;
            }
        }
    
        public double getError (double pos) {
            return setpoint - pos;
        }
    
        public double update (double pos, double errorAllowed, int dt) {
            double previous_error = 0;
            double integral = 0;
            double derivative = 0;
            double output = 0;
            double error = errorAllowed + 2;
            while (error > errorAllowed) {
                   error = setpoint - pos;
                integral = integral + error * dt;
                   derivative = (error - previous_error) / dt;
                output = kP*error + kI*integral + kD*derivative;
                previous_error = error;
                try {
                    Thread.sleep(dt);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
            return output;
        }
    }
}

/*public class PIDF {
    private double kP;
    private double kD;
    private double kI;
    private double F;
    private double setpoint;
    public PIDF (double kP. double kD, double kI, double F) {
        this.kP = kP;
        this.kD = kD;
        this.kI = kI;
        this.F = F;
    }

    public double get (int id) {
        switch (id){
            case 1: 
                return kP;
            case 2: 
                return kD;
            case 3:
                return kI;
            case 4:
                return F;
            case 5: 
                return setpoint;
            default: 
                return -1;
        }
    }

    public void set (int id, double value) {
        switch (id){
            case 1: 
                kP = value;
                break;
            case 2: 
                kD = value;
                break;
            case 3:
                kI = value;
                break;
            case 4:
                F = value;
                break;
            case 5:
                setpoint = value;
                break;
            default: 
                break;
        }
    }

    public double getError (double pos) {
        return setpoint - pos;
    }

    public double update (double pos, double errorAllowed, int dt) {
        double previous_error = 0;
        double integral = 0;
        double derivative = 0;
        double output = 0;
        double error = errorAllowed + 2;
        while (error > errorAllowed) {
       	    error = setpoint - pos;
        	integral = integral + error * dt;
       	    derivative = (error - previous_error) / dt;
    	    output = kP*error + kI*integral + kD*derivative;
    	    previous_error = error;
            try {
                Thread.sleep(dt);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        return output;
    }
}*/