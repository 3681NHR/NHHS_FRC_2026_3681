package frc.utils.controlWrappers;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.utils.PIDTuner;

public class PIDGains {
    public static class Gains {
        String key;
        LoggedNetworkBoolean apply;
        Runnable applyCallback = ()->{};

        public void update() {
        }

        public Gains makeTunable(String key) {
            this.key = key;
            apply = new LoggedNetworkBoolean(key + "apply", false);
            PIDTuner.tunableGains.add(this);
            return this;
        }
        /**
         * set a callback for when applied, 
         * @param onApply called when the gains are set using the live tuning
         * @return 
         */
        public Gains withCallback(Runnable onApply){
            this.applyCallback = onApply;
            return this;
        }
    }

    /**
     * simple class for PID gains
     */
    public static class PID extends Gains {
        public double kP;
        public double kI;
        public double kD;
        LoggedNetworkNumber setkP;
        LoggedNetworkNumber setkI;
        LoggedNetworkNumber setkD;

        public PID(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            if (kP < 0 || kI < 0 || kD < 0) {
                throw new IllegalArgumentException("PID values must be positive");
            }
        }

        @Override
        public Gains makeTunable(String key){
            super.makeTunable(key);

            setkP = new LoggedNetworkNumber(key + "/kp", kP);
            setkI = new LoggedNetworkNumber(key + "/ki", kI);
            setkD = new LoggedNetworkNumber(key + "/kd", kD);

            return this;
        }

        @Override
        public void update(){
            if(apply != null){
                if(apply.get()){
                    kP = setkP.get();                    
                    kI = setkI.get();                        
                    kD = setkD.get();   

                    apply.set(false);
                    applyCallback.run();
                }
            }
        }
    }

    /**
     * simple class for PID gains, includes max speed and max acceleration
     */
    public static class ProfiledPID extends Gains {
        public double kP;
        public double kI;
        public double kD;
        public double maxSpeed;
        public double maxAccel;
        LoggedNetworkNumber setkP;
        LoggedNetworkNumber setkI;
        LoggedNetworkNumber setkD;
        LoggedNetworkNumber setmaxSpeed;
        LoggedNetworkNumber setmaxAccel;

        public ProfiledPID(double kP, double kI, double kD, double maxSpeed, double maxAccel) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.maxSpeed = maxSpeed;
            this.maxAccel = maxAccel;
            if (kP < 0 || kI < 0 || kD < 0) {
                throw new IllegalArgumentException("PID values must be positive");
            }
        }

        public PID getPID() {
            return new PID(kP, kI, kD);
        }
        

        @Override
        public Gains makeTunable(String key){
            super.makeTunable(key);

            setkP = new LoggedNetworkNumber(key + "/kp", kP);
            setkI = new LoggedNetworkNumber(key + "/ki", kI);
            setkD = new LoggedNetworkNumber(key + "/kd", kD);
            setmaxSpeed = new LoggedNetworkNumber(key + "/max speed", maxSpeed);
            setmaxAccel = new LoggedNetworkNumber(key + "/max acceleration", maxAccel);

            return this;
        }

        @Override
        public void update(){
            if(apply != null){
                if(apply.get()){
                    kP = setkP.get();                    
                    kI = setkI.get();                        
                    kD = setkD.get();   
                    maxSpeed = setmaxSpeed.get();   
                    maxAccel = setmaxAccel.get();   
                
                    apply.set(false);
                    applyCallback.run();
                }
            }
        }
    }

    /**
     * simple class for feedforward gains
     */
    public static class SimpleFF extends Gains {
        public double kS;
        public double kV;
        public double kA;
        LoggedNetworkNumber setkS;
        LoggedNetworkNumber setkV;
        LoggedNetworkNumber setkA;

        public SimpleFF(double kS, double kV, double kA) {
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            if (kS < 0 || kV < 0 || kA < 0) {
                throw new IllegalArgumentException("FF values must be positive");
            }
        }

        @Override
        public Gains makeTunable(String key){
            super.makeTunable(key);

            setkS = new LoggedNetworkNumber(key + "/ks", kS);
            setkV = new LoggedNetworkNumber(key + "/kv", kV);
            setkA = new LoggedNetworkNumber(key + "/ka", kA);

            return this;
        }

        @Override
        public void update(){
            if(apply != null){
                if(apply.get()){
                    kS = setkS.get();                    
                    kV = setkV.get();                        
                    kA = setkA.get();   

                    apply.set(false);
                    applyCallback.run();
                }
            }
        }
    }

    /**
     * simple class for feedforward gains, includes kG for elevator and arm ff
     */
    public static class GravityFF extends Gains {
        public double kS;
        public double kV;
        public double kA;
        public double kG;
        LoggedNetworkNumber setkS;
        LoggedNetworkNumber setkV;
        LoggedNetworkNumber setkA;
        LoggedNetworkNumber setkG;

        public GravityFF(double kS, double kG, double kV, double kA) {
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            this.kG = kG;
            if (kS < 0 || kV < 0 || kA < 0) {
                throw new IllegalArgumentException("FF values must be positive");
            }
        }

        

        @Override
        public Gains makeTunable(String key){
            super.makeTunable(key);

            setkS = new LoggedNetworkNumber(key + "/ks", kS);
            setkV = new LoggedNetworkNumber(key + "/kv", kV);
            setkA = new LoggedNetworkNumber(key + "/ka", kA);
            setkG = new LoggedNetworkNumber(key + "/kg", kG);

            return this;
        }

        @Override
        public void update(){
            if(apply != null){
                if(apply.get()){
                    kS = setkS.get();                    
                    kV = setkV.get();                        
                    kA = setkA.get();   
                    kG = setkG.get();   

                    apply.set(false);
                    applyCallback.run();
                }
            }
        }
    }
}
