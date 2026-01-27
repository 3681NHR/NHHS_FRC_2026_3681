package frc.utils.rumble;

public class Rumble{
    public double powR;
    public double powL;
    public double time;
    public Rumble(double time, double pow){
        this.powR = pow;
        this.powL = pow;
        this.time = time;
    }
    public Rumble(double time, double powR, double powL){
        this.powR = powR;
        this.powL = powL;
        this.time = time;
    }
}