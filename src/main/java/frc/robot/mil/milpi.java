package frc.robot.mil;

public class MilPi {
    
    private String ip;
    private boolean active;

    public MilPi(String ipAddr){
        ip = ipAddr;
    }

    public boolean isActive(){
        return active;
    }

    public boolean check(){
        try{
            Process p1 = java.lang.Runtime.getRuntime().exec("ping -c 1 " + ip);
            int retVal = p1.waitFor();
            active = retVal != 0;
            return !active;
        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }
    }

} 
