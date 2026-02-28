/**
 * RawTrajectoryCalc - Calculates elevation angle and shooter speed for dual roller shooter.
 * 
 * FRC 2026 REBUILT - Raw trajectory calculator for throwing balls to arbitrary targets.
 * No lip clearance, no funnel - just hit a target point.
 * 
 * Shooter configuration:
 *   - Dual coned TPR rollers (3"→4" cones), 88.9mm effective diameter
 *   - 1:1 counter-rotating (opposite directions, same speed)
 *   - Zero backspin (counter-rotation cancels spin)
 *   - Exit velocity = π × 0.0889 × RPS (100% velocity transfer)
 * 
 * Usage:
 *   RawTrajectoryCalc calc = new RawTrajectoryCalc();
 *   calc.setShooterHeightM(0.533);
 *   calc.setTargetDistanceM(15.0);
 *   calc.setTargetHeightM(0.5);
 *   calc.setSlipPercentage(1.0);
 *   calc.setMaxElevationDeg(85.0);
 *   calc.setMinElevationDeg(45.0);
 *   
 *   if (calc.run()) {
 *       double angle = calc.getElevationDeg();
 *       double speed = calc.getShooterSpeedRps();
 *       double peak = calc.getPeakHeightM();
 *   }
 * 
 * @author FRC Team [Your Number]
 */
public class RawTrajectoryCalc {
    
    // ========== BALL PHYSICS ==========
    private static final double BALL_DIAMETER_M = 0.1501;
    private static final double BALL_RADIUS_M = BALL_DIAMETER_M / 2;
    private static final double BALL_MASS_KG = 0.215;
    private static final double BALL_AREA_M2 = Math.PI * Math.pow(BALL_RADIUS_M, 2);
    private static final double DRAG_COEFF = 0.47;
    private static final double AIR_DENSITY = 1.225;
    private static final double GRAVITY = 9.81;
    
    // ========== SHOOTER CONSTANTS ==========
    private static final double ROLLER_DIAMETER_M = 0.0889;
    
    // ========== SIMULATION PARAMETERS ==========
    private static final double DT = 0.0005;
    private static final double MAX_SIM_TIME = 5.0;
    
    // ========== INPUTS (Properties) ==========
    private double shooterHeightM = 0.533;
    private double targetDistanceM = 15.0;
    private double targetHeightM = 0.5;
    private double slipPercentage = 1.0;
    private double maxElevationDeg = 85.0;
    private double minElevationDeg = 45.0;
    
    // ========== OUTPUTS (Properties) ==========
    private double elevationDeg = 0.0;
    private double shooterSpeedRps = 0.0;
    private double peakHeightM = 0.0;
    
    // ========== CONSTRUCTORS ==========
    
    /**
     * Default constructor. Set input properties before calling run().
     */
    public RawTrajectoryCalc() {
    }
    
    // ========== INPUT SETTERS ==========
    
    /**
     * Sets the shooter exit height above ground.
     * @param value Height in meters
     */
    public void setShooterHeightM(double value) {
        this.shooterHeightM = value;
    }
    
    /**
     * Sets the horizontal distance to target.
     * @param value Distance in meters
     */
    public void setTargetDistanceM(double value) {
        this.targetDistanceM = value;
    }
    
    /**
     * Sets the target height above ground.
     * @param value Height in meters
     */
    public void setTargetHeightM(double value) {
        this.targetHeightM = value;
    }
    
    /**
     * Sets the slip percentage (1.0 = no slip, <1.0 = slip reduces velocity).
     * @param value Slip factor (typically 0.9-1.0)
     */
    public void setSlipPercentage(double value) {
        this.slipPercentage = value;
    }
    
    /**
     * Sets the maximum allowed elevation angle.
     * @param value Angle in degrees
     */
    public void setMaxElevationDeg(double value) {
        this.maxElevationDeg = value;
    }
    
    /**
     * Sets the minimum allowed elevation angle.
     * @param value Angle in degrees
     */
    public void setMinElevationDeg(double value) {
        this.minElevationDeg = value;
    }
    
    // ========== INPUT GETTERS ==========
    
    public double getShooterHeightM() {
        return shooterHeightM;
    }
    
    public double getTargetDistanceM() {
        return targetDistanceM;
    }
    
    public double getTargetHeightM() {
        return targetHeightM;
    }
    
    public double getSlipPercentage() {
        return slipPercentage;
    }
    
    public double getMaxElevationDeg() {
        return maxElevationDeg;
    }
    
    public double getMinElevationDeg() {
        return minElevationDeg;
    }
    
    // ========== OUTPUT GETTERS ==========
    
    /**
     * @return Calculated elevation angle in degrees
     */
    public double getElevationDeg() {
        return elevationDeg;
    }
    
    /**
     * @return Calculated shooter roller speed in RPS
     */
    public double getShooterSpeedRps() {
        return shooterSpeedRps;
    }
    
    /**
     * @return Maximum height of ball trajectory in meters
     */
    public double getPeakHeightM() {
        return peakHeightM;
    }
    
    // ========== RUN METHOD ==========
    
    /**
     * Calculates the optimal elevation and shooter speed based on current inputs.
     * Finds the trajectory that requires minimum speed (energy efficient).
     * Results are available via getElevationDeg(), getShooterSpeedRps(), and getPeakHeightM().
     * 
     * @return true if a valid trajectory solution was found, false otherwise
     */
    public boolean run() {
        // Reset outputs
        elevationDeg = 0.0;
        shooterSpeedRps = 0.0;
        peakHeightM = 0.0;
        
        double bestAngle = -1;
        double bestSpeed = Double.MAX_VALUE;
        double bestPeak = 0;
        
        // Search all angles with fine step, find minimum speed solution
        for (double angle = minElevationDeg; angle <= maxElevationDeg; angle += 1.0) {
            
            double speedResult = findSpeedForAngle(angle, targetDistanceM, targetHeightM);
            
            if (speedResult > 0 && speedResult < bestSpeed) {
                TrajectoryResult traj = simulateTrajectory(speedResult, angle);
                
                if (traj.valid) {
                    double error = Math.abs(traj.landingX - targetDistanceM);
                    
                    if (error < 0.2) {
                        bestSpeed = speedResult;
                        bestAngle = angle;
                        bestPeak = traj.peakHeight;
                    }
                }
            }
        }
        
        if (bestAngle > 0 && bestSpeed < Double.MAX_VALUE) {
            this.elevationDeg = bestAngle;
            this.shooterSpeedRps = velocityToRps(bestSpeed);
            this.peakHeightM = bestPeak;
            return true;
        }
        
        return false;
    }
    
    // ========== PRIVATE METHODS ==========
    
    private double findSpeedForAngle(double angleDeg, double targetX, double targetZ) {
        double vMin = 3.0;
        double vMax = 35.0;
        
        for (int i = 0; i < 50; i++) {
            double vMid = (vMin + vMax) / 2;
            TrajectoryResult traj = simulateTrajectory(vMid, angleDeg);
            
            // If invalid AND landed short, need more speed
            // If invalid AND overshot, need less speed
            if (!traj.valid) {
                // Check where ball ended up
                if (traj.landingX < targetX) {
                    vMin = vMid;  // Short - need more speed
                } else {
                    vMax = vMid;  // Overshot or other issue - need less speed
                }
                continue;
            }
            
            double landingError = traj.landingX - targetX;
            
            if (Math.abs(landingError) < 0.02) {
                return vMid;
            }
            
            if (landingError < 0) {
                vMin = vMid;
            } else {
                vMax = vMid;
            }
        }
        
        // Check if final value is close enough
        TrajectoryResult finalTraj = simulateTrajectory((vMin + vMax) / 2, angleDeg);
        if (finalTraj.valid && Math.abs(finalTraj.landingX - targetX) < 0.2) {
            return (vMin + vMax) / 2;
        }
        
        return -1;
    }
    
    private TrajectoryResult simulateTrajectory(double v0, double angleDeg) {
        double angleRad = Math.toRadians(angleDeg);
        double vx = v0 * Math.cos(angleRad);
        double vz = v0 * Math.sin(angleRad);
        
        double x = 0;
        double z = shooterHeightM;
        double peakHeight = z;
        boolean passedPeak = false;
        double prevZ = z;
        double prevX = x;
        
        double t = 0;
        while (t < MAX_SIM_TIME && z >= -0.5 && x < targetDistanceM + 10.0) {
            double v = Math.sqrt(vx * vx + vz * vz);
            double drag = 0.5 * AIR_DENSITY * v * v * DRAG_COEFF * BALL_AREA_M2;
            
            double ax = -drag / BALL_MASS_KG * (vx / v);
            double az = -drag / BALL_MASS_KG * (vz / v) - GRAVITY;
            
            vx += ax * DT;
            vz += az * DT;
            
            prevZ = z;
            prevX = x;
            x += vx * DT;
            z += vz * DT;
            
            if (vz < 0 && !passedPeak) {
                passedPeak = true;
            }
            if (z > peakHeight) {
                peakHeight = z;
            }
            
            // Check when ball crosses target height (descending)
            // Must be past peak AND currently descending (vz < 0)
            if (passedPeak && vz < 0 && z <= targetHeightM && prevZ > targetHeightM) {
                double frac = (targetHeightM - prevZ) / (z - prevZ);
                double xAtTarget = prevX + frac * (x - prevX);
                return new TrajectoryResult(true, xAtTarget, targetHeightM, peakHeight);
            }
            
            // Also check if we're past target distance and descending through target height
            if (passedPeak && vz < 0 && x >= targetDistanceM - 0.5 && x <= targetDistanceM + 0.5) {
                if (z <= targetHeightM + 0.1 && z >= targetHeightM - 0.1) {
                    return new TrajectoryResult(true, x, z, peakHeight);
                }
            }
            
            t += DT;
        }
        
        return new TrajectoryResult(false, x, z, peakHeight);
    }
    
    private double velocityToRps(double velocity) {
        return velocity / (Math.PI * ROLLER_DIAMETER_M * slipPercentage);
    }
    
    // ========== INNER CLASS ==========
    
    private static class TrajectoryResult {
        final boolean valid;
        final double landingX;
        final double landingZ;
        final double peakHeight;
        
        TrajectoryResult(boolean valid, double landingX, double landingZ, double peakHeight) {
            this.valid = valid;
            this.landingX = landingX;
            this.landingZ = landingZ;
            this.peakHeight = peakHeight;
        }
    }
    
    // ========== TEST ==========
    
    public static void Test() {
        System.out.println("RawTrajectoryCalc Test");
        System.out.println("======================");
        System.out.println();
        
        System.out.printf("Shooter height: 21\" (0.533m)%n");
        System.out.printf("Target height: 0.5m%n");
        System.out.println();
        
        System.out.printf("%-12s %-12s %-12s %-12s %-10s%n", 
                          "Distance", "Elevation", "Speed", "Peak Ht", "Valid");
        System.out.printf("%-12s %-12s %-12s %-12s %-10s%n", 
                          "(m)", "(deg)", "(RPS)", "(m)", "");
        System.out.println("-".repeat(58));
        
        double[] distances = {5.0, 10.0, 15.0, 20.0};
        
        RawTrajectoryCalc calc = new RawTrajectoryCalc();
        calc.setShooterHeightM(0.533);  // 21"
        calc.setTargetHeightM(0.5);     // 0.5m above ground
        calc.setSlipPercentage(1.0);
        calc.setMaxElevationDeg(85.0);
        calc.setMinElevationDeg(20.0);
        
        for (double dist : distances) {
            calc.setTargetDistanceM(dist);
            
            boolean valid = calc.run();
            
            if (valid) {
                System.out.printf("%-12.1f %-12.1f %-12.1f %-12.2f %-10s%n",
                    dist, 
                    calc.getElevationDeg(),
                    calc.getShooterSpeedRps(),
                    calc.getPeakHeightM(),
                    "YES");
            } else {
                System.out.printf("%-12.1f %-12s %-12s %-12s %-10s%n",
                    dist, "-", "-", "-", "NO");
            }
        }
    }
}
