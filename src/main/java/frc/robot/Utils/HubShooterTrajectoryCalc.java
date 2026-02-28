/**
 * HubShooterTrajectoryCalc - Calculates elevation angle and shooter speed for dual roller shooter.
 * 
 * FRC 2026 REBUILT - Alliance Hub lob shot trajectory calculator.
 * 
 * Shooter configuration:
 *   - Dual coned TPR rollers (3"→4" cones), 88.9mm effective diameter
 *   - 1:1 counter-rotating (opposite directions, same speed)
 *   - Zero backspin (counter-rotation cancels spin)
 *   - Exit velocity = π × 0.0889 × RPS (100% velocity transfer)
 * 
 * Target: Center of hub funnel opening at 1.829m (72") height
 * Ball must clear funnel edge and land in center
 * 
 * Usage:
 *   HubShooterTrajectoryCalc calc = new HubShooterTrajectoryCalc();
 *   calc.setShooterHeightM(0.6);
 *   calc.setHubDistanceM(5.36);
 *   calc.setSlipPercentage(1.0);
 *   calc.setMaxElevationDeg(80.0);
 *   calc.setMinElevationDeg(45.0);
 *   
 *   if (calc.run()) {
 *       double angle = calc.getElevationDeg();
 *       double speed = calc.getShooterSpeedRps();
 *   }
 * 
 * @author FRC Team [Your Number]
 */
public class HubShooterTrajectoryCalc {
    
    // ========== FIELD CONSTANTS ==========
    private static final double HUB_HEIGHT_M = 1.829;
    private static final double HUB_OPENING_M = 1.065;
    
    // ========== BALL PHYSICS ==========
    private static final double BALL_DIAMETER_M = 0.1501;
    private static final double BALL_RADIUS_M = BALL_DIAMETER_M / 2;
    private static final double BALL_MASS_KG = 0.215;
    private static final double BALL_AREA_M2 = Math.PI * Math.pow(BALL_RADIUS_M, 2);
    private static final double DRAG_COEFF = 0.47;
    private static final double AIR_DENSITY = 1.225;
    private static final double GRAVITY = 9.81;
    
    // ========== CLEARANCE ==========
    /** Extra clearance beyond ball radius for lip clearance (m) */
    private static final double LIP_CLEARANCE_M = 0.025; // 1"
    
    // ========== SHOOTER CONSTANTS ==========
    private static final double ROLLER_DIAMETER_M = 0.0889;
    
    // ========== SIMULATION PARAMETERS ==========
    private static final double DT = 0.0005;
    private static final double MAX_SIM_TIME = 3.0;
    
    // ========== INPUTS (Properties) ==========
    private double shooterHeightM = 0.6;
    private double hubDistanceM = 3.0;
    private double slipPercentage = 1.0;
    private double maxElevationDeg = 80.0;
    private double minElevationDeg = 45.0;
    
    // ========== OUTPUTS (Properties) ==========
    private double elevationDeg = 0.0;
    private double shooterSpeedRps = 0.0;
    private double peakHeightM = 0.0;
    
    // ========== CONSTRUCTORS ==========
    
    /**
     * Default constructor. Set input properties before calling run().
     */
    public HubShooterTrajectoryCalc() {
    }
    
    // ========== INPUT SETTERS ==========
    
    /**
     * Sets the shooter exit height above carpet.
     * @param value Height in meters
     */
    public void setShooterHeightM(double value) {
        this.shooterHeightM = value;
    }
    
    /**
     * Sets the horizontal distance to hub center.
     * @param value Distance in meters
     */
    public void setHubDistanceM(double value) {
        this.hubDistanceM = value;
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
    
    public double getHubDistanceM() {
        return hubDistanceM;
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
     * Results are available via getElevationDeg() and getShooterSpeedRps().
     * 
     * Ball center must clear the hub lip by ball radius + clearance margin.
     * 
     * @return true if a valid trajectory solution was found, false otherwise
     */
    public boolean run() {
        // Reset outputs
        elevationDeg = 0.0;
        shooterSpeedRps = 0.0;
        peakHeightM = 0.0;
        
        // Target point: center of hub
        // Ball center must clear lip, so effective target is higher
        double targetX = hubDistanceM;
        double lipClearanceHeight = HUB_HEIGHT_M + BALL_RADIUS_M + LIP_CLEARANCE_M;
        
        // For close shots, ball enters while ascending - must clear lip on way up
        // For far shots (lob), ball descends into hub - must clear lip on way down
        boolean isCloseShot = (hubDistanceM < 0.6);
        
        // Minimum peak height for lob shots (must be above lip clearance height)
        double minPeakHeight = lipClearanceHeight + 0.05; // 5cm above clearance
        
        double bestAngle = -1;
        double bestSpeed = -1;
        double bestError = Double.MAX_VALUE;
        
        // Search direction based on distance
        double searchStart = (hubDistanceM < 1.5) ? maxElevationDeg : minElevationDeg;
        double searchEnd = (hubDistanceM < 1.5) ? minElevationDeg : maxElevationDeg;
        double searchStep = (hubDistanceM < 1.5) ? -0.5 : 0.5;
        
        for (double angle = searchStart; 
             (searchStep > 0) ? (angle <= searchEnd) : (angle >= searchEnd); 
             angle += searchStep) {
            
            double speedResult = findSpeedForAngle(angle, targetX, lipClearanceHeight,
                                                    isCloseShot ? 0 : minPeakHeight,
                                                    isCloseShot);
            
            if (speedResult > 0) {
                TrajectoryResult traj = simulateTrajectory(speedResult, angle, isCloseShot, 
                                                            lipClearanceHeight);
                
                if (traj.valid && traj.clearsLip) {
                    double error = Math.abs(traj.landingX - targetX);
                    
                    if (error < bestError) {
                        bestError = error;
                        bestAngle = angle;
                        bestSpeed = speedResult;
                    }
                    
                    if (error < 0.05) {
                        break;
                    }
                }
            }
        }
        
        if (bestAngle > 0 && bestSpeed > 0) {
            this.elevationDeg = bestAngle;
            this.shooterSpeedRps = velocityToRps(bestSpeed);
            // Get peak height from final trajectory
            TrajectoryResult finalTraj = simulateTrajectory(bestSpeed, bestAngle, isCloseShot, 
                                                             lipClearanceHeight);
            this.peakHeightM = finalTraj.peakHeight;
            return true;
        }
        
        return false;
    }
    
    // ========== PRIVATE METHODS ==========
    
    private double findSpeedForAngle(double angleDeg, double targetX, double lipClearanceHeight, 
                                      double minPeakHeight, boolean isCloseShot) {
        double vMin = 3.0;
        double vMax = 15.0;
        
        for (int i = 0; i < 30; i++) {
            double vMid = (vMin + vMax) / 2;
            TrajectoryResult traj = simulateTrajectory(vMid, angleDeg, isCloseShot, lipClearanceHeight);
            
            if (!traj.valid) {
                vMin = vMid;
                continue;
            }
            
            double landingError = traj.landingX - targetX;
            
            if (Math.abs(landingError) < 0.02) {
                if (traj.clearsLip && (isCloseShot || traj.peakHeight >= minPeakHeight)) {
                    return vMid;
                } else {
                    return -1;
                }
            }
            
            if (landingError < 0) {
                vMin = vMid;
            } else {
                vMax = vMid;
            }
        }
        
        return -1;
    }
    
    private TrajectoryResult simulateTrajectory(double v0, double angleDeg, boolean isCloseShot,
                                                 double lipClearanceHeight) {
        double angleRad = Math.toRadians(angleDeg);
        double vx = v0 * Math.cos(angleRad);
        double vz = v0 * Math.sin(angleRad);
        
        double x = 0;
        double z = shooterHeightM;
        double peakHeight = z;
        boolean passedPeak = false;
        double prevZ = z;
        double prevX = x;
        
        // Track if ball clears the lip
        // Lip is at hubDistanceM - BALL_RADIUS_M - clearance horizontally
        double lipX = hubDistanceM - BALL_RADIUS_M - LIP_CLEARANCE_M;
        boolean clearsLip = false;
        boolean passedLipX = false;
        
        double t = 0;
        while (t < MAX_SIM_TIME && z > 0 && x < hubDistanceM + 2.0) {
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
            
            // Check lip clearance when crossing lip X position
            if (!passedLipX && x >= lipX && prevX < lipX) {
                double frac = (lipX - prevX) / (x - prevX);
                double heightAtLip = prevZ + frac * (z - prevZ);
                passedLipX = true;
                clearsLip = (heightAtLip >= lipClearanceHeight);
            }
            
            // For close shots: check when ball center is at or past hub distance
            // while still ascending or near peak
            if (isCloseShot && x >= hubDistanceM && prevX < hubDistanceM) {
                // Interpolate height at hub distance
                double frac = (hubDistanceM - prevX) / (x - prevX);
                double heightAtHub = prevZ + frac * (z - prevZ);
                // Accept if ball is at or above hub height
                if (heightAtHub >= HUB_HEIGHT_M) {
                    return new TrajectoryResult(true, hubDistanceM, heightAtHub, peakHeight, clearsLip);
                }
            }
            
            // Descending entry into hub (lob shots)
            if (!isCloseShot && passedPeak && z <= HUB_HEIGHT_M && prevZ > HUB_HEIGHT_M) {
                return new TrajectoryResult(true, x, z, peakHeight, clearsLip);
            }
            
            t += DT;
        }
        
        return new TrajectoryResult(false, x, z, peakHeight, false);
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
        final boolean clearsLip;
        
        TrajectoryResult(boolean valid, double landingX, double landingZ, double peakHeight, 
                         boolean clearsLip) {
            this.valid = valid;
            this.landingX = landingX;
            this.landingZ = landingZ;
            this.peakHeight = peakHeight;
            this.clearsLip = clearsLip;
        }
    }
    
    // ========== TEST ==========
    
    public static void Test() {
        System.out.println("HubShooterTrajectoryCalc Test");
        System.out.println("==============================");
        System.out.println();
        
        // Close shot: 17.5" = 0.445m (robot against hub)
        // Far shot: 5.36m (corner)
        double[] distances = {0.445, 1.0, 2.0, 3.5, 5.36};
        
        System.out.printf("Shooter height: 21\" (0.533m)%n");
        System.out.printf("Hub lip: 72\" (1.829m)%n");
        System.out.printf("Ball clearance: radius + 1\"%n");
        System.out.println();
        
        System.out.printf("%-12s %-12s %-12s %-12s %-10s%n", 
                          "Distance", "Elevation", "Speed", "Peak Ht", "Valid");
        System.out.printf("%-12s %-12s %-12s %-12s %-10s%n", 
                          "(m)", "(deg)", "(RPS)", "(m)", "");
        System.out.println("-".repeat(58));
        
        HubShooterTrajectoryCalc calc = new HubShooterTrajectoryCalc();
        calc.setShooterHeightM(0.533);  // 21"
        calc.setSlipPercentage(1.0);
        calc.setMaxElevationDeg(85.0);
        calc.setMinElevationDeg(45.0);
        
        for (double dist : distances) {
            calc.setHubDistanceM(dist);
            
            boolean valid = calc.run();
            
            if (valid) {
                System.out.printf("%-12.3f %-12.1f %-12.1f %-12.2f %-10s%n",
                    dist, 
                    calc.getElevationDeg(),
                    calc.getShooterSpeedRps(),
                    calc.getPeakHeightM(),
                    "YES");
            } else {
                System.out.printf("%-12.3f %-12s %-12s %-12s %-10s%n",
                    dist, "-", "-", "-", "NO");
            }
        }
    }
}
