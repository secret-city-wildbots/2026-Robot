package frc.robot.Utils;
/**
 * MovingHubShooterTrajectory - Field-centric turret aiming with motion compensation.
 * 
 * Uses HubShooterTrajectoryCalc's pre-computed cache for O(1) trajectory lookups.
 * Compensates for robot translational motion during ball flight.
 * 
 * FIELD-CENTRIC MODE: Outputs azimuth in field frame. The turret servo is 
 * responsible for maintaining field orientation (compensating for robot rotation).
 * This eliminates the need for rotation prediction in the trajectory calculation.
 * 
 * Coordinate Conventions:
 *   Field: 0° = +X = downfield (toward opponent), 90° = +Y = left, CCW positive
 * 
 * Naming Conventions:
 *   _field_m   = field frame, meters
 *   _field_deg = field frame, degrees
 *   _mps       = meters per second
 *   _rps       = rotations per second
 *   _s         = seconds
 *   _m         = meters (distance)
 * 
 * Usage:
 *   MovingHubShooterTrajectory shooter = new MovingHubShooterTrajectory();
 *   shooter.initialize();  // Call once at robot init (loads trajectory cache)
 *   
 *   // In periodic loop:
 *   shooter.setRobotPosition(x_field_m, y_field_m);
 *   shooter.setRobotVelocity(vx_field_mps, vy_field_mps);
 *   shooter.setAlliance(Alliance.BLUE);
 *   if (shooter.run()) {
 *       double azimuth = shooter.getAzimuth_field_deg();  // Turret servo target
 *       double elevation = shooter.getElevation_deg();
 *       double speed = shooter.getShooterSpeed_rps();
 *   }
 */
public class MovingHubShooterTrajectory {

    // ========== ENUMS ==========
    public enum Alliance { RED, BLUE }

    // ========== FIELD CONSTANTS ==========
    // Field dimensions: ~16.46m x 8.23m (54ft x 27ft)
    private static final double FIELD_LENGTH_M = 16.46;
    private static final double FIELD_WIDTH_M = 8.23;
    private static final double HUB_DISTANCE_FROM_WALL_M = 4.03;  // 158.6"
    
    // Hub positions in field coordinates
    // Blue hub: 4.03m from blue wall (X=0), centered on Y
    private static final double BLUE_HUB_X_FIELD_M = HUB_DISTANCE_FROM_WALL_M;  // 4.03m
    private static final double BLUE_HUB_Y_FIELD_M = FIELD_WIDTH_M / 2;         // 4.115m
    
    // Red hub: 4.03m from red wall (X=16.46m), centered on Y  
    private static final double RED_HUB_X_FIELD_M = FIELD_LENGTH_M - HUB_DISTANCE_FROM_WALL_M;  // 12.43m
    private static final double RED_HUB_Y_FIELD_M = FIELD_WIDTH_M / 2;          // 4.115m

    // ========== TURRET LIMITS ==========
    private static final double MIN_ELEVATION_TURRET_DEG = 40.0;
    private static final double MAX_ELEVATION_TURRET_DEG = 85.0;

    // ========== LEAD COMPENSATION ==========
    private static final int MAX_ITERATIONS = 5;
    private static final double CONVERGENCE_THRESHOLD_M = 0.01;

    // ========== STATE ==========
    private boolean initialized = false;

    // ========== INPUTS ==========
    private double robotX_field_m = 0.0;
    private double robotY_field_m = 0.0;
    private double velocityX_field_mps = 0.0;  // Robot velocity in field X
    private double velocityY_field_mps = 0.0;  // Robot velocity in field Y
    private Alliance alliance = Alliance.BLUE;

    // ========== OUTPUTS ==========
    private double azimuth_field_deg = 0.0;      // Field-centric turret target (with lead)
    private double azimuthToHub_field_deg = 0.0; // Direct angle to hub (no lead)
    private double azimuthDelta_deg = 0.0;       // Lead compensation delta
    private double elevation_deg = 0.0;
    private double shooterSpeed_rps = 0.0;
    private double airtime_s = 0.0;
    private double targetDistance_m = 0.0;
    private double shotQuality = 0.0;            // Shot quality 0.0-1.0
    private boolean valid = false;

    // ========== SHOT QUALITY PARAMETERS ==========
    // Based on ball physics: 22 RPM spin, 1/4" gouges, off-center mass wobble
    private static final double WOBBLE_ACCEL_MPS2 = 0.1;    // Lateral acceleration from wobble (m/s²)
    private static final double TARGET_MARGIN_M = 0.41;     // Effective target radius: (41.7" - 9.5")/2 = 16"
    private static final double MAX_VELOCITY_MPS = 10.0;    // Velocity where quality degrades significantly

    // ========== CONSTRUCTOR ==========
    public MovingHubShooterTrajectory() {}

    // ========== INITIALIZATION ==========
    
    /**
     * Initialize by loading HubShooterTrajectoryCalc cache.
     * Call once at robot startup. Loads from JSON file if available.
     */
    public void initialize() {
        System.out.println("MovingHubShooterTrajectory: Initializing...");
        long startTime = System.currentTimeMillis();

        // Initialize the trajectory cache (loads from JSON or builds)
        HubShooterTrajectoryCalc.initializeCache();

        initialized = true;
        long elapsed = System.currentTimeMillis() - startTime;
        System.out.printf("MovingHubShooterTrajectory: Ready in %dms%n", elapsed);
    }

    // ========== INPUT SETTERS ==========

    /**
     * Set robot position in field coordinates.
     * @param x_field_m X position in field frame (meters)
     * @param y_field_m Y position in field frame (meters)
     */
    public void setRobotPosition(double x_field_m, double y_field_m) {
        this.robotX_field_m = x_field_m;
        this.robotY_field_m = y_field_m;
    }

    /**
     * Set robot velocity vector in field coordinates.
     * @param vx_field_mps velocity in field X direction (m/s, positive = downfield)
     * @param vy_field_mps velocity in field Y direction (m/s, positive = left)
     */
    public void setRobotVelocity(double vx_field_mps, double vy_field_mps) {
        this.velocityX_field_mps = vx_field_mps;
        this.velocityY_field_mps = vy_field_mps;
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    // ========== OUTPUT GETTERS ==========

    /** @return turret azimuth in FIELD frame with lead (degrees, 0=downfield, 90=left) */
    public double getAzimuth_field_deg() { return azimuth_field_deg; }
    
    /** @return direct angle to hub in FIELD frame, no lead (degrees) */
    public double getAzimuthToHub_field_deg() { return azimuthToHub_field_deg; }
    
    /** @return lead compensation delta to add to hub angle (degrees) */
    public double getAzimuthDelta_deg() { return azimuthDelta_deg; }
    
    /** @return turret elevation angle (degrees from horizontal) */
    public double getElevation_deg() { return elevation_deg; }
    
    /** @return shooter wheel speed (rotations per second) */
    public double getShooterSpeed_rps() { return shooterSpeed_rps; }
    
    /** @return estimated ball flight time (seconds) */
    public double getAirtime_s() { return airtime_s; }
    
    /** @return distance to target from shooting position (meters) */
    public double getTargetDistance_m() { return targetDistance_m; }
    
    /** 
     * @return shot quality 0.0-1.0 based on distance and lead compensation
     *         1.0 = optimal (2-5m, low velocity)
     *         0.0 = poor (extreme distance or high lead angle)
     */
    public double getShotQuality() { return shotQuality; }
    
    /** @return true if valid solution was found */
    public boolean isValid() { return valid; }

    // ========== MAIN CALCULATION (OPTIMIZED) ==========

    /**
     * Calculate turret aim with translational motion compensation.
     * Outputs field-centric azimuth for turret servo to track.
     * @return true if valid solution found
     */
    public boolean run() {
        valid = false;

        // Get hub position based on alliance (branch-free for predictability)
        final double hubX = (alliance == Alliance.BLUE) ? BLUE_HUB_X_FIELD_M : RED_HUB_X_FIELD_M;
        final double hubY = (alliance == Alliance.BLUE) ? BLUE_HUB_Y_FIELD_M : RED_HUB_Y_FIELD_M;

        // Direct vector to hub (reused multiple times)
        final double directDx = hubX - robotX_field_m;
        final double directDy = hubY - robotY_field_m;
        final double directDistSq = directDx * directDx + directDy * directDy;
        final double directDist = Math.sqrt(directDistSq);

        // Fast path: if stationary, skip lead compensation iteration
        final boolean isMoving = (velocityX_field_mps != 0.0 || velocityY_field_mps != 0.0);
        
        double aimDist;
        double aimDx, aimDy;
        HubShooterTrajectoryCalc.CacheEntry entry;

        if (!isMoving) {
            // Stationary: aim directly at hub
            aimDist = directDist;
            aimDx = directDx;
            aimDy = directDy;
            entry = HubShooterTrajectoryCalc.lookupCache(aimDist);
            if (entry == null || !entry.valid) return false;
        } else {
            // Moving: iterative lead compensation (typically converges in 2-3 iterations)
            double aimX = robotX_field_m;
            double aimY = robotY_field_m;
            double prevDistSq = 0;
            entry = null;

            for (int iter = 0; iter < MAX_ITERATIONS; iter++) {
                aimDx = hubX - aimX;
                aimDy = hubY - aimY;
                double distSq = aimDx * aimDx + aimDy * aimDy;

                // Convergence check using squared distance (avoids sqrt)
                if (iter > 0) {
                    double deltaSq = distSq - prevDistSq;
                    if (deltaSq < 0) deltaSq = -deltaSq;
                    if (deltaSq < CONVERGENCE_THRESHOLD_M * CONVERGENCE_THRESHOLD_M * 4) break;
                }
                prevDistSq = distSq;

                // Single cache lookup per iteration
                double dist = Math.sqrt(distSq);
                entry = HubShooterTrajectoryCalc.lookupCache(dist);
                if (entry == null || !entry.valid) return false;

                // Predict future position
                aimX = robotX_field_m + velocityX_field_mps * entry.airtime_s;
                aimY = robotY_field_m + velocityY_field_mps * entry.airtime_s;
            }

            // Final distance calculation
            aimDx = hubX - aimX;
            aimDy = hubY - aimY;
            aimDist = Math.sqrt(aimDx * aimDx + aimDy * aimDy);
            
            // Final cache lookup if distance changed significantly
            entry = HubShooterTrajectoryCalc.lookupCache(aimDist);
            if (entry == null || !entry.valid) return false;
        }

        // Extract results from single cache entry
        targetDistance_m = aimDist;
        elevation_deg = entry.elevation_deg;
        shooterSpeed_rps = entry.speed_rps;
        airtime_s = entry.airtime_s;

        // Validate elevation limits
        if (elevation_deg < MIN_ELEVATION_TURRET_DEG || elevation_deg > MAX_ELEVATION_TURRET_DEG) {
            return false;
        }

        // Calculate angles (these atan2 calls are unavoidable)
        azimuthToHub_field_deg = Math.toDegrees(Math.atan2(directDy, directDx));
        
        if (isMoving) {
            // Recalculate aim vector with final airtime
            double finalAimDx = hubX - (robotX_field_m + velocityX_field_mps * airtime_s);
            double finalAimDy = hubY - (robotY_field_m + velocityY_field_mps * airtime_s);
            azimuth_field_deg = Math.toDegrees(Math.atan2(finalAimDy, finalAimDx));
        } else {
            azimuth_field_deg = azimuthToHub_field_deg;
        }

        // Delta calculation (simple subtraction, normalize only if needed)
        azimuthDelta_deg = azimuth_field_deg - azimuthToHub_field_deg;
        if (azimuthDelta_deg > 180) azimuthDelta_deg -= 360;
        else if (azimuthDelta_deg < -180) azimuthDelta_deg += 360;

        // Calculate shot quality (0.0 - 1.0) based on airtime and velocity
        shotQuality = calculateShotQuality(airtime_s, velocityX_field_mps, velocityY_field_mps);

        valid = true;
        return true;
    }

    /**
     * Calculate shot quality based on airtime (ball wobble/drift) and velocity (prediction uncertainty).
     * 
     * Physics model:
     * - Ball wobble from off-center mass and gouges causes lateral drift
     * - Drift = 0.5 × wobble_accel × airtime² (quadratic growth)
     * - Longer airtime = more accumulated drift = lower quality
     * - Higher velocity = more motion prediction uncertainty = lower quality
     * 
     * @param airtime_s ball flight time in seconds
     * @param vx_mps robot velocity X component (m/s)
     * @param vy_mps robot velocity Y component (m/s)
     * @return quality 0.0-1.0 (1.0 = optimal)
     */
    private double calculateShotQuality(double airtime_s, double vx_mps, double vy_mps) {
        // Airtime factor: based on expected ball drift from wobble
        // drift = 0.5 × accel × t², quality degrades as drift approaches target margin
        double expectedDrift = 0.5 * WOBBLE_ACCEL_MPS2 * airtime_s * airtime_s;
        double airtimeQuality = 1.0 - (expectedDrift / TARGET_MARGIN_M);
        airtimeQuality = Math.max(0.0, Math.min(1.0, airtimeQuality));

        // Velocity factor: motion prediction uncertainty
        // At 0 m/s = 100%, degrades linearly to 50% at MAX_VELOCITY_MPS
        double velocity = Math.sqrt(vx_mps * vx_mps + vy_mps * vy_mps);
        double velQuality = 1.0 - (velocity / MAX_VELOCITY_MPS) * 0.5;
        velQuality = Math.max(0.5, Math.min(1.0, velQuality));

        // Combined quality (geometric mean)
        return Math.sqrt(airtimeQuality * velQuality);
    }

    // ========== LOOKUP HELPERS (via HubShooterTrajectoryCalc) ==========

    private double lookupElevation(double distance_m) {
        HubShooterTrajectoryCalc.CacheEntry entry = HubShooterTrajectoryCalc.lookupCache(distance_m);
        return (entry != null && entry.valid) ? entry.elevation_deg : -1;
    }

    private double lookupSpeed(double distance_m) {
        HubShooterTrajectoryCalc.CacheEntry entry = HubShooterTrajectoryCalc.lookupCache(distance_m);
        return (entry != null && entry.valid) ? entry.speed_rps : -1;
    }

    private double lookupAirtime(double distance_m) {
        HubShooterTrajectoryCalc.CacheEntry entry = HubShooterTrajectoryCalc.lookupCache(distance_m);
        return (entry != null && entry.valid) ? entry.airtime_s : 0.5;  // Default 0.5s if invalid
    }

    // ========== UTILITY ==========

    private double normalizeAngle(double angle_deg) {
        while (angle_deg > 180) angle_deg -= 360;
        while (angle_deg < -180) angle_deg += 360;
        return angle_deg;
    }

    // ========== TEST ==========

    public static void main(String[] args) {
        MovingHubShooterTrajectory shooter = new MovingHubShooterTrajectory();
        shooter.initialize();

        System.out.println("\n=== FIELD-CENTRIC TURRET MODE ===");
        System.out.println("Outputs: ToHub (direct), Delta (lead), Final (ToHub + Delta)");
        
        shooter.setAlliance(Alliance.BLUE);

        // Test from position (2.0, 4.11)
        shooter.setRobotPosition(2.0, 4.11);

        System.out.println("\n=== Position (2.0, 4.11) - Various velocities ===");
        System.out.println("Velocity      ToHub   Delta   Final   Elev    RPS   Quality");
        System.out.println("-------------------------------------------------------------");
        
        double[][] velocities = {
            {0.0, 0.0},    // Stationary
            {2.0, 0.0},    // +X (toward hub)
            {-2.0, 0.0},   // -X (away from hub)
            {0.0, 2.0},    // +Y (left)
            {0.0, -2.0},   // -Y (right)
            {5.0, 0.0},    // Fast +X
            {0.0, 5.0},    // Fast +Y
            {0.0, 10.0},   // Very fast +Y
            {0.0, 15.0},   // Max speed +Y
        };

        for (double[] vel : velocities) {
            shooter.setRobotVelocity(vel[0], vel[1]);
            if (shooter.run()) {
                System.out.printf("(%+5.1f,%+5.1f) %6.1f° %+6.1f° %6.1f° %5.1f° %5.1f  %.0f%%%n",
                    vel[0], vel[1],
                    shooter.getAzimuthToHub_field_deg(),
                    shooter.getAzimuthDelta_deg(),
                    shooter.getAzimuth_field_deg(),
                    shooter.getElevation_deg(),
                    shooter.getShooterSpeed_rps(),
                    shooter.getShotQuality() * 100);
            } else {
                System.out.printf("(%+5.1f,%+5.1f) NO SOLUTION%n", vel[0], vel[1]);
            }
        }

        // Test from position (2.0, 0.0) - angled to hub
        shooter.setRobotPosition(2.0, 0.0);
        System.out.println("\n=== Position (2.0, 0.0) - Angled to hub (4.6m away) ===");
        System.out.println("Velocity      ToHub   Delta   Final   Elev    RPS   Quality");
        System.out.println("-------------------------------------------------------------");
        
        for (double[] vel : velocities) {
            shooter.setRobotVelocity(vel[0], vel[1]);
            if (shooter.run()) {
                System.out.printf("(%+5.1f,%+5.1f) %6.1f° %+6.1f° %6.1f° %5.1f° %5.1f  %.0f%%%n",
                    vel[0], vel[1],
                    shooter.getAzimuthToHub_field_deg(),
                    shooter.getAzimuthDelta_deg(),
                    shooter.getAzimuth_field_deg(),
                    shooter.getElevation_deg(),
                    shooter.getShooterSpeed_rps(),
                    shooter.getShotQuality() * 100);
            } else {
                System.out.printf("(%+5.1f,%+5.1f) NO SOLUTION%n", vel[0], vel[1]);
            }
        }
    }
}
