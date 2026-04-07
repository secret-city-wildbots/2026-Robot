package frc.robot.Utils;

import java.io.*;

/**
 * HubShooterTrajectoryCalc - Calculates elevation angle and shooter speed for hub shots.
 * 
 * FRC 2026 REBUILT - Alliance Hub trajectory calculator with lookup cache.
 * 
 * Trajectory Requirements:
 *   1. Clear the lip: Ball must pass INSIDE the 41.7" opening at 72" (clear lip edges)
 *   2. Target center: Ball crosses hub CENTER at 60" height (funnel bottom)
 *   3. Descending: Ball must be descending when passing through lip opening
 * 
 * Hub Geometry (funnel faces UP):
 *   - Lip height: 72" (1.829m) - top edge of funnel opening
 *   - Opening diameter: 41.7" (1.059m) at lip height
 *   - Funnel bottom: 60" (1.524m) - target height for ball crossing
 *   - Target: Center of hub at funnel bottom height, ±6" tolerance
 * 
 * Shooter configuration:
 *   - Dual coned TPR rollers, 88.9mm effective diameter
 *   - Exit velocity = PI * 0.0889 * RPS * slip_factor
 * 
 * Usage:
 *   // Option 1: Direct calculation
 *   HubShooterTrajectoryCalc calc = new HubShooterTrajectoryCalc();
 *   calc.setShooterHeightM(0.533);
 *   calc.setHubDistanceM(3.0);
 *   if (calc.run()) { ... }
 * 
 *   // Option 2: Use cache for fast lookup
 *   HubShooterTrajectoryCalc.initializeCache();
 *   CacheEntry entry = HubShooterTrajectoryCalc.lookupCache(3.0);
 */
public class HubShooterTrajectoryCalc {
    
    // ========== HUB GEOMETRY CONSTANTS ==========
    private static final double HUB_LIP_HEIGHT_M = 1.829;       // 72" lip height (top of funnel)
    private static final double HUB_OPENING_DIAMETER_M = 1.059; // 41.7" opening at top
    private static final double HUB_OPENING_RADIUS_M = HUB_OPENING_DIAMETER_M / 2;  // 0.529m
    private static final double TARGET_HEIGHT_M = 1.524;        // 60" - target height at hub center
    private static final double TARGET_TOLERANCE_M = 0.1524;    // 6" tolerance for landing
    
    // ========== BALL PHYSICS ==========
    private static final double BALL_DIAMETER_M = 0.150;        // 5.91" ball (not 6")
    private static final double BALL_RADIUS_M = BALL_DIAMETER_M / 2;
    private static final double BALL_MASS_KG = 0.215;
    private static final double BALL_AREA_M2 = Math.PI * BALL_RADIUS_M * BALL_RADIUS_M;
    private static final double DRAG_COEFF = 0.47;
    private static final double AIR_DENSITY_KG_M3 = 1.225;
    private static final double GRAVITY_MPS2 = 9.81;
    
    // ========== CLEARANCE ==========
    private static final double LIP_FUDGE_M = 0.3048;  // 12" extra clearance
    private static final double LIP_CLEARANCE_HEIGHT_M = HUB_LIP_HEIGHT_M + BALL_RADIUS_M + LIP_FUDGE_M;
    
    // ========== SHOOTER CONSTANTS ==========
    private static final double ROLLER_DIAMETER_M = 0.0508;  // 2.25" effective
    
    // ========== SIMULATION PARAMETERS ==========
    private static final double DT_S = 0.0005;       // 0.5ms timestep
    private static final double MAX_SIM_TIME_S = 3.0;
    
    // ========== CACHE PARAMETERS ==========
    private static final double CACHE_MIN_DISTANCE_M = 1.0;
    private static final double CACHE_MAX_DISTANCE_M = 8.0;
    private static final int CACHE_SIZE = 50;
    private static final double CACHE_STEP_M = (CACHE_MAX_DISTANCE_M - CACHE_MIN_DISTANCE_M) / (CACHE_SIZE - 1);
    private static final String CACHE_FILE = System.getProperty("user.home", "/home/lvuser") + "/trajectory_cache.json";
    
    // ========== CACHE STORAGE ==========
    private static CacheEntry[] cache = null;
    private static boolean cacheInitialized = false;
    private static double cacheShooterHeight_m = 0.533;  // Shooter height used for cache
    
    // ========== INPUTS ==========
    private double shooterHeight_m = 0.533;   // 21"
    private double hubDistance_m = 3.0;
    private double slipFactor = 0.8;
    private double maxElevation_deg = 85.0;
    private double minElevation_deg = 40.0;
    
    // ========== OUTPUTS ==========
    private double elevation_deg = 0.0;
    private double shooterSpeed_rps = 0.0;
    private double peakHeight_m = 0.0;
    private double airtime_s = 0.0;
    private double landingX_m = 0.0;
    
    // ========== CACHE ENTRY CLASS ==========
    public static class CacheEntry {
        public final double distance_m;
        public final double elevation_deg;
        public final double speed_rps;
        public final double airtime_s;
        public final double peakHeight_m;
        public final boolean valid;
        
        public CacheEntry(double distance_m, double elevation_deg, double speed_rps, 
                          double airtime_s, double peakHeight_m, boolean valid) {
            this.distance_m = distance_m;
            this.elevation_deg = elevation_deg;
            this.speed_rps = speed_rps;
            this.airtime_s = airtime_s;
            this.peakHeight_m = peakHeight_m;
            this.valid = valid;
        }
    }
    
    // ========== CONSTRUCTOR ==========
    public HubShooterTrajectoryCalc() {}
    
    // ========== INPUT SETTERS ==========
    public void setShooterHeightM(double value) { this.shooterHeight_m = value; }
    public void setHubDistanceM(double value) { this.hubDistance_m = value; }
    public void setSlipPercentage(double value) { this.slipFactor = value; }
    public void setMaxElevationDeg(double value) { this.maxElevation_deg = value; }
    public void setMinElevationDeg(double value) { this.minElevation_deg = value; }
    
    // ========== INPUT GETTERS ==========
    public double getShooterHeightM() { return shooterHeight_m; }
    public double getHubDistanceM() { return hubDistance_m; }
    public double getSlipPercentage() { return slipFactor; }
    public double getMaxElevationDeg() { return maxElevation_deg; }
    public double getMinElevationDeg() { return minElevation_deg; }
    
    // ========== OUTPUT GETTERS ==========
    public double getElevationDeg() { return elevation_deg; }
    public double getShooterSpeedRps() { return shooterSpeed_rps; }
    public double getPeakHeightM() { return peakHeight_m; }
    public double getAirtimeS() { return airtime_s; }
    public double getLandingXM() { return landingX_m; }
    
    // ========== CACHE METHODS ==========
    
    /**
     * Initialize the cache - loads from file if exists and params match, otherwise builds and saves.
     * Call once at robot startup.
     * @param shooterHeight_m Shooter height to use for all cache entries
     */
    public static void initializeCache(double shooterHeight_m) {
        // Try to load from file first (will verify parameters match)
        if (loadCacheFromFile(shooterHeight_m)) {
            System.out.println("HubShooterTrajectoryCalc: Cache loaded from " + CACHE_FILE);
            return;
        }
        
        // File doesn't exist, params don't match, or invalid - build the cache
        buildCache(shooterHeight_m);
        
        // Save to file for next time
        saveCacheToFile();
    }
    
    /** Initialize cache with default shooter height (0.533m / 21") */
    public static void initializeCache() {
        initializeCache(0.533);
    }
    
    /**
     * Build the cache from scratch (computation-intensive).
     */
    private static void buildCache(double shooterHeight_m) {
        System.out.println("HubShooterTrajectoryCalc: Building cache (" + CACHE_SIZE + " entries)...");
        long startTime = System.currentTimeMillis();
        
        cacheShooterHeight_m = shooterHeight_m;
        cache = new CacheEntry[CACHE_SIZE];
        
        HubShooterTrajectoryCalc calc = new HubShooterTrajectoryCalc();
        calc.setShooterHeightM(shooterHeight_m);
        calc.setSlipPercentage(1.0);
        calc.setMinElevationDeg(40.0);
        calc.setMaxElevationDeg(85.0);
        
        int validCount = 0;
        double hintAngle = -1;
        double hintSpeed = -1;
        
        for (int i = 0; i < CACHE_SIZE; i++) {
            double distance = CACHE_MIN_DISTANCE_M + i * CACHE_STEP_M;
            calc.setHubDistanceM(distance);
            
            boolean success;
            if (hintAngle < 0) {
                success = calc.run();
            } else {
                success = calc.runWithHint(hintAngle, hintSpeed);
            }
            
            if (success) {
                cache[i] = new CacheEntry(distance, calc.getElevationDeg(), calc.getShooterSpeedRps(),
                                          calc.getAirtimeS(), calc.getPeakHeightM(), true);
                hintAngle = calc.getElevationDeg();
                hintSpeed = calc.getShooterSpeedRps() * Math.PI * ROLLER_DIAMETER_M;
                validCount++;
            } else {
                cache[i] = new CacheEntry(distance, 0, 0, 0, 0, false);
                hintAngle = -1;
                hintSpeed = -1;
            }
            
            if ((i + 1) % 100 == 0) {
                System.out.printf("  ... %d/%d entries%n", i + 1, CACHE_SIZE);
            }
        }
        
        cacheInitialized = true;
        long elapsed = System.currentTimeMillis() - startTime;
        System.out.printf("HubShooterTrajectoryCalc: Cache built in %dms (%d/%d valid)%n",
                          elapsed, validCount, CACHE_SIZE);
    }
    
    /**
     * Save cache to JSON file with all configuration parameters.
     */
    private static void saveCacheToFile() {
        try (PrintWriter writer = new PrintWriter(new FileWriter(CACHE_FILE))) {
            writer.println("{");
            writer.println("  \"version\": 2,");
            writer.println("  \"config\": {");
            writer.println("    \"shooterHeight_m\": " + cacheShooterHeight_m + ",");
            writer.println("    \"hubLipHeight_m\": " + HUB_LIP_HEIGHT_M + ",");
            writer.println("    \"hubOpeningDiameter_m\": " + HUB_OPENING_DIAMETER_M + ",");
            writer.println("    \"targetHeight_m\": " + TARGET_HEIGHT_M + ",");
            writer.println("    \"targetTolerance_m\": " + TARGET_TOLERANCE_M + ",");
            writer.println("    \"ballDiameter_m\": " + BALL_DIAMETER_M + ",");
            writer.println("    \"rollerDiameter_m\": " + ROLLER_DIAMETER_M);
            writer.println("  },");
            writer.println("  \"cache\": {");
            writer.println("    \"size\": " + CACHE_SIZE + ",");
            writer.println("    \"minDistance\": " + CACHE_MIN_DISTANCE_M + ",");
            writer.println("    \"maxDistance\": " + CACHE_MAX_DISTANCE_M);
            writer.println("  },");
            writer.println("  \"entries\": [");
            
            for (int i = 0; i < CACHE_SIZE; i++) {
                CacheEntry e = cache[i];
                StringBuilder line = new StringBuilder("    {");
                line.append("\"d\":").append(String.format("%.4f", e.distance_m)).append(",");
                line.append("\"e\":").append(String.format("%.2f", e.elevation_deg)).append(",");
                line.append("\"r\":").append(String.format("%.2f", e.speed_rps)).append(",");
                line.append("\"a\":").append(String.format("%.4f", e.airtime_s)).append(",");
                line.append("\"p\":").append(String.format("%.3f", e.peakHeight_m)).append(",");
                line.append("\"v\":").append(e.valid);
                line.append("}");
                if (i < CACHE_SIZE - 1) line.append(",");
                writer.println(line.toString());
            }
            
            writer.println("  ]");
            writer.println("}");
            System.out.println("HubShooterTrajectoryCalc: Cache saved to " + CACHE_FILE);
        } catch (IOException e) {
            System.err.println("Warning: Could not save cache file: " + e.getMessage());
        }
    }
    
    /**
     * Load cache from JSON file if it exists and parameters match.
     * @param shooterHeight_m Expected shooter height
     * @return true if successfully loaded, false if file doesn't exist, params mismatch, or invalid
     */
    private static boolean loadCacheFromFile(double shooterHeight_m) {
        try {
            File file = new File(CACHE_FILE);
            if (!file.exists()) {
                return false;
            }
            
            // Read entire file
            StringBuilder sb = new StringBuilder();
            try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
                String line;
                while ((line = reader.readLine()) != null) {
                    sb.append(line).append("\n");
                }
            }
            String content = sb.toString();
            
            // Verify configuration parameters match
            double fileShooterHeight = parseJsonDouble(content, "\"shooterHeight_m\":");
            double fileHubLipHeight = parseJsonDouble(content, "\"hubLipHeight_m\":");
            double fileTargetHeight = parseJsonDouble(content, "\"targetHeight_m\":");
            double fileTargetTolerance = parseJsonDouble(content, "\"targetTolerance_m\":");
            double fileBallDiameter = parseJsonDouble(content, "\"ballDiameter_m\":");
            int fileSize = (int) parseJsonDouble(content, "\"size\":");
            
            // Check if parameters match (with small tolerance for floating point)
            if (Math.abs(fileShooterHeight - shooterHeight_m) > 0.001 ||
                Math.abs(fileHubLipHeight - HUB_LIP_HEIGHT_M) > 0.001 ||
                Math.abs(fileTargetHeight - TARGET_HEIGHT_M) > 0.001 ||
                Math.abs(fileTargetTolerance - TARGET_TOLERANCE_M) > 0.001 ||
                Math.abs(fileBallDiameter - BALL_DIAMETER_M) > 0.001 ||
                fileSize != CACHE_SIZE) {
                System.out.println("HubShooterTrajectoryCalc: Cache params mismatch, rebuilding...");
                return false;
            }
            
            // Parameters match - parse entries
            cache = new CacheEntry[CACHE_SIZE];
            cacheShooterHeight_m = shooterHeight_m;
            
            // Find entries array
            int entriesStart = content.indexOf("\"entries\":");
            if (entriesStart < 0) return false;
            
            int arrayStart = content.indexOf("[", entriesStart);
            int arrayEnd = content.lastIndexOf("]");
            if (arrayStart < 0 || arrayEnd < 0) return false;
            
            String entriesStr = content.substring(arrayStart + 1, arrayEnd);
            
            // Parse each entry
            int idx = 0;
            int pos = 0;
            while (idx < CACHE_SIZE && pos < entriesStr.length()) {
                int objStart = entriesStr.indexOf("{", pos);
                if (objStart < 0) break;
                int objEnd = entriesStr.indexOf("}", objStart);
                if (objEnd < 0) break;
                
                String obj = entriesStr.substring(objStart + 1, objEnd);
                
                double d = parseJsonDouble(obj, "\"d\":");
                double e = parseJsonDouble(obj, "\"e\":");
                double r = parseJsonDouble(obj, "\"r\":");
                double a = parseJsonDouble(obj, "\"a\":");
                double p = parseJsonDouble(obj, "\"p\":");
                boolean v = obj.contains("\"v\":true");
                
                cache[idx] = new CacheEntry(d, e, r, a, p, v);
                idx++;
                pos = objEnd + 1;
            }
            
            if (idx == CACHE_SIZE) {
                cacheInitialized = true;
                return true;
            } else {
                System.err.println("Warning: Cache file has wrong number of entries: " + idx);
                return false;
            }
        } catch (Exception e) {
            System.err.println("Warning: Could not load cache file: " + e.getMessage());
            return false;
        }
    }
    
    /** Helper to parse a double value from simple JSON */
    private static double parseJsonDouble(String json, String key) {
        int idx = json.indexOf(key);
        if (idx < 0) return 0;
        int start = idx + key.length();
        // Skip whitespace after the colon
        while (start < json.length() && Character.isWhitespace(json.charAt(start))) {
            start++;
        }
        int end = start;
        while (end < json.length() && (Character.isDigit(json.charAt(end)) || 
               json.charAt(end) == '.' || json.charAt(end) == '-')) {
            end++;
        }
        try {
            return Double.parseDouble(json.substring(start, end));
        } catch (NumberFormatException e) {
            return 0;
        }
    }
    
    /** Check if cache is initialized */
    public static boolean isCacheInitialized() {
        return cacheInitialized;
    }
    
    /**
     * Lookup trajectory from cache with linear interpolation.
     * @param distance_m Distance to hub center
     * @return CacheEntry with trajectory parameters (check .valid)
     */
    public static CacheEntry lookupCache(double distance_m) {
        if (!cacheInitialized) {
            System.err.println("ERROR: Cache not initialized! Call initializeCache() first.");
            return new CacheEntry(distance_m, 0, 0, 0, 0, false);
        }
        
        // Clamp to cache bounds
        if (distance_m < CACHE_MIN_DISTANCE_M) distance_m = CACHE_MIN_DISTANCE_M;
        if (distance_m > CACHE_MAX_DISTANCE_M) distance_m = CACHE_MAX_DISTANCE_M;
        
        // Find index
        int idx = (int)((distance_m - CACHE_MIN_DISTANCE_M) / CACHE_STEP_M);
        if (idx >= CACHE_SIZE - 1) idx = CACHE_SIZE - 2;
        
        CacheEntry e1 = cache[idx];
        CacheEntry e2 = cache[idx + 1];
        
        // If either is invalid, return the valid one or invalid
        if (!e1.valid && !e2.valid) {
            return new CacheEntry(distance_m, 0, 0, 0, 0, false);
        }
        if (!e1.valid) return e2;
        if (!e2.valid) return e1;
        
        // Linear interpolation
        double t = (distance_m - e1.distance_m) / CACHE_STEP_M;
        return new CacheEntry(
            distance_m,
            e1.elevation_deg + t * (e2.elevation_deg - e1.elevation_deg),
            e1.speed_rps + t * (e2.speed_rps - e1.speed_rps),
            e1.airtime_s + t * (e2.airtime_s - e1.airtime_s),
            e1.peakHeight_m + t * (e2.peakHeight_m - e1.peakHeight_m),
            true
        );
    }
    
    // ========== MAIN CALCULATION ==========
    
    /**
     * Calculate optimal trajectory for current inputs.
     * Finds the LOWEST RPS valid solution.
     * 
     * Trajectory must:
     *   1. Land at CENTER of FUNNEL BOTTOM (target)
     *   2. Pass INSIDE the 41.7" lip opening (clear lip edges at 72" + ball + fudge)
     *   3. Be DESCENDING when passing through lip opening
     * 
     * @return true if valid solution found
     */
    public boolean run() {
        // Reset outputs
        elevation_deg = 0.0;
        shooterSpeed_rps = 0.0;
        peakHeight_m = 0.0;
        airtime_s = 0.0;
        landingX_m = 0.0;
        
        // Target: CENTER of HUB at TARGET_HEIGHT_M (60")
        double targetX_m = hubDistance_m;
        double nearEdgeX_m = hubDistance_m - HUB_OPENING_RADIUS_M;
        
        double bestAngle = -1;
        double bestSpeed = -1;
        double bestRps = Double.MAX_VALUE;
        TrajectoryResult bestTraj = null;
        
        // Search all angles to find lowest RPS solution
        for (double angle = minElevation_deg; angle <= maxElevation_deg; angle += 0.5) {
            
            // Try multiple speed values for this angle to find one that works
            // Speed search: find speed that lands at target AND clears lip
            for (double speed = 3.0; speed <= 25.0; speed += 0.2) {
                TrajectoryResult traj = simulateFullTrajectory(speed, angle);
                
                if (traj == null) continue;
                
                // Check if lands close to target
                double landError = Math.abs(traj.landingX_m - targetX_m);
                if (landError > TARGET_TOLERANCE_M) continue;
                
                // Check all constraints
                boolean clearsLip = traj.heightAtNearEdge_m >= LIP_CLEARANCE_HEIGHT_M;
                boolean descendingAtLip = traj.isDescendingAtLip;
                boolean insideOpening = traj.xAtLipHeight_m >= nearEdgeX_m && 
                                       traj.xAtLipHeight_m <= (hubDistance_m + HUB_OPENING_RADIUS_M);
                
                if (clearsLip && descendingAtLip && insideOpening) {
                    double rps = velocityToRps(speed);
                    if (rps < bestRps) {
                        bestRps = rps;
                        bestAngle = angle;
                        bestSpeed = speed;
                        bestTraj = traj;
                    }
                    break;  // Found valid for this angle, move to next angle
                }
            }
        }
        
        if (bestAngle > 0 && bestTraj != null) {
            this.elevation_deg = bestAngle;
            this.shooterSpeed_rps = velocityToRps(bestSpeed);
            this.peakHeight_m = bestTraj.peakHeight_m;
            this.airtime_s = bestTraj.airtime_s;
            this.landingX_m = bestTraj.landingX_m;
            return true;
        }
        
        return false;
    }
    
    /**
     * Optimized calculation using hints from previous result.
     * Searches near the hinted angle/speed first for faster convergence.
     * Falls back to full search if hint doesn't work.
     * 
     * @param hintAngle_deg Previous successful angle (starting point)
     * @param hintSpeed_mps Previous successful speed in m/s (starting point)
     * @return true if valid solution found
     */
    public boolean runWithHint(double hintAngle_deg, double hintSpeed_mps) {
        // Reset outputs
        elevation_deg = 0.0;
        shooterSpeed_rps = 0.0;
        peakHeight_m = 0.0;
        airtime_s = 0.0;
        landingX_m = 0.0;
        
        double targetX_m = hubDistance_m;
        double targetY_m = TARGET_HEIGHT_M;
        double nearEdgeX_m = hubDistance_m - HUB_OPENING_RADIUS_M;
        
        double bestAngle = -1;
        double bestSpeed = -1;
        double bestScore = Double.MAX_VALUE;
        TrajectoryResult bestTraj = null;
        
        // Search near hint first (±5°), then expand if needed
        double searchRadius = 5.0;
        double angleStart = Math.max(minElevation_deg, hintAngle_deg - searchRadius);
        double angleEnd = Math.min(maxElevation_deg, hintAngle_deg + searchRadius);
        
        for (double angle = angleStart; angle <= angleEnd; angle += 0.25) {
            // Use hint speed as starting point for binary search
            double speed = findSpeedForTargetWithHint(angle, targetX_m, targetY_m, hintSpeed_mps);
            
            if (speed > 0) {
                TrajectoryResult traj = simulateFullTrajectory(speed, angle);
                
                if (traj != null && traj.valid) {
                    boolean clearsLip = traj.heightAtNearEdge_m >= LIP_CLEARANCE_HEIGHT_M;
                    boolean descendingAtLip = traj.isDescendingAtLip;
                    boolean insideOpening = traj.xAtLipHeight_m >= nearEdgeX_m && 
                                           traj.xAtLipHeight_m <= (hubDistance_m + HUB_OPENING_RADIUS_M);
                    boolean hitsCenter = Math.abs(traj.landingX_m - targetX_m) < TARGET_TOLERANCE_M;
                    
                    if (clearsLip && descendingAtLip && insideOpening && hitsCenter) {
                        // Score: PRIORITIZE lowest RPS (speed)
                        double rps = velocityToRps(speed);
                        
                        if (rps < bestScore) {
                            bestScore = rps;
                            bestAngle = angle;
                            bestSpeed = speed;
                            bestTraj = traj;
                        }
                    }
                }
            }
        }
        
        // If hint-based search found solution, use it
        if (bestAngle > 0 && bestTraj != null) {
            this.elevation_deg = bestAngle;
            this.shooterSpeed_rps = velocityToRps(bestSpeed);
            this.peakHeight_m = bestTraj.peakHeight_m;
            this.airtime_s = bestTraj.airtime_s;
            this.landingX_m = bestTraj.landingX_m;
            return true;
        }
        
        // Fall back to full search
        return run();
    }
    
    // ========== PRIVATE METHODS ==========
    
    /**
     * Find speed with hint - binary search near hint for efficiency.
     */
    private double findSpeedForTargetWithHint(double angle_deg, double targetX_m, double targetY_m, double hintSpeed_mps) {
        // Binary search near hint (±3 m/s range)
        double vMin = Math.max(3.0, hintSpeed_mps - 3.0);
        double vMax = Math.min(25.0, hintSpeed_mps + 3.0);
        
        for (int iter = 0; iter < 30; iter++) {
            double vMid = (vMin + vMax) / 2;
            TrajectoryResult traj = simulateFullTrajectory(vMid, angle_deg);
            
            double landX = (traj != null) ? traj.landingX_m : -1;
            
            if (landX < 0 || landX < targetX_m - 0.01) {
                vMin = vMid;
            } else if (landX > targetX_m + 0.01) {
                vMax = vMid;
            } else {
                return vMid;
            }
        }
        
        // Check final result from hint search
        double vFinal = (vMin + vMax) / 2;
        TrajectoryResult traj = simulateFullTrajectory(vFinal, angle_deg);
        if (traj != null && traj.landingX_m > 0 && Math.abs(traj.landingX_m - targetX_m) < TARGET_TOLERANCE_M) {
            return vFinal;
        }
        
        // Fall back to full search
        return findSpeedForTarget(angle_deg, targetX_m, targetY_m);
    }
    
    /**
     * Find the speed that makes ball cross target height at target X distance.
     * Uses binary search, landing X is monotonic with speed at fixed angle.
     */
    private double findSpeedForTarget(double angle_deg, double targetX_m, double targetY_m) {
        double vMin = 3.0;
        double vMax = 25.0;
        
        // Binary search: landingX increases with speed (monotonic)
        for (int iter = 0; iter < 40; iter++) {
            double vMid = (vMin + vMax) / 2;
            TrajectoryResult traj = simulateFullTrajectory(vMid, angle_deg);
            
            // Get landing position (may be -1 if ball doesn't reach target height)
            double landX = (traj != null) ? traj.landingX_m : -1;
            
            if (landX < 0 || landX < targetX_m - 0.01) {
                // Ball lands short or doesn't reach - need more speed
                vMin = vMid;
            } else if (landX > targetX_m + 0.01) {
                // Ball lands past target - need less speed
                vMax = vMid;
            } else {
                // Close enough - return this speed
                return vMid;
            }
        }
        
        // Check final result
        double vFinal = (vMin + vMax) / 2;
        TrajectoryResult traj = simulateFullTrajectory(vFinal, angle_deg);
        if (traj != null && traj.landingX_m > 0 && Math.abs(traj.landingX_m - targetX_m) < TARGET_TOLERANCE_M) {
            return vFinal;
        }
        
        return -1;
    }
    
    /**
     * Simulate full trajectory with air drag.
     * Tracks:
     *   - Height at near edge of opening (for lip clearance)
     *   - X position when crossing lip height (to verify inside opening)
     *   - Landing position at funnel bottom (the target)
     */
    private TrajectoryResult simulateFullTrajectory(double v0_mps, double angle_deg) {
        double angle_rad = Math.toRadians(angle_deg);
        double vx = v0_mps * Math.cos(angle_rad);
        double vy = v0_mps * Math.sin(angle_rad);
        
        double x = 0;
        double y = shooterHeight_m;
        double peakHeight = y;
        boolean passedPeak = false;
        
        // Near edge position for lip clearance check
        double nearEdgeX = hubDistance_m - HUB_OPENING_RADIUS_M;
        double heightAtNearEdge = -1;
        
        // Track crossing of LIP height while descending
        double xAtLipHeight = -1;
        boolean isDescendingAtLip = false;
        
        // Track landing at FUNNEL BOTTOM
        double landingX = -1;
        double landingTime = 0;
        
        double prevX = x;
        double prevY = y;
        
        double t = 0;
        while (t < MAX_SIM_TIME_S && y > 0) {
            // Air drag
            double v = Math.sqrt(vx * vx + vy * vy);
            double dragForce = 0.5 * AIR_DENSITY_KG_M3 * v * v * DRAG_COEFF * BALL_AREA_M2;
            double ax = -dragForce / BALL_MASS_KG * (vx / v);
            double ay = -dragForce / BALL_MASS_KG * (vy / v) - GRAVITY_MPS2;
            
            // Update velocity
            vx += ax * DT_S;
            vy += ay * DT_S;
            
            // Update position
            prevX = x;
            prevY = y;
            x += vx * DT_S;
            y += vy * DT_S;
            t += DT_S;
            
            // Track peak
            if (y > peakHeight) {
                peakHeight = y;
            }
            if (vy < 0 && !passedPeak) {
                passedPeak = true;
            }
            
            // Check height at near edge of opening
            if (heightAtNearEdge < 0 && prevX < nearEdgeX && x >= nearEdgeX) {
                double frac = (nearEdgeX - prevX) / (x - prevX);
                heightAtNearEdge = prevY + frac * (y - prevY);
            }
            
            // Check crossing of LIP height while DESCENDING
            if (passedPeak && xAtLipHeight < 0 && prevY > HUB_LIP_HEIGHT_M && y <= HUB_LIP_HEIGHT_M) {
                double frac = (HUB_LIP_HEIGHT_M - prevY) / (y - prevY);
                xAtLipHeight = prevX + frac * (x - prevX);
                isDescendingAtLip = (vy < 0);
            }
            
            // Check crossing at TARGET height (60")
            if (passedPeak && landingX < 0 && prevY > TARGET_HEIGHT_M && y <= TARGET_HEIGHT_M) {
                double frac = (TARGET_HEIGHT_M - prevY) / (y - prevY);
                landingX = prevX + frac * (x - prevX);
                landingTime = t;
            }
            
            // Stop if we've gone past the hub
            if (x > hubDistance_m + 2.0) {
                break;
            }
        }
        
        // If we never crossed near edge, use current height
        if (heightAtNearEdge < 0 && x >= nearEdgeX) {
            heightAtNearEdge = y;
        }
        
        // Valid if ball lands at funnel bottom and was descending at lip
        boolean valid = (landingX > 0) && isDescendingAtLip && (xAtLipHeight > 0);
        
        return new TrajectoryResult(valid, heightAtNearEdge, xAtLipHeight, isDescendingAtLip,
                                    landingX, peakHeight, landingTime);
    }
    
    private double velocityToRps(double velocity_mps) {
        return velocity_mps / (Math.PI * ROLLER_DIAMETER_M * slipFactor);
    }
    
    // ========== RESULT CLASS ==========
    
    private static class TrajectoryResult {
        final boolean valid;
        final double heightAtNearEdge_m;    // Height when crossing near edge (for lip clearance)
        final double xAtLipHeight_m;        // X position when crossing lip height (descending)
        final boolean isDescendingAtLip;    // Was ball descending when crossing lip?
        final double landingX_m;            // X where ball lands at funnel bottom
        final double peakHeight_m;
        final double airtime_s;
        
        TrajectoryResult(boolean valid, double heightAtNearEdge_m, double xAtLipHeight_m,
                         boolean isDescendingAtLip, double landingX_m, 
                         double peakHeight_m, double airtime_s) {
            this.valid = valid;
            this.heightAtNearEdge_m = heightAtNearEdge_m;
            this.xAtLipHeight_m = xAtLipHeight_m;
            this.isDescendingAtLip = isDescendingAtLip;
            this.landingX_m = landingX_m;
            this.peakHeight_m = peakHeight_m;
            this.airtime_s = airtime_s;
        }
    }
    
    // ========== MAIN ==========
    
    public static void main(String[] args) {
        Test();
    }
    
    // ========== TEST ==========
    
    public static void Test() {
        System.out.println("HubShooterTrajectoryCalc Test");
        System.out.println("=============================");
        System.out.println();
        System.out.println("Target: Land at CENTER of FUNNEL BOTTOM, passing through lip opening");
        System.out.println();
        System.out.printf("Hub lip height:    72\" (%.3fm) - top of funnel%n", HUB_LIP_HEIGHT_M);
        System.out.printf("Target height:     60\" (%.3fm) - hub center target%n", TARGET_HEIGHT_M);
        System.out.printf("Target tolerance:  6\" (%.3fm)%n", TARGET_TOLERANCE_M);
        System.out.printf("Hub opening:       41.7\" (%.3fm) diameter at lip%n", HUB_OPENING_DIAMETER_M);
        System.out.printf("Lip clearance:     %.3fm (lip + ball + fudge)%n", LIP_CLEARANCE_HEIGHT_M);
        System.out.printf("Shooter height:    21\" (0.533m)%n");
        System.out.println();

        // Test direct calculation
        System.out.println("=== Direct Calculation Test ===");
        double startDist = 1.0;
        double endDist = 8.0;
        double step = 0.5;
        
        System.out.printf("%-8s %-8s %-8s %-8s %-8s %-8s %-6s%n", 
                          "Dist(m)", "Dist(ft)", "Elev", "RPS", "Peak(m)", "Air(s)", "Valid");
        System.out.println("--------------------------------------------------------------");
        
        HubShooterTrajectoryCalc calc = new HubShooterTrajectoryCalc();
        calc.setShooterHeightM(0.533);
        calc.setSlipPercentage(1.0);
        calc.setMinElevationDeg(40.0);
        calc.setMaxElevationDeg(85.0);
        
        for (double dist = startDist; dist <= endDist; dist += step) {
            calc.setHubDistanceM(dist);
            boolean valid = calc.run();
            
            if (valid) {
                System.out.printf("%-8.2f %-8.1f %-8.1f %-8.1f %-8.2f %-8.3f %-6s%n",
                    dist, dist / 0.3048,
                    calc.getElevationDeg(), calc.getShooterSpeedRps(),
                    calc.getPeakHeightM(), calc.getAirtimeS(), "YES");
            } else {
                System.out.printf("%-8.2f %-8.1f %-8s %-8s %-8s %-8s %-6s%n",
                    dist, dist / 0.3048, "-", "-", "-", "-", "NO");
            }
        }
        
        // Test cache
        System.out.println();
        System.out.println("=== Cache Test ===");
        initializeCache(0.533);
        
        System.out.println();
        System.out.printf("%-8s %-8s %-8s %-8s %-8s%n", 
                          "Dist(m)", "Elev", "RPS", "Peak(m)", "Air(s)");
        System.out.println("------------------------------------------");
        
        for (double dist = 1.0; dist <= 8.0; dist += 0.5) {
            CacheEntry entry = lookupCache(dist);
            if (entry.valid) {
                System.out.printf("%-8.2f %-8.1f %-8.1f %-8.2f %-8.3f%n",
                    dist, entry.elevation_deg, entry.speed_rps, 
                    entry.peakHeight_m, entry.airtime_s);
            } else {
                System.out.printf("%-8.2f %-8s %-8s %-8s %-8s%n",
                    dist, "-", "-", "-", "-");
            }
        }
        
        System.out.println();
        System.out.println("Cache ready for fast lookups.");
    }
}