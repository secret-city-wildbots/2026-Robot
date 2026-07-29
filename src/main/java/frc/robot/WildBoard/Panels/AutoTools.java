package frc.robot.WildBoard.Panels;

import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Utils.PathPlannerAnalysis;
import frc.robot.WildBoard.WBPanel;

/**
 * The "Autos" tab: a field visualiser and a path/auto cross-reference map.
 *
 * <p>The heavy data (every path's geometry, markers and analysis) is written
 * once to {@code /dynamic/autoanalysis.json} and fetched by the frontend, so
 * it never travels over the websocket. The socket carries only the armed auto
 * and rescan requests.
 *
 * <p>Messages from the frontend:
 * <ul>
 *   <li>{@code arm:<autoName>} — arm this auto for the match</li>
 *   <li>{@code rescan} — re-read the deploy folder and rewrite the JSON</li>
 *   <li>{@code hello} — ask what is currently armed (sent on page load)</li>
 * </ul>
 *
 * <p>Messages to the frontend: {@code armed:<name>} (empty name means nothing
 * is armed), {@code refused:<name>}, {@code rescanned}.
 *
 * <p>Incoming messages arrive on the websocket thread. They are parked in
 * volatile fields and acted on in {@link #update()}, which the WildBoard loop
 * calls on the robot thread — building a {@code PathPlannerAuto} off-thread
 * would race the command scheduler.
 */
public class AutoTools extends WBPanel {

    private Consumer<String> onArm = name -> { };
    private PathPlannerAnalysis.Result scan = PathPlannerAnalysis.Result.empty();

    private volatile String armRequest = null;
    private volatile boolean rescanRequest = false;
    private volatile boolean helloRequest = false;

    private String armedAuto = "";
    private String lastBroadcast = null;

    /** Auto-rescan bookkeeping (simulation only — see update()). */
    private long folderStamp = 0;
    private int pollCounter = 0;
    private static final int POLL_EVERY_LOOPS = 100;   // ~2 s at 20 ms

    public AutoTools() {
        this.usesML = true;
        this.setPanelName("AutoTools");
    }

    /** Called on the robot thread with the auto name when one is armed. */
    public AutoTools onArm(Consumer<String> handler) {
        this.onArm = handler;
        return this;
    }

    /** The auto currently armed, or "" if none. */
    public String getArmed() {
        return this.armedAuto;
    }

    /** Auto names that will not load — arming these is refused. */
    public List<String> getUnloadable() {
        return this.scan.unloadable();
    }

    /** Warning count the analysis found for an auto (0 if unknown). */
    public int getWarnCount(String name) {
        return this.scan.warnCounts().getOrDefault(name, 0);
    }

    @Override
    public void start() {
        this.folderStamp = PathPlannerAnalysis.folderStamp();
        this.scan = PathPlannerAnalysis.writeJson();
        if (!this.scan.unloadable().isEmpty()) {
            System.err.println("[AutoTools] these autos will NOT load: "
                    + String.join(", ", this.scan.unloadable()));
        }
    }

    @Override
    public void onMsg(String msg) {
        if (msg == null) return;
        if (msg.equals("hello")) {
            this.helloRequest = true;
        } else if (msg.equals("rescan")) {
            this.rescanRequest = true;
        } else if (msg.startsWith("arm:")) {
            this.armRequest = msg.substring(4).trim();
        }
    }

    @Override
    public void update() {
        if (this.ml == null) return;

        // In simulation PathPlanner writes straight into the deploy folder we
        // read, so poll for edits and rescan on our own. On a real robot the
        // deploy folder cannot change while the code runs, so don't waste the
        // directory listing.
        if (RobotBase.isSimulation() && ++this.pollCounter >= POLL_EVERY_LOOPS) {
            this.pollCounter = 0;
            long now = PathPlannerAnalysis.folderStamp();
            if (now != this.folderStamp) {
                this.folderStamp = now;
                this.rescanRequest = true;
                System.out.println("[AutoTools] pathplanner folder changed, rescanning");
            }
        }

        if (this.rescanRequest) {
            this.rescanRequest = false;
            this.folderStamp = PathPlannerAnalysis.folderStamp();
            this.scan = PathPlannerAnalysis.writeJson();
            this.ml.send("rescanned");
        }

        String req = this.armRequest;
        if (req != null) {
            this.armRequest = null;
            if (!req.isEmpty()) {
                // Refuse anything the analysis says cannot be constructed,
                // rather than letting PathPlannerAuto throw during a match.
                if (this.scan.unloadable().contains(req)) {
                    System.err.println("[AutoTools] refused to arm '" + req
                            + "' — it has errors and would fail to load");
                    this.ml.send("refused:" + req);
                } else {
                    try {
                        this.onArm.accept(req);
                        this.armedAuto = req;
                        System.out.println("[AutoTools] armed auto: " + req);
                    } catch (Exception e) {
                        System.err.println("[AutoTools] failed to arm '" + req + "': " + e);
                        this.ml.send("refused:" + req);
                    }
                }
            }
        }

        // Broadcast the armed auto when it changes, or when a freshly loaded
        // page asks. Without the hello handshake a browser refresh would show
        // nothing armed even though the robot still has one.
        if (this.helloRequest || !this.armedAuto.equals(this.lastBroadcast)) {
            this.helloRequest = false;
            this.lastBroadcast = this.armedAuto;
            this.ml.send("armed:" + this.armedAuto);
        }
    }
}
