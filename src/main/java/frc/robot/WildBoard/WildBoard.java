package frc.robot.WildBoard;

import frc.robot.WildBoard.Panels.*;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicInteger;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

public class WildBoard {
    private Server server;
    private int PORT = 5804;
    private ArrayList<WBPanel> panels = new ArrayList<WBPanel>();
    private ArrayList<WBPanel> updatePanels = new ArrayList<WBPanel>();
    private ArrayList<WBPanel> updatePanelsTeleOp = new ArrayList<WBPanel>();
    private ArrayList<Tab> tabs = new ArrayList<Tab>();
    private Overrides overrides;
    private double lastTime_s;
    public static double loopTime_ms;

    public WildBoard() {
    }

    public WildBoard(int PORT) {
        this.PORT = PORT;
    }

    public void addPanel(WBPanel panel) {
        panels.add(panel);
    }

    public void addTab(Tab tab) {
        tabs.add(tab);
    }

    public void start() {
        serverStart();
        clientBuild();
        this.server.start();
        lastTime_s = Timer.getTimestamp();
    }

    private void clientBuild() {
        File wildboardHome;
        if (RobotBase.isSimulation()) {
            wildboardHome = new File(Filesystem.getOperatingDirectory(), "sim/home");
        } else {
            wildboardHome = new File("/home/lvuser/WildBoard");
        }
        File deployDir = Filesystem.getDeployDirectory();
        File frontendDir = new File(deployDir, "WildBoard/frontend");

        File indexStub = new File(frontendDir, "src/pages/indexStub.tsx");
        File indexFinalDir = new File(wildboardHome, "frontend/src/pages");
        File indexFinal = new File(wildboardHome, "frontend/src/pages/index.tsx");

        File indexLoader = new File(frontendDir, "src/pages/indexLoader.tsx");
        File indexLoaderFinal = new File(wildboardHome, "frontend/src/pages/indexLoader.tsx");

        indexStub.setReadable(true);
        indexFinal.setReadable(true);
        indexFinal.setWritable(true);

        try {
            // fill in all the blanks for tabs and sidepanels
            String index = Files.readString(indexStub.toPath());
            String tabs = "";
            String sidepanels = "";
            String imports = "";

            for (Tab tab : this.tabs) {
                tabs = tabs + tab.generate();
                imports += tab.genImport() + "\n";
            }
            for (WBPanel panel : this.panels) {
                sidepanels = sidepanels + "<div class=\"column-item\" style=\"padding-bottom: 0;\">\r\n"
                        + panel.generate() + "\n</div>";
                imports += panel.genImport() + "\n";
            }

            index = index.replace("[TABS]", tabs)
                    .replace("[SIDEPANELS]", sidepanels)
                    .replace("[IMPORTS]", imports)
                    .replaceAll("\\[DEPLOY\\]", deployDir.getCanonicalPath().replaceAll("\\\\", "/"));

            indexFinalDir.mkdirs();
            Files.writeString(indexFinal.toPath(), index);

            // fix loader import for sim
            Files.writeString(indexLoaderFinal.toPath(),
                    Files.readString(indexLoader.toPath())
                            .replaceAll("\\[DEPLOY\\]", deployDir.getCanonicalPath().replaceAll("\\\\", "/"))
                            .replaceAll("\\[HOME\\]", wildboardHome.getCanonicalPath().replaceAll("\\\\", "/")));

            System.out.println(wildboardHome.getCanonicalPath());
        } catch (IOException err) {
            System.out.println("couldn't read/write file in frontend construction process.");
            System.err.println(err);
        }

        FrontendBuilder.buildFrontend();
    }

    private void serverStart() {
        server = new Server(PORT);

        AtomicInteger mlId = new AtomicInteger(0);

        // startup panels
        for (int i2 = 0; i2 < panels.size(); i2++) {
            WBPanel wbPanel = panels.get(i2);

            if (wbPanel.usesML) {
                int id = mlId.getAndIncrement();
                System.out.println("assigned ml id " + id);
                wbPanel.assignML(new MessageLayer(server, id), id);

                this.updatePanels.add(wbPanel);
                this.updatePanelsTeleOp.add(wbPanel);
            }
            wbPanel.start();
        }

        // recursive panels
        for (int i2 = 0; i2 < tabs.size(); i2++) {
            Tab tab = tabs.get(i2);

            WBPanelUtils.traverse(tab, panel -> {
                if (panel.usesML) {
                    int id = mlId.getAndIncrement();
                    panel.assignML(new MessageLayer(server, id), id);

                    this.updatePanels.add(panel);
                    if (tab.getTitle() == "TeleOp") {
                        this.updatePanelsTeleOp.add(panel);
                    }
                    if (panel.getPanelName() == "Overrides") {
                        this.overrides = (Overrides) panel;
                    }
                }

                panel.start();
            });
        }

    }

    /*
     * Call this method within RobotPeriodic in order for the dashboard
     * to function properly
     */
    public void update() {
        // update all panels
        // System.out.println(this.overrides.compMode);
        if (!(this.overrides != null && this.overrides.compMode)) {
            for (WBPanel wbPanel : updatePanels) {
                wbPanel.update();
                wbPanel.ml.update();
            }
        } else {
            for (WBPanel wbPanel : updatePanelsTeleOp) {
                wbPanel.update();
                wbPanel.ml.update();
            }
        }

        server.ws.flush();

        double curTime_s = Timer.getTimestamp();
        loopTime_ms = Math.floor((curTime_s - lastTime_s) * 1000);
        lastTime_s = curTime_s;
    }
}