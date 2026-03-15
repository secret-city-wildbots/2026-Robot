package frc.robot.WildBoard;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;

public class FrontendBuilder {
    public static void buildFrontend() {
        File deployDir = Filesystem.getDeployDirectory();
        File frontendDir = new File(deployDir, "WildBoard/frontend");

        File tmp;
        if (RobotBase.isSimulation()) {
            tmp = new File(Filesystem.getOperatingDirectory(), "sim/tmp");
        } else {
            tmp = new File("/tmp");
        }

        File wildboardHome;
        if (RobotBase.isSimulation()) {
            wildboardHome = new File(Filesystem.getOperatingDirectory(), "sim/home");
        } else {
            wildboardHome = new File("/home/lvuser/WildBoard");
        }

        tmp.mkdirs();

        File esbuildSrc = new File(frontendDir, "src/esbuild");
        File esbuildTmp = new File(tmp, "esbuild");

        // Copy esbuild only if missing or file size differs
        try {
            boolean copy = true;
            // check file size diff to ensure new versions arent ignored
            if (esbuildTmp.exists() && esbuildTmp.length() == esbuildSrc.length()) {
                copy = false;
            }
            if (copy) {
                Files.copy(esbuildSrc.toPath(), esbuildTmp.toPath(), StandardCopyOption.REPLACE_EXISTING);
                esbuildTmp.setExecutable(true);
            }
        } catch (IOException e) {
            System.err.println("Failed to copy esbuild to /tmp: " + e.getMessage());
            e.printStackTrace();
            return;
        }

        // Output directory in /tmp (writable)
        File outputDir = new File(tmp, "frontend-public/dynamic");
        outputDir.mkdirs();

        File indexLoader = new File(wildboardHome, "frontend/src/pages/indexLoader.tsx");;

        if (RobotBase.isSimulation()) {
            esbuildTmp = new File(tmp, "esbuild.exe");
        }
        
        // Build command using the /tmp copy
        ProcessBuilder pb = new ProcessBuilder(
            esbuildTmp.getAbsolutePath(),
            indexLoader.getAbsolutePath(),
            "--bundle",
            //"--sourcemap", //UNCOMMENT FOR DEBUGGING
            "--outfile=" + new File(outputDir, "index.js").getAbsolutePath(),
            "--format=esm",
            "--platform=browser",
            "--jsx-factory=h",
            "--jsx-fragment=Fragment",
            "--jsx=transform",
            "--minify"
        );

        pb.directory(frontendDir);
        pb.redirectErrorStream(true);

        try {
            Process process = pb.start();
            process.getInputStream().transferTo(System.out);
            int exitCode = process.waitFor();
            System.out.println("Frontend build complete (exit " + exitCode + ")");
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
        }
    }
}