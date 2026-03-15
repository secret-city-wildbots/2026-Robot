package frc.robot.WildBoard;

import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import com.sun.net.httpserver.HttpServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;

import java.io.*;
import java.net.InetSocketAddress;
import java.net.URLConnection;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;

public class Server {

    private final Path publicDir;
    public WsServer ws;
    private HttpServer httpServer;

    public Server(int port) {
        try {
            File tmp;
            if (RobotBase.isSimulation()) {
                tmp = new File(Filesystem.getOperatingDirectory(), "sim/tmp");
            } else {
                tmp = new File("/tmp");
            }

            publicDir = Filesystem.getDeployDirectory()
                    .toPath()
                    .resolve("WildBoard/frontend/public");
            Path dynamicDir = new File(tmp, "frontend-public").toPath();

            httpServer = HttpServer.create(new InetSocketAddress(port), 0);
            httpServer.createContext("/dynamic/", new StaticFileHandler(dynamicDir));
            httpServer.createContext("/", new StaticFileHandler(publicDir));
            httpServer.setExecutor(null);

            ws = new WsServer(port + 1);

            System.out.println("HTTP + WebSocket server running on port " + (port + 1));
            System.out.println("Serving files from: " + publicDir.toAbsolutePath());

        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public void start() {
        httpServer.start();
        ws.start();
    }

    // --------------------------
    // Static File Handler
    // --------------------------
    static class StaticFileHandler implements HttpHandler {
        private final Path rootPath;
        private final Path tmpIndexJs;

        public StaticFileHandler(Path rootDir) {
            this.rootPath = rootDir.toAbsolutePath();
            this.tmpIndexJs = Path.of("/tmp/frontend-public/index.js").toAbsolutePath();
        }

        @Override
        public void handle(HttpExchange exchange) throws IOException {
            if ("websocket".equalsIgnoreCase(exchange.getRequestHeaders().getFirst("Upgrade"))) {
                exchange.sendResponseHeaders(400, 0);
                exchange.close();
                return;
            }

            String requestPath = exchange.getRequestURI().getPath();
            if (requestPath.equals("/"))
                requestPath = "/index.html";

            Path filePath;
            if (requestPath.equals("/index.js")) {
                filePath = tmpIndexJs;
            } else {
                filePath = rootPath.resolve("." + requestPath).normalize();
                if (!filePath.startsWith(rootPath)) {
                    sendResponse(exchange, 403, "Forbidden");
                    return;
                }
            }

            if (Files.exists(filePath) && !Files.isDirectory(filePath)) {
                String mimeType = URLConnection.guessContentTypeFromName(filePath.toString());
                if (mimeType == null)
                    mimeType = "application/octet-stream";

                byte[] fileBytes = Files.readAllBytes(filePath);
                exchange.getResponseHeaders().set("Content-Type", mimeType);
                exchange.sendResponseHeaders(200, fileBytes.length);
                try (OutputStream os = exchange.getResponseBody()) {
                    os.write(fileBytes);
                }
            } else {
                sendResponse(exchange, 404, "404 Not Found");
            }
        }

        private void sendResponse(HttpExchange exchange, int code, String message) throws IOException {
            byte[] bytes = message.getBytes();
            exchange.getResponseHeaders().set("Content-Type", "text/plain");
            exchange.sendResponseHeaders(code, bytes.length);
            try (OutputStream os = exchange.getResponseBody()) {
                os.write(bytes);
            }
        }
    }

    // -----------------------
    // WebSocket Server
    // -----------------------
    static class WsServer extends WebSocketServer {

        private final Set<WebSocket> clients = Collections.synchronizedSet(new HashSet<>());
        private final ObjectMapper mapper = new ObjectMapper();
        private final Map<String, List<BiConsumer<WebSocket, Object>>> eventHandlers = new ConcurrentHashMap<>();
        public Map<Integer, Consumer<String>> onMsg;

        // ===== BATCHING =====
        private final StringBuilder batch = new StringBuilder();
        private final Object batchLock = new Object();

        public WsServer(int port) {
            super(new InetSocketAddress(port));
            
            onMsg = new ConcurrentHashMap<>();
        }

        @Override
        public void onOpen(WebSocket conn, ClientHandshake handshake) {
            clients.add(conn);
            System.out.println("Client connected: " + conn.getRemoteSocketAddress());
        }

        @Override
        public void onClose(WebSocket conn, int code, String reason, boolean remote) {
            clients.remove(conn);
            System.out.println("Client disconnected: " + conn.getRemoteSocketAddress());
        }

        @Override
        public void onMessage(WebSocket conn, String message) {
            if (message.equals("p")) {
                this.ping();
                return;
            }
            if (onMsg != null) {
                try {
                    int dot = message.indexOf('.');
                    if (dot <= 1)
                        throw new RuntimeException();

                    int id = Integer.parseInt(message.substring(1, dot));
                    String payload = message.substring(dot + 1);

                    onMsg.get(id).accept(payload);
                } catch (Exception e) {
                    System.err.println("Invalid message: " + message);
                }
            }
        }

        @Override
        public void onError(WebSocket conn, Exception ex) {
            ex.printStackTrace();
        }

        @Override
        public void onStart() {
            System.out.println("WebSocket server ready");
        }

        public void enqueue(String msg) {
            synchronized (batchLock) {
                batch.append(msg.length()).append(':').append(msg);
            }
        }

        public void bind(int id, Consumer<String> handler) {
            onMsg.put(id, handler);
        }

        public void flush() {
            String out;

            synchronized (batchLock) {
                out = batch.toString();
                batch.setLength(0);
            }

            if (out.isEmpty())
                return;

            //System.out.println(out);

            synchronized (clients) {
                for (WebSocket client : clients) {
                    client.send(out);
                }
            }
        }

        public void ping() {
            try {
                synchronized (clients) {
                for (WebSocket client : clients) {
                    client.send("p");
                }
            }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
}