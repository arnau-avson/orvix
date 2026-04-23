package com.example.lens;

import android.app.Activity;
import android.os.Bundle;
import android.text.Editable;
import android.text.TextWatcher;
import android.widget.EditText;
import android.widget.TextView;

/**
 * Captura cada cambio del EditText propio (consentimiento implicito al usar
 * la app) y lo envia al backend HTTP. Solo se observa lo que el usuario
 * teclea DENTRO de esta actividad; nada del resto del sistema.
 */
public class MainActivity extends Activity {

    private TextView statusView;
    private BackendClient backend;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        statusView = findViewById(R.id.status);
        EditText input = findViewById(R.id.input);

        backend = new BackendClient(
                "http://127.0.0.1:8080/keys",
                msg -> runOnUiThread(() -> statusView.setText(msg))
        );

        input.addTextChangedListener(new TextWatcher() {
            private String beforeText = "";

            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {
                beforeText = s.toString();
            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                // sin uso; calculamos el delta en afterTextChanged
            }

            @Override
            public void afterTextChanged(Editable s) {
                String afterText = s.toString();
                if (!beforeText.equals(afterText)) {
                    backend.sendKeystroke(System.currentTimeMillis(), beforeText, afterText);
                }
            }
        });
    }
}
