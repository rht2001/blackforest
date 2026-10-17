# Digital Twin auf dem Team-main

Basis: `origin/main` bei `5851e49` (17.10.2026). Aktueller 3D-Viewer,
2D-Taktilkarte und lokale HTTP-Bridge aus Karls Twin-Version.
Steuerung, Register, GUI, Datenbank und Abhängigkeiten entsprechen main.
Die frühere separate CSV-Integration ist nicht enthalten.

## Start

Terminal 1 im Repository-Hauptverzeichnis:

```powershell
uv run python digital_twin/bridge_server.py
```

Browser: http://127.0.0.1:18765/ und **Live aus Python** wählen.
2D-Karte: http://127.0.0.1:18765/tactile.
Die Viewer können auch ihre eigene Simulation ohne Hardware anzeigen.

Terminal 2, vorhandene Hardware-App mit optionaler Twin-Ausgabe:

```powershell
uv run python src/main.py --twin
```

Ohne `--twin` startet die Team-App ohne Hintergrundsender. Die Hardware-IP
bleibt wie in main bei `192.168.123.210`. Startup, Standardkraft und automatische
Greifsteuerung stammen unverändert aus main. `--twin-url` überschreibt das
HTTP-Ziel. Bridge und Browser öffnen keine Hardwareverbindung.

Der Adapter verwendet ausschließlich `DataDict` und `last_sensor_data`:
keine zusätzlichen Sensorabfragen oder Bewegungsbefehle. Fehlende oder
unvollständige Regionen werden ausgelassen, Padding bleibt ungültig.
Kraftregister werden nur für die Anzeige vorzeichenbehaftet interpretiert.
Der Hintergrundsender hält höchstens ein wartendes Paket. Eine nicht erreichbare
Bridge blockiert die Datenbank nicht. Von main als Nullen ersetzte Lesefehler
kann der Adapter nicht nachträglich von echten Nullmessungen unterscheiden.

## Tests ohne Hardware

```powershell
uv run python -m unittest discover -s tests -v
```

Tests verwenden künstliche Telemetrie und einen lokalen HTTP-Server;
sie instanziieren keinen Hardwarecontroller.
