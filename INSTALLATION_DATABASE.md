# CSV-Integration für Branch karl

Mit `main` (24b501c) am 17.10.2026 abgeglichen. Die aktuelle Command-API,
Greifzustände und `DataDict` bleiben erhalten. `main()` liefert den aktuellen
`DataDict`; `capture_telemetry()` bildet denselben Erfassungszyklus auf das
CSV-/Twin-Schema ab, ohne Sensoren erneut zu lesen. Die Rohdaten für CSV und
Twin sind von den offsetbereinigten Matrizen der Greifsteuerung getrennt.
`database_publish` und `environment-setup` sind in diesem main-Stand bereits
enthalten. Die zusätzlich am 17.10. abgerufenen Stände von `fixing` (85303c8)
und `fixing_dataset` (256bd71) sind ebenfalls integriert. Aus `fixing` kommen Fehlerdiagnose und Roadmap-
Aufteilung hinzu. Bei den überlappenden Steuerungsänderungen hat der neuere
main-Stand Vorrang (Greifbestätigung und Standard-IP); die CSV wird beim
Schließen weiterhin vollständig geschrieben. Beim Zurücksetzen wird die
automatische Kontakterkennung erneut aktiviert.
`state` und `state_change` stehen auch im CSV-Snapshot. `state_change` ist nur
bei einem tatsächlichen Zustandswechsel wahr. Übergänge nach `steady_state`
werden zusätzlich in `state_changes.csv` neben der Haupt-CSV gespeichert;
`--state-changes` überschreibt diesen Pfad. Beide Puffer werden beim Schließen
geschrieben. Der neue Upstream-Logger unter `interface/robot_dataset.py` bleibt
weiterhin nutzbar; die Anwendung nutzt die abgesicherte `RobotDataset`-Persistenz.
Die Dateien dieses Pakets werden relativ zum Repository-Hauptverzeichnis übernommen.
Nicht nach documents entpacken. Die vorhandenen README, Dokumente und Mock-Funktionen
bleiben erhalten. Tests importieren die Module direkt aus src; kein manuelles
Setzen von PYTHONPATH nötig.

```powershell
uv sync
uv run python -m unittest discover -s tests -v
uv run blackforest --simulate --dataset test_dataset.csv
```

Alternativer Einstieg: `uv run python src/main.py --simulate --dataset test_dataset.csv`.
--simulate verhindert jeden Verbindungsversuch zur Hand. Der bisherige automatische
Fallback bei fehlender Verbindung bleibt erhalten. Die CSV-Spalte simulated markiert
Mock-Datensätze explizit. tactile_valid bedeutet vollständig gelesene Werte in der
jeweiligen Quelle (Hardware oder Simulation).

Das X am Fenster stoppt den Zyklus und schreibt den Puffer. Nach erneutem Start mit
demselben Dateipfad wird der Index fortgesetzt. `uv run blackforest` startet mit
der bisherigen Hardware-IP; dabei öffnet das bestehende Startup die Hand und
die automatische Kontaktschließung bleibt aktiv.

## Datenfluss

- src/logic/robot_outputs.py: TypedDict-Schema für einen Lauf.
- src/logic/robot_dataset.py: unabhängiger pandas-Logger, CSV-Trennzeichen `;`.
- src/interface/robot_control.py: einmalige Erfassung pro Zyklus, POSITION aus 1534,
  Kräfte vorzeichenbehaftet, None bei fehlgeschlagenen oder unvollständigen Lesungen.
- src/ui/appInterface.py: zeichnet denselben Snapshot, getrennte GUI-Slider.
- src/main.py: Tk mainloop/after und Flush in on_close sowie finally-Absicherung.
- pyproject.toml: pandas und explizite src-Paketzuordnung für blackforest/main:main.
- tests/: Hardwarefreie Tests für Logger, Controller, Simulation und Shutdown.

CSV-Standardpfad: main_dataset.csv im Arbeitsverzeichnis, überschreibbar mit --dataset.
Neustarts setzen den höchsten gespeicherten Index fort. Neue Felder werden vor dem
Lauf ergänzt; alte Spalten bleiben erhalten. Listen/Matrizen stehen als JSON in
CSV-Zellen (json.loads zum Lesen). Fehlende skalare Werte sind leere Zellen.
Die Matrix hat 112 Zeilen und 17 Spalten in sensor_groups-Reihenfolge. Padding und
fehlende Messwerte sind JSON-null, tactile_valid markiert die gültigen Regionen.
Die CSV enthält Rohwerte, Anzeige-Offsets wirken nur beim Zeichnen.
timestamp_ns ist UTC-Unixzeit beim Beginn des Zyklus; die Sensoren werden nacheinander
gelesen. Winkel, Positionen und Kräfte sind Registerwerte ohne zusätzliche Umrechnung.

Bei 1000 gepufferten Zeilen werden die ältesten 900 geschrieben und entfernt.
100 bleiben, der nächste Batch folgt nach 900 neuen Zeilen. Normales Schließen
schreibt den Rest; ein Prozessabbruch kann gepufferte Zeilen verlieren. Nur ein
Prozess darf eine CSV schreiben. Hardwarelesen und CSV-Schreiben erfolgen im
GUI-Thread: keine harte Echtzeit, Zyklusdauer ist IO-Laufzeit plus 30 ms.

uv sync aktualisiert uv.lock; diese mit committen. main_dataset.csv und test_dataset.csv
sind ignoriert. Für andere Dateinamen die .gitignore ergänzen.
