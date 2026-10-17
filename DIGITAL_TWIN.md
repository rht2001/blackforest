# App, CSV und Digital Twin gemeinsam testen

Die neue Telemetrie ist optional (`--twin`), liest keine zusätzlichen Sensoren und
sendet keine Hardwarebefehle. Derselbe Snapshot versorgt CSV, Tk-Anzeige und Twin.
CSV speichert jeden Zyklus; der Hintergrundsender überträgt den jeweils neuesten
Snapshot, mit höchstens einem wartenden Paket. Eine fehlende Bridge unterbricht
die Erfassung nicht. Positionswerte bleiben in der CSV; die aktuelle Bridge nutzt
Winkel, Kräfte und die 17 taktilen Regionen. Simulationsdaten sind im Viewer markiert.

## Tests ohne Hardware

Im Repository-Hauptverzeichnis:

```powershell
uv run python -m unittest discover -s tests -v
```

## Gemeinsamer manueller Test

Terminal 1, Repository-Hauptverzeichnis:

```powershell
uv run python digital_twin/bridge_server.py
```

Browser öffnen: http://127.0.0.1:18765/ und **Live aus Python** wählen.
Optional http://127.0.0.1:18765/tactile öffnen und **Live-Bridge** wählen.

Terminal 2, Repository-Hauptverzeichnis:

```powershell
uv run blackforest --simulate --twin --dataset test_dataset.csv
```

GUI-Regler bewegen und loslassen: dieselben Winkel sollten im Twin ankommen.
Der vorhandene Mock liefert taktile Nullen, also keine Kontaktflecken. Die 2D-Karte
kann ihre eigenen synthetischen Kontakte mit **Simulation** anzeigen, diese werden
aber nicht als Robotermessungen in die CSV übernommen.

GUI über X schließen, erneut starten, wieder schließen. CSV prüfen:

```powershell
uv run python -c "import pandas as pd; d=pd.read_csv('test_dataset.csv',sep=';'); print(d[['index','simulated','angles']].tail()); print('Zeilen:',len(d),'Index eindeutig:',d['index'].is_unique)"
Invoke-RestMethod http://127.0.0.1:18765/api/state | Select-Object seq,simulated,angles_act
```

Der Index läuft weiter und simulated ist True. Nach vier Sekunden ohne Daten zeigt
der Twin die Verbindung als veraltet. Bridge mit Ctrl+C beenden. Bleibt die App
offen, läuft die CSV-Erfassung weiter; nach Bridge-Neustart treffen neue Pakete ein.
Keine zweite Hardware-App parallel starten.

## Mit Hardware

Nach dem Simulationstest --simulate weglassen. Dabei gilt das bisherige Startup
mit Öffnen der Hand und automatischem Kontaktschließen. Nur die blackforest-App
verbindet sich zur Hand, Bridge und Browser sind reine Anzeige.

Winkelbereich der Bridge: 0–1000, Kraftbereich: -4000–4000, Taxels: 0–4095.
Ungültige Werte werden für die Anzeige ausgelassen; die CSV behält die Originaldaten.
Fehlende Regionen werden nicht als gültige Nullmatrizen übertragen. Taktile Rohdaten
bleiben in Erfassungsreihenfolge, einschließlich der speziellen Palm-Zuordnung im Viewer.
Der Twin bleibt ein geometrisches Modell mit teilweise angenäherten Sensorlagen
und Gelenkkennlinien; die Anbindung kalibriert diese Geometrie nicht.
