# skywatcher – Sky-Watcher Montierungs-Steuerung

C-Bibliothek zur Steuerung von Sky-Watcher GoTo-Montierungen über die
Serialschnittstelle (TCP/IP). Bietet Initialisierung, Positionierung,
Goto, Tracking und Geschwindigkeitssteuerung für beide Achsen.

## Umfang

- Verbindungsaufbau/Termination (`skywatcher_open`, `skywatcher_close`)
- Automatische Gerätesuche per UDP-Broadcast (`skywatcher_discover`)
- Achsen-Initialisierung und Kalibrierung (`skywatcher_initialize_axis`)
- Positionierung: `skywatcher_get_position`, `skywatcher_set_position`,
  `skywatcher_get_axis_position`
- Goto: `skywatcher_goto_deg`, `skywatcher_goto_home`,
  `skywatcher_set_goto_target`, `skywatcher_get_goto_target`
- Geschwindigkeit: `skywatcher_set_axis_speed_and_start`,
  `skywatcher_get_speed`, `skywatcher_set_ra_siderial_speed`
- Motion: `skywatcher_start_motion`, `skywatcher_stop_motion`,
  `skywatcher_instant_stop`
- Achsen-Status: `skywatcher_get_axis_status`
- Hintergrund-Thread für Achsen-Monitoring (`skywatcher_start_thread`,
  `skywatcher_stop_thread`)

## Nutzung

```c
#include <skywatcher/skywatcher.h>

int main(void)
{
    char ip[64] = {0};
    if (skywatcher_discover(ip, sizeof(ip)))
    {
        skywatcher_open(ip);
        skywatcher_initialize_axis();
        skywatcher_goto_deg(SA_AXIS_1, 45.0);
        // ...
        skywatcher_close();
    }
    return 0;
}
```

## Abhängigkeiten

- `api` (Callback-/Sichtbarkeits-Makros), `threading`, `string`, `logging`,
  `socket`, `physics`

## Build

```bash
cmake -S . -B build
cmake --build build
```

In ein Projekt einbinden: `add_subdirectory(../../libraries/skywatcher …)`,
Einbindung des Headers über den Include-Pfad `<skywatcher/skywatcher.h>`.

## Tests

Test-App `tests/` (führt die öffentliche API aus, ohne Test-Framework).
Ohne angeschlossene Montierung wird der Hardware-Teil übersprungen:

```bash
cmake -S tests -B tests/build
cmake --build tests/build
./tests/build/bin/skywatcher_tests
```

## Wiki

Dokumentation: [skywatcher – Sky-Watcher Montierungs-Steuerung](https://czybor.i234.me/wiki/sw-module/skywatcher/) (Quartz-Wiki)
