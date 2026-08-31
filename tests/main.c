/*
 * skywatcher_tests – Test-App für die skywatcher-Bibliothek
 *
 * Führt die öffentliche API von skywatcher aus. Jede Funktion wird durchlaufen
 * und das Ergebnis protokolliert. Der Exit-Status ist:
 *   0  – alle Aufrufe ausgeführt bzw. korrekt übersprungen (kein Mount erreichbar)
 *   1  – ein Test ist fehlgeschlagen
 *
 * Ohne angeschlossene Montung (skywatcher_open == false) werden die
 * Netzwerk-Funktionen nicht ausgeführt und als "SKIPPED" gemeldet.
 */

#include <skywatcher/skywatcher.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <threading/threading.h>

#define TEST_IP "192.168.178.37"

static int g_failures = 0;
static int g_checks = 0;

static void report(const char* name, bool ok)
{
    g_checks++;
    if (ok)
    {
        printf("  [PASS] %s\n", name);
    }
    else
    {
        printf("  [FAIL] %s\n", name);
        g_failures++;
    }
}

static void test_enums_and_types(void)
{
    printf("Test: Enum-/Typ-Definitionen\n");

    report("skywatcher_axis: SA_NONE..SA_AXIS_2 diskret",
        SA_NONE == 0 && SA_AXIS_1 == 1 && SA_AXIS_2 == 2);
    report("skywatcher_mode: SM_TRACKING..SM_GOTO diskret",
        SM_TRACKING == 0 && SM_GOTO == 1);

    skywatcher_goto_callback_t cb = NULL;
    report("goto_callback-Typ ist zuweisbar (NULL)", cb == NULL);
}

static void test_axis_status(void)
{
    printf("Test: skywatcher_get_axis_status (Flags-Auswertung)\n");

    struct skywatcher_axis_status status = {0};

    skywatcher_get_axis_status(SA_AXIS_1, &status);
    report("Mode ist ein gültiger skywatcher_mode-Wert",
        (status.mode == SM_TRACKING) || (status.mode == SM_GOTO));
    report("Direction ist ein gültiger skywatcher_direction-Wert",
        (status.direction == SD_CW) || (status.direction == SD_CCW));
    report("Speed ist ein gültiger skywatcher_speed_mode-Wert",
        (status.speed == SSM_SLOW) || (status.speed == SSM_FAST));
    report("Action ist ein gültiger skywatcher_axis_action-Wert",
        (status.action == SAA_STOPPED) || (status.action == SAA_RUNNING));
    report("Axis-State ist ein gültiger skywatcher_axis_state-Wert",
        (status.axis_state == SAS_NORMAL) || (status.axis_state == SAS_BLOCKED));
    report("Init-State ist ein gültiger skywatcher_init_state-Wert",
        (status.init_state == SIS_NOT_INIT) || (status.init_state == SIS_DONE));
}

static bool wait_axis_stopped(enum skywatcher_axis axis, int timeout_s)
{
    struct skywatcher_axis_status status = {0};
    for (int i = 0; i < timeout_s * 10; i++)
    {
        skywatcher_get_axis_status(axis, &status);
        if (status.action == SAA_STOPPED)
            return true;
        threading_thread_sleep(TTR_MILLI, 100);
    }
    return false;
}

static void test_goto_motion(void)
{
    printf("Test: Goto-Bewegung (45° → 0° → -45° → 0°)\n");

    const double angles[] = {45.0, 0.0, -45.0, 0.0};
    const int steps = sizeof(angles) / sizeof(angles[0]);

    for (int i = 0; i < steps; i++)
    {
        printf("  → %.0f° ...\n", angles[i]);
        skywatcher_goto_deg(SA_AXIS_1, angles[i]);
        skywatcher_goto_deg(SA_AXIS_2, angles[i]);

        bool a1 = wait_axis_stopped(SA_AXIS_1, 60);
        bool a2 = wait_axis_stopped(SA_AXIS_2, 60);

        char name[64];
        snprintf(name, sizeof(name), "Achse 1 bei %.0f° gestoppt", angles[i]);
        report(name, a1);
        snprintf(name, sizeof(name), "Achse 2 bei %.0f° gestoppt", angles[i]);
        report(name, a2);
    }
}

int main(void)
{
    printf("=== skywatcher-Tests ===\n\n");

    test_enums_and_types();

    printf("\nTest: Verbindung (skywatcher_discover)\n");
    char discovered_ip[64] = {0};
    bool discovered = skywatcher_discover(discovered_ip, sizeof(discovered_ip));

    if (discovered)
    {
        printf("  [PASS] Montierung unter %s entdeckt\n", discovered_ip);
    }
    else
    {
        printf("  [SKIP] Keine Montierung per UDP-Broadcast gefunden – Fallback auf %s\n", TEST_IP);
        snprintf(discovered_ip, sizeof(discovered_ip), "%s", TEST_IP);
    }

    printf("\nTest: Verbindung (skywatcher_open)\n");
    bool connected = skywatcher_open(discovered_ip);

    if (!connected)
    {
        printf("  [SKIP] Kein Mount unter %s erreichbar – Hardware-Tests übersprungen\n", discovered_ip);
    }
    else
    {
        printf("  [PASS] Mount erreichbar\n");

        test_axis_status();

        printf("\nTest: Initialisierung\n");
        report("skywatcher_initialize_axis", skywatcher_initialize_axis());

        printf("\nTest: Position\n");
        int32_t position = 0;
        report("skywatcher_get_position", skywatcher_get_position(SA_AXIS_1, &position));
        report("skywatcher_set_position", skywatcher_set_position(SA_AXIS_1, position));
        report("skywatcher_get_axis_position", skywatcher_get_axis_position(SA_AXIS_1, &position));

        printf("\nTest: Goto\n");
        report("skywatcher_set_goto_target", skywatcher_set_goto_target(SA_AXIS_1, 0));
        int32_t target = 0;
        report("skywatcher_get_goto_target", skywatcher_get_goto_target(SA_AXIS_1, &target));

        printf("\nTest: Geschwindigkeit\n");
        double speed = 0.0;
        report("skywatcher_get_speed", skywatcher_get_speed(SA_AXIS_1, &speed));
        report("skywatcher_set_ra_siderial_speed", skywatcher_set_ra_siderial_speed());

        printf("\nTest: Motion\n");
        report("skywatcher_start_motion", skywatcher_start_motion(SA_AXIS_1));
        report("skywatcher_stop_motion", skywatcher_stop_motion(SA_AXIS_1));
        report("skywatcher_instant_stop", skywatcher_instant_stop(SA_AXIS_1));

        printf("\nTest: Achsen-Zustand\n");
        report("skywatcher_set_axis_sleep", skywatcher_set_axis_sleep(SA_AXIS_1, false));
        report("skywatcher_set_aux", skywatcher_set_aux(SA_AXIS_1, false));

        printf("\nTest: Goto-Hilfsfunktionen\n");
        skywatcher_goto_home(SA_AXIS_1, true);
        skywatcher_goto_deg(SA_AXIS_2, 45.0);

        test_goto_motion();

        printf("\nStoppe Montierung...\n");
        skywatcher_instant_stop(SA_AXIS_1);
        skywatcher_instant_stop(SA_AXIS_2);
        skywatcher_close();
    }

    printf("\n=== Ergebnis: %d Checks, %d Fehler ===\n", g_checks, g_failures);
    return g_failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
