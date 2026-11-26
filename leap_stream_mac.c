#include <LeapC.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

static double seconds(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return now.tv_sec + now.tv_nsec / 1e9;
}

int main(int argc, char **argv) {
    const bool motor_tracking = argc == 2 && strcmp(argv[1], "--motor-tracking") == 0;
    const double period = motor_tracking ? 0.05 : 0.2;
    LEAP_CONNECTION connection = NULL;
    eLeapRS result = LeapCreateConnection(NULL, &connection);
    if (result == eLeapRS_Success)
        result = LeapOpenConnection(connection);

    double last_frame = 0, next_output = 0;
    LEAP_HAND hand = {0};
    bool visible = false;
    while (result == eLeapRS_Success || result == eLeapRS_Timeout) {
        LEAP_CONNECTION_MESSAGE event = {0};
        result = LeapPollConnection(connection, 100, &event);
        double now = seconds();
        if (result == eLeapRS_Success) {
            if (event.type == eLeapEventType_ConnectionLost)
                break;
            if (event.type == eLeapEventType_Tracking) {
                last_frame = now;
                visible = event.tracking_event->nHands == 1;
                if (visible)
                    hand = event.tracking_event->pHands[0];
            }
        }
        if (now >= next_output) {
            if (visible && now - last_frame < (motor_tracking ? 0.15 : 0.5) && result == eLeapRS_Success) {
                LEAP_VECTOR palm = hand.palm.position;
                printf("%g,%g,%g,%g\n", palm.x, palm.y, palm.z, hand.pinch_strength);
            } else
                puts("MISSING");
            fflush(stdout);
            next_output = now + period;
        }
    }
    fprintf(stderr, "Leap tracking stopped (status %#x).\n", (unsigned)result);
    if (connection)
        LeapDestroyConnection(connection);
    return 1;
}
