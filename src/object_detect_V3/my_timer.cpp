#include "my_timer.h"
#include <Arduino.h>

#define MAX_TASKS 5

struct TaskRecord {
    void (*callback)();
    unsigned long interval;
    unsigned long lastExecution;
};

static TaskRecord tasks[MAX_TASKS];
static uint8_t taskCount = 0;

void runTaskPeriodically(void (*callback)(), unsigned long intervalMs) {
    unsigned long now = millis();
    
    // Buscar si la tarea ya está registrada
    for (uint8_t i = 0; i < taskCount; i++) {
        if (tasks[i].callback == callback) {
            if (now - tasks[i].lastExecution >= tasks[i].interval) {
                tasks[i].lastExecution = now;
                callback();
            }
            return;
        }
    }
    
    // Si no existe y hay espacio, la registramos por primera vez
    if (taskCount < MAX_TASKS) {
        tasks[taskCount].callback = callback;
        tasks[taskCount].interval = intervalMs;
        tasks[taskCount].lastExecution = now; // Se ejecuta en el próximo ciclo
        taskCount++;
    }
}