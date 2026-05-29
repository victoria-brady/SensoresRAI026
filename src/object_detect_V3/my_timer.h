#pragma once

// Permite ejecutar una función específica cada X milisegundos
void runTaskPeriodically(void (*callback)(), unsigned long intervalMs);