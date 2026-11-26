#pragma once

// Initialises the mouse-mode state machine and prints the current mode.
void mouseControllerBegin();

// Call continuously from loop() on core 0. Drains USB mouse reports, handles
// shake/mode transitions and drives the presentation pointer.
void mouseControllerUpdate();
