//Menu navigation. The menu structure itself is the table at the top of menu.cpp.
#pragma once

void updateMenu(); //draws the current menu and handles a key press. Call this from loop()

//shows an editor for a positive number of up to 8 digits. Returns true if the user accepted (#), false if they canceled (*)
bool valueEditMenu(long* value, const char* label);
