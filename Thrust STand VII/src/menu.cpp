#include <Arduino.h>
#include "menu.h"
#include "state.h"
#include "screens.h"
#include "tests.h"
#include "load_cells.h"
#include "analog.h"

//////////////////////////////////////////////////////////////////////////////////////////////////
//MENU STRUCTURE
/*
Each item has an ID, and lists the ID of the menu it lives in (its parent).
Submenu items open another menu, value items edit a setting, and action items call a function.

0 MAIN MENU
    1 Run Test
    2 Configure Test
        21 Select Profile
        22 Configure Profiles
            221 Smooth
            222 Intervals
            223 Battery Test
        23 Configure Hardware
    3 Tare Sensors
    4 Debug
*/
enum ItemType {TYPE_SUBMENU, TYPE_TOGGLE, TYPE_VALUE, TYPE_ACTION};

struct MenuItem {
    int itemId;
    const char* label;
    ItemType type;
    int parentId;
    long* variable; //for TYPE_VALUE items
    void (*action)(); //for TYPE_ACTION items
};

static MenuItem menus[] = {
    {0, "Main Menu", TYPE_SUBMENU, 1000, NULL, NULL},

    {1, "Run Test", TYPE_ACTION, 0, NULL, runTest},

    {2, "Configure Test", TYPE_SUBMENU, 0, NULL, NULL},
        {21, "Select Profile", TYPE_ACTION, 2, NULL, selectProfile},
        {22, "Configure Profiles", TYPE_SUBMENU, 2, NULL, NULL},
            {221, "Smooth", TYPE_SUBMENU, 22, NULL, NULL},
                {2211, "Ramp Up Time (s)", TYPE_VALUE, 221, &settings.rampTime, NULL},
                {2212, "Top Hold Time (s)", TYPE_VALUE, 221, &settings.topTime, NULL},
                {2213, "Max Throttle (0-100%)", TYPE_VALUE, 221, &settings.throttleMax, NULL},
            {222, "Intervals", TYPE_SUBMENU, 22, NULL, NULL},
                {2221, "Interval Amount", TYPE_VALUE, 222, &settings.intervalCount, NULL},
                {2222, "Interval Time", TYPE_VALUE, 222, &settings.intervalTime, NULL},
                {2223, "Max Throttle (0-100%)", TYPE_VALUE, 222, &settings.throttleMax, NULL},
                {2224, "Ramp Settle Time (ms)", TYPE_VALUE, 222, &settings.rampSettleTime, NULL},
            {223, "Battery Test", TYPE_SUBMENU, 22, NULL, NULL},
                {2231, "Discharge Amount (mAh)", TYPE_VALUE, 223, &settings.dischargeAmount, NULL},
                {2232, "Current Draw (A)", TYPE_VALUE, 223, &settings.targetAmpDraw, NULL},
                {2233, "Gain (ms)", TYPE_VALUE, 223, &settings.currentTestGain, NULL},
                {2234, "Voltage Cutoff (V)", TYPE_VALUE, 223, &settings.voltageCutoff, NULL},
                {2235, "Sag Recover Time (s)", TYPE_VALUE, 223, &settings.batteryRecoveryTime, NULL},
        {23, "Configure Hardware", TYPE_SUBMENU, 2, NULL, NULL},
            {231, "RPM Marker Count", TYPE_VALUE, 23, &hardware.pulsesPerRev, NULL},
            {232, "RPM Update Rate (ms)", TYPE_VALUE, 23, &hardware.rpmUpdateRate, NULL},
            {233, "A-Spd Override (m/s)", TYPE_VALUE, 23, &hardware.airspeedOverride, NULL},
            {234, "Moving AVG Gain (1-100)", TYPE_VALUE, 23, &hardware.averageGain, NULL},

    {3, "Tare Sensors", TYPE_SUBMENU, 0, NULL, NULL},
        {32, "Zero Thrust", TYPE_ACTION, 3, NULL, tareThrust},
        {33, "Zero Torque", TYPE_ACTION, 3, NULL, tareTorque},
        {34, "Calibrate Thrust Sensor", TYPE_ACTION, 3, NULL, calibrateThrust},
        {35, "Calibrate Torque 1", TYPE_ACTION, 3, NULL, calibrateTorque},
        {37, "Calibrate Torque 2", TYPE_ACTION, 3, NULL, calibrateTorque2},
        {36, "Zero Analog", TYPE_ACTION, 3, NULL, zeroAnalog},

    {4, "Debug", TYPE_ACTION, 0, NULL, debugMenu},
};

static const int MENU_COUNT = sizeof(menus)/sizeof(menus[0]);
static int currentMenuId = 0; //the menu currently on screen

//////////////////////////////////////////////////////////////////////////////////////////////////
//MENU FUNCTIONS

static MenuItem* getMenu(int menuId) { //returns a pointer to the item with that ID, or null if there isn't one
    for (int i = 0; i < MENU_COUNT; i++) {
        if (menus[i].itemId == menuId) {
            return &menus[i];
        }
    }
    return nullptr;
}

static void drawMenu(int menuId) { //pass the ID of the parent menu. Will draw it and all of its items
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tr); //big font for the menu title

    MenuItem* parentMenu = getMenu(menuId);
    if (parentMenu) {
        u8g2.drawStr(2, 9, parentMenu->label);
    }

    u8g2.drawLine(0, 10, 128, 10);

    int menusDrawn = 1; //keep track of how many items we draw so that we can keep moving down
    const int menuOffset = 7; //pixel height of one menu item
    u8g2.setFont(u8g2_font_squeezed_r6_tr); //small font for the items
    for (int i = 0; i < MENU_COUNT; i++) {
        if (menus[i].parentId == menuId) {
            MenuItem* item = &menus[i];
            int y = 12 + menusDrawn*menuOffset;

            u8g2.setCursor(4, y);
            u8g2.print(menusDrawn); //the option number
            u8g2.drawStr(12, y, item->label);

            if (item->type == TYPE_VALUE && item->variable != nullptr) { //show the current value at the end
                u8g2.setCursor(95, y);
                u8g2.print("= ");
                u8g2.print(*(item->variable));
            }

            menusDrawn++;
        }
    }

    u8g2.drawStr(4, 63, "Back: *");
    u8g2.sendBuffer();
}

static int getChosenMenuId(int choice) { //given the number the user picked (1 indexed), returns the ID of that item in the current menu, or -1 if invalid
    if (choice < 1) {
        return -1;
    }
    for (int i = 0; i < MENU_COUNT; i++) {
        if (menus[i].parentId == currentMenuId) {
            if (choice == 1) {
                return menus[i].itemId;
            }
            choice = choice - 1;
        }
    }
    return -1;
}

static void executeMenu(int targetMenuId) {
    MenuItem* targetMenu = getMenu(targetMenuId);
    if (targetMenu->type == TYPE_SUBMENU) {
        currentMenuId = targetMenuId; //open the submenu
    } else if (targetMenu->type == TYPE_ACTION) {
        if (targetMenu->action){
            targetMenu->action();
        }
    } else if (targetMenu->type == TYPE_VALUE) {
        valueEditMenu(targetMenu->variable, targetMenu->label);
    } else if (targetMenu->type == TYPE_TOGGLE) {
        //write bool change function here
    }
}

void updateMenu() {
    drawMenu(currentMenuId);

    char userInput = customKeypad.getKey();
    if (!userInput) {
        return;
    }
    Serial.println(userInput);

    if (userInput >= '0' && userInput <= '9') { //a number picks a menu item
        int choice = getChosenMenuId(userInput - '0');
        if (choice != -1){
            executeMenu(choice);
        }
    } else if (userInput == '*') { //asterisk is the back button
        if (currentMenuId != 0){ //do nothing on the main menu
            currentMenuId = getMenu(currentMenuId)->parentId;
        }
    }

    Serial.print("Active ID is now: ");
    Serial.println(currentMenuId);
}

bool valueEditMenu(long* value, const char* label){
    if (!value){
        return false;
    }

    unsigned long startTime = millis(); //used to blink the cursor
    String input = String(*value);

    while(1){
        u8g2.clearBuffer();
        u8g2.setFontMode(1);
        u8g2.setBitmapMode(1);
        u8g2.setFont(u8g2_font_t0_12b_tr);
        u8g2.drawStr(2, 11, label);
        u8g2.drawLine(0, 13, 127, 13);
        u8g2.setFont(u8g2_font_4x6_tr);
        u8g2.drawStr(89, 51, "Accept: # ");
        u8g2.drawStr(89, 57, "Delete: D");
        u8g2.drawStr(89, 63, "Cancel: *");
        u8g2.setFont(u8g2_font_t0_22b_tr);

        u8g2.setCursor(3, 40);
        u8g2.print(input);

        if ((millis()-startTime)/300 % 2 == 1){ //blink the cursor every 300ms
            u8g2.print("|");
        }

        char userInput = customKeypad.getKey();
        if (userInput >= '0' && userInput <= '9') {
            if (input.length() < 8){ //make sure the number doesn't get too long for long overflow!
                input += userInput;
            }
        } else if (userInput == '*') { //cancel
            return false;
        } else if (userInput == '#') { //accept
            *value = input.toInt();
            return true;
        } else if (userInput == 'D') { //delete
            if (input.length() > 0) {
                input.remove(input.length() - 1);
            }
        }
        u8g2.sendBuffer();
    }
}
