/*
 * Test suite.
 */

#include <Arduino.h>
#include <Adafruit_ILI9341.h>
#include <FEH.h>
#include "../private_include/FEHInternal.h"
#include "../private_include/scheduler.h"

#include "FEHDefines.h"

class uiText
{
private:
    char text[32];
    int16_t x, y;
    uint16_t color;
    uint8_t text_size;
    bool centered;
    void drawInternal(bool useBgColor);

public:
    uiText(int16_t x, int16_t y, uint16_t color, uint8_t size, bool centered);
    void draw(const char *newText);
};

/*
 * User Interface (UI) API.
 *
 * Has text, button, and slider widgets.
 */

// TODO: Button widget
// TODO: Slider widget

#define UI_BG_COLOR FEHLCD::Black

#define SLIDER_MIN_X 40
#define SLIDER_MAX_X 280

uiText::uiText(int16_t x, int16_t y, uint16_t color, uint8_t text_size, bool centered)
    : x(x), y(y), color(color), text_size(text_size), centered(centered)
{
    memset(this->text, 0, sizeof(this->text));
}

void uiText::drawInternal(bool eraseMode)
{
    /* Disable text wrapping so it doesn't mess with printing the label */
    /* or LCD.getTextBounds(). */
    ILI9341.setTextWrap(false);

    /* Set text size first so LCD.getTextBounds() below can use it */
    LCD.SetFontSize(this->text_size);

    int16_t x = this->x;
    if (this->centered)
    {
        int16_t x1, y1;
        uint16_t width, height;
        ILI9341.getTextBounds(this->text, 0, 0, &x1, &y1, &width, &height);

        x -= width / 2;
    }

    LCD.SetTextCursor(x, this->y);

    if (eraseMode)
    {
        LCD.SetFontColor(UI_BG_COLOR);
    }
    else
    {
        LCD.SetFontColor(this->color, UI_BG_COLOR);
    }

    LCD.Write(this->text);

    ILI9341.setTextWrap(true);
}

void uiText::draw(const char *newText)
{
    if (strlen(newText) < strlen(this->text))
    {
        /* If text size becomes smaller erase old text first */
        this->drawInternal(true);
    }

    /* Otherwise, just draw over old text */
    strlcpy(this->text, newText, sizeof(this->text));
    this->drawInternal(false);
}

// Takes a value as a percentage and maps it to the slider's x-coordinates
void drawTickMark(int value, int y, int endY)
{
    static constexpr int tickDist = 5;
    int x = map(value, 0, 100, SLIDER_MIN_X, SLIDER_MAX_X);
    LCD.DrawLine(x + 0, y - tickDist, x + 0, endY + tickDist);
    LCD.DrawLine(x + 1, y - tickDist, x + 1, endY + tickDist);
}

void drawSlider(int y)
{
    LCD.SetFontColor(WHITE);

    /* Draw slider*/
    LCD.DrawLine(40, y + 0, LCD_WIDTH - 40, y + 0);
    LCD.DrawLine(40, y + 1, LCD_WIDTH - 40, y + 1);
    LCD.DrawLine(40, y + 2, LCD_WIDTH - 40, y + 2);

    int endY = y + 2;

    drawTickMark(0, y, endY);
    drawTickMark(25, y, endY);
    drawTickMark(50, y, endY);
    drawTickMark(75, y, endY);
    drawTickMark(100, y, endY);
}

int forceBounds(int val, int min, int max, bool wrap)
{
    if (wrap)
    {
        if (val < min)
            return max;
        if (val > max)
            return min;
    }
    else
    {
        if (val < min)
            return min;
        if (val > max)
            return max;
    }
    return val;
}

void labelSlider(int y, const char *minLabel, const char *midLabel, const char *maxLabel)
{
    int labelY = y + 20;

    uiText tSliderMin(SLIDER_MIN_X, labelY, FEHLCD::White, 2, true);
    uiText tSliderMid((SLIDER_MAX_X + SLIDER_MIN_X) / 2, labelY, FEHLCD::White, 2, true);
    uiText tSliderMax(SLIDER_MAX_X, labelY, FEHLCD::White, 2, true);

    tSliderMin.draw(minLabel);
    tSliderMid.draw(midLabel);
    tSliderMax.draw(maxLabel);
}

static FEHMotor motors[] = {
    FEHMotor(FEHMotor::FEHMotorPort::Motor0, 12),
    FEHMotor(FEHMotor::FEHMotorPort::Motor1, 12),
    FEHMotor(FEHMotor::FEHMotorPort::Motor2, 12),
    FEHMotor(FEHMotor::FEHMotorPort::Motor3, 12),
};

static volatile int8_t targetMotorSpeeds[NUM_MOTORS] = {0};
static volatile int8_t rampedMotorSpeeds[NUM_MOTORS] = {0};

void motorRampingCallback()
{
    for (int i = 0; i < 4; i++)
    {
        if (rampedMotorSpeeds[i] < targetMotorSpeeds[i])
            rampedMotorSpeeds[i]++;
        else if (rampedMotorSpeeds[i] > targetMotorSpeeds[i])
            rampedMotorSpeeds[i]--;
        motors[i].SetPercent(rampedMotorSpeeds[i]);
    }
    scheduleEvent(motorRampingCallback, schedulerMsToTicks(2));
}

#define MENU_C WHITE
#define TEXT_C GOLD
#define SELT_C RED
#define SHOW_C BLUE
#define HI_C GREEN

typedef enum
{
    MAIN,
    MOTOR,
    SERVO,
    DIGITAL,
    ANALOG,
    BATTERY,
    TOUCH
} selectedMenu;

class TestingMenu
{

private:
    char _backLabel[1][20] = {"Back"};


public:
    TestingMenu() {}

    selectedMenu _sel_menu = MAIN;

    int sel_motor = 0;
    int sel_servo = 0;

    int x, y; // Touch coordinates

    void NewMenu()
    {
        switch (_sel_menu)
        {
        case MAIN:
            MainMenu();
            break;
        case MOTOR:
            MotorMenu();
            break;
        case SERVO:
            ServoMenu();
            break;
        case DIGITAL:
            DigitalMenu();
            break;
        case ANALOG:
            AnalogMenu();
            break;
        case BATTERY:
            BatteryMenu();
            break;
        case TOUCH:
            TouchMenu();
            break;
        default:
            MainMenu();
            break;
        }
    }

    void MainMenu()
    {
        LCD.Clear(BLACK);
        LCD.SetFontSize(2);

        FEHIcon::Icon _main_title_icon_arr[1];
        char _main_title[1][20] = {"ERC2 TEST GUI"};
        FEHIcon::DrawIconArray(_main_title_icon_arr, 1, 1, 1, 201, 1, 1, _main_title, HI_C, TEXT_C);
        _main_title_icon_arr[0].Select();

        static constexpr int NUM_MAIN_ICONS = 6;
        FEHIcon::Icon _main_icon_arr[NUM_MAIN_ICONS];
        char _main_icon_labels[NUM_MAIN_ICONS][20] = {"Motor", "Servo", "Digital In", "Analog In", "Battery", "Touch"};
        FEHIcon::DrawIconArray(_main_icon_arr, (NUM_MAIN_ICONS + 1) / 2, 2, 40, 20, 1, 1, _main_icon_labels, MENU_C, TEXT_C);

        while (_sel_menu == MAIN)
        {
            if (LCD.Touch(&x, &y))
            {
                /* Check to see if a main menu icon has been touched */
                for (int i = 0; i < NUM_MAIN_ICONS; i++)
                {
                    if (_main_icon_arr[i].Pressed(x, y, 0))
                    {
                        _main_icon_arr[i].WhilePressed(x, y);
                        _sel_menu = (selectedMenu)(i + 1);
                        break;
                    }
                }
            }
        }
    }

    void MotorMenu()
    {

        static constexpr int TOP_LABEL_Y = 50;
        static constexpr int SLIDER_Y = 190;

        LCD.Clear(BLACK);

        // Create back to main menu icon
        FEHIcon::Icon _motor_back_icon_arr[1];
        FEHIcon::DrawIconArray(_motor_back_icon_arr, 1, 1, 1, 201, 1, 1, _backLabel, MENU_C, TEXT_C);
        _motor_back_icon_arr[0].Select();

        scheduleEvent(motorRampingCallback, 0);

        char label[64];

        int motorUnderTest = 0;

        drawSlider(SLIDER_Y);
        labelSlider(SLIDER_Y, "-100%", "0%", "100%");

        uiText leftArrow(SLIDER_MIN_X, TOP_LABEL_Y, FEHLCD::White, 4, true);
        uiText rightArrow(SLIDER_MAX_X, TOP_LABEL_Y, FEHLCD::White, 4, true);
        leftArrow.draw("<");
        rightArrow.draw(">");

        uiText tMotor(160, TOP_LABEL_Y, FEHLCD::White, 4, true);
        uiText tPercent(160, 110, FEHLCD::White, 6, true);

        while (_sel_menu == MOTOR)
        {
            snprintf(label, 64, "Motor%d", motorUnderTest);
            tMotor.draw(label);

            snprintf(label, 64, "%d%%", targetMotorSpeeds[motorUnderTest]);
            tPercent.draw(label);

            while (true)
            {
                int touchX, touchY;
                if (LCD.Touch(&touchX, &touchY))
                {

                    // Check if back button is pressed
                    if (_motor_back_icon_arr[0].Pressed(touchX, touchY, 0))
                    {
                        _motor_back_icon_arr[0].WhilePressed(touchX, touchY);
                        _sel_menu = MAIN;
                        break;
                    }

                    if (touchY > 120)
                    {
                        int p = map(touchX, SLIDER_MIN_X, SLIDER_MAX_X, -100, 100);
                        p = forceBounds(p, -100, 100, false);

                        /* Convenience deadzone */
                        if (abs(p) < 10)
                            p = 0;

                        snprintf(label, 64, "%d%%", p);
                        tPercent.draw(label);

                        targetMotorSpeeds[motorUnderTest] = p;
                    }
                    else
                    {
                        LCD.WaitForTouchToEnd();

                        if (touchX >= 160)
                            motorUnderTest++;
                        else
                            motorUnderTest--;
                        motorUnderTest = forceBounds(motorUnderTest, 0, NUM_MOTORS - 1, true);
                        break;
                    }
                }
            }
        }

        // Stop motor ramping and turn off motors
        cancelEvents(motorRampingCallback);
        for (int i = 0; i < NUM_MOTORS; i++)
        {
            motors[i].Stop();
        }
    }

    void ServoMenu()
    {

        FEHServo servos[] = {
            FEHServo(FEHServo::FEHServoPort::Servo0),
            FEHServo(FEHServo::FEHServoPort::Servo1),
            FEHServo(FEHServo::FEHServoPort::Servo2),
            FEHServo(FEHServo::FEHServoPort::Servo3),
            FEHServo(FEHServo::FEHServoPort::Servo4),
            FEHServo(FEHServo::FEHServoPort::Servo5),
            FEHServo(FEHServo::FEHServoPort::Servo6),
            FEHServo(FEHServo::FEHServoPort::Servo7),
        };
        

        static constexpr int TOP_LABEL_Y = 50;
        static constexpr int SLIDER_Y = 190;

        LCD.Clear(BLACK);

        // Create back to main menu icon
        FEHIcon::Icon _servo_back_icon_arr[1];
        FEHIcon::DrawIconArray(_servo_back_icon_arr, 1, 1, 1, 201, 1, 1, _backLabel, MENU_C, TEXT_C);
        _servo_back_icon_arr[0].Select();

        char label[64];

        int servoUnderTest = 0;

        drawSlider(SLIDER_Y);
        labelSlider(SLIDER_Y, "0", "90", "180");

        uiText leftArrow(SLIDER_MIN_X, TOP_LABEL_Y, FEHLCD::White, 4, true);
        uiText rightArrow(SLIDER_MAX_X, TOP_LABEL_Y, FEHLCD::White, 4, true);
        leftArrow.draw("<");
        rightArrow.draw(">");

        uiText tServo(160, TOP_LABEL_Y, FEHLCD::White, 4, true);
        uiText tAngle(160, 110, FEHLCD::White, 6, true);

        int8_t targetServoPositions[NUM_SERVOS] = {0};

        while (_sel_menu == SERVO)
        {
            snprintf(label, 64, "Servo%d", servoUnderTest);
            tServo.draw(label);

            snprintf(label, 64, "%d", targetServoPositions[servoUnderTest]);
            tAngle.draw(label);

            while (true)
            {
                int touchX, touchY;
                if (LCD.Touch(&touchX, &touchY))
                {

                    // Check if back button is pressed
                    if (_servo_back_icon_arr[0].Pressed(touchX, touchY, 0))
                    {
                        _servo_back_icon_arr[0].WhilePressed(touchX, touchY);
                        _sel_menu = MAIN;
                        break;
                    }

                    if (touchY > 120)
                    {
                        int p = map(touchX, SLIDER_MIN_X, SLIDER_MAX_X, 0, 180);
                        p = forceBounds(p, 0, 180, false);

                        snprintf(label, 64, "%d", p);
                        tAngle.draw(label);
                        targetServoPositions[servoUnderTest] = p;
                        servos[servoUnderTest].SetDegree(p);
                    }
                    else
                    {
                        LCD.WaitForTouchToEnd();

                        if (touchX >= 160)
                            servoUnderTest++;
                        else
                            servoUnderTest--;
                        servoUnderTest = forceBounds(servoUnderTest, 0, NUM_SERVOS - 1, true);
                        break;
                    }
                }
            }
        }

        // Turn off servos
        for (int i = 0; i < NUM_SERVOS; i++)
        {
            servos[i].Off();
        }

    }

    void DigitalMenu()
    {

        static constexpr int TOP_LABEL_Y = 47;

        LCD.Clear(BLACK);

        // Create back to main menu icon
        FEHIcon::Icon _digital_back_icon_arr[1];
        FEHIcon::DrawIconArray(_digital_back_icon_arr, 1, 1, 1, 201, 1, 1, _backLabel, MENU_C, TEXT_C);
        _digital_back_icon_arr[0].Select();

        char label[64];

        // // Label the screen
        uiText tTitle(160, TOP_LABEL_Y, FEHLCD::White, 3, true);
        tTitle.draw("Digital In");
        uiText leftArrow(20, TOP_LABEL_Y, FEHLCD::White, 4, true);
        uiText rightArrow(300, TOP_LABEL_Y, FEHLCD::White, 4, true);
        leftArrow.draw("<");
        rightArrow.draw(">");

        LCD.SetFontSize(2);

        FEHIcon::Icon _iconGroup0[8];
        FEHIcon::Icon _iconGroup1[8];
        char _nullLabels[8][20] = {""};

        FEHIcon::DrawIconArray(_iconGroup0, 2, 4, 80, 90, 10, 10, _nullLabels, MENU_C, TEXT_C);
        FEHIcon::DrawIconArray(_iconGroup1, 2, 4, 160, 10, 10, 10, _nullLabels, MENU_C, TEXT_C);


        DigitalInputPin digitalPins[NUM_STUDENT_GPIO] = {
            DigitalInputPin(FEHIO::Pin0),
            DigitalInputPin(FEHIO::Pin1),
            DigitalInputPin(FEHIO::Pin2),
            DigitalInputPin(FEHIO::Pin3),
            DigitalInputPin(FEHIO::Pin4),
            DigitalInputPin(FEHIO::Pin5),
            DigitalInputPin(FEHIO::Pin6),
            DigitalInputPin(FEHIO::Pin7),
            DigitalInputPin(FEHIO::Pin8),
            DigitalInputPin(FEHIO::Pin9),
            DigitalInputPin(FEHIO::Pin10),
            DigitalInputPin(FEHIO::Pin11),
            DigitalInputPin(FEHIO::Pin12),
            DigitalInputPin(FEHIO::Pin13),
            DigitalInputPin(FEHIO::Pin14),
            DigitalInputPin(FEHIO::Pin15),
        };

        Serial.println("D");


        int page = 0;
        bool pageswitch = true;

        while (_sel_menu == DIGITAL)
        {

            while (true)
            {

                int touchX, touchY;
                if (LCD.Touch(&touchX, &touchY))
                {
                    // Check if back button is pressed
                    if (_digital_back_icon_arr[0].Pressed(touchX, touchY, 0))
                    {
                        _digital_back_icon_arr[0].WhilePressed(touchX, touchY);
                        _sel_menu = MAIN;
                        break;
                    }
                    // Scroll through pages
                    else
                    {
                        LCD.WaitForTouchToEnd();
                        page++;
                        page %= 2;
                        pageswitch = true;
                    }
                }

                if (pageswitch)
                {
                    // Change the labels of the icons
                    for (size_t i = 0; i < 4; i++)
                    {
                        snprintf(label, 64, "Pin %d", i + 8 * page);
                        _iconGroup0[i].ChangeLabelString(label);
                        snprintf(label, 64, "Pin %d", i + 4 + 8 * page);
                        _iconGroup1[i].ChangeLabelString(label);
                    }
                    pageswitch = false;
                }

                // Update digital input values
                for (size_t i = 0; i < 4; i++)
                {
                    _iconGroup0[i + 4].ChangeLabelString(digitalPins[i + 8 * page].Value() ? "T" : "F");
                    _iconGroup1[i + 4].ChangeLabelString(digitalPins[i + 4 + 8 * page].Value() ? "T" : "F");
                }
            }
        }
    }

    void AnalogMenu()
    {

        static constexpr int TOP_LABEL_Y = 47;

        LCD.Clear(BLACK);

        // Create back to main menu icon
        FEHIcon::Icon _servo_back_icon_arr[1];
        FEHIcon::DrawIconArray(_servo_back_icon_arr, 1, 1, 1, 201, 1, 1, _backLabel, MENU_C, TEXT_C);
        _servo_back_icon_arr[0].Select();

        char label[64];

        // Label the screen
        uiText tTitle(160, TOP_LABEL_Y, FEHLCD::White, 3, true);
        tTitle.draw("Analog In");
        uiText leftArrow(20, TOP_LABEL_Y, FEHLCD::White, 4, true);
        uiText rightArrow(300, TOP_LABEL_Y, FEHLCD::White, 4, true);
        leftArrow.draw("<");
        rightArrow.draw(">");

        LCD.SetFontSize(2);

        FEHIcon::Icon _iconGroup0[8];
        FEHIcon::Icon _iconGroup1[8];
        char _nullLabels[8][20] = {""};
        FEHIcon::DrawIconArray(_iconGroup0, 2, 4, 80, 90, 10, 10, _nullLabels, MENU_C, TEXT_C);
        FEHIcon::DrawIconArray(_iconGroup1, 2, 4, 160, 10, 10, 10, _nullLabels, MENU_C, TEXT_C);

        AnalogInputPin analogPins[NUM_STUDENT_GPIO - 1] = {
            AnalogInputPin(FEHIO::Pin0),
            AnalogInputPin(FEHIO::Pin1),
            AnalogInputPin(FEHIO::Pin2),
            AnalogInputPin(FEHIO::Pin3),
            AnalogInputPin(FEHIO::Pin4),
            AnalogInputPin(FEHIO::Pin5),
            AnalogInputPin(FEHIO::Pin6),
            AnalogInputPin(FEHIO::Pin7),
            AnalogInputPin(FEHIO::Pin8),
            AnalogInputPin(FEHIO::Pin9),
            AnalogInputPin(FEHIO::Pin10),
            AnalogInputPin(FEHIO::Pin11),
            AnalogInputPin(FEHIO::Pin12),
            AnalogInputPin(FEHIO::Pin13),
            AnalogInputPin(FEHIO::Pin14)};

        DigitalInputPin pin15 = DigitalInputPin(FEHIO::Pin15);

        int page = 0;
        bool pageswitch = true;

        while (_sel_menu == ANALOG)
        {

            while (true)
            {

                int touchX, touchY;
                if (LCD.Touch(&touchX, &touchY))
                {
                    // Check if back button is pressed
                    if (_servo_back_icon_arr[0].Pressed(touchX, touchY, 0))
                    {
                        _servo_back_icon_arr[0].WhilePressed(touchX, touchY);
                        _sel_menu = MAIN;
                        break;
                    }
                    // Scroll through pages
                    else
                    {
                        LCD.WaitForTouchToEnd();
                        page++;
                        page %= 2;
                        pageswitch = true;
                    }
                }

                if (pageswitch)
                {
                    // Change the labels of the icons
                    for (size_t i = 0; i < 4; i++)
                    {
                        snprintf(label, 64, "Pin %d", i + 8 * page);
                        _iconGroup0[i].ChangeLabelString(label);
                        snprintf(label, 64, "Pin %d", i + 4 + 8 * page);
                        _iconGroup1[i].ChangeLabelString(label);
                    }
                    pageswitch = false;
                }

                // Update digital input values
                for (size_t i = 0; i < 4; i++)
                {
                    _iconGroup0[i + 4].ChangeLabelFloat(analogPins[i + 8 * page].Value());
                    if (i == 3 && page == 1)
                    {
                        _iconGroup1[i + 4].ChangeLabelString(pin15.Value() ? "T" : "F");
                    }
                    else
                    {
                        _iconGroup1[i + 4].ChangeLabelFloat(analogPins[i + 4 + 8 * page].Value());
                    }
                }
            }
        }
    }

    void BatteryMenu()
    {
        LCD.Clear(BLACK);

        // Create back to main menu icon
        FEHIcon::Icon _motor_back_icon_arr[1];
        FEHIcon::DrawIconArray(_motor_back_icon_arr, 1, 1, 1, 201, 1, 1, _backLabel, MENU_C, TEXT_C);
        _motor_back_icon_arr[0].Select();

        char label[64];

        uiText tPercent(160, 110, FEHLCD::White, 6, true);

        while (_sel_menu == BATTERY)
        {
            while (true)
            {
                int touchX, touchY;

                snprintf(label, 64, "%0.2fV", (double)BatteryVoltage());
                tPercent.draw(label);

                if (LCD.Touch(&touchX, &touchY))
                {

                    // Check if back button is pressed
                    if (_motor_back_icon_arr[0].Pressed(touchX, touchY, 0))
                    {
                        _motor_back_icon_arr[0].WhilePressed(touchX, touchY);
                        _sel_menu = MAIN;
                        break;
                    }
                }
            }
        }
    }

    void TouchMenu()
    {
        LCD.Clear(BLACK);

        // Create back to main menu icon
        FEHIcon::Icon _motor_back_icon_arr[1];
        FEHIcon::DrawIconArray(_motor_back_icon_arr, 1, 1, 1, 201, 1, 1, _backLabel, MENU_C, TEXT_C);
        _motor_back_icon_arr[0].Select();

        while (_sel_menu == TOUCH)
        {
            while (true)
            {
                int touchX, touchY;

                if (LCD.Touch(&touchX, &touchY))
                {

                    // Check if back button is pressed
                    if (_motor_back_icon_arr[0].Pressed(touchX, touchY, 0))
                    {
                        _motor_back_icon_arr[0].WhilePressed(touchX, touchY);
                        _sel_menu = MAIN;
                        break;
                    }

                    // Draw the pixel that was touched
                    LCD.SetFontColor(COLORPINK);
                    LCD.DrawPixel(touchX, touchY);
                }
            }
        }
    }
};

void TestGUI()
{
    TestingMenu menu;

    while (true)
    {
        menu.NewMenu();
    }
}
