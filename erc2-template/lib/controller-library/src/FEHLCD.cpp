/**
 * FEHLCD.cpp
 *
 * FEH LCD library.
 *
 * A thin wrapper around the Adafruit ILI9341 display library.
 *
 * @author Brian Jia <jia.659@osu.edu>
 */

#include <FEH.h>
#include <Wire.h> // this is needed for FT6206
#include <Adafruit_ILI9341.h>
#include <Adafruit_FT6206.h>

#define LCD_CS 53
#define LCD_DC 42
#define LCD_RST 48

/* LCD Singleton */
Adafruit_ILI9341 ILI9341 = Adafruit_ILI9341(LCD_CS, LCD_DC, LCD_RST);

/* Touchscreen Singleton */
Adafruit_FT6206 FT6206 = Adafruit_FT6206();

/* LCD Singleton */
FEHLCD LCD;

void FEHLCD::SetTextCursor(int x, int y)
{
    ILI9341.setCursor(x, y);
}

void FEHLCD::SetFontColor(uint16_t color)
{
    ILI9341.setTextColor(color);
    _foregroundColor = color;
}

void FEHLCD::SetFontColor(uint16_t color, uint16_t highlight_color)
{
    ILI9341.setTextColor(color, highlight_color);
    _foregroundColor = color;
}

void FEHLCD::SetFontSize(uint8_t size)
{
    ILI9341.setTextSize(size);
}

bool FEHLCD::Touch(int *x_pos, int *y_pos)
{
    TS_Point point = FT6206.getPoint();
    // DO NOT USE FT6206.touched() TO DETERMINE TOUCH STATUS
    // IF THE USER STOPS TOUCHING THE SCREEN BETWEEN YOU CALLING FT6206.touched() AND YOUR FT6206.getPoint() CALL
    // THAT WILL RESULT IN AN INVALID TOUCH READ
    // THE LIBRARY RETURNS Z=0 WHEN NOTHING IS BEING TOUCHED SO USE THAT TO DETERMINE TOUCH STATUS
    bool touched = point.z != 0;

    if (touched)
    {
        /* *x_pos = point.y IS CORRECT */
        /* THE TOUCHSCREEN IS ROTATED 90% FROM THE SCREEN */
        /* Flip x */
        *x_pos = LCD_WIDTH - point.y;
        *y_pos = point.x;
    }

    return touched;
}

void FEHLCD::WaitForTouchToStart()
{
    while (!FT6206.touched())
    {
    }
}

void FEHLCD::WaitForTouchToEnd()
{
    while (FT6206.touched())
    {
    }
}

void FEHLCD::SetOrientation(FEHLCDOrientation orientation)
{
    ILI9341.setRotation(orientation);
}

void FEHLCD::Clear()
{
    this->Clear(BLACK);
}

void FEHLCD::Clear(uint16_t color)
{
    /* Set text cursor to 0,0 to match Proteus LCD.Clear() behavior */
    ILI9341.setCursor(0, 0);
    ILI9341.fillScreen(color);
}

void FEHLCD::Write(const char *str)
{
    ILI9341.print(str);
}

void FEHLCD::Write(int i)
{
    ILI9341.print(i);
}

void FEHLCD::Write(float f)
{
    ILI9341.print(f);
}

void FEHLCD::Write(double d)
{
    ILI9341.print(d);
}

void FEHLCD::Write(bool b)
{
    if (b)
    {
        ILI9341.print("true");
    }
    else
    {
        ILI9341.print("false");
    }
}

void FEHLCD::Write(char c)
{
    ILI9341.print(c);
}

void FEHLCD::WriteLine()
{
    ILI9341.println();
}

void FEHLCD::WriteLine(const char *str)
{
    ILI9341.println(str);
}

void FEHLCD::WriteLine(int i)
{
    ILI9341.println(i);
}

void FEHLCD::WriteLine(float f)
{
    ILI9341.println(f);
}

void FEHLCD::WriteLine(double d)
{
    ILI9341.println(d);
}

void FEHLCD::WriteLine(bool b)
{
    if (b)
    {
        ILI9341.println("true");
    }
    else
    {
        ILI9341.println("false");
    }
}

void FEHLCD::WriteLine(char c)
{
    ILI9341.println(c);
}

void FEHLCD::WriteAt(const char *str, int x, int y)
{
    this->SetTextCursor(x, y);
    this->Write(str);
}

void FEHLCD::WriteAt(int i, int x, int y)
{
    this->SetTextCursor(x, y);
    this->Write(i);
}

void FEHLCD::WriteAt(float f, int x, int y)
{
    this->SetTextCursor(x, y);
    this->Write(f);
}

void FEHLCD::WriteAt(double d, int x, int y)
{
    this->SetTextCursor(x, y);
    this->Write(d);
}

void FEHLCD::WriteAt(bool b, int x, int y)
{
    this->SetTextCursor(x, y);
    this->Write(b);
}

void FEHLCD::WriteAt(char c, int x, int y)
{
    this->SetTextCursor(x, y);
    this->Write(c);
}

void FEHLCD::setTextCursorRC(int row, int col)
{
    int16_t x1, y1;
    uint16_t w, h;
    ILI9341.getTextBounds(" ", 0, 0, &x1, &y1, &w, &h);
    int x = col * w;
    int y = row * h;
    this->SetTextCursor(x, y);
}

void FEHLCD::WriteRC(const char *str, int row, int col)
{
    this->setTextCursorRC(row, col);
    this->Write(str);
}

void FEHLCD::WriteRC(int i, int row, int col)
{
    this->setTextCursorRC(row, col);
    this->Write(i);
}

void FEHLCD::WriteRC(float f, int row, int col)
{
    this->setTextCursorRC(row, col);
    this->Write(f);
}

void FEHLCD::WriteRC(double d, int row, int col)
{
    this->setTextCursorRC(row, col);
    this->Write(d);
}

void FEHLCD::WriteRC(bool b, int row, int col)
{
    this->setTextCursorRC(row, col);
    this->Write(b);
}

void FEHLCD::WriteRC(char c, int row, int col)
{
    this->setTextCursorRC(row, col);
    this->Write(c);
}

void FEHLCD::DrawPixel(int x, int y)
{
    ILI9341.drawPixel(x, y, _foregroundColor);
}

void FEHLCD::DrawHorizontalLine(int y, int x1, int x2)
{
    ILI9341.drawFastHLine(x1, y, x2 - x1 + 1, _foregroundColor);
}

void FEHLCD::DrawVerticalLine(int x, int y1, int y2)
{
    ILI9341.drawFastVLine(x, y1, y2 - y1 + 1, _foregroundColor);
}

void FEHLCD::DrawLine(int x0, int y0, int x1, int y1)
{
    ILI9341.drawLine(x0, y0, x1, y1, _foregroundColor);
}

void FEHLCD::DrawRectangle(int x, int y, int w, int h)
{
    ILI9341.drawRect(x, y, w, h, _foregroundColor);
}

void FEHLCD::FillRectangle(int x, int y, int w, int h)
{
    ILI9341.fillRect(x, y, w, h, _foregroundColor);
}

void FEHLCD::DrawCircle(int x0, int y0, int r)
{
    ILI9341.drawCircle(x0, y0, r, _foregroundColor);
}

void FEHLCD::FillCircle(int x0, int y0, int r)
{
    ILI9341.fillCircle(x0, y0, r, _foregroundColor);
}

/*
 * FEHIcon API.
 */

/* Icon constructor function */
FEHIcon::Icon::Icon() {}

/* Icon function to set position, size, label, and color */
void FEHIcon::Icon::SetProperties(char name[20], int start_x, int start_y, int w, int h, unsigned int c, unsigned int tc)
{
    strcpy(label, name);
    x_start = start_x;
    y_start = start_y;
    width = w;
    height = h;
    x_end = x_start + width;
    y_end = y_start + height;
    color = c;
    textcolor = tc;
    set = 0;
}

/* Icon function to draw it and write label */
void FEHIcon::Icon::Draw()
{
    LCD.SetFontColor(color);
    LCD.DrawRectangle(x_start, y_start, width, height);
    LCD.SetFontColor(textcolor);
    LCD.WriteAt(label, x_start + ((width - (strlen(label) * 12)) / 2), y_start + ((height - 17) / 2)); // equation to center text inside the icon
}

/* Icon function to make the icon selected and set */
void FEHIcon::Icon::Select()
{
    LCD.SetFontColor(color);
    LCD.DrawRectangle(x_start + 1, y_start + 1, width - 2, height - 2);
    LCD.DrawRectangle(x_start + 2, y_start + 2, width - 4, height - 4);
    LCD.DrawRectangle(x_start + 3, y_start + 3, width - 6, height - 6);
    set = 1;
}

/* Icon function to make the icon deselected and not set */
void FEHIcon::Icon::Deselect()
{
    LCD.SetFontColor(BLACK);
    LCD.DrawRectangle(x_start + 3, y_start + 3, width - 6, height - 6);
    LCD.DrawRectangle(x_start + 2, y_start + 2, width - 4, height - 4);
    LCD.DrawRectangle(x_start + 1, y_start + 1, width - 2, height - 2);
    set = 0;
}

/* Icon function to see if it has been pressed */
int FEHIcon::Icon::Pressed(int x, int y, int mode)
{
    if (x >= x_start && x <= x_end && y >= y_start && y <= y_end)
    {
        if (!LCD.Touch(&x, &y))
            return 0;

        if (x >= x_start && x <= x_end && y >= y_start && y <= y_end) // check twice to avoid buggy touch screen issues
        {
            if (!mode) // if mode is 0, then alternate selecting and deselecting as it is pressed again and again; otherwise, the icon does not select and deselect
            {
                if (!set)
                {
                    Select();
                }
                else if (set)
                {
                    Deselect();
                }
            }
            return 1;
        }
        return 0;
    }
    else
    {
        return 0;
    }
}

/* Icon function to wait while it is pressed */
void FEHIcon::Icon::WhilePressed(int xi, int yi)
{
    int x = xi, y = yi;
    while (Pressed(x, y, 1))
    {
        LCD.Touch(&x, &y);
    }
}

/* Icon function to change the label of an icon with a string */
void FEHIcon::Icon::ChangeLabelString(const char new_label[])
{
    if (strcmp(label, new_label))
    {
        strcpy(label, new_label);
        LCD.SetFontColor(BLACK);
        LCD.FillRectangle(x_start + 1, y_start + 1, width - 2, height - 2);
        Draw();
    }
}

/* Icon function to change the label of an icon with a float */
void FEHIcon::Icon::ChangeLabelFloat(float val)
{
    size_t length_i = strlen(label);
    // Copy the label to a temporary variable
    char temp_label[32];
    strcpy(temp_label, label);
    int d, r;
    /* Convert float to string so it can be auto-centered in icon */
    if (val >= 0)
    {
        d = (int)val;
        r = (int)((val - d) * 1000);
        sprintf(label, "%d.%03d", d, r);
    }
    else
    {
        val *= -1;
        d = (int)val;
        r = (int)((val - d) * 1000);
        sprintf(label, "-%d.%03d", d, r);
    }

    LCD.SetFontColor(BLACK);
    /* If the new label is not the same length as the old one, then erase the old one so that it does not show up behind the new one */
    if (strlen(label) != length_i)
    {
        LCD.FillRectangle(x_start + 1, y_start + 1, width - 2, height - 2);
    }
    else
    {
        // TODO: Move this functionality into Draw()
        //  If the same length, erase any characters that are different
        for (uint8_t i = 0; i < length_i; i++)
        {
            // If character is different, erase just this digit
            if (temp_label[i] != label[i])
            {
                LCD.FillRectangle(x_start + ((width - (strlen(label) * 12)) / 2) + i * 12, y_start + ((height - 17) / 2), 12, height - 2);
            }
        }
    }

    Draw();
}

/* Icon function to change the label of an icon with a int */
void FEHIcon::Icon::ChangeLabelInt(int val)
{
    size_t length_i = strlen(label);

    // Copy the label to a temporary variable
    char temp_label[32];
    strcpy(temp_label, label);

    /* Convert int to string so it can be auto-centered in icon */
    sprintf(label, "%d", val);

    LCD.SetFontColor(BLACK);
    /* If the new label is not the same length as the old one, then erase the old one so that it does not show up behind the new one */
    if (strlen(label) != length_i)
    {
        LCD.FillRectangle(x_start + 1, y_start + 1, width - 2, height - 2);
    }
    else
    {
        // TODO: Move this functionality into Draw()
        //  If the same length, erase any characters that are different
        for (uint8_t i = 0; i < length_i; i++)
        {
            // If character is different, erase just this digit
            if (temp_label[i] != label[i])
            {
                LCD.FillRectangle(x_start + ((width - (strlen(label) * 12)) / 2) + i * 12, y_start + ((height - 17) / 2), 12, height - 2);
            }
        }
    }

    Draw();
}

/* Function to draw an array of icons in a given space and size and label them */
void FEHIcon::DrawIconArray(Icon icon[], int rows, int cols, int top, int bot, int left, int right, char labels[][20], unsigned int col, unsigned int txtcol)
{
    int xs = left;
    int ys = top;
    float total_w = (320. - left - right);
    float total_h = (240. - top - bot);
    int w = total_w / cols;
    int h = total_h / rows;
    int nx, ny, N = 0;
    for (ny = 1; ny <= rows; ny++)
    {
        for (nx = 1; nx <= cols; nx++)
        {
            icon[N].SetProperties(labels[N], xs, ys, w, h, col, txtcol);
            icon[N].Draw();
            N = N + 1;
            xs = xs + w;
        }
        ys = ys + h;
        xs = left;
    }
}