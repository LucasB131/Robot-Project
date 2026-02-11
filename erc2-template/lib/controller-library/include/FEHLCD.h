/**
 * FEHLCD.h
 */

#ifndef FEHLCD_H
#define FEHLCD_H

#include <stdint.h>

/*
 * LCD Colors
 *
 * A world of color for the LCD!
 *
 * Copied from Proteus LCDColors.h, then the colors converted to 16-bit from 24-bit using a Python script.
 */

#define BLACK 0x0000
#define NAVY 0x0010
#define DARKBLUE 0x0011
#define MEDIUMBLUE 0x0019
#define BLUE 0x001f
#define DARKGREEN 0x0320
#define GREEN 0x0400
#define TEAL 0x0410
#define DARKCYAN 0x0451
#define DEEPSKYBLUE 0x05ff
#define DARKTURQUOISE 0x0679
#define MEDIUMSPRINGGREEN 0x07d3
#define LIME 0x07e0
#define SPRINGGREEN 0x07ef
#define AQUA 0x07ff
#define CYAN 0x07ff
#define MIDNIGHTBLUE 0x18ce
#define DODGERBLUE 0x249f
#define LIGHTSEAGREEN 0x2595
#define FORESTGREEN 0x2444
#define SEAGREEN 0x344b
#define DARKSLATEGRAY 0x328a
#define LIMEGREEN 0x3666
#define MEDIUMSEAGREEN 0x3d8e
#define TURQUOISE 0x46f9
#define ROYALBLUE 0x435b
#define STEELBLUE 0x4c16
#define DARKSLATEBLUE 0x49f1
#define MEDIUMTURQUOISE 0x4e99
#define INDIGO 0x4810
#define DARKOLIVEGREEN 0x5346
#define CADETBLUE 0x64f3
#define CORNFLOWERBLUE 0x64bd
#define GRAY 0x632c
#define MEDIUMAQUAMARINE 0x6675
#define DIMGRAY 0x6b4d
#define SLATEBLUE 0x6ad9
#define OLIVEDRAB 0x6c64
#define SLATEGRAY 0x7412
#define LIGHTSLATEGRAY 0x7453
#define MEDIUMSLATEBLUE 0x7b5d
#define LAWNGREEN 0x7fc0
#define CHARTREUSE 0x7fe0
#define AQUAMARINE 0x7ffa
#define MAROON 0x8000
#define PURPLE 0x8010
#define OLIVE 0x8400
#define SKYBLUE 0x867d
#define LIGHTSKYBLUE 0x867e
#define BLUEVIOLET 0x897b
#define DARKRED 0x8800
#define DARKMAGENTA 0x8811
#define SADDLEBROWN 0x8a22
#define DARKSEAGREEN 0x8dd1
#define LIGHTGREEN 0x9772
#define MEDIUMPURPLE 0x939b
#define DARKVIOLET 0x901a
#define PALEGREEN 0x97d2
#define DARKORCHID 0x9999
#define YELLOWGREEN 0x9e66
#define SIENNA 0x9a85
#define BROWN 0xa145
#define DARKGRAY 0xad55
#define LIGHTBLUE 0xaebc
#define GREENYELLOW 0xafe6
#define PALETURQUOISE 0xaf7d
#define LIGHTSTEELBLUE 0xae1b
#define POWDERBLUE 0xaefc
#define FIREBRICK 0xb104
#define DARKGOLDENROD 0xb421
#define MEDIUMORCHID 0xbaba
#define SCARLET 0xb800
#define ROSYBROWN 0xbc71
#define DARKKHAKI 0xbdad
#define SILVER 0xbdf7
#define MEDIUMVIOLETRED 0xc0b0
#define INDIANRED 0xcaeb
#define PERU 0xcc28
#define CHOCOLATE 0xd344
#define TAN 0xd591
#define LIGHTGRAY 0xd69a
#define THISTLE 0xd5fa
#define ORCHID 0xdb9a
#define GOLDENROD 0xdd24
#define PALEVIOLETRED 0xdb92
#define CRIMSON 0xd8a7
#define GAINSBORO 0xdedb
#define PLUM 0xdd1b
#define BURLYWOOD 0xddb0
#define LIGHTCYAN 0xdfff
#define LAVENDER 0xe73e
#define DARKSALMON 0xe4af
#define VIOLET 0xec1d
#define PALEGOLDENROD 0xef35
#define LIGHTCORAL 0xec10
#define KHAKI 0xef31
#define ALICEBLUE 0xefbf
#define HONEYDEW 0xeffd
#define AZURE 0xefff
#define SANDYBROWN 0xf52c
#define WHEAT 0xf6f6
#define BEIGE 0xf7bb
#define WHITESMOKE 0xf7be
#define MINTCREAM 0xf7fe
#define GHOSTWHITE 0xf7bf
#define SALMON 0xf40e
#define ANTIQUEWHITE 0xf75a
#define LINEN 0xf77c
#define LIGHTGOLDENRODYELLOW 0xf7da
#define OLDLACE 0xffbc
#define RED 0xf800
#define FUCHSIA 0xf810
#define MAGENTA 0xf81f
#define DEEPPINK 0xf8b2
#define ORANGERED 0xfa20
#define TOMATO 0xfb09
#define HOTPINK 0xfb56
#define CORAL 0xfbea
#define DARKORANGE 0xfc60
#define LIGHTSALMON 0xfd0f
#define ORANGE 0xfd20
#define LIGHTPINK 0xfdb7
#define COLORPINK 0xfdf9
#define GOLD 0xfea0
#define PEACHPUFF 0xfed6
#define NAVAJOWHITE 0xfef5
#define MOCCASIN 0xff16
#define BISQUE 0xff18
#define MISTYROSE 0xff1b
#define BLANCHEDALMOND 0xff59
#define PAPAYAWHIP 0xff7a
#define LAVENDERBLUSH 0xff7e
#define SEASHELL 0xffbd
#define CORNSILK 0xffbb
#define LEMONCHIFFON 0xffd9
#define FLORALWHITE 0xffdd
#define SNOW 0xffde
#define YELLOW 0xffe0
#define LIGHTYELLOW 0xfffb
#define IVORY 0xfffd
#define WHITE 0xffff

/**
 * @brief Access to the Proteus LCD
 *
 * Allows user to edit and interact with Proteus LCD screen
 */
class FEHLCD
{
public:
    // Create color states
    typedef enum
    {
        Black = 0x0000,
        White = 0xFFFF,
        Red = 0xF800,
        Green = 0x07E0,
        Blue = 0x001F,
        Scarlet = 0xF8C0,
        Gray = 0xAAAA
    } FEHLCDColor;

    // Create directional states
    typedef enum
    {
        North = 0,
        South,
        East,
        West
    } FEHLCDOrientation;

    /**
     * @brief Processes user touch on the LCD screen
     *
     * @param x_pos
     *      X-coordinate that the user touches
     * @param y_pos
     *      Y-coordinate that the user touches
     *
     * @return the boolean value of whether or not the user has
     * touched the board.
     */
    bool Touch(int *x_pos, int *y_pos);

    /**
     * @brief Waits for user touch to start on the LCD screen.
     */
    void WaitForTouchToStart();

    /**
     * @brief Waits for user touch to end on the LCD screen.
     */
    void WaitForTouchToEnd();

    /**
     * @brief Sets orientation of LCD screen
     *
     * @param Orientation
     *      Current orientation of LCD screen
     */
    void SetOrientation(FEHLCDOrientation orientation);

    /**
     * @brief Clears LCD screen to a given color
     *
     * @param color
     *      Color that the LCD background is cleared to, in RGB565 format
     */
    void Clear(uint16_t color);

    /**
     * @brief Clears LCD screen to black
     */
    void Clear();

    /**
     * @brief Sets the coordinate on screen to write text to
     */
    void SetTextCursor(int x, int y);

    /**
     * @brief Sets LCD font color
     *
     * @param color
     *      Color of font to be set
     *
     */
    void SetFontColor(uint16_t color);

    /**
     * @brief Sets LCD font color and highlight color
     *
     * @param color
     *      Color of font to be set
     *
     * @param highlight_color
     *      Highlight color of font to be set
     *
     */
    void SetFontColor(uint16_t color, uint16_t highlight_color);

    /**
     * @brief Sets LCD background color
     *
     * @param color
     *      Color of the background to be set, in RGB565 format
     */
    void SetBackgroundColor(unsigned int color);

    /**
     * @brief   Set text 'magnification' size. Each increase in size makes 1 pixel that much bigger.
     * @param  size  Desired text size. 1 is default 6x8, 2 is 12x16, 3 is 18x24, etc
     *
     */
    void SetFontSize(uint8_t size);

    /**
     * @brief Draws shapes to LCD screen
     *
     * Note: This comment applies to all public member variations of
     * {@code} DrawXXX() and FillXXX()
     *
     * Draws or fills pixels on the LCD screen based on given
     * coordinates and dimensions.
     */
    void DrawPixel(int x, int y);
    void DrawHorizontalLine(int y, int x1, int x2);
    void DrawVerticalLine(int x, int y1, int y2);
    void DrawLine(int x1, int y1, int x2, int y2);
    void DrawRectangle(int x, int y, int width, int height);
    void FillRectangle(int x, int y, int width, int height);
    void DrawCircle(int x0, int y0, int r);
    void FillCircle(int x0, int y0, int r);

    /**
     * @brief Writes message to LCD screen
     *
     * Note: This comment applies to all public member variations of
     * {@code} Write()
     *
     * @param 1
     *      message to be printed to the screen
     */
    void Write(const char *str);
    void Write(int i);
    void Write(float f);
    void Write(double d);
    void Write(bool b);
    void Write(char c);

    /**
     * @brief Writes message to LCD and returns to a new line
     *
     * Note: This comment applies to all public member variations of
     * {@code} WriteLine()
     *
     * @param 1
     *      message to be printed to the screen before a new line
     */
    void WriteLine();
    void WriteLine(const char *str);
    void WriteLine(int i);
    void WriteLine(float f);
    void WriteLine(double d);
    void WriteLine(bool b);
    void WriteLine(char c);

    /**
     * @brief Writes message to LCD at specific coordinates
     *
     * Note: This comment applies to all public member variations of
     * {@code} WriteAt()
     *
     * @param 1
     *      message to be printed to the screen
     *
     * @param x
     *      X-coordinate where a message will be printed
     *
     * @param y
     *      Y-coordinate where a message will be printed
     */
    void WriteAt(const char *str, int x, int y);
    void WriteAt(int i, int x, int y);
    void WriteAt(float f, int x, int y);
    void WriteAt(double d, int x, int y);
    void WriteAt(bool b, int x, int y);
    void WriteAt(char c, int x, int y);

    // Write to Row, Column

    /**
     * @brief Writes message to LCD at a specific row and column
     *
     * Note: This comment applies to all public member variations of
     * {@code} WriteRC()
     *
     * @param 1
     *      message to be printed to the screen
     *
     * @param row
     *      Row where a message will be printed
     *
     * @param column
     *      Column where a message will be printed
     */
    void WriteRC(const char *str, int row, int col);
    void WriteRC(int i, int row, int col);
    void WriteRC(float f, int row, int col);
    void WriteRC(double d, int row, int col);
    void WriteRC(bool b, int row, int col);
    void WriteRC(char c, int row, int col);

private:
    uint16_t _foregroundColor = 0;

    void setTextCursorRC(int row, int col);
};

/* LCD Singleton */
extern FEHLCD LCD;

namespace FEHIcon
{
    /* Class definition for software icons */
    class Icon
    {
    private:
        int x_start, x_end;
        int y_start, y_end;
        int width;
        int height;
        unsigned int color;
        unsigned int textcolor;
        char label[20];
        int set;

    public:
        Icon();
        void SetProperties(char name[20], int start_x, int start_y, int w, int h, unsigned int c, unsigned int tc);
        void Draw();
        void Select();
        void Deselect();
        int Pressed(int x, int y, int mode);
        void WhilePressed(int xi, int yi);
        void ChangeLabelString(const char new_label[20]);
        void ChangeLabelFloat(float val);
        void ChangeLabelInt(int val);
    };

    /* Function prototype for drawing an array of icons in a rows by cols array with top, bot, left, and right margins from edges of screen, labels for each icon from top left across each row to the bottom right, and color for the rectangle and the text color */
    void DrawIconArray(Icon icon[], int rows, int cols, int top, int bot, int left, int right, char labels[][20], unsigned int col, unsigned int txtcol);
}

#endif // FEHLCD_H