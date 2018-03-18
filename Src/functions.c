// LoadCell_STM32_RAM v2 functions
// (C) Tapio Valli 2018-02-17

#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <ctype.h>

#include "main.h"
#include "dwt_stm32_delay.h"


void initArray(Array *a, size_t initialSize) {
  a->array = (uint16_t *)malloc(initialSize * sizeof(uint16_t));
  a->used = 0;
  a->size = initialSize;
}

void insertArray(Array *a, uint32_t newsize) {
    a->size = newsize;
    a->array = (uint16_t *)realloc(a->array, a->size * sizeof(uint16_t));
    a->used = 0;
}

void freeArray(Array *a) {
  free(a->array);
  a->array = NULL;
  a->used = a->size = 0;
}

size_t string_parser(char *input, char ***word_array)
{
    size_t n = 0;
    const char *p = input;

    while ( *p )
    {
        while ( isspace( ( unsigned char )*p ) ) ++p;
        n += *p != '\0';
        while ( *p && !isspace( ( unsigned char )*p ) ) ++p;
    }

    if ( n )
    {
        size_t i = 0;

        *word_array = (char**)malloc( n * sizeof( char * ) );

        p = input;

        while ( *p )
        {
            while ( isspace( ( unsigned char )*p ) ) ++p;
            if ( *p )
            {
                const char *q = p;
                while ( *p && !isspace( ( unsigned char )*p ) ) ++p;

                size_t length = p - q;

                ( *word_array )[i] = ( char * )malloc( length + 1 );

                strncpy( ( *word_array )[i], q, length );
                ( *word_array )[i][length] = '\0';

                ++i;
            }
        }
    }

    return n;
}

void PlotData(uint32_t XCoordinate,uint32_t YCoordinate)
{
    // Plot at x,y
    BSP_LCD_DrawPixel((uint16_t) XCoordinate,(uint16_t) YCoordinate,1);
}

void InitScreen(uint32_t BackGroundColor,uint32_t ForeGroundColor)
{

// #define LCD_COLOR_BLUE          ((uint32_t)0xFF0000FF)
// #define LCD_COLOR_GREEN         ((uint32_t)0xFF00FF00)
// #define LCD_COLOR_RED           ((uint32_t)0xFFFF0000)
// #define LCD_COLOR_CYAN          ((uint32_t)0xFF00FFFF)
// #define LCD_COLOR_MAGENTA       ((uint32_t)0xFFFF00FF)
// #define LCD_COLOR_YELLOW        ((uint32_t)0xFFFFFF00)
// #define LCD_COLOR_LIGHTBLUE     ((uint32_t)0xFF8080FF)
// #define LCD_COLOR_LIGHTGREEN    ((uint32_t)0xFF80FF80)
// #define LCD_COLOR_LIGHTRED      ((uint32_t)0xFFFF8080)
// #define LCD_COLOR_LIGHTCYAN     ((uint32_t)0xFF80FFFF)
// #define LCD_COLOR_LIGHTMAGENTA  ((uint32_t)0xFFFF80FF)
// #define LCD_COLOR_LIGHTYELLOW   ((uint32_t)0xFFFFFF80)
// #define LCD_COLOR_DARKBLUE      ((uint32_t)0xFF000080)
// #define LCD_COLOR_DARKGREEN     ((uint32_t)0xFF008000)
// #define LCD_COLOR_DARKRED       ((uint32_t)0xFF800000)
// #define LCD_COLOR_DARKCYAN      ((uint32_t)0xFF008080)
// #define LCD_COLOR_DARKMAGENTA   ((uint32_t)0xFF800080)
// #define LCD_COLOR_DARKYELLOW    ((uint32_t)0xFF808000)
// #define LCD_COLOR_WHITE         ((uint32_t)0xFFFFFFFF)
// #define LCD_COLOR_LIGHTGRAY     ((uint32_t)0xFFD3D3D3)
// #define LCD_COLOR_GRAY          ((uint32_t)0xFF808080)
// #define LCD_COLOR_DARKGRAY      ((uint32_t)0xFF404040)
// #define LCD_COLOR_BLACK         ((uint32_t)0xFF000000)
// #define LCD_COLOR_BROWN         ((uint32_t)0xFFA52A2A)
// #define LCD_COLOR_ORANGE        ((uint32_t)0xFFFFA500)
// #define LCD_COLOR_TRANSPARENT   ((uint32_t)0xFF000000)

	BSP_LCD_Clear(BackGroundColor);
	BSP_LCD_SetBackColor(BackGroundColor);
	BSP_LCD_SetTextColor(ForeGroundColor);
	BSP_LCD_SetFont(&Font20);
}

void LCDWrite(uint32_t Line,char Str[])
{
    char IntStr[50];

    // InitScreen(LCD_COLOR_BLACK,LCD_COLOR_WHITE,Font20);

    BSP_LCD_ClearStringLine(Line);
    snprintf(IntStr,50,Str);
    BSP_LCD_DisplayStringAtLine((uint16_t)Line,(uint8_t *) IntStr);
}

void CountDown(uint32_t millisecs)
{
    InitScreen(LCD_COLOR_BLACK,LCD_COLOR_WHITE);

    LCDWrite(5," ");
    DWT_Delay_us(1000);

    LCDWrite(5,"Starting in 3... ");
    DWT_Delay_us(millisecs*1000);

    LCDWrite(5,"Starting in 2... ");
    DWT_Delay_us(millisecs*1000);

    LCDWrite(5,"Starting in 1... ");
    DWT_Delay_us(millisecs*1000);

    InitScreen(LCD_COLOR_BLACK,LCD_COLOR_WHITE);
    LCDWrite(5,"GO!");
}

void SamplePoints(Array *Data,uint32_t NoOfPoints,uint32_t Period_us)
{
    uint32_t i;

    HAL_ADC_Start(&hadc3);

    // Measure NoOfPoints values (f.ex. 19200)
    for(i=0;i<NoOfPoints;i++) {
        Data->array[i]=(uint16_t)HAL_ADC_GetValue(&hadc3);
    	DWT_Delay_us(Period_us);
    }

    InitScreen(LCD_COLOR_BLACK,LCD_COLOR_WHITE);
    LCDWrite(5,"DONE!");

    printf("Sampling done.\r\n");
}

void AvgAndPlotPoints(Array *Data,uint32_t NoOfPoints, uint32_t AvgSize) {

    uint32_t i1,i2;

    uint32_t BufferSum,BufferAvg;
    uint32_t XCoord,YCoord;
    char MyStr[50];

    printf("Start reading... \r\n");
    InitScreen(LCD_COLOR_BLACK,LCD_COLOR_RED);

    for(i1=0;i1<NoOfPoints;i1++) {
        BufferSum=0;

        // Read AvgSize samples
        for(i2=i1;i2<i1+AvgSize;i2++) {
            BufferSum=BufferSum+(uint32_t)Data->array[i2];
        }

        BufferAvg=BufferSum/AvgSize;

        // Calculate two coords and plot
        XCoord=((i1*480.0)/NoOfPoints);
        YCoord=(272.0*(BufferAvg/65536.0));

        PlotData(XCoord,YCoord);
    }

    printf("Done all, Points = %lu Avg = %lu \r\n", i1,AvgSize);

    LCDWrite(0,"");
    snprintf(MyStr,50,"Pnts = %lu Avg = %lu",NoOfPoints,AvgSize);
    LCDWrite(0,MyStr);
}
