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
	a->array = (uint16_t *) malloc(initialSize * sizeof(uint16_t));
	a->used = 0;
	a->size = initialSize;
}

void insertArray(Array *a, uint32_t newsize) {
	a->size = newsize;
	a->array = (uint16_t *) realloc(a->array, a->size * sizeof(uint16_t));
	a->used = 0;
}

void freeArray(Array *a) {
	free(a->array);
	a->array = NULL;
	a->used = a->size = 0;
}

size_t string_parser(char *input, char ***word_array) {
	size_t n = 0;
	const char *p = input;

	while (*p) {
		while (isspace((unsigned char )*p))
			++p;
		n += *p != '\0';
		while (*p && !isspace((unsigned char )*p))
			++p;
	}

	if (n) {
		size_t i = 0;

		*word_array = (char**) malloc(n * sizeof(char *));

		p = input;

		while (*p) {
			while (isspace((unsigned char )*p))
				++p;
			if (*p) {
				const char *q = p;
				while (*p && !isspace((unsigned char )*p))
					++p;

				size_t length = p - q;

				(*word_array)[i] = (char *) malloc(length + 1);

				strncpy((*word_array)[i], q, length);
				(*word_array)[i][length] = '\0';

				++i;
			}
		}
	}

	return n;
}

void PlotData(uint32_t XCoordinate, uint32_t YCoordinate) {
	// Plot at x,y
	BSP_LCD_DrawPixel((uint16_t) XCoordinate, (uint16_t) YCoordinate, 1);
}

void InitScreen(uint32_t BackGroundColor, uint32_t ForeGroundColor) {

	BSP_LCD_Clear(BackGroundColor);
	BSP_LCD_SetBackColor(BackGroundColor);
	BSP_LCD_SetTextColor(ForeGroundColor);
	BSP_LCD_SetFont(&Font20);
}

void LCDWrite(uint32_t Line, char Str[]) {
	char IntStr[50];

	BSP_LCD_ClearStringLine(Line);
	snprintf(IntStr, 50, Str);
	BSP_LCD_DisplayStringAtLine((uint16_t) Line, (uint8_t *) IntStr);
}

void CountDown(uint32_t millisecs) {
	InitScreen(LCD_COLOR_BLACK, LCD_COLOR_WHITE);

	LCDWrite(5, " ");
	DWT_Delay_us(1000);

	LCDWrite(5, "Starting in 3... ");
	DWT_Delay_us(millisecs * 1000);

	LCDWrite(5, "Starting in 2... ");
	DWT_Delay_us(millisecs * 1000);

	LCDWrite(5, "Starting in 1... ");
	DWT_Delay_us(millisecs * 1000);

	InitScreen(LCD_COLOR_BLACK, LCD_COLOR_RED);
	LCDWrite(5, "GO!");
}

FRESULT scan_files(char* path
// Start node to be scanned, also used as work area
		) {
	FRESULT res;
	DIR dir;
	UINT i;
	static FILINFO fno;
	char buffer[1000] = " ";

	res = f_opendir(&dir, path); /* Open the directory */
	if (res == FR_OK) {
		for (;;) {
			res = f_readdir(&dir, &fno); /* Read a directory item */
			if (res != FR_OK || fno.fname[0] == 0)
				break; /* Break on error or end of dir */
			if (fno.fattrib & AM_DIR) { /* It is a directory */
				i = strlen(path);
				sprintf(&path[i], "/%s", fno.fname);
				res = scan_files(path); /* Enter the directory */
				if (res != FR_OK)
					break;
				path[i] = 0;
			} else { /* It is a file. */
				sprintf(buffer, "%s/%s 		%lu \r\n", path, fno.fname, fno.fsize);
				HAL_UART_Transmit(&huart1, (uint8_t *) buffer, 1000, 0xFFFF);
			}
		}
		f_closedir(&dir);
	}

	return res;
}

void SamplePoints(Array *Data, uint32_t NoOfPoints, uint32_t Period_us) {
	uint32_t i;
	char buffer[1000] = " ";

	HAL_ADC_Start(&hadc3);

	// Measure NoOfPoints values (f.ex. 19200)
	for (i = 0; i < NoOfPoints; i++) {
		Data->array[i] = (uint16_t) HAL_ADC_GetValue(&hadc3);
		DWT_Delay_us(Period_us);
	}

	InitScreen(LCD_COLOR_BLACK, LCD_COLOR_WHITE);
	LCDWrite(5, "DONE!");

	sprintf(buffer, "Sampling done.\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *) buffer, 1000, 0xFFFF);
}

void AvgAndPlotPoints(Array *Data, uint32_t NoOfPoints, uint32_t AvgSize) {

	uint32_t i1, i2;
	char buffer[1000] = " ";

	uint32_t BufferSum, BufferAvg;
	uint32_t XCoord, YCoord;
	char MyStr[50];

	sprintf(buffer, "Start averaging... \r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *) buffer, 1000, 0xFFFF);

	InitScreen(LCD_COLOR_BLACK, LCD_COLOR_RED);

	for (i1 = 0; i1 < NoOfPoints; i1++) {
		BufferSum = 0;

		// Read AvgSize samples
		for (i2 = i1; i2 < i1 + AvgSize; i2++) {
			BufferSum = BufferSum + (uint32_t) Data->array[i2];
		}

		BufferAvg = BufferSum / AvgSize;

		// Calculate two coords and plot
		XCoord = ((i1 * 480.0) / NoOfPoints);
		YCoord = (272.0 * (BufferAvg / 65536.0));

		PlotData(XCoord, YCoord);
	}

	sprintf(buffer, "Averaging done, Points = %lu Avg = %lu \r\n", i1, AvgSize);
	HAL_UART_Transmit(&huart1, (uint8_t *) buffer, 1000, 0xFFFF);

	LCDWrite(0, "");
	snprintf(MyStr, 50, "Pnts = %lu Avg = %lu", NoOfPoints, AvgSize);
	LCDWrite(0, MyStr);
}

void WriteData2FS(Array *Data, uint32_t NoOfPoints, uint32_t MeasNo) {

	// Create file for data, as meas#.txt

	FRESULT res; // FatFs function common result code
	uint32_t byteswritten, totalbytes; //File write counts
	char buffer[1000] = " ";

	char* fname = sprintf("1:\\meas%lu.txt", MeasNo);
	FIL MyFile;
	uint32_t idx;

	totalbytes = 0;

	if (f_open(&MyFile, fname, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {

		// File Open for write Error
		_Error_Handler(__FILE__, __LINE__);
	} else {
		sprintf(buffer, "Opened file %s OK \r\n", fname);
			HAL_UART_Transmit(&huart1, (uint8_t *) buffer, 1000, 0xFFFF);

		// Write data to the text file line by line
		for (idx = 0; idx < NoOfPoints; idx++) {
			sprintf(buffer, "%lu \r\n", ((uint32_t) Data->array[idx]));
			res = f_write(&MyFile, buffer, strlen(buffer),
					(void *) &byteswritten);
			totalbytes += byteswritten;
			if ((byteswritten == 0) || (res != FR_OK)) {
				// File Write Error
				_Error_Handler(__FILE__, __LINE__);
			}
		}

		sprintf(buffer, "File %s, %lu bytes written \r\n", fname, totalbytes);
		HAL_UART_Transmit(&huart1, (uint8_t *) buffer, 1000, 0xFFFF);

		/*##-6- Close the open text file #################################*/
		f_close(&MyFile);

		sprintf(buffer, "Closed file %s OK \r\n", fname);
		HAL_UART_Transmit(&huart1, (uint8_t *) buffer, 1000, 0xFFFF);
	}
}

void DirList(void) {

	FATFS fs;
	FRESULT res;
	char buff[256];

	// Should be mounted already
	// res = f_mount(&fs, "", 1);
	// if (res == FR_OK) {
			strcpy(buff, "/");
			res = scan_files(buff);
	//}
	}

