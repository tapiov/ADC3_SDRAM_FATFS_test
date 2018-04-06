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

#include "stm32f7xx_hal.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "usb_host.h"

#include "stm32746g_discovery.h"
#include "stm32746g_discovery_lcd.h"

ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

CRC_HandleTypeDef hcrc;

DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c3;

LTDC_HandleTypeDef hltdc;

UART_HandleTypeDef huart1;

DMA_HandleTypeDef hdma_memtomem_dma2_stream2;
DMA_HandleTypeDef hdma_memtomem_dma2_stream1;
SDRAM_HandleTypeDef hsdram1;

// Mod from https://stackoverflow.com/questions/23191203/convert-float-to-string-without-sprintf
// prints a number with 2 digits following the decimal place
// creates the string backwards, before printing it character-by-character from
// the end to the start
//
// Usage: myPrintf(270.458)
//  Output: 270.45
char * myPrintf(float fVal) {
	static char result[10];
	uint8_t dVal, dec, i;

	fVal += 0.005;   // added after a comment from Matt McNabb, see below.

	dVal = fVal;
	dec = (uint8_t) (fVal * 100) % 100;

	memset(result, 0, 10);
	result[0] = (dec % 10) + '0';
	result[1] = (dec / 10) + '0';
	result[2] = '.';

	i = 3;
	while (dVal > 0) {
		result[i] = (char) (dVal % 10) + '0';
		dVal /= 10;
		i++;
	}

	return result;
}


void initArray(Array *a, size_t initialSize) {
	a->array = (uint16_t *) pvPortMalloc(initialSize * sizeof(uint16_t));
	a->used = 0;
	a->size = initialSize;
}

void insertArray(Array *a, uint32_t newsize) {
	a->size = newsize;
	vPortFree(a->array);
	a->array = (uint16_t *) pvPortMalloc(a->size * sizeof(uint16_t));
	a->used = 0;
}

void freeArray(Array *a) {
	vPortFree(a->array);
	a->array = NULL;
	a->used = a->size = 0;
}

// mod from
// https://stackoverflow.com/questions/13230253/how-do-i-parse-a-string-in-c
char * string_parse(char * parse_string, uint8_t idx) {

	char *work_string;
	uint8_t stringArray[30];
	static uint8_t wordArray[30];
	uint8_t i = 0;
	uint8_t counter = 0;
	uint8_t argCounter = 0;
	uint8_t wordCounter = 0;

	// Let's make a copy for work
	work_string = parse_string;

	// And move it to array
	while (*work_string != '\0') {
		stringArray[i] = (int) *work_string;
		i++;
		work_string++;
	}

	// Terminate
	stringArray[i] = '\0';

	// Main function.
	counter = 0;
	while (stringArray[wordCounter] != '\0') {
		// Puts first word into temporary wordArray.
		while ((stringArray[wordCounter] != ' ')
				&& (stringArray[wordCounter] != '\0')) {
			wordArray[counter++] = stringArray[wordCounter++];
		}
		wordArray[counter] = '\0';

		// Return the content of wordArray at requested index.
		if (argCounter == idx) {
			return wordArray;
		} else if (argCounter > idx) {
			// Too many arguments
			_Error_Handler(__FILE__, __LINE__);
		}

		//Clears temporary wordArray for new use.
		for (counter = 0; counter < 30; counter++) {
			wordArray[counter] = '\0';
		}

		if (stringArray[wordCounter] != '\n') {
			wordCounter++;
		}

		counter = 0;
		argCounter++;
	}

	// If we return here. the parse_string had less arguments as requested
	// In that case, return empty string
	return '\0';
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
	HAL_Delay(1);

	LCDWrite(5, "Starting in 3... ");
	HAL_Delay(millisecs);

	LCDWrite(5, "Starting in 2... ");
	HAL_Delay(millisecs);

	LCDWrite(5, "Starting in 1... ");
	HAL_Delay(millisecs);

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
				printf("%s/%s 		%lu \r\n", path, fno.fname, fno.fsize);
			}
		}
		f_closedir(&dir);
	}

	return res;
}

void SamplePoints(Array *Data, uint32_t NoOfPoints, uint32_t Period_us) {

	TIM_HandleTypeDef htim2;
	uint32_t i;

	HAL_ADC_Start(&hadc3);
	htim2.Instance = TIM2;

	// Measure NoOfPoints values (f.ex. 19200)
	for (i = 0; i < NoOfPoints; i++) {

		// Set the TIM2 to zero
		__HAL_TIM_SET_COUNTER(&htim2, 0);

		// Sample ADC3[0] and store to array[i]
		// Takes apprx 15 ADCClk cycles for 12 bits = 15 * (1/25 MHz) = 0.6 us
		// Lets substract 1 us from Period_us to compensate for loop + conversion
		Data->array[i] = (uint16_t) HAL_ADC_GetValue(&hadc3);

		// Start TIM2
		if ((HAL_TIM_Base_Start(&htim2)) != HAL_OK) {
			_Error_Handler(__FILE__, __LINE__);
		}

		// Wait for Period_us-1 us
		while ((__HAL_TIM_GET_COUNTER(&htim2)) < (Period_us - 1)) {
			;;
		}

		// Stop TIM2
		if ((HAL_TIM_Base_Stop(&htim2)) != HAL_OK) {
			_Error_Handler(__FILE__, __LINE__);

		}
	}

	InitScreen(LCD_COLOR_BLACK, LCD_COLOR_WHITE);
	LCDWrite(5, "DONE!");

	printf("Sampling done.\r\n");
}

void AvgAndPlotPoints(Array *Data, uint32_t NoOfPoints, uint32_t AvgSize) {

	uint32_t i1, i2;

	uint32_t BufferSum, BufferAvg;
	uint32_t XCoord, YCoord;
	char MyStr[50];

	printf("Start averaging... \r\n");

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

	printf("Averaging done, Points = %lu Avg = %lu \r\n", i1, AvgSize);

	LCDWrite(0, "");
	snprintf(MyStr, 50, "Pnts = %lu Avg = %lu", NoOfPoints, AvgSize);
	LCDWrite(0, MyStr);
}

void WriteData2FS(Array *Data, uint32_t NoOfPoints, uint32_t MeasNo) {

	// Create file for data, as meas#.txt

	FRESULT res; // FatFs function common result code
	char buffer[1000] = " ";
	uint32_t byteswritten, totalbytes; //File write counts

	char* fname = (char *) (sprintf("meas_%lu.txt", (char *) MeasNo));
	FIL MyFile;
	uint32_t idx;

	totalbytes = 0;

	if (f_open(&MyFile, fname, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {

		// File Open for write Error
		_Error_Handler(__FILE__, __LINE__);
	} else {
		printf("Opened file %s OK \r\n", fname);

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

		printf("File %s, %lu bytes written \r\n", fname, totalbytes);

		/*##-6- Close the open text file #################################*/
		f_close(&MyFile);

		printf("Closed file %s OK \r\n", fname);
	}
}

void DirList(void) {

	FATFS fs;
	FRESULT res;
	char buff[256];

	// Should be mounted already
	res = f_mount(&fs, "", 1);
	if (res == FR_OK) {
		strcpy(buff, "");

		// File list
		res = scan_files(buff);

		// Disk free space
		DWORD fre_clust, fre_sect, tot_sect;
		FATFS *fsp;

		// Get volume information and free clusters of drive 1
		res = f_getfree(buff, &fre_clust, &fsp);
		if (res) {
			printf("Error: Filesystem free space check failed \r\n");
			_Error_Handler(__FILE__, __LINE__);
		}

		// Get total sectors and free sectors
		tot_sect = (fsp->n_fatent - 2) * fsp->csize;
		fre_sect = fre_clust * fsp->csize;

		// Print the free space (assuming 512 bytes/sector)
		printf(
				"%10lu KiB total drive space.\n%10lu KiB available (%s\%%). \n",
				tot_sect / 2, fre_sect / 2,
				myPrintf(((float) (fre_sect)) / ((float) (tot_sect)) * 100.0));

	} else {
		printf("Error: Filesystem mount failed \r\n");
		_Error_Handler(__FILE__, __LINE__);
	}
}

