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

// https://www.experts-exchange.com/questions/29003980/Display-a-Float-Variable-in-C-without-using-the-function-printf.html
char *myPrintf(float source) {
	int iValue, dValue;
	static char Display[20];

	// iValue contains the integer portion of the
	// floating point number.
	iValue = source;

	// dValue contains the first 2 digits of the fractional part
	//of the floating point number.
	dValue = (source * 100) - (iValue * 100);

	snprintf(Display, 20, "%d%c%d", iValue, '.', dValue);

	return Display;
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

		if (stringArray[wordCounter] != '\0') {
			wordCounter++;
		}

		counter = 0;
		argCounter++;
	}

	// If we return here. the parse_string had less arguments as requested
	// In that case, return empty string
	return NULL;
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

		// printf("End Counter %lu \r\n", __HAL_TIM_GET_COUNTER(&htim2));

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

void WriteData2FS(const Diskio_drvTypeDef SDRAMDISK_Driver, FATFS SDRAMFatFs,
		char SDRAMPath[4], FIL MyFile, Array *Data, uint32_t NoOfPoints,
		uint32_t MeasNo) {

	// Create file for data, as meas#.txt

	FRESULT res; // FatFs function common result code
	uint32_t byteswritten, totalbytes; //File write counts

	//FATFS SDRAMFatFs;
	//const Diskio_drvTypeDef drv;
	//char SDRAMPath[4]; /* SDRAM card logical drive path */

	//char fname[30] = " ";

	//sprintf(fname, "meas_%lu.txt", MeasNo);

	//FIL MyFile;
	uint32_t idx;

	totalbytes = 0;

	// Unlink the SDRAM disk I/O driver
	FATFS_UnLinkDriver(SDRAMPath);


	printf("Starting file write 1 \r\n");

	// Link the SDRAM disk I/O driver
	if (FATFS_LinkDriver(&SDRAMDISK_Driver, SDRAMPath) == 0) {
		printf("SDRAM FATFS link Success \r\n");
	}
	// Register the file system object to the FatFs module
	if (f_mount(&SDRAMFatFs, (TCHAR const*) SDRAMPath, 0) != FR_OK) {
		// FatFs Initialization Error
		_Error_Handler(__FILE__, __LINE__);
	} else {
		printf("SDRAM FATFS mount Success \r\n");
	}
	// Create and Open a new text file object with write access
	if (f_open(&MyFile, "MEAS1.TXT", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
		// File Open for write Error
		_Error_Handler(__FILE__, __LINE__);
	} else {
		printf("SDRAM FATFS fopen Success \r\n");
	}

	char buffer[30];

	// Write data to the text file
	for (int idx = 0; idx < NoOfPoints; idx++) {
		sprintf(buffer, "%lu \r\n", ((uint32_t) Data->array[idx]));
		res = f_write(&MyFile, buffer, sizeof(buffer), (void *) &byteswritten);
	}

	if ((byteswritten == 0) || (res != FR_OK)) {
		_Error_Handler(__FILE__, __LINE__);
	} else {
		printf("SDRAM FATFS write Success \r\n");
		// Close the text file
		f_close(&MyFile);
		printf("SDRAM FATFS fclose Success \r\n");
	}

	printf("Closed file %s OK \r\n", "MEAS1.TXT");
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



static DWORD pn( /* Pseudo random number generator */
DWORD pns /* 0:Initialize, !0:Read */
) {
	static DWORD lfsr;
	UINT n;

	if (pns) {
		lfsr = pns;
		for (n = 0; n < 32; n++)
			pn(0);
	}
	if (lfsr & 1) {
		lfsr >>= 1;
		lfsr ^= 0x80200003;
	} else {
		lfsr >>= 1;
	}
	return lfsr;
}

// ChaN's test file IO
int test_diskio(BYTE pdrv, /* Physical drive number to be checked (all data on the drive will be lost) */
UINT ncyc, /* Number of test cycles */
DWORD* buff, /* Pointer to the working buffer */
UINT sz_buff /* Size of the working buffer in unit of byte */
) {
	UINT n, cc, ns;
	DWORD sz_drv, lba, lba2, sz_eblk, pns = 1;
	WORD sz_sect;
	BYTE *pbuff = (BYTE*) buff;
	DSTATUS ds;
	DRESULT dr;

	printf("test_diskio(%u, %u, 0x%08X, 0x%08X)\n", pdrv, ncyc, (UINT) buff,
			sz_buff);

	if (sz_buff < _MAX_SS + 4) {
		printf("Insufficient work area to run program.\n");
		return 1;
	}

	for (cc = 1; cc <= ncyc; cc++) {
		printf("**** Test cycle %u of %u start ****\n", cc, ncyc);

		printf(" disk_initalize(%u)", pdrv);
		ds = disk_initialize(pdrv);
		if (ds & STA_NOINIT) {
			printf(" - failed.\n");
			return 2;
		} else {
			printf(" - ok.\n");
		}

		printf("**** Get drive size ****\n");
		printf(" disk_ioctl(%u, GET_SECTOR_COUNT, 0x%08X)", pdrv,
				(UINT) &sz_drv);
		sz_drv = 0;
		dr = disk_ioctl(pdrv, GET_SECTOR_COUNT, &sz_drv);
		if (dr == RES_OK) {
			printf(" - ok.\n");
		} else {
			printf(" - failed.\n");
			return 3;
		}
		if (sz_drv < 128) {
			printf("Failed: Insufficient drive size to test.\n");
			return 4;
		}
		printf(" Number of sectors on the drive %u is %lu.\n", pdrv, sz_drv);

#if FF_MAX_SS != FF_MIN_SS
		printf("**** Get sector size ****\n");
		printf(" disk_ioctl(%u, GET_SECTOR_SIZE, 0x%X)", pdrv, (UINT)&sz_sect);
		sz_sect = 0;
		dr = disk_ioctl(pdrv, GET_SECTOR_SIZE, &sz_sect);
		if (dr == RES_OK) {
			printf(" - ok.\n");
		} else {
			printf(" - failed.\n");
			return 5;
		}
		printf(" Size of sector is %u bytes.\n", sz_sect);
#else
		sz_sect = _MAX_SS;
#endif

		printf("**** Get block size ****\n");
		printf(" disk_ioctl(%u, GET_BLOCK_SIZE, 0x%X)", pdrv, (UINT) &sz_eblk);
		sz_eblk = 0;
		dr = disk_ioctl(pdrv, GET_BLOCK_SIZE, &sz_eblk);
		if (dr == RES_OK) {
			printf(" - ok.\n");
		} else {
			printf(" - failed.\n");
		}
		if (dr == RES_OK || sz_eblk >= 2) {
			printf(" Size of the erase block is %lu sectors.\n", sz_eblk);
		} else {
			printf(" Size of the erase block is unknown.\n");
		}

		/* Single sector write test */
		printf("**** Single sector write test 1 ****\n");
		lba = 0;
		for (n = 0, pn(pns); n < sz_sect; n++)
			pbuff[n] = (BYTE) pn(0);
		printf(" disk_write(%u, 0x%X, %lu, 1)", pdrv, (UINT) pbuff, lba);
		dr = disk_write(pdrv, pbuff, lba, 1);
		if (dr == RES_OK) {
			printf(" - ok.\n");
		} else {
			printf(" - failed.\n");
			return 6;
		}
		printf(" disk_ioctl(%u, CTRL_SYNC, NULL)", pdrv);
		dr = disk_ioctl(pdrv, CTRL_SYNC, 0);
		if (dr == RES_OK) {
			printf(" - ok.\n");
		} else {
			printf(" - failed.\n");
			return 7;
		}
		memset(pbuff, 0, sz_sect);
		printf(" disk_read(%u, 0x%X, %lu, 1)", pdrv, (UINT) pbuff, lba);
		dr = disk_read(pdrv, pbuff, lba, 1);
		if (dr == RES_OK) {
			printf(" - ok.\n");
		} else {
			printf(" - failed.\n");
			return 8;
		}
		for (n = 0, pn(pns); n < sz_sect && pbuff[n] == (BYTE) pn(0); n++)
			;
		if (n == sz_sect) {
			printf(" Data matched.\n");
		} else {
			printf("Failed: Read data differs from the data written.\n");
			return 10;
		}
		pns++;

		printf("**** Multiple sector write test ****\n");
		lba = 1;
		ns = sz_buff / sz_sect;
		if (ns > 4)
			ns = 4;
		for (n = 0, pn(pns); n < (UINT) (sz_sect * ns); n++)
			pbuff[n] = (BYTE) pn(0);
		printf(" disk_write(%u, 0x%X, %lu, %u)", pdrv, (UINT) pbuff, lba, ns);
		dr = disk_write(pdrv, pbuff, lba, ns);
		if (dr == RES_OK) {
			printf(" - ok.\n");
		} else {
			printf(" - failed.\n");
			return 11;
		}
		printf(" disk_ioctl(%u, CTRL_SYNC, NULL)", pdrv);
		dr = disk_ioctl(pdrv, CTRL_SYNC, 0);
		if (dr == RES_OK) {
			printf(" - ok.\n");
		} else {
			printf(" - failed.\n");
			return 12;
		}
		memset(pbuff, 0, sz_sect * ns);
		printf(" disk_read(%u, 0x%X, %lu, %u)", pdrv, (UINT) pbuff, lba, ns);
		dr = disk_read(pdrv, pbuff, lba, ns);
		if (dr == RES_OK) {
			printf(" - ok.\n");
		} else {
			printf(" - failed.\n");
			return 13;
		}
		for (n = 0, pn(pns);
				n < (UINT) (sz_sect * ns) && pbuff[n] == (BYTE) pn(0); n++)
			;
		if (n == (UINT) (sz_sect * ns)) {
			printf(" Data matched.\n");
		} else {
			printf("Failed: Read data differs from the data written.\n");
			return 14;
		}
		pns++;

		printf("**** Single sector write test (misaligned address) ****\n");
		lba = 5;
		for (n = 0, pn(pns); n < sz_sect; n++)
			pbuff[n + 3] = (BYTE) pn(0);
		printf(" disk_write(%u, 0x%X, %lu, 1)", pdrv, (UINT) (pbuff + 3), lba);
		dr = disk_write(pdrv, pbuff + 3, lba, 1);
		if (dr == RES_OK) {
			printf(" - ok.\n");
		} else {
			printf(" - failed.\n");
			return 15;
		}
		printf(" disk_ioctl(%u, CTRL_SYNC, NULL)", pdrv);
		dr = disk_ioctl(pdrv, CTRL_SYNC, 0);
		if (dr == RES_OK) {
			printf(" - ok.\n");
		} else {
			printf(" - failed.\n");
			return 16;
		}
		memset(pbuff + 5, 0, sz_sect);
		printf(" disk_read(%u, 0x%X, %lu, 1)", pdrv, (UINT) (pbuff + 5), lba);
		dr = disk_read(pdrv, pbuff + 5, lba, 1);
		if (dr == RES_OK) {
			printf(" - ok.\n");
		} else {
			printf(" - failed.\n");
			return 17;
		}
		for (n = 0, pn(pns); n < sz_sect && pbuff[n + 5] == (BYTE) pn(0); n++)
			;
		if (n == sz_sect) {
			printf(" Data matched.\n");
		} else {
			printf("Failed: Read data differs from the data written.\n");
			return 18;
		}
		pns++;

		printf("**** 4GB barrier test ****\n");
		if (sz_drv >= 128 + 0x80000000 / (sz_sect / 2)) {
			lba = 6;
			lba2 = lba + 0x80000000 / (sz_sect / 2);
			for (n = 0, pn(pns); n < (UINT) (sz_sect * 2); n++)
				pbuff[n] = (BYTE) pn(0);
			printf(" disk_write(%u, 0x%X, %lu, 1)", pdrv, (UINT) pbuff, lba);
			dr = disk_write(pdrv, pbuff, lba, 1);
			if (dr == RES_OK) {
				printf(" - ok.\n");
			} else {
				printf(" - failed.\n");
				return 19;
			}
			printf(" disk_write(%u, 0x%X, %lu, 1)", pdrv,
					(UINT) (pbuff + sz_sect), lba2);
			dr = disk_write(pdrv, pbuff + sz_sect, lba2, 1);
			if (dr == RES_OK) {
				printf(" - ok.\n");
			} else {
				printf(" - failed.\n");
				return 20;
			}
			printf(" disk_ioctl(%u, CTRL_SYNC, NULL)", pdrv);
			dr = disk_ioctl(pdrv, CTRL_SYNC, 0);
			if (dr == RES_OK) {
				printf(" - ok.\n");
			} else {
				printf(" - failed.\n");
				return 21;
			}
			memset(pbuff, 0, sz_sect * 2);
			printf(" disk_read(%u, 0x%X, %lu, 1)", pdrv, (UINT) pbuff, lba);
			dr = disk_read(pdrv, pbuff, lba, 1);
			if (dr == RES_OK) {
				printf(" - ok.\n");
			} else {
				printf(" - failed.\n");
				return 22;
			}
			printf(" disk_read(%u, 0x%X, %lu, 1)", pdrv,
					(UINT) (pbuff + sz_sect), lba2);
			dr = disk_read(pdrv, pbuff + sz_sect, lba2, 1);
			if (dr == RES_OK) {
				printf(" - ok.\n");
			} else {
				printf(" - failed.\n");
				return 23;
			}
			for (n = 0, pn(pns);
					pbuff[n] == (BYTE) pn(0) && n < (UINT) (sz_sect * 2); n++)
				;
			if (n == (UINT) (sz_sect * 2)) {
				printf(" Data matched.\n");
			} else {
				printf("Failed: Read data differs from the data written.\n");
				return 24;
			}
		} else {
			printf(" Test skipped.\n");
		}
		pns++;

		printf("**** Test cycle %u of %u completed ****\n\n", cc, ncyc);
	}

	return 0;
}

