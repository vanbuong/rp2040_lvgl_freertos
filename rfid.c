#include "FreeRTOS.h"
#include "task.h"
#include "rfid.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "string.h"

#define CS_LOW()    gpio_put(RFID_CS, 0)
#define CS_HIGH()   gpio_put(RFID_CS, 1)

uint32_t _RxBits;

void rfid_init()
{
    spi_init(RFID_SPI, RFID_SPI_SPEED);
    gpio_set_function(RFID_SCK, GPIO_FUNC_SPI);
    gpio_set_function(RFID_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(RFID_MISO, GPIO_FUNC_SPI);

    gpio_init(RFID_CS);
    gpio_set_dir(RFID_CS, GPIO_OUT);
    gpio_put(RFID_CS, 1);

    gpio_init(RFID_RST);
    gpio_set_dir(RFID_RST, GPIO_OUT);
    gpio_put(RFID_RST, 1);

    // Reset RFID module
    gpio_put(RFID_RST, 0);  //Pull down for short time
    sleep_ms(1);
    gpio_put(RFID_RST, 1);
    sleep_ms(1);

	//Timer: TPrescaler*TreloadVal/6.78MHz = 24ms
	rfid_write(TModeReg, 0x8D);		//Tauto=1; f(Timer) = 6.78MHz/TPreScaler
	rfid_write(TPrescalerReg, 0x3E);	//TModeReg[3..0] + TPrescalerReg
	rfid_write(TReloadRegL, 0xE8);
	rfid_write(TReloadRegH, 0x03);
	rfid_write(TxAutoReg, 0x40);		//100%ASK
	rfid_write(ModeReg, 0x3D);		//CRC Initial value 0x6363	???
	rfid_antenna_on();		//Open the antenna
	rfid_write(ComIrqReg, 0x80);
	rfid_write(ComIEnReg, 0x7f);
	rfid_write(DivlEnReg, 0x90);

	/* Need delay to RFReader get ready to read the RF card */
	sleep_ms(1);
}

void rfid_task(void *param)
{
    unsigned char str[RFID_DATA_MAX_LEN];
    uint8_t out_len;
    do {
        rfid_get_card_info(param, str, RFID_DATA_MAX_LEN, &out_len);
        vTaskDelay(pdMS_TO_TICKS(250));
    } while (1);
}

void rfid_write(uint8_t addr, uint8_t val)
{
	uint8_t spiData[2];

	spiData[0] = (addr << 1) & 0x7E;  // Address Format: 0XXXXXX0, the left most "0" indicates a write
	spiData[1] = val;

	CS_LOW();
	spi_write_blocking(RFID_SPI, spiData, 2);
	CS_HIGH();
}

uint8_t rfid_read(uint8_t addr)
{
	uint8_t spiData;

	spiData = ((addr << 1) & 0x7E) | 0x80;  // Address Format: 1XXXXXX0, the first "1" indicates a read

	CS_LOW();
	spi_write_blocking(RFID_SPI, &spiData, 1);
	spi_read_blocking(RFID_SPI, 0, &spiData, 1);
	CS_HIGH();

    return spiData;
}

void set_bit_mask(uint8_t reg, uint8_t mask)
{
    uint8_t tmp;
    tmp = rfid_read(reg);
    rfid_write(reg, tmp | mask);  // set bit mask
}


/*
 * Function Name: clear_bit_mask
 * Description: clear RC522 register bit
 * Input parameters: reg - register address; mask - clear bit value
 * Return value: None
*/
void clear_bit_mask(uint8_t reg, uint8_t mask)
{
    uint8_t tmp;
    tmp = rfid_read(reg);
    rfid_write(reg, tmp & (~mask));  // clear bit mask
}


/*
 * Function Name: rfid_antenna_on
 * Description: Open antennas, each time you start or shut down the natural barrier between the transmitter should be at least 1ms interval
 * Input: None
 * Return value: None
 */
void rfid_antenna_on(void)
{
	uint8_t temp;

	temp = rfid_read(TxControlReg);
	if (!(temp & 0x03))
	{
		set_bit_mask(TxControlReg, 0x03);
	}
}


/*
  * Function Name: rfid_antenna_off
  * Description: Close antennas, each time you start or shut down the natural barrier between the transmitter should be at least 1ms interval
  * Input: None
  * Return value: None
 */
void rfid_antenna_off(void)
{
	clear_bit_mask(TxControlReg, 0x03);
}


/*
 * Function Name: rfid_softreset
 * Description: Perform soft reset of RFID Module
 * Input: None
 * Return value: None
 */
void rfid_softreset(void)
{
    rfid_write(CommandReg, PCD_SOFTRESET);
}


/*
 * Function Name: rfid_request
 * Description: Find cards, read the card type number
 * Input parameters: reqMode - find cards way
 *			 TagType - Return Card Type
 *			 	0x4400 = Mifare_UltraLight
 *				0x0400 = Mifare_One(S50)
 *				0x0200 = Mifare_One(S70)
 *				0x0800 = Mifare_Pro(X)
 *				0x4403 = Mifare_DESFire
 * Return value: the successful return MI_OK
 */
uint8_t rfid_request(uint8_t reqMode, uint8_t *TagType)
{
	uint8_t status;

	rfid_write(BitFramingReg, 0x07);		//TxLastBists = BitFramingReg[2..0]	???

	TagType[0] = reqMode;
	status = rfid_write_to_card(PCD_TRANSCEIVE, TagType, 1, TagType, &_RxBits);
    if (status == MI_OK) printf("_RxBits = %ld\r\n", _RxBits);
	if ((status != MI_OK) || (_RxBits != 0x10))
	{
		status = MI_ERR;
	}

	return status;
}


/*
 * Function Name: rfid_write_to_card
 * Description: RC522 and ISO14443 card communication
 * Input Parameters: command - MF522 command word,
 *			 sendData--RC522 sent to the card by the data
 *			 sendLen--Length of data sent
 *			 backData--Data returned from the card
 *			 backLen--Returned data bit length
 * Return value: the successful return MI_OK
 */
uint8_t rfid_write_to_card(uint8_t command, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint32_t *backLen)
{
    uint8_t status = MI_ERR;
    uint8_t irqEn = 0x00;
    uint8_t waitIRq = 0x00;
    uint8_t lastBits;
    uint8_t n;
    uint32_t i;

    switch (command)
    {
        case PCD_MFAUTHENT:		//Certification cards close
		{
			irqEn = 0x12;
			waitIRq = 0x10;
			break;
		}
		case PCD_TRANSCEIVE:	//Transmit FIFO data
		{
			irqEn = 0x77;
			waitIRq = 0x30;
			break;
		}
		default:
			break;
    }

    rfid_write(ComIrqReg, irqEn|0x80);	//Interrupt request
    clear_bit_mask(ComIrqReg, 0x80);			//Clear all interrupt request bit
    set_bit_mask(FIFOLevelReg, 0x80);			//FlushBuffer=1, FIFO Initialization

	rfid_write(CommandReg, PCD_IDLE);	//NO action; Cancel the current command???

	//Writing data to the FIFO
    for (i=0; i<sendLen; i++)
    {
		rfid_write(FIFODataReg, sendData[i]);
	}

	//Execute the command
	rfid_write(CommandReg, command);
    if (command == PCD_TRANSCEIVE)
    {
		set_bit_mask(BitFramingReg, 0x80);		//StartSend=1,transmission of data starts
	}

	//Waiting to receive data to complete
	i = 2000;	// according to the clock frequency adjustment, the operator M1 card maximum waiting time 25ms???
    do
    {
		//ComIrqReg[7..0]
		//Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
        n = rfid_read(ComIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x01) && !(n&waitIRq));

    clear_bit_mask(BitFramingReg, 0x80);			//StartSend=0

    if (i != 0)
    {
        if(!(rfid_read(ErrorReg) & 0x1B))	//BufferOvfl Collerr CRCErr ProtecolErr
        {
            status = MI_OK;
            if (n & irqEn & 0x01)
            {
				status = MI_NO_TAG_ERR;			//??
			}

            if (command == PCD_TRANSCEIVE)
            {
               	n = rfid_read(FIFOLevelReg);
              	lastBits = rfid_read(ControlReg) & 0x07;
                if (lastBits)
                {
					*backLen = (n-1)*8 + lastBits;
				}
                else
                {
					*backLen = n*8;
				}

                if (n == 0)
                {
					n = 1;
				}
                if (n > RFID_DATA_MAX_LEN)
                {
					n = RFID_DATA_MAX_LEN;
				}
                printf("RFID: rd %d bytes fr FIFO\n", n);
				//Reading the received data in FIFO
                for (i=0; i<n; i++)
                {
					backData[i] = rfid_read(FIFODataReg);
				}
            }
        }
        else
        {
        	printf("ErrReg: 0x%02X\n", rfid_read(ErrorReg));
			status = MI_ERR;
		}

    }
    return status;
}


/*
 * Function Name: rfid_anticoll
 * Description: Anti-collision detection, reading selected card serial number card
 * Input parameters: serNum - returns 4 bytes card serial number, the first 5 bytes for the checksum byte
 * Return value: the successful return MI_OK
 */
uint8_t rfid_anticoll(uint8_t *serNum)
{
    uint8_t status;
    uint8_t i;
	uint8_t serNumCheck=0;

	rfid_write(BitFramingReg, 0x00);		//TxLastBists = BitFramingReg[2..0]

    serNum[0] = PICC_ANTICOLL;
    serNum[1] = 0x20;
    status = rfid_write_to_card(PCD_TRANSCEIVE, serNum, 2, serNum, &_RxBits);

    if (status == MI_OK)
	{
		//Check card serial number
		for (i=0; i<4; i++)
		{
		 	serNumCheck ^= serNum[i];
		}
		if (serNumCheck != serNum[i])
		{
			status = MI_ERR;
		}
    }
    return status;
}


/*
 * Function Name: rfid_calculate_crc
 * Description: CRC calculation with MF522
 * Input parameters: pIndata - To read the CRC data, len - the data length, pOutData - CRC calculation results
 * Return value: None
 */
void rfid_calculate_crc(uint8_t *pIndata, uint8_t len, uint8_t *pOutData)
{
    uint8_t i, n;

    clear_bit_mask(DivIrqReg, 0x04);			//CRCIrq = 0
    set_bit_mask(FIFOLevelReg, 0x80);			//Clear the FIFO pointer
    //Write_RFID(CommandReg, PCD_IDLE);

	//Writing data to the FIFO
    for (i=0; i<len; i++)
    {
		rfid_write(FIFODataReg, *(pIndata+i));
	}
    rfid_write(CommandReg, PCD_CALCCRC);

	//Wait CRC calculation is complete
    i = 0xFF;
    do
    {
        n = rfid_read(DivIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x04));			//CRCIrq = 1

	//Read CRC calculation result
    pOutData[0] = rfid_read(CRCResultRegL);
    pOutData[1] = rfid_read(CRCResultRegM);
}


/*
 * Function Name: rfid_select_tag
 * Description: Selection card, read the card memory capacity
 * Input parameters: serNum - Incoming card serial number
 * Return value: the successful return of card capacity
 */
uint32_t rfid_select_tag(uint8_t *serNum)
{
    uint8_t i;
	uint8_t status;
	uint8_t size;
    uint32_t recvBits;
    uint8_t buffer[9];

	//clear_bit_mask(Status2Reg, 0x08);			//MFCrypto1On=0

    buffer[0] = PICC_SElECTTAG;
    buffer[1] = 0x70;
    for (i=0; i<5; i++)
    {
    	buffer[i+2] = *(serNum+i);
    }
	rfid_calculate_crc(buffer, 7, &buffer[7]);		//??
    status = rfid_write_to_card(PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);

    if ((status == MI_OK) && (recvBits == 0x18))
    {
		size = buffer[0];
	}
    else
    {
		size = 0;
	}

    return size;
}


/*
 * Function Name: rfid_authentication
 * Description: Verify card password
 * Input parameters: authMode - Password Authentication Mode
                 0x60 = A key authentication
                 0x61 = Authentication Key B
             BlockAddr--Block address
             Sectorkey--Sector password
             serNum--Card serial number, 4-byte
 * Return value: the successful return MI_OK
 */
uint8_t rfid_authentication(uint8_t authMode, uint8_t BlockAddr, uint8_t *Sectorkey, uint8_t *serNum)
{
    uint8_t status;
    uint32_t recvBits;
    uint8_t i;
	uint8_t buff[12];

	//Verify the command block address + sector + password + card serial number
    buff[0] = authMode;
    buff[1] = BlockAddr;
    for (i=0; i<6; i++)
    {
		buff[i+2] = *(Sectorkey+i);
	}
    for (i=0; i<4; i++)
    {
		buff[i+8] = *(serNum+i);
	}
    status = rfid_write_to_card(PCD_MFAUTHENT, buff, 12, buff, &recvBits);

    if ((status != MI_OK) || (!(rfid_read(Status2Reg) & 0x08)))
    {
		status = MI_ERR;
	}

    return status;
}


/*
 * Function Name: rfid_read_block
 * Description: Read block data
 * Input parameters: blockAddr - block address; recvData - read block data
 * Return value: the successful return MI_OK
 */
uint8_t rfid_read_block(uint8_t blockAddr, uint8_t *recvData)
{
    uint8_t status;
    uint32_t unLen;

    recvData[0] = PICC_READ;
    recvData[1] = blockAddr;
    rfid_calculate_crc(recvData,2, &recvData[2]);
    status = rfid_write_to_card(PCD_TRANSCEIVE, recvData, 4, recvData, &unLen);

    if ((status != MI_OK) || (unLen != 0x90))
    {
        status = MI_ERR;
    }

    return status;
}


/*
 * Function Name: rfid_write_block
 * Description: Write block data
 * Input parameters: blockAddr - block address; writeData - to 16-byte data block write
 * Return value: the successful return MI_OK
 */
uint8_t rfid_write_block(uint8_t blockAddr, uint8_t *_writeData)
{
    uint8_t status;
    uint32_t recvBits;
    uint8_t i;
	uint8_t buff[18];

    buff[0] = PICC_WRITE;
    buff[1] = blockAddr;
    rfid_calculate_crc(buff, 2, &buff[2]);
    status = rfid_write_to_card(PCD_TRANSCEIVE, buff, 4, buff, &recvBits);

    if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
    {
		status = MI_ERR;
	}

    if (status == MI_OK)
    {
        for (i=0; i<16; i++)		//Data to the FIFO write 16Byte
        {
        	buff[i] = *(_writeData+i);
        }
        rfid_calculate_crc(buff, 16, &buff[16]);
        status = rfid_write_to_card(PCD_TRANSCEIVE, buff, 18, buff, &recvBits);

		if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
        {
			status = MI_ERR;
		}
    }

    return status;
}


/*
 * Function Name: rfid_halt
 * Description: Command card into hibernation
 * Input: None
 * Return value: None
 */
uint8_t rfid_halt(void)
{
	uint8_t status;
    uint32_t unLen;
    uint8_t buff[4];

    buff[0] = PICC_HALT;
    buff[1] = 0;
    rfid_calculate_crc(buff, 2, &buff[2]);

    status = rfid_write_to_card(PCD_TRANSCEIVE, buff, 4, buff,&unLen);

    return status;
}

uint8_t get_num_rxbits(void)
{
	return _RxBits;
}

void rfid_clear_interrupt()
{
	rfid_write(ComIrqReg,0x7f);
}

int rfid_get_card_info(callback cb, unsigned char* str_, uint8_t inlen, uint8_t *olen)
{
	uint8_t status;
	int ret = MI_OK;
	unsigned char str[RFID_DATA_MAX_LEN];

	//rfid_init();

	*olen = 0;
	{
		status = rfid_request(PICC_REQALL, str); //
		if (status == MI_OK)
		{
			printf("RFID tag detected\r\n");
			printf("Tag Type: ");
			uint16_t tagType = (uint16_t) str[0] << 8;
			tagType = tagType + str[1];
			switch (tagType)
			{
				case 0x4400:
				printf("MF UL\r\n");
				break;
				case 0x400:
				printf("MF 1 (S50)\r\n");
				break;
				case 0x200:
				printf("MF 1 (S70)\r\n");
				break;
				case 0x800:
				printf("MF P (X)\r\n");
				break;
				case 0x4403:
				printf("MF DESFire\r\n");
				break;
				default:
				printf("Unk\r\n");
				break;
			}
		}

		//Anti-collision, return tag serial number 4 bytes
		status = rfid_anticoll(str);
		if (status == MI_OK)
		{
			ret = MI_OK;
			printf("The tag's number is:\r\n");
			printf("0x%02X", str[0]);
			printf(" , ");
			printf("0x%02X", str[1]);
			printf(" , ");
			printf("0x%02X", str[2]);
			printf(" , ");
			printf("0x%02X\r\n", str[3]);
			uint8_t checksum1 = str[0] ^ str[1] ^ str[2] ^ str[3];
			printf("Read Checksum: ");
			printf("0x%02X\r\n", str[4]);
			printf("Calculated Checksum: ");
			printf("0x%02X\r\n", checksum1);
            cb(str);
			if (inlen > RFID_DATA_MAX_LEN) {
				inlen = RFID_DATA_MAX_LEN;
			}
			memset(&str[4], 0, RFID_DATA_MAX_LEN - 4);
			memcpy(str_, str, inlen);
			*olen = inlen;
		} else {
			ret = MI_NO_TAG_ERR;
		}
	}
	return ret;
}
