

#include <serial.h>
#include <stdarg.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sched.h>
#include <pthread.h>
#include <signal.h>
#include <termios.h>	/* POSIX terminal control definitons */
#include <unistd.h>		/* UNIX standard function definitios */
#include <fcntl.h>		/* File control definitions */
#include <sys/ioctl.h>	/* System I/O definitions and structures */
#include <sys/types.h>	/* Standard system types. Ex.: types for mutex */
#include <sys/stat.h>	/* Used to facilitate getting information about file attributes */


int serial_handle = 0;

#define FiO2_ADDR       40000
#define INSP_ADDR       40001
#define BPM_ADDR        40002
#define PAW_ADDR        40003
#define MAX_PAW_ADDR    40004
#define MIN_PEEP_ADDR   40005
#define MAX_PEEP_ADDR   40006
#define PEEP_ADDR   	40007
#define PTR_ADDR   		40008
#define ON_OFF_ADDR   	40009
#define SP3_ZERO_ADDR   40010
#define SP3_FC_ADDR   	40011

/*
 * serialOpen:
 *	Open and initialise the serial port, setting all the right
 *	port parameters - or as many as are required.
 *********************************************************************************
 */

int serialOpen (char *device, int baud)
{
	struct termios options ;
	speed_t myBaud ;
	int     status, fd ;

	switch (baud)
	{
	case     50:	myBaud =     B50 ; break ;
	case     75:	myBaud =     B75 ; break ;
	case    110:	myBaud =    B110 ; break ;
	case    134:	myBaud =    B134 ; break ;
	case    150:	myBaud =    B150 ; break ;
	case    200:	myBaud =    B200 ; break ;
	case    300:	myBaud =    B300 ; break ;
	case    600:	myBaud =    B600 ; break ;
	case   1200:	myBaud =   B1200 ; break ;
	case   1800:	myBaud =   B1800 ; break ;
	case   2400:	myBaud =   B2400 ; break ;
	case   9600:	myBaud =   B9600 ; break ;
	case  19200:	myBaud =  B19200 ; break ;
	case  38400:	myBaud =  B38400 ; break ;
	case  57600:	myBaud =  B57600 ; break ;
	case 115200:	myBaud = B115200 ; break ;
	case 230400:	myBaud = B230400 ; break ;

	default:
	  return -2 ;
	}

	// Open serial port "device"
	// O_RDWR 	- allows to read and write to the serial port
	// O_NOCTTY - tells UNIX that this program doesn't want to be the controlling entity for that port. If you do not specify this, the device file
	//			  will be owned by you, and any input (such as keyboard abort signals) will afect your process.
	// O_NDELAY - this flag tells that we don't care whether the other end of the port is up and running (i.e. if a data carrier detect (DCD) signal is present)
	//			  If you do not specify this flag, your process will be put to sleep until the DCD signal line is set to the space voltage.
	if ((fd = open (device, O_RDWR | O_NOCTTY | O_NDELAY)) == -1)
		return -1 ;

	// Get the current options:
	tcgetattr (fd, &options) ;

    // Set the terminal to the raw mode, i.e., input is available character by character, echoing is disabled and special processing
    // of terminal input and output character is disabled.
    // The terminal attributes are set as follows:
    // termios_p->c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    // termios_p->c_oflag &= ~OPOST;
    // termios_p->c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    // termios_p->c_lflag &= ~(CSIZE | PARENB);
    // termios_p->c_lflag |= CS8;
    cfmakeraw   (&options) ;
    
    // Set input baud rate speed
    cfsetispeed (&options, myBaud) ;
    
    // Set output baud rate speed
    cfsetospeed (&options, myBaud) ;

    // CLOCAL - Ignore modem control lines
    // CREAD  - Enable receiver
    options.c_cflag |= (CLOCAL | CREAD) ;
    
    // PARENB - Enable parity generation on output and parity checking for input (disabled)
    options.c_cflag &= ~PARENB ;
    
    // CSTOPB - Set two stop bits, rather than one (disabled)
    options.c_cflag &= ~CSTOPB ;
    
    // CSIZE - Character size mask (CS5, CS6, CS7, or CS8) - Clear character size info
    options.c_cflag &= ~CSIZE ;
    
    // Set 8 bit character
    options.c_cflag |= CS8 ;
    
    // Disable the canonical mode, the echo input characters and some signal related to INTR, QUIT, SUSP and DSUSP characters
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG) ;
    
    // Disable implementation-defined output processing
    options.c_oflag &= ~OPOST ;

    // There is no limit of characters in order to return a read command
    // returns either when at least one byte of data is available, or when the timer expires.
    options.c_cc [VMIN]  =   0 ;
    options.c_cc [VTIME] = 100 ;	// Ten seconds (100 deciseconds)

	// Set modify options
	// TCSANOW - set now or ...
	// TCSAFLUSH - set after all output has been transmitted. Also, all input that has been received but not read will be discarted
	// 			   before the change is made.
	tcsetattr (fd, TCSANOW | TCSAFLUSH, &options) ;

	// Get the status of the modem bits.
	ioctl (fd, TIOCMGET, &status);

	// Set data terminal ready (DTR) in the status flag
	status |= TIOCM_DTR ;
	
	// Set request to send (RTS) in the status flag
	status |= TIOCM_RTS ;

	// Set this new status flag
	ioctl (fd, TIOCMSET, &status);

	usleep (10000) ;	// 10mS

	// Return the file descriptor
	return fd ;
}


/*
 * serialFlush:
 *	Discard data written to the serial port but not transmitted, or data received but not read; Flush both buffers (tx & rx)
 *********************************************************************************
 */

void serialFlush (int fd)
{
  tcflush (fd, TCIOFLUSH) ;
}


/*
 * serialClose:
 *	Release the serial port
 *********************************************************************************
 */

void serialClose (int fd)
{
  close (fd) ;
}


/*
 * serialPutchar:
 *	Send a single character to the serial port
 *********************************************************************************
 */

int serialPutchar (int fd, unsigned char c)
{
  return write (fd, &c, 1) ;
}


/*
 * serialPutchar:
 *	Send a single character to the serial port
 *********************************************************************************
 */

int serialPutBuffer (int fd, char *s, int len)
{
   return write (fd, s, len) ;
}

/*
 * serialPuts:
 *	Send a string to the serial port
 *********************************************************************************
 */

int serialPuts (int fd, char *s)
{
  return write (fd, s, strlen (s)) ;
}

/*
 * serialPrintf:
 *	Printf over Serial
 *********************************************************************************
 */

void serialPrintf (int fd, char *message, ...)
{
  va_list argp ;
  char buffer [1024] ;

  va_start (argp, message) ;
    vsnprintf (buffer, 1023, message, argp) ;
  va_end (argp) ;

  serialPuts (fd, buffer) ;
}


/*
 * serialDataAvail:
 *	Return the number of bytes of data avalable to be read in the serial port
 *********************************************************************************
 */

int serialDataAvail (int fd)
{
  int result ;

  if (ioctl (fd, FIONREAD, &result) == -1)
    return -1 ;

  return result ;
}


/*
 * serialGetchar:
 *	Get a single character from the serial device.
 *	Note: Zero is a valid character and this function will time-out after
 *	10 seconds.
 *********************************************************************************
 */

int serialGetchar (int fd, char *data)
{
  uint8_t x ;
  
  fcntl(fd, F_SETFL, 0);			/* causes read to block until new characters are present */
  //fcntl(fd, F_SETFL, FNDELAY);	/* return immediately from the read function */

  if (read (fd, &x, 1) != 1)
    return -1 ;
    
  *data = ((char)x) & 0xFF;

  return 0;
}


static const uint8_t aucCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40
};

static const uint8_t aucCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
    0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
    0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
    0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
    0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 
    0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
    0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
    0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
    0x41, 0x81, 0x80, 0x40
};

uint16_t usMBCRC16( uint8_t * pucFrame, uint16_t usLen )
{
    uint8_t           ucCRCHi = 0xFF;
    uint8_t           ucCRCLo = 0xFF;
    int             iIndex;

    while( usLen-- )
    {
        iIndex = ucCRCLo ^ *( pucFrame++ );
        ucCRCLo = ( uint8_t )( ucCRCHi ^ aucCRCHi[iIndex] );
        ucCRCHi = aucCRCLo[iIndex];
    }
    return ( uint16_t )( ucCRCHi << 8 | ucCRCLo );
}



//Função para ler os holding registers
uint8_t read_holding_reg(uint8_t endereco_slave, uint16_t init_address, uint16_t num_reg_read, uint16_t *regs){

	uint8_t funcao=3; //função leitura de múltiplos registradores
	uint8_t mensagem[8]; //vetor  mensagem
	uint16_t  tamanho_resposta= 5+(2*num_reg_read);
	//vetor resposta (c não suporta tamanho de vetor determinado por parâmetro)
	uint8_t resposta[148]; // Suporta a leitura de um modelo com até 70 registradores
	uint16_t crc_resp;//variavel auxiliar CRC parte alta
	uint16_t CRC;
	int i;
	int j;

	//monta vetor mensagem
	init_address -= 1;
	mensagem[0] = endereco_slave;
	mensagem[1] = funcao;
	mensagem[2] = (uint8_t)(init_address >> 8);
	mensagem[3] = (uint8_t)init_address;
	mensagem[4] = (uint8_t)(num_reg_read >> 8);
	mensagem[5] = (uint8_t)num_reg_read;

	CRC=usMBCRC16(mensagem,6);

	mensagem[6] = (uint8_t)CRC;
	mensagem[7] = (uint8_t)(CRC>>8);

	resposta[0]=0;

	// Transmite a pergunta
	(void)serialPutBuffer(serial_handle, (char *)mensagem, 8) ;

	/* Recebe a resposta */
	serialGetchar(serial_handle, (char *)&resposta[0]);	// Slave addr
	serialGetchar(serial_handle, (char *)&resposta[1]);	// Function code
	if (resposta[1] != funcao){
		return -1;
	}

	for(j=2;j<tamanho_resposta;j++){
        serialGetchar(serial_handle, (char *)&resposta[j]);	// Dados
	}

	// Teste de CRC
	CRC=usMBCRC16(resposta, (tamanho_resposta-2));
	crc_resp=(resposta[(tamanho_resposta-1)]<<8) | (resposta[(tamanho_resposta-2)]);

	if(CRC==crc_resp){
		for ( i = 0; i < ((tamanho_resposta - 5)>>1); i++){
			*regs++ =  (resposta[4+(i*2)] << 8) + resposta[3+(i*2)];
		}
		return 0;
	}else{
		return -1;
	}
}//fim da função de leitura do slave




//Função para escrever os holding registers
uint8_t write_holding_reg(uint8_t endereco_slave, uint16_t init_address, uint16_t num_reg_write, uint16_t *regs){

	uint8_t funcao=16; 		//função escrita de múltiplos registradores
	uint8_t mensagem[148]; 	//vetor mensagem, suporta escrita de até 69 registradores
	uint16_t  tamanho_pergunta = 7+(2*num_reg_write);

	// Tamanho permite a escrita de até 8 registradores
	uint8_t resposta[8];
	uint16_t crc_resp;//variavel auxiliar CRC parte alta
	uint16_t CRC;
	int i;
	int j;

	//monta vetor mensagem
        init_address -= 1;
	mensagem[0] = endereco_slave;
	mensagem[1] = funcao;
	mensagem[2] = (uint8_t)(init_address >> 8);
	mensagem[3] = (uint8_t)init_address;
	mensagem[4] = (uint8_t)(num_reg_write >> 8);
	mensagem[5] = (uint8_t)num_reg_write;
	mensagem[6] = (uint8_t)(num_reg_write * 2);

	for (i=0;i<num_reg_write;i++){
		mensagem[7+(i*2)] = (uint8_t)(regs[i] & 0xFF);
		mensagem[8+(i*2)] = (uint8_t)(regs[i] >> 8);
	}

	CRC=usMBCRC16(mensagem,tamanho_pergunta);

	mensagem[tamanho_pergunta] = (uint8_t)CRC;
	mensagem[tamanho_pergunta+1] = (uint8_t)(CRC>>8);

	/* Esvaziar buffer da serial (caso existe algum lixo) */
	resposta[0]=0;

	// Transmite a pergunta
	(void)serialPutBuffer(serial_handle, (char *)mensagem, tamanho_pergunta+2) ;

	/* Recebe a resposta */
	serialGetchar(serial_handle, (char *)&resposta[0]);	// Slave addr
	serialGetchar(serial_handle, (char *)&resposta[1]);	// Function code
	if (resposta[1] != funcao){
		return -1;
	}

	for(j=2;j<8;j++){
		serialGetchar(serial_handle, (char *)&resposta[j]);	// Dados
	}

	// Teste de CRC
	CRC=usMBCRC16(resposta, (6));
	crc_resp=(resposta[(7)]<<8) | (resposta[(6)]);

	if(CRC==crc_resp){
		return 0;
	}else{
		return -1;
	}
}//fim da função de escrita do slave



void signal_callback_handler(int signum)
{
	// Close serial port
	serialClose(serial_handle);
	
	printf("Caught signal %d\n\r",signum);

    // Terminate program
	exit(signum);
}


#define SWAP(x) ((((x) & 0x00ffUL) << 8) | (((x) & 0xff00UL) >> 8))

uint8_t escreve_FiO2(uint16_t FiO2){
    // Escreve novo valor para o respirador    
    printf("Atualizando valor de FiO2 para %u\n\r", (unsigned int)SWAP(FiO2));
    write_holding_reg(10, FiO2_ADDR, 1, &FiO2);

    // Confere se o valor foi atualizado
    uint16_t updated_FiO2;
    read_holding_reg(10, FiO2_ADDR, 1, &updated_FiO2);

    if (FiO2 == updated_FiO2){
        printf("Confirmada atualização de FiO2 para %u\n\r", (unsigned int)SWAP(updated_FiO2));
        return 0;
    }else{
        printf("Erro na atualização de FiO2 para %u\n\r", (unsigned int)SWAP(updated_FiO2));
        return -1;
    }
}


uint8_t escreve_insp(uint16_t insp){
    // Escreve novo valor para o respirador    
    printf("Atualizando valor de insp para %u\n\r", (unsigned int)SWAP(insp));
    write_holding_reg(10, INSP_ADDR, 1, &insp);
    fflush(stdout);

    // Confere se o valor foi atualizado
    uint16_t updated_insp;
    read_holding_reg(10, INSP_ADDR, 1, &updated_insp);
    fflush(stdout);

    if (insp == updated_insp){
        printf("Confirmada atualização de bpm para %u\n\r", (unsigned int)SWAP(updated_insp));
        return 0;
    }else{
        printf("Erro na atualização de bpm para %u\n\r", (unsigned int)SWAP(updated_insp));
        return -1;
    }
}


uint8_t escreve_bpm(uint16_t bpm){
    // Escreve novo valor para o respirador  
    printf("Atualizando valor de bpm para %u\n\r", (unsigned int)SWAP(bpm));  
    write_holding_reg(10, BPM_ADDR, 1, &bpm);

    // Confere se o valor foi atualizado
    uint16_t updated_bpm;
    read_holding_reg(10, BPM_ADDR, 1, &updated_bpm);

    if (bpm == updated_bpm){
        printf("Confirmada atualização de bpm para %u\n\r", (unsigned int)SWAP(updated_bpm));
        return 0;
    }else{
        printf("Erro na atualização de bpm para %u\n\r", (unsigned int)SWAP(updated_bpm));
        return -1;
    }
}


uint8_t escreve_max_paw(uint16_t max_paw){
    // Escreve novo valor para o respirador
    printf("Atualizando valor de max_paw para %u\n\r", (unsigned int)SWAP(max_paw));    
    write_holding_reg(10, MAX_PAW_ADDR, 1, &max_paw);

    // Confere se o valor foi atualizado
    uint16_t updated_max_paw;
    read_holding_reg(10, MAX_PAW_ADDR, 1, &updated_max_paw);

    if (max_paw == updated_max_paw){
        printf("Confirmada atualização de max_paw para %u\n\r", (unsigned int)SWAP(updated_max_paw));
        return 0;
    }else{
        printf("Erro na autalização de max_paw para %u\n\r", (unsigned int)SWAP(updated_max_paw));
        return -1;
    }
}

uint8_t escreve_onoff(uint16_t onoff){
    printf("Escreve onoff com valor %d\n\r", onoff);
    // Escreve novo valor para o respirador
    onoff = SWAP(onoff);    
    write_holding_reg(10, ON_OFF_ADDR, 1, &onoff);

    // Confere se o valor foi atualizado
    uint16_t updated_onoff;
    read_holding_reg(10, ON_OFF_ADDR, 1, &updated_onoff);

    if (onoff == updated_onoff){
	printf("Deu certo. \n\r");
        return 0;
    }else{
	printf("Não deu certo. \n\r");
        return -1;
    }
}

/*
 *********************************************************************************
 * main:
 *	Let the fun begin
 *********************************************************************************
 */
int modbusOpen(char *string)
{ 
  printf("\n\rTentando abrir a porta serial %s\n\r", string);
  serial_handle = serialOpen (string, 115200);
  if (serial_handle < 0)
  {
	printf("Não deu certo. \n\r");
	return -1;    
  }else{
	printf("Porta serial %s aberta com sucesso!\n\r\n\r", string);  
	return 0;
  }
}


