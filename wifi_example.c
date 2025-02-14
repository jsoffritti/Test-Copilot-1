/*
 * wifi.c
 *
 * Created: 
 *  Author: 
 *
 * Driver for the wifi controller
 */ 

/* --------------------------- Includes -------------------------------------*/
#include "wifi.h"

#include <conf_board.h>

#include "utils/error_codes.h"

#ifdef CONF_BOARD_WIFI
#include <stdio.h>
#include <string.h>
#include <delay.h>
#include <sleep.h>

#include <board.h>

#include "services/error/error.h"
#include "services/k_string/k_string.h"
#include "services/serial/serial_gw.h"
#include "services/time/time.h"

#include "conf_wifi.h"

/* ----------------------- Defines ------------------------------------------*/
// Ethernet card response limitations
#define RX_BUFLEN_M              512
#define RX_BUFLEN                256
#define TX_BUFLEN                64
#define FTP_REQ_LEN              1024
#define RX_BUFLEN_OTA            ( FTP_REQ_LEN + 20 )

#define WIFI_TX_TIMEOUT           10UL
#define WIFI_RX_TIMEOUT           10UL
#define WIFI_MIN_SERIAL_TIMEOUT   3UL
#define WIFI_MAX_SERIAL_TIMEOUT   30UL
#define WIFI_MAX_OTA_TIMEOUT      60UL
#define WIFI_STARTUP_TIME_MS          100UL
#define WIFI_EXT_MODEM_STARTUP_TIME_s 60UL

//Commands Utilities
#define AT_COMMAND               "AT"
#define ERROR_RESPONSE           "ERROR"
#define OK_RESPONSE              "OK"

// Concatenate arguments and send command
#define wifi_send_AT(SCOMM, SARG, ...)  wifi_send_command_P(PSTR(AT_COMMAND SCOMM SARG), __VA_ARGS__);
// Receive command response
#define wifi_rcv_AT_resp(buf, OK_R, ERR_R, maxln, to) wifi_receive_resp_P(buf, PSTR(OK_R), PSTR(ERR_R), maxln, to)

#define USB_WIFI_LOG
//#define USB_WIFI_PACKET_LOG

#ifdef USB_WIFI_LOG
	#define wifi_printf(...) printf(__VA_ARGS__)
#else
	#define wifi_printf(...)
#endif

#ifdef USB_WIFI_PACKET_LOG
	#define wifi_pkt_printf(...) printf(__VA_ARGS__)
#else
	#define wifi_pkt_printf(...)
#endif

/* ----------------------- Static variables ---------------------------------*/
static USART_data_t                 _wifi_usart;

/* ----------------------- Start implementation -----------------------------*/
/* ------------------------- Private Functions ------------------------------*/

static void wifi_gpio_init(void)
{

	// Set communication port in low power mode
	gpio_set_direction(GPIO_TX_USART_WIFI, IOPORT_DIR_OUTPUT);
	gpio_set_level(GPIO_TX_USART_WIFI, IOPORT_PIN_LEVEL_HIGH);
	gpio_set_direction(GPIO_RX_USART_WIFI, IOPORT_DIR_INPUT);
}

/*****************************************************************************************
 * @brief
 *    Send command to Ethernet card
 *
 * @param format
 *    command to send as formatted string (stored in PROGMEM space)
 *
 *****************************************************************************************/
static void wifi_send_command_P(const char* format, ...)
{
	char tx_buffer[TX_BUFLEN];
	va_list args;
	
	// Compose command
	va_start(args, format);
	vsprintf_P(tx_buffer, format, args);
	va_end(args);
	
	// Clear serial port reception buffer
	serial_flush(&(_wifi_usart));
	
	wifi_printf("LOG -> TX Command : %s ", tx_buffer );
	
	// Send command
	if (_wifi_usart.flow_control == FLOW_CONTROL_NONE)  delay_ms(5);
	serial_print(&(_wifi_usart), tx_buffer);
}


/*****************************************************************************************
 * @brief
 *    It reads the bytes received from the Ethernet modem serial_ port until either OK_str or 
 *    ERR_str string is received. If none of them are received, it waits 
 *    up to timeout seconds and then exits.
 *
 * @param rx_buffer
 *    Reception buffer to be filled with the received string
 *
 * @param OK_str
 *    String pointer with the OK string expected (stored in PROGMEM space).
 *
 * @param ERR_str
 *    String pointer with the possible error message expected (stored in PROGMEM space).
 *
 * @param max_len
 *    Maximum number of bytes allowed into rx_buffer
 *
 * @param timeout
 *    Max number of seconds any answer will be waited for
 *
 * @return
 *    Number of bytes received.
 *****************************************************************************************/
static uint16_t wifi_receive_resp_P(char *rx_buffer, const char *OK_resp, const char *ERR_resp, uint16_t max_len, uint8_t timeout)
{
	uint32_t init_time;
	uint16_t rcvd_bytes = 0;

	// Check input variables are valid
	if ((rx_buffer == NULL) || (OK_resp == NULL) || (ERR_resp == NULL)) return (0);
	if (max_len == 0) return (0);
	
	if (timeout < WIFI_MIN_SERIAL_TIMEOUT ) timeout = WIFI_MIN_SERIAL_TIMEOUT;   
	if (timeout > WIFI_MAX_SERIAL_TIMEOUT)  timeout = WIFI_MAX_SERIAL_TIMEOUT; 
	
	// Initialize response buffer
	memset(rx_buffer, 0, max_len);
	init_time = time_get();
	
	// Wait until OK_str or ERR_str is found in response buffer (for timeout seconds)
	while ((strstr_P(rx_buffer, OK_resp) == NULL) && (strstr_P(rx_buffer, ERR_resp) == NULL)) {
		// If timeout seconds and expected response not found, exit loop
		if ((time_get() - init_time) > timeout) break;
		
		// Whenever a byte is available, read it into response buffer
		if (serial_available(&(_wifi_usart)) > 0) {
			rx_buffer[rcvd_bytes++] = serial_read(&(_wifi_usart));
						
			// Avoid overflowing the buffer
			if (rcvd_bytes >= (max_len-1)) break;
		}
	}
	
	wifi_printf("LOG -> RX : %s", rx_buffer );
	
	return (rcvd_bytes);
}


// Send Byte array through serial port
static uint8_t wifi_send_byte_array(uint8_t *buffer, uint16_t length)
{
	for (uint16_t i = 0; i < length; i++) {
		serial_print_c(&_wifi_usart, buffer[i]);
	}
	
	  wifi_pkt_printf("LOG -> Packet Tx : \r\n");
	  for( uint16_t i = 0 ; i < length; i++ )
	  wifi_pkt_printf("%02x", buffer[i] );
	  wifi_pkt_printf("\r\n");
	
	return (NO_ERROR);
}

// Receive Byte array through serial port
static uint8_t wifi_receive_byte_array(uint8_t *buffer, uint16_t length)
{
	uint32_t init_time = time_get();
	uint16_t bytes_read = 0;
	
	while ((bytes_read < length) && (time_get() < init_time + WIFI_RX_TIMEOUT)) {
		if (serial_available(&_wifi_usart) > 0) {
			// Store received byte in reception buffer
			buffer[bytes_read++] = serial_read(&_wifi_usart);
			// Restart timeout with every received byte
			init_time = time_get();
		}
	}
	
#ifdef USB_WIFI_PACKET_LOG
	
	if( bytes_read )
	{
	  wifi_pkt_printf("LOG -> Packet Rx : \r\n");
      for( uint16_t i = 0 ; i < bytes_read; i++ )
	    wifi_pkt_printf("%02x", buffer[i] );
	  wifi_pkt_printf("\r\n");
	}
#endif
	
	if (bytes_read == length) return (NO_ERROR);
	
	return (ERROR);
}

static void wifi_usart_start(void)
{
	irqflags_t flags = cpu_irq_save();
	
	// Set communication port in active mode since it is normally 
	// off to save power
	gpio_set_direction(GPIO_TX_USART_WIFI, IOPORT_DIR_OUTPUT);
	gpio_set_level(GPIO_TX_USART_WIFI, IOPORT_PIN_LEVEL_HIGH);
	gpio_set_direction(GPIO_RX_USART_WIFI, IOPORT_DIR_INPUT);
	
	// Start USART service
	serial_begin(&_wifi_usart);
	cpu_irq_restore(flags);
}

static void wifi_usart_stop(void)
{
	irqflags_t flags = cpu_irq_save();
	
	// Stop USART service
	serial_stop(&_wifi_usart);
	
	// Set communication port in low power mode
	gpio_set_direction(GPIO_TX_USART_WIFI, IOPORT_DIR_OUTPUT);
	gpio_set_level(GPIO_TX_USART_WIFI, IOPORT_PIN_LEVEL_HIGH);
	gpio_set_direction(GPIO_RX_USART_WIFI, IOPORT_DIR_INPUT);
	cpu_irq_restore(flags);
}

/*****************************************************************************************
 * @brief
 *    ISR for the serial peripheral we rely on.
 *****************************************************************************************/
ISR(USART_RX_WIFI_ISR_VECT)
{
  serial_RXComplete(&(_wifi_usart));
}
#endif /* CONF_BOARD_WIFI */

/* -------------------------- Public Functions ------------------------------*/

/*****************************************************************************************
 * @brief
 *    Initializes the modem and its structures.
 *
 * @note
 *    This function must be called before performing any other operation with the
 *    library. It initializes the abstract data types.
 *****************************************************************************************/
void wifi_init(void) {
#ifdef CONF_BOARD_WIFI
	wifi_gpio_init();

	// USART Configuration
	_wifi_usart.usart        = USART_WIFI;
	_wifi_usart.rxIntLevel   = USART_WIFI_INT_LVL;
	_wifi_usart.baudrate     = USART_WIFI_BAUDRATE;
	_wifi_usart.charlength   = USART_WIFI_CHAR_LENGTH;
	_wifi_usart.paritytype   = USART_WIFI_PARITY;
	_wifi_usart.stopbits     = USART_WIFI_STOP_BIT;
	_wifi_usart.flow_control = USART_WIFI_FLOW_CONTROL_TYPE;
	
#endif
}

/*****************************************************************************************
 * @brief
 *    Tests the modem command interface.
 *
 * @note
 *    
 *    
 *****************************************************************************************/
void wifi_test_command(char* cmd)
{
#ifdef CONF_BOARD_WIFI	
	char buffer[RX_BUFLEN];

	wifi_usart_start();

    wifi_send_AT("","+%s\r\n", cmd);

    // Wait for expected response
    wifi_rcv_AT_resp(buffer, OK_RESPONSE, ERROR_RESPONSE, RX_BUFLEN, WIFI_MIN_SERIAL_TIMEOUT);
    if (kstrstr(buffer, OK_RESPONSE) != NULL) {
		wifi_printf("Wifi Command OK Rx: %s \r\n", buffer);
    }
	else{
		wifi_printf("Wifi Resp Rx: %s \r\n", buffer);
	}
	
	wifi_usart_stop();
#endif	
}
