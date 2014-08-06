#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/builtins.h>
#include "fifo.h"
// ��������� FIFO ������ ��� ���������� � �������� ������ � UART
static FIFO(128) uart_rx_fifo;
static FIFO(128) uart_tx_fifo;


/*!
 * @brief ������ ���������� �� ������ �����
 */
ISR( USART0_RX_vect )
{
  unsigned char rxbyte = UDR0;
  if( !FIFO_IS_FULL( uart_rx_fifo ) ) {
    FIFO_PUSH( uart_rx_fifo, rxbyte );
  }
}

/*!
 * @brief ������ ���������� �� ������� �������� �������� ����� (���� ���������)
 */
ISR( USART0_UDRE_vect )
{
  if( FIFO_IS_EMPTY( uart_tx_fifo ) ) {
    //���� ������ � fifo ������ ��� �� ��������� ��� ����������
    UCSR0B &= ~( 1 << UDRIE0 );
  }
  else {
    //����� �������� ��������� ����
    char txbyte = FIFO_FRONT( uart_tx_fifo );
    FIFO_POP( uart_tx_fifo );
    UDR0 = txbyte;
  }
}

/*!
 * @brief ���������� �������� ����� � ���� ��� ������������ ������
 */
int uart_putc(  char c, FILE *file )
{
  int ret;
  __builtin_avr_cli(); //��������� ����������
  if( !FIFO_IS_FULL( uart_tx_fifo ) ) {
    //���� � ������ ���� �����, �� ��������� ���� ����
    FIFO_PUSH( uart_tx_fifo, c );
    //� ��������� ���������� �� ������������ �����������
    UCSR0B |= ( 1 << UDRIE0 );
    ret = 0;
  }
  else {
    ret = -1; //����� ����������
  }
  __builtin_avr_sei(); //��������� ����������
  return ret;
}

/*!
 * @brief ���������� ������ ������ �� ������������ �����
 * @warning ������� ����� ������� ��������� ������� �� UART
 */
int uart_getc( FILE* file )
{
  int ret;
  //���� ���� �������� ������ � FIFO, ���� ��� ������ ���
  while(FIFO_IS_EMPTY( uart_rx_fifo ) );
  __builtin_avr_cli(); //��������� ����������
  ret = FIFO_FRONT( uart_rx_fifo );
  FIFO_POP( uart_rx_fifo );
  __builtin_avr_sei(); //��������� ����������
  return ret;
}

/*!
 * @brief ������������� ���������� ����� UART
 */
void uart0_hw_init()
{
  #define UART_CALC_BR( br ) ( ( uint16_t )( ( F_CPU / ( 16UL * (br) ) ) - 1 ) )
  uint16_t br = UART_CALC_BR(9600);
  //��������� �������� ������
  UBRR0H = br >> 8;
  UBRR0L = br & 0xFF;
  //8 ��� ������, 1 ���� ���, ��� �������� ��������
  UCSR0C = ( 1 << USBS0 ) | ( 1 << UCSZ01 ) | ( 1 << UCSZ00 );
  //��������� ����� � �������� ������
  UCSR0B = ( 1 << TXEN0 ) | ( 1 << RXEN0 ) | (1 << RXCIE0 );
}

//��������� ����� �����/������, ������� ����� ������������ ��� ��������������� stdio
static FILE uart_stream = FDEV_SETUP_STREAM(uart_putc, uart_getc, _FDEV_SETUP_RW);

void init_uart0()
{
    uart0_hw_init();
    stdout = stdin = &uart_stream;
}
