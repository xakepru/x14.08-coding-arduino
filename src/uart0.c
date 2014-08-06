#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/builtins.h>
#include "fifo.h"
// объявляем FIFO буферы для считывания и отправки данных в UART
static FIFO(128) uart_rx_fifo;
static FIFO(128) uart_tx_fifo;


/*!
 * @brief Вектор прерывания по приему байта
 */
ISR( USART0_RX_vect )
{
  unsigned char rxbyte = UDR0;
  if( !FIFO_IS_FULL( uart_rx_fifo ) ) {
    FIFO_PUSH( uart_rx_fifo, rxbyte );
  }
}

/*!
 * @brief Вектор прерывания по очистке регистра отправки байта (байт отправлен)
 */
ISR( USART0_UDRE_vect )
{
  if( FIFO_IS_EMPTY( uart_tx_fifo ) ) {
    //если данных в fifo больше нет то запрещаем это прерывание
    UCSR0B &= ~( 1 << UDRIE0 );
  }
  else {
    //иначе передаем следующий байт
    char txbyte = FIFO_FRONT( uart_tx_fifo );
    FIFO_POP( uart_tx_fifo );
    UDR0 = txbyte;
  }
}

/*!
 * @brief обработчик отправки байта в порт для стандартного вывода
 */
int uart_putc(  char c, FILE *file )
{
  int ret;
  __builtin_avr_cli(); //запрещаем прерывания
  if( !FIFO_IS_FULL( uart_tx_fifo ) ) {
    //если в буфере есть место, то добавляем туда байт
    FIFO_PUSH( uart_tx_fifo, c );
    //и разрешаем прерывание по освобождению передатчика
    UCSR0B |= ( 1 << UDRIE0 );
    ret = 0;
  }
  else {
    ret = -1; //буфер переполнен
  }
  __builtin_avr_sei(); //разрешаем прерывания
  return ret;
}

/*!
 * @brief обработчик чтения данных из стандартного ввода
 * @warning функция будет ожидать получения символа из UART
 */
int uart_getc( FILE* file )
{
  int ret;
  //ждем пока появятся данные в FIFO, если там ничего нет
  while(FIFO_IS_EMPTY( uart_rx_fifo ) );
  __builtin_avr_cli(); //запрещаем прерывания
  ret = FIFO_FRONT( uart_rx_fifo );
  FIFO_POP( uart_rx_fifo );
  __builtin_avr_sei(); //разрешаем прерывания
  return ret;
}

/*!
 * @brief инициализация аппаратной части UART
 */
void uart0_hw_init()
{
  #define UART_CALC_BR( br ) ( ( uint16_t )( ( F_CPU / ( 16UL * (br) ) ) - 1 ) )
  uint16_t br = UART_CALC_BR(9600);
  //настройка скорости обмена
  UBRR0H = br >> 8;
  UBRR0L = br & 0xFF;
  //8 бит данных, 1 стоп бит, без контроля четности
  UCSR0C = ( 1 << USBS0 ) | ( 1 << UCSZ01 ) | ( 1 << UCSZ00 );
  //разрешить прием и передачу данных
  UCSR0B = ( 1 << TXEN0 ) | ( 1 << RXEN0 ) | (1 << RXCIE0 );
}

//объявляем поток ввода/вывода, который будем использовать для перенаправления stdio
static FILE uart_stream = FDEV_SETUP_STREAM(uart_putc, uart_getc, _FDEV_SETUP_RW);

void init_uart0()
{
    uart0_hw_init();
    stdout = stdin = &uart_stream;
}
