/*
 */

#include <avr/io.h>
#include <avr/builtins.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include "timer1.h"
#include "discr_io.h"
#include "uart0.h"
#include "command.h"

void user_timer_ISR()
{
    discr_io_ISR();
}

void init_hw()
{
    //��������� ���������� ����������� ��� ��������� ��������� ���������
    __builtin_avr_cli();
    MCUCR |= ( 1 << IVCE );
    MCUCR &= ~(( 1 << IVSEL ) | (1 << IVCE)); // ... 2.Within four cycles, write the desired value to IVSEL while writing a zero to IVCE.

    //����������� ��������� ������� �������� ������������
    init_discr_io();

    //�������������� ������
    timer1_init();

    //��������� UART0
    init_uart0();

    //�������� ���������� �����������
    __builtin_avr_sei();
}

int main(void)
{
    //����������� ������
    init_hw();

    printf_P(PSTR("\r\nStarted...\r\n"));

    while(1)
    {
        process_command();
        continue;
    }

    return 0;
}
