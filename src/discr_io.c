#include <avr/io.h>
#include <avr/builtins.h>
#include <avr/eeprom.h>
#include "discr_io.h"
#include <string.h>

//��������� EEPROM
//������������ ��������� ��������� �������:
//<��������� �������� ��� �����>
//<�������� ������ ��������� ��� ����� 1 ���� (0 - ��������, �� ��������� - �� ��������)>
//<��������� �������� ��� ������>
//<�������� ������ ��������� ��� ������ 1 ���� (0 - ��������, �� ��������� - �� ��������)>
#define EE_ADDR_NORM    ( 0x100 )
#define EE_ADDR_ALARM   ( (EE_ADDR_NORM) + (BLINK_SIZE*sizeof(uint16_t) + 1) )


static uint8_t blink_idx = 0;
static volatile uint16_t blink_delay = 0;

//����������� ������������
static uint16_t blink_seq_norm[BLINK_SIZE]={500, 500, 0};

//S-O-S ���������
static uint16_t blink_seq_alarm[BLINK_SIZE]={200, 100, 200, 100, 200, 100,
                                   500, 100, 500, 100, 500, 100,
                                   200, 100, 200, 100, 200, 500, 0};

static uint16_t *blink_seq = blink_seq_norm;

void blink_led()
{
  // ������� �����������
  if ( blink_seq != 0)
  {
    if ( blink_delay != 0)
      --blink_delay; // ����������� ��������
    else
    {
      if ( blink_seq[ blink_idx ] == 0 ) // ����� ������������������
        blink_idx = 0;
      // ������ ������� ��������, �������� �������
      if (((blink_idx ^ 1) & 0x1) == 1)
        PORTB |= _BV(PB7);
      else
        PORTB &= ~_BV(PB7);

      blink_delay = blink_seq[ blink_idx ];
      ++blink_idx;
    }
  }
}

void clr_led_blink()
{
    uint8_t ee[32];
    uint8_t idx = 0;
    for (idx = 0; idx < 32; ++idx) ee[idx]=0xFF;
    uint16_t addr = EE_ADDR_NORM;
    while (addr < EE_ADDR_ALARM + sizeof(uint16_t)*BLINK_SIZE + sizeof(uint8_t))
    {
        eeprom_write_block (ee, (void*)addr, 32);
        addr += 32;
    }
}

void load_led_blink()
{
    uint8_t ctrl_norm = 0, ctrl_alarm = 0;
    eeprom_busy_wait();
    ctrl_norm = eeprom_read_byte((uint8_t*)(EE_ADDR_NORM + BLINK_SIZE*sizeof(uint16_t) ));
    eeprom_busy_wait();
    ctrl_alarm = eeprom_read_byte((uint8_t*)(EE_ADDR_ALARM + BLINK_SIZE*sizeof(uint16_t)));

    //�������� ����������� �����, ���� ��� �������� - ������ ���������
    __builtin_avr_cli();

    if (ctrl_norm == 0 )
    {
        eeprom_busy_wait();
        eeprom_read_block(blink_seq_norm, (void*)EE_ADDR_NORM, BLINK_SIZE*sizeof(uint16_t));
    }
    if (ctrl_alarm == 0)
    {
        eeprom_busy_wait();
        eeprom_read_block(blink_seq_alarm, (void*)EE_ADDR_ALARM, BLINK_SIZE*sizeof(uint16_t));
    }
    blink_idx = 0;
    blink_delay = blink_seq[blink_idx];
    ++blink_idx;
    __builtin_avr_sei();
}

void save_led_blink(worker_mode_t mode, uint16_t src[BLINK_SIZE])
{
    uint16_t addr = 0xFFFF;
    switch (mode)
    {
    case wm_normal:
        addr = EE_ADDR_NORM;
        break;
    case wm_alarm:
        addr = EE_ADDR_ALARM;
        break;
    default:
        addr = 0xFFFF;
        break;
    }
    if (addr != 0xFFFF)
    {
        __builtin_avr_cli();

        //�������� ����, ��� �����������, ����� � ������ ������������ �� ����� ������, �� �����,
        //��� ���� ���� ����������
        eeprom_busy_wait();
        eeprom_write_byte((uint8_t*)(addr + BLINK_SIZE*sizeof(uint16_t)), 0x1);

        //����� ���� ��������
        eeprom_busy_wait();
        eeprom_write_block(src, (void*)addr, BLINK_SIZE*sizeof(uint16_t));

        //�������� ����, ��� ���������
        eeprom_busy_wait();
        eeprom_write_byte((uint8_t*)(addr + BLINK_SIZE*sizeof(uint16_t)), 0x0);

        __builtin_avr_sei();
    }
}


void ch_blink_mode(worker_mode_t mode)
{
    __builtin_avr_cli();
    switch (mode)
    {
    case wm_normal:
        blink_seq = blink_seq_norm;
        break;
    case wm_alarm:
        blink_seq = blink_seq_alarm;
        break;
    default:
        blink_seq = blink_seq_norm;
        break;
    }
    blink_idx = 0;
    blink_delay = blink_seq[blink_idx];
    ++blink_idx;
    __builtin_avr_sei();
}

int get_led_blink(worker_mode_t mode, uint16_t dst[BLINK_SIZE])
{
    void* addr = NULL;
    switch (mode)
    {
    case wm_normal:
        addr = (void*)blink_seq_norm;
        break;
    case wm_alarm:
        addr = (void*)blink_seq_alarm;
        break;
    default:
        addr = NULL;
        break;
    }
    if (addr != NULL)
    {
        memcpy(dst, addr, sizeof(uint16_t)*BLINK_SIZE);
    }
    return addr == NULL ? 0 : 1;
}

void process_input()
{
    static uint8_t prev_state = 0;
    // ��������� �� ����� ��������� ������ ����������� �����
    uint8_t cur_state = (PINL & _BV(PL6));

    //���������, ���������� �� ���������?
    if (prev_state != cur_state)
    {
        //����������, �������� ������� � ������������ � ����� �������
        if (cur_state == 0)
            ch_blink_mode(wm_alarm);
        else
            ch_blink_mode(wm_normal);

        prev_state = cur_state;
    }
}

void discr_io_ISR()
{
    //������� �����������
    blink_led();

    //����������� ��������� ���������� ������
    process_input();
}

void init_discr_io(void)
{
    //�������� ���������� �������� � "1"
    PORTL |= _BV(PL6);
    //����������� ��� PB6 �� ����
    DDRL &= ~_BV(PL6);


    //�������� �� ����� ��� PB7 ����������
    DDRB |= _BV(PB7);
    PORTB &= ~_BV(PB7);

    //��������� ��������� ��������
    load_led_blink();
}
