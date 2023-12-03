// eeprom.c
// PJ 2023-12-03

#include <xc.h>
#include <avr/pgmspace.h>
#include <avr/cpufunc.h>
#include <stdint.h>

void eeprom_write_byte(uint16_t addr, uint8_t data)
{
    // Data sheet recommends NoOp to avoid collisions.
    _PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_NOOP_gc);
    // Write byte to page buffer.
    *(uint8_t *)(EEPROM_START + addr) = data;
    _PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_EEPERW_gc);
    if (NVMCTRL.STATUS & NVMCTRL_ERROR_gm) {
        NVMCTRL.STATUS &= ~NVMCTRL_ERROR_gm;
    }
    while (NVMCTRL.STATUS & NVMCTRL_EEBUSY_bm) {
        /* [TODO] clr wdt */
    }
}

uint8_t eeprom_read_byte(uint16_t addr)
{
    return *(uint8_t *)(EEPROM_START + addr);
}
