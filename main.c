/*
 * main.c
 *
 * 2 channel MODBUS switch
 *
 *  Created on: 24.05.2019
 *      Author: andru
 */

#include <string.h>

#include "ch.h"
#include "hal.h"

#include "main.h"
#include "crc8.h"
#include "modbus_slave.h"
#include "sensors.h"
#include "hal_flash_lld.h"
#include "stm32_flash.h"

#define SWITCH_POLL		250	// mS
#define LineRead(l)		palReadLine(l)
#define LineSet(l, n)	((n) ? palSetLine(l) : palClearLine(l))

#define LED		PAL_LINE(GPIOB, GPIOB_LED)
#define IN1		PAL_LINE(GPIOB, GPIOB_IN1)
#define IN2		PAL_LINE(GPIOB, GPIOB_IN2)
#define OUT1	PAL_LINE(GPIOB, GPIOB_OUT1)
#define OUT2	PAL_LINE(GPIOB, GPIOB_OUT2)

volatile thd_check_t thd_state;		// thread state mask
CONFIG_T config;					// configuration data
event_source_t event_src;			// event thread event sources

volatile uint8_t in1, in2;
volatile uint8_t out1, out2;
volatile bool cfg_write = false;
volatile uint16_t timer1, timer2;

/*
 * Watchdog deadline set to 2000 / (LSI=40000 / 64) = ~3.2s.
 */
static const WDGConfig wdgcfg = {
  STM32_IWDG_PR_64,
  STM32_IWDG_RL(2000),
};

void halt(void);
static void modbus_coil(void);
static void modbus_hold(void);

/*===========================================================================*/
/* Main and generic code.                                                    */
/*===========================================================================*/

static THD_WORKING_AREA(waSwitchThread, 256);
static THD_FUNCTION(SwitchThread, arg) {
	(void)arg;

	chRegSetThreadName("Switch");
	while (true) {
		uint8_t state;

		thd_state |= THD_SWITCH;

		state = LineRead(IN1);
		if (in1 != state) {
			in1 = state;
			setDiscBit(MB_DI_IN1, in1);
			if (timer1 > 0) {
				timer1 = 0;
				writeHoldingReg(MB_HO_TIMER1, timer1);
			}
			if (config.in1_on)
				chEvtBroadcastFlags(&event_src, EVT_IN1);
		}
		state = LineRead(IN2);
		if (in2 != state) {
			in2 = state;
			setDiscBit(MB_DI_IN2, in2);
			if (timer2 > 0) {
				timer2 = 0;
				writeHoldingReg(MB_HO_TIMER2, timer2);
			}
			if (config.in2_on)
				chEvtBroadcastFlags(&event_src, EVT_IN2);
		}
		chThdSleepMilliseconds(SWITCH_POLL);
	}
}

static THD_WORKING_AREA(waEventThread, 512);
static THD_FUNCTION(eventThread, arg) {
	(void)arg;

	event_listener_t event_el;

	chRegSetThreadName("Event");
	chEvtObjectInit(&event_src);
	chEvtRegisterMask(&event_src, &event_el, ALL_EVENTS);

	while (true) {
		chEvtWaitAny(ALL_EVENTS);
		eventflags_t flags = chEvtGetAndClearFlags(&event_el);

		if (flags & EVT_TEST)
		    thd_state |= THD_EVENT;

		if ((flags & EVT_IN1) &&  (out1 != in1)) {
	    	LineSet(OUT1, in1);
	    	out1 = in1;
			setDiscBit(MB_DI_OUT1, out1);
			setCoilBit(MB_CO_OUT1, out1);
	    }

	    if ((flags & EVT_IN2) &&  (out2 != in2)) {
	    	LineSet(OUT2, in2);
	    	out2 = in2;
			setDiscBit(MB_DI_OUT2, out2);
			setCoilBit(MB_CO_OUT2, out2);
	    }

	    // Modbus coil write event
	    if (flags & EVT_MBCOIL) {
	    	modbus_coil();
	    } //EVT_MBCOIL

	    // Modbus holding write event
	    if (flags & EVT_MBHOLD) {
	    	modbus_hold();
	    } //EVT_MBHOLD

	} //while
}

static void default_config(void) {
  config.mb_addr = MB_ADDRESS;
  config.mb_bitrate = MB_BITRATE;
  config.mb_parity = MB_PARITY_NONE;
  config.in1_on = config.in2_on = true;
}

static void modbus_coil(void) {
	uint8_t bit, write = false;

	bit = getCoilBit(MB_CO_OUT1) != 0;
	if (out1 != bit) {
		out1 = bit;
		LineSet(OUT1, out1);
		setDiscBit(MB_DI_OUT1, out1);
		setCoilBit(MB_CO_OUT1, out1);
		timer1 = 0;
		if (out1)
			timer1 = readHoldingReg(MB_HO_TIMER1);
		writeHoldingReg(MB_HO_TIMER1, timer1);
	}

	bit = getCoilBit(MB_CO_OUT2) != 0;
	if (out2 != bit) {
		out2 = bit;
		LineSet(OUT2, out2);
		setDiscBit(MB_DI_OUT2, out2);
		setCoilBit(MB_CO_OUT2, out2);
		timer2 = 0;
		if (out2)
			timer2 = readHoldingReg(MB_HO_TIMER2);
		writeHoldingReg(MB_HO_TIMER2, timer2);
	}

	if (getCoilBit(MB_CO_MBADDR)) {
		setCoilBit(MB_CO_MBADDR, OFF);
		uint16_t reg = readHoldingReg(MB_HO_MBADDR);
		if ((reg != config.mb_addr) && (reg > 0) && (reg < 247)) {
			config.mb_addr = reg;
			write = true;
		}
	}

	if (getCoilBit(MB_CO_MBCONF)) {
		setCoilBit(MB_CO_MBCONF, OFF);
		uint16_t reg = readHoldingReg(MB_HO_MBCONF);
		uint8_t bitrate = reg & 0xFF;
		uint8_t parity = reg >> 8;
		if ((bitrate != config.mb_bitrate) && (bitrate >= MB_BITRATE_1200) && (bitrate <= MB_BITRATE_115200)) {
			config.mb_bitrate = bitrate;
			write = true;
		}
		if ((parity != config.mb_parity) && (parity <= MB_PARITY_ODD)) {
			config.mb_parity = parity;
			write = true;
		}
	}

	bit = getCoilBit(MB_CO_IN1) != 0;
	if (config.in1_on != bit) {
		config.in1_on = bit;
		write = true;
	}

	bit = getCoilBit(MB_CO_IN2) != 0;
	if (config.in2_on != bit) {
		config.in2_on = bit;
		write = true;
	}

	if (write) cfg_write = true;
}

static void modbus_hold(void) {
	uint16_t val;

	val = readHoldingReg(MB_HO_TIMER1);
	if (out1 && val > 0) {
		timer1 = val;
		writeInputReg(MB_IN_TIMER1, timer1);
	}

	val = readHoldingReg(MB_HO_TIMER2);
	if (out2 && val > 0) {
		timer2 = val;
		writeInputReg(MB_IN_TIMER2, timer2);
	}
}

// called on kernel panic
void halt(void) {
  port_disable();
  while (true)	{
	palToggleLine(LED);
	chThdSleepMilliseconds(250);
  }
}

/*
 * Application entry point.
 */
int main(void) {
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  // PB3 - JTDO / TRACESWO, PB4 - NJTRST need to be re-mapped
  // 010: JTAG-DP Disabled and SW-DP Enabled, RM008m page 171
  AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_1;

  palSetLineMode(LED, PAL_MODE_OUTPUT_PUSHPULL);

  palSetLineMode(IN1, PAL_MODE_INPUT);
  palSetLineMode(OUT1, PAL_MODE_OUTPUT_PUSHPULL);
  in1 = LineRead(IN1);
  out1 = in1;
  LineSet(OUT1, out1);

  palSetLineMode(IN2, PAL_MODE_INPUT);
  palSetLineMode(OUT2, PAL_MODE_OUTPUT_PUSHPULL);
  in2 = LineRead(IN2);
  out2 = in2;
  LineSet(OUT2, out2);

  wdgStart(&WDGD1, &wdgcfg);

  if (!initFlashInfo()) {
	  halt();
  }

  // firmware & config size changed
#if FLASHERASE
  if (!flashErase()) {
	  halt();
  }
  if (!initFlashInfo()) {
	  halt();
  }
#endif

  // read config from FLASH
  if (!readFlash((uint8_t *) &config)) {
	  setDiscBit(MB_DI_FLASH, ON);
	  default_config();
	  if (!writeFlash((uint8_t *) &config)) {
		  halt();
	  }
  }

  writeInputReg(MB_IN_FIRMWARE, FIRMWARE);

  setDiscBit(MB_DI_IN1, in1);
  setDiscBit(MB_DI_OUT1, out1);
  setCoilBit(MB_CO_OUT1, out1);
  setDiscBit(MB_DI_IN2, in2);
  setDiscBit(MB_DI_OUT2, out2);
  setCoilBit(MB_CO_OUT2, out2);

  setCoilBit(MB_CO_IN1, config.in1_on);
  setCoilBit(MB_CO_IN2, config.in2_on);
  writeHoldingReg(MB_HO_MBADDR, config.mb_addr);
  uint16_t reg = ((uint16_t) config.mb_parity << 8) | config.mb_bitrate;
  writeHoldingReg(MB_HO_MBCONF, reg);

  timer1 = timer2 = 0;
  writeInputReg(MB_IN_TIMER1, timer1);
  writeInputReg(MB_IN_TIMER2, timer2);
  writeHoldingReg(MB_HO_TIMER1, timer1);
  writeHoldingReg(MB_HO_TIMER2, timer2);

  // event thread
  chThdCreateStatic(waEventThread, sizeof(waEventThread), NORMALPRIO+1, eventThread, NULL);

  // switch polling thread
  chThdCreateStatic(waSwitchThread, sizeof(waSwitchThread), NORMALPRIO, SwitchThread, NULL);

  // Creates the MODBUS thread.
  modbus_init();

  // ADC read thread
  sensors_init();

  uint16_t dummy=0;
  systime_t time = chVTGetSystemTime();
  while (TRUE) {
	time += TIME_MS2I(1000);

    palToggleLine(LED);

	chEvtBroadcastFlags(&event_src, EVT_TEST);

	writeInputReg(MB_IN_COUNTER, dummy++);

    thd_state |= THD_MAIN;
    if (thd_state == THD_GOOD) {
    	wdgReset(&WDGD1);
    }

    if (timer1 > 0) {
    	if (--timer1 == 0) {
    		out1 = 0;
    		LineSet(OUT1, out1);
    		setDiscBit(MB_DI_OUT1, out1);
    		setCoilBit(MB_CO_OUT1, out1);
    	}
    }
	writeInputReg(MB_IN_TIMER1, timer1);

    if (timer2 > 0) {
    	if (--timer2 == 0) {
    		out2 = 0;
    		LineSet(OUT2, out2);
    		setDiscBit(MB_DI_OUT2, out2);
    		setCoilBit(MB_CO_OUT2, out2);
    	}
    }
	writeInputReg(MB_IN_TIMER2, timer2);

    // ADC read
    if (sensors_read() == ADC_NO_ERROR) {
    	// mcu internal temperature calculation
    	float temp = TEMPCALC(adc_tempint, adc_vrefint);
    	writeInputReg(MB_IN_TEMPERATURE, 10 * temp);
		setDiscBit(MB_DI_TEMP, OFF);
    } else {
    	setDiscBit(MB_DI_TEMP, ON);
    }

    if (cfg_write) {
    	cfg_write = false;
    	setDiscBit(MB_DI_FLASH, OFF);
    	if (!writeFlash((uint8_t *) &config))
   		  setDiscBit(MB_DI_FLASH, ON);
    }

    chThdSleepUntil(time);
  }
}
