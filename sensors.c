/*
 * sensors.c
 *
 *  Created on: 01.03.2015
 *      Author: a_pol@hotbox.ru
 */

#include "ch.h"
#include "hal.h"

#include "main.h"
#include "sensors.h"
#include "modbus_slave.h"

#define SENSORS_TIMEOUT_MS		2				// задержка опроса датчиков, mS
#define ADC_TIMEOUT_US			100				// задержка опроса датчиков, uS

static volatile adc_error_t adcerror;			// флаг преобразования ADC
uint16_t samples[SENSORSALL];					// описание датчиков
static binary_semaphore_t adcsem;				// семафор доступа к ADC
static binary_semaphore_t adc_cbsem;			// семафор чтения ADC

volatile uint16_t adc_tempint;
volatile uint16_t adc_vrefint;

#define ADCRead_PRIO			(NORMALPRIO+1)
static THD_WORKING_AREA(waADCReadThread, 256);

void adccallback(ADCDriver *adcp, adcsample_t *buffer, size_t n);
void adcerrcallback(ADCDriver *adcp, adcerror_t err);

/*
 * ADC conversion group.
 */
#define ADC_SAMPLE ADC_SAMPLE_239P5
static const ADCConversionGroup adcgrpcfg = {
  FALSE,			//circular
  SENSORSALL,		//number of channels
  adccallback,		//adc callback function
  adcerrcallback,	//error callback function
  /* HW dependent part.*/
  0,				//cr1
  ADC_CR2_TSVREFE,	//cr2
  //SMPR1 register
  ADC_SMPR1_SMP_VREF(ADC_SAMPLE) |
  ADC_SMPR1_SMP_SENSOR(ADC_SAMPLE) |
  0,
  //SMPR2 register
  0,
  //SQR1 register
  ADC_SQR1_NUM_CH(SENSORSALL),
  //SQR2 register
  0,
  //SQR3 register
  ADC_SQR3_SQ2_N(ADC_CHANNEL_VREFINT) |
  ADC_SQR3_SQ1_N(ADC_CHANNEL_SENSOR) |
  0,
};


/*
 * ADC end conversion callback
 */
void adccallback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {
	(void) buffer;
	(void) n;
	if (adcp->state == ADC_COMPLETE){
		chSysLockFromISR();
		chBSemSignalI(&adc_cbsem);
		chSysUnlockFromISR();
	}
}

void adcerrcallback(ADCDriver *adcp, adcerror_t err) {
	(void)adcp;
	(void)err;
	if (adcp->state == ADC_ERROR){
		adcerror = ADC_CONV_ERROR;
	}
	chSysLockFromISR();
	chBSemSignalI(&adc_cbsem);
	chSysUnlockFromISR();
}

/*
 *  ADC read process
 */
static thread_t *ADCReadThread_p;
static THD_FUNCTION(ADCReadThread, arg) {
	(void)arg;

	chRegSetThreadName("ADCRead");
	chBSemObjectInit(&adc_cbsem,TRUE);

	while (TRUE) {
		msg_t req;
		thread_t *tp;

		tp = chMsgWait();
		req = chMsgGet(tp);
		chMsgRelease(tp, (msg_t) req);

		adcerror = ADC_NO_ERROR;
		chBSemReset(&adc_cbsem,TRUE);
		adcStartConversion(&ADCD1, &adcgrpcfg, samples, 1);
		if (chBSemWaitTimeout(&adc_cbsem, TIME_US2I(ADC_TIMEOUT_US)) == MSG_TIMEOUT) {
			adcerror = ADC_TIMEOUT;
		}
		if (adcerror == ADC_NO_ERROR) {
			adc_tempint = samples[TEMPINT];
			adc_vrefint = samples[VREFINT];
		}
		chBSemSignal(&adcsem);
	}
}

/*
 * Initializes the ADC driver 1.
 */
void sensors_init(void){

	adcStart(&ADCD1, NULL);

	chBSemObjectInit(&adcsem,FALSE);
	ADCReadThread_p = chThdCreateStatic(waADCReadThread, sizeof(waADCReadThread), ADCRead_PRIO, ADCReadThread, NULL);
}

adc_error_t sensors_read(void) {
	msg_t *adc_read_p = 0;

	chBSemWait(&adcsem); /* to be sure */

	chMsgSend(ADCReadThread_p, (msg_t) adc_read_p);

	/* wait for reply */
	if(chBSemWaitTimeout(&adcsem, TIME_MS2I(SENSORS_TIMEOUT_MS)) == MSG_TIMEOUT) {
		return ADC_TIMEOUT;
	}
	chBSemReset(&adcsem, FALSE);

	return adcerror;
}
