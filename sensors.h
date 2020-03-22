/*
 * sensors.h
 */
#ifndef SENSORS_H_
#define SENSORS_H_

#define SENSORSALL			2
#define TEMPINT				0		// номер датчика контроля внутр.температуры
#define VREFINT				1		// номер датчика контроля VREFINT

#define V25					(1.43)		// V
#define AVG_SLOPE			(0.0043)	// V/C
#define VREF_INT			(1200)		// V * 1000
#define TEMPCALC(d,v)		((V25 - d * VREF_INT / v / 1000.0) / AVG_SLOPE + 25.0)

typedef enum {
	ADC_NO_ERROR,
	ADC_CONV_ERROR,
	ADC_TIMEOUT,
} adc_error_t;

extern volatile uint16_t adc_tempint;
extern volatile uint16_t adc_vrefint;

#ifdef __cplusplus
extern "C" {
#endif

void sensors_init(void);
adc_error_t sensors_read(void);

#ifdef __cplusplus
}
#endif
#endif /* SENSORS_H_ */
