#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"
#include "doom.h"

/************************************************************************/
/* pins  config                                                          */
/************************************************************************/
/* BUZZER */
#define BUZZER_PIO           PIOC
#define BUZZER_PIO_ID        ID_PIOC
#define BUZZER_PIO_IDX       13
#define BUZZER_PIO_IDX_MASK  (1u << BUZZER_PIO_IDX)

/* LED 1 da placa OLED */
#define LED1_PIO          PIOA
#define LED1_PIO_ID       ID_PIOA
#define LED1_PIO_IDX      0
#define LED1_PIO_IDX_MASK  (1 << LED1_PIO_IDX)

/* LED 2 da placa OLED */
#define LED2_PIO          PIOC
#define LED2_PIO_ID       ID_PIOC
#define LED2_PIO_IDX      30
#define LED2_PIO_IDX_MASK  (1 << LED2_PIO_IDX)

/* LED 3 da placa OLED */
#define LED3_PIO          PIOB
#define LED3_PIO_ID       ID_PIOB
#define LED3_PIO_IDX      2
#define LED3_PIO_IDX_MASK  (1 << LED3_PIO_IDX)

/* Botao 1 da placa OLED */
#define BUT1_PIO          PIOD
#define BUT1_PIO_ID       ID_PIOD
#define BUT1_PIO_IDX      28
#define BUT1_PIO_IDX_MASK (1 << BUT1_PIO_IDX)

/* Botao 2 da placa OLED */
#define BUT2_PIO      PIOC
#define BUT2_PIO_ID   ID_PIOC
#define BUT2_IDX      31
#define BUT2_PIO_IDX_MASK (1 << BUT2_IDX)

/* Botao 3 da placa OLED */
#define BUT3_PIO      PIOA
#define BUT3_PIO_ID   ID_PIOA
#define BUT3_IDX      19
#define BUT3_PIO_IDX_MASK (1 << BUT3_IDX)
/************************************************************************/
/* globals                                                              */
/************************************************************************/

/************************************************************************/
/* prototypes and types                                                 */
/************************************************************************/

void pin_toggle(Pio *pio, uint32_t mask);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq) ;
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
void io_init(void);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) { }
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

typedef struct {
	uint Note;
	uint Duration;
} Musica;

SemaphoreHandle_t xSemaphoreStart;
SemaphoreHandle_t xSemaphoreRTT;
QueueHandle_t xQueueNote;

/************************************************************************/
/* recursos RTOS                                                        */
/************************************************************************/

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but1_callback(void) {
	int x1 = 1;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	//xQueueSendFromISR(xSemaphoreStart, &x1, &xHigherPriorityTaskWoken);
	xSemaphoreGiveFromISR(xSemaphoreStart, &xHigherPriorityTaskWoken);
}

void TC0_Handler(void) {
	/* A leitura do periferico informa que a interrupcao foi satisfeita */
	volatile uint32_t status = tc_get_status(TC0, 0);
	int pulsos = 1000;//(double) 1000* (double) 100;
	for (int i = 0; i < pulsos; i++) {
		pin_toggle(BUZZER_PIO, BUZZER_PIO_IDX_MASK);
	}
	//RTT_init(100,1000, RTT_MR_ALMIEN);
}

void RTT_Handler(void) {
	uint32_t ul_status;
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		//BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		//xSemaphoreGiveFromISR(xSemaphoreRTT,xHigherPriorityTaskWoken);
		//xQueueSendFromISR(xQueueCompressor, &modo, 0);
		BaseType_t xHigherPriorityTaskWoken = pdTRUE;
		xSemaphoreGiveFromISR(xSemaphoreRTT, &xHigherPriorityTaskWoken);
	}
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_music(void *pvParameters) {
	io_init();
	
	int divider;
	int noteDuration;
	Musica musica;
	for (;;) {
		if(xSemaphoreTake(xSemaphoreStart, 0)) {
			for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {
				// calculates the duration of each note
				divider = melody[thisNote + 1];
				if (divider > 0) {
					// regular note, just proceed
					noteDuration = (wholenote) / divider;
					} else if (divider < 0) {
					// dotted notes are represented with negative durations!!
					noteDuration = (wholenote) / abs(divider);
					noteDuration *= 1.5; // increases the duration in half for dotted notes
				}
			
				musica.Note = melody[thisNote];
				musica.Duration = noteDuration;
				xQueueSend(xQueueNote, (void *)&musica, 10);
			}
		}
	}
}

static void task_play(void *pvParameters) {
	io_init();
	Musica dado;
	
	for (;;) {
		if (xQueueReceive(xQueueNote, &dado, 0)) {
			printf("%d",dado.Note);
			TC_init(TC0, ID_TC0, 0, 4*dado.Note);
			tc_start(TC0, 0);
			RTT_init(1000,dado.Duration, RTT_MR_ALMIEN);
		}
		
		if(xSemaphoreTake(xSemaphoreRTT, 1000)) {
			tc_stop(TC0, 0);
		}
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void io_init(void) {
	pmc_enable_periph_clk(BUZZER_PIO_ID);
	
	pmc_enable_periph_clk(BUT1_PIO_ID);

	pio_configure(BUZZER_PIO, PIO_OUTPUT_0, BUZZER_PIO_IDX_MASK, PIO_DEFAULT);
	
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);
	
	pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_IDX_MASK, 40);
	
	pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but1_callback);

	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);

	pio_get_interrupt_status(BUT1_PIO);

	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 4);
}

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);

	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);

	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);

}

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq) {
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);

	/** ATIVA clock canal 0 TC */
	if(ul_tcclks == 0 )
		pmc_enable_pck(PMC_PCK_6);

	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura NVIC*/
	NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

void pin_toggle(Pio *pio, uint32_t mask) {
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/
int main(void) {
	sysclk_init();
	board_init();
	configure_console();

	/* Create task to control oled */
	if (xTaskCreate(task_music, "music", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create game task\r\n");
	}
	
	/* Create task to control oled */
	if (xTaskCreate(task_play, "game", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create game task\r\n");
	}
	
	xQueueNote = xQueueCreate(100, sizeof(Musica));
	if (xQueueNote == NULL)
	printf("falha em criar a queue AFEC \n");
	
	xSemaphoreRTT = xSemaphoreCreateBinary();
	xSemaphoreStart = xSemaphoreCreateBinary();

	vTaskStartScheduler();

	while(1){}

	return 0;
}
