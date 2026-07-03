/* =====================================================================
 *  PROJEKAT: Kontrola stanja sigurnosnih pojaseva u automobilu
 *  Fajl:     main_application.c  (jedini fajl koji se ocenjuje / menja)
 *  Platforma: FreeRTOS Windows simulator (Visual Studio, MSVC v142)
 *
 *  Periferije (pokrecu se kao zasebni .exe simulatori):
 *    - UniCom kanal 0  -> senzori (auto-odgovor preko Trigera, svakih 200 ms)
 *    - UniCom kanal 1  -> PC: komande (START/STOP/PRAG_x) + status na 2000 ms
 *    - UniCom kanal 2  -> PC: upozorenja kad neko nije vezan (na ~100 ms)
 *    - LED bar         -> stub 0 ulazni (kontakt), stub 1 i 2 izlazni
 *    - Seg7Mux displej -> osvezavanje na 100 ms
 * ===================================================================== */

 /* STANDARD INCLUDES */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

/* KERNEL INCLUDES */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "extint.h"

/* HARDWARE SIMULATOR UTILITY FUNCTIONS */
#include "HW_access.h"

void main_demo(void);

/* ---------------------------------------------------------------------
 * Definicije kanala
 * ------------------------------------------------------------------- */
#define SENZOR_CH   0U   /* senzori                         */
#define PC_CH1      1U   /* PC: komande + status            */
#define PC_CH2      2U   /* PC: upozorenja                  */

#define MOJ_STEK    (configMINIMAL_STACK_SIZE * 2U)

 /* Podrazumevani prag pritiska (AD 0..1023) */
#define DEFAULT_PRAG        512U

/* Tajming blinkanja LED-ova (period) */
#define BLINK_NORMAL_MS     2000U   /* period 2 s   (manje od 20 s nevezan) */
#define BLINK_FAST_MS       400U    /* period 400 ms (vise od 20 s nevezan) */
#define UNBELT_THRESHOLD_MS 20000U  /* prag od 20 s za brzo blinkanje       */

/* Odziv pri kome se anketira ulazni LED stub kad nema blinkanja */
#define IDLE_POLL_MS        100U

/* MISRA dev: jedna staticka kontrolna promenljiva za beskonacne petlje
 * taskova, da bi se izbegao while(1) (MISRA 14.3 / pravilo iz vezbi).  */
static volatile uint8_t LINT_WHILE = 1U;

/* ---------------------------------------------------------------------
 * Struktura sa podacima senzora
 * ------------------------------------------------------------------- */
typedef struct {
    uint8_t  vozac_pojas;       /* 1 = vezan, 0 = nije vezan        */
    uint8_t  suvozac_pojas;     /* 1 = vezan, 0 = nije vezan        */
    uint16_t suvozac_pritisak;  /* pritisak na sediste (AD 0..1023) */
} SenzorPojasevi;

static volatile SenzorPojasevi TrenutniPodaci = { 0U, 0U, 0U };

/* ---------------------------------------------------------------------
 * Stanje sistema
 * ------------------------------------------------------------------- */
static volatile uint8_t  sistemUkljucen = 0U;
static volatile uint16_t pragPritiska = DEFAULT_PRAG;

/* ---------------------------------------------------------------------
 * 7-SEG kodovi cifara i slova
 * ------------------------------------------------------------------- */
static const uint8_t hexnum[] = {
    0x3FU, 0x06U, 0x5BU, 0x4FU, 0x66U,
    0x6DU, 0x7DU, 0x07U, 0x7FU, 0x6FU
};

#define SEG_BLANK  0x00U   /* prazan segment */
#define SEG_DASH   0x40U   /* crtica "-"      */
#define SEG_F      0x71U   /* F = Fastened (vezan) */
#define SEG_P      0x73U   /* P = prisutan suvozac */

/* ---------------------------------------------------------------------
 * Baferi za prijem sa serijske
 * ------------------------------------------------------------------- */
#define RX_BUFFER_SIZE  64U

static char     sensor_rx_buffer[RX_BUFFER_SIZE];
static uint32_t sensor_rx_index = 0U;

static char     pc1_rx_buffer[RX_BUFFER_SIZE];
static uint32_t pc1_rx_index = 0U;

/* ---------------------------------------------------------------------
 * FreeRTOS objekti
 * ------------------------------------------------------------------- */
static QueueHandle_t     SensorQueue;     /* prijem senzora -> obrada     */
static QueueHandle_t     DisplayQueue;    /* obrada -> displej            */
static QueueHandle_t     PC1SendQueue;    /* obrada -> PC1 (status)       */

static SemaphoreHandle_t RXC0_Sem;        /* primljen karakter na kanalu 0 */
static SemaphoreHandle_t RXC1_Sem;        /* primljen karakter na kanalu 1 */
static SemaphoreHandle_t SerialTx_Mutex;  /* serijalizuje slanje na UniCom */
static SemaphoreHandle_t SerialRx_Mutex;  /* stiti get_serial_character sqn */

/* Velicina poruka koje se salju kroz redove */
#define STATUS_MSG_LEN  128U
#define SERIAL_TBE_TIMEOUT_MS  5U

/* ---------------------------------------------------------------------
 * PRIORITETI TASKOVA (configMAX_PRIORITIES = 7 -> validno 1..6)
 * Prijemni taskovi imaju veci prioritet od predajnih (preporuka iz vezbi).
 * ------------------------------------------------------------------- */
#define PRI_SENSOR_RX   5U
#define PRI_PROCESSING  4U
#define PRI_DISPLAY     3U
#define PRI_PC1_RX      3U
#define PRI_LED         2U
#define PRI_PC1_SEND    1U
#define PRI_PC2_SEND    1U
#define PRI_SENSOR_TX   1U

 /* ---------------------------------------------------------------------
  * FORWARD DEKLARACIJE
  * ------------------------------------------------------------------- */
static void SenzorReceive_Task(void* pvParameters);
static void SenzorSend_Task(void* pvParameters);
static void Processing_Task(void* pvParameters);
static void Display_Task(void* pvParameters);
static void PC1Send_Task(void* pvParameters);
static void PC1Receive_Task(void* pvParameters);
static void PC2Send_Task(void* pvParameters);
static void LED_Task(void* pvParameters);

/* =====================================================================
 *  PREKIDNI HANDLER (TBE - transmit buffer empty)
 *  Signalizira da je UniCom spreman za sledeci karakter.
 *  portYIELD_FROM_ISR(x) se razvija u "return x".
 * ===================================================================== */
static uint32_t prvProcessTBEInterrupt(void)
{
    return 0U;
}

/* =====================================================================
 *  PREKIDNI HANDLER (RXC - reception complete)
 *
 *  Jedan RXC prekid stize kad BILO KOJI kanal primi karakter.
 *  U ISR-u se proverava koji kanal ima spreman karakter i budi se samo
 *  odgovarajuci prijemni task. Time se izbegava da pogresan task cita
 *  tudji kanal i remeti prijem komandi.
 *
 *  portYIELD_FROM_ISR(x) se razvija u "return x", zato NEMA return ispod.
 * ===================================================================== */
static uint32_t prvProcessRXCInterrupt(void)
{
    BaseType_t xHigherPTW = pdFALSE;

    if (get_RXC_status(SENZOR_CH) != 0)
    {
        (void)xSemaphoreGiveFromISR(RXC0_Sem, &xHigherPTW);
    }

    if (get_RXC_status(PC_CH1) != 0)
    {
        (void)xSemaphoreGiveFromISR(RXC1_Sem, &xHigherPTW);
    }

    portYIELD_FROM_ISR(xHigherPTW);
}

/* ---------------------------------------------------------------------
 * Pomocne funkcije za slanje preko serijskog kanala.
 * Svi kanali dele TBE interrupt, zato slanje mora biti serijalizovano.
 * Posle svakog poslatog znaka ceka se TBE prekid. Timeout sprecava
 * trajno zakljucavanje ako simulator ne posalje prekid.
 * ------------------------------------------------------------------- */
static void serial_wait_tbe(uint8_t ch)
{
    TickType_t start = xTaskGetTickCount();

    while (get_TBE_status(ch) == 0)
    {
        if ((xTaskGetTickCount() - start) >=
            pdMS_TO_TICKS(SERIAL_TBE_TIMEOUT_MS))
        {
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1U));
    }
}

static void serial_send_char(uint8_t ch, uint8_t c)
{
    (void)xSemaphoreTake(SerialTx_Mutex, portMAX_DELAY);

    (void)send_serial_character(ch, c);
    serial_wait_tbe(ch);

    (void)xSemaphoreGive(SerialTx_Mutex);
}

static void serial_send_string(uint8_t ch, const char* str)
{
    uint16_t i = 0U;

    (void)xSemaphoreTake(SerialTx_Mutex, portMAX_DELAY);

    while (str[i] != '\0')
    {
        (void)send_serial_character(ch, (uint8_t)str[i]);
        serial_wait_tbe(ch);
        i++;
    }

    (void)xSemaphoreGive(SerialTx_Mutex);
}

static int serial_get_char(uint8_t ch, uint8_t* c)
{
    int ret;

    (void)xSemaphoreTake(SerialRx_Mutex, portMAX_DELAY);
    ret = get_serial_character(ch, c);
    (void)xSemaphoreGive(SerialRx_Mutex);

    return ret;
}

/* =====================================================================
 *  main_demo - inicijalizacija i pokretanje
 * ===================================================================== */
void main_demo(void)
{
    /* Periferije */
    (void)init_7seg_comm();
    (void)init_LED_comm();

    (void)init_serial_uplink(SENZOR_CH);
    (void)init_serial_downlink(SENZOR_CH);

    (void)init_serial_uplink(PC_CH1);
    (void)init_serial_downlink(PC_CH1);

    (void)init_serial_uplink(PC_CH2);    /* kanal 2 koristimo samo za slanje */
    (void)init_serial_downlink(PC_CH2);  /* potreban za proveru TBE statusa */

    /* Semafori */
    RXC0_Sem = xSemaphoreCreateBinary();
    RXC1_Sem = xSemaphoreCreateBinary();
    SerialTx_Mutex = xSemaphoreCreateMutex();
    SerialRx_Mutex = xSemaphoreCreateMutex();

    /* Redovi */
    SensorQueue = xQueueCreate(5U, sizeof(SenzorPojasevi));
    DisplayQueue = xQueueCreate(5U, sizeof(SenzorPojasevi));
    PC1SendQueue = xQueueCreate(8U, STATUS_MSG_LEN * sizeof(char));

    /* Prekidni handleri */
    vPortSetInterruptHandler(portINTERRUPT_SRL_RXC, prvProcessRXCInterrupt);
    vPortSetInterruptHandler(portINTERRUPT_SRL_TBE, prvProcessTBEInterrupt);

    /* Taskovi */
    (void)xTaskCreate(SenzorSend_Task, "SensorTX", MOJ_STEK, NULL, PRI_SENSOR_TX, NULL);
    (void)xTaskCreate(SenzorReceive_Task, "SensorRX", MOJ_STEK, NULL, PRI_SENSOR_RX, NULL);
    (void)xTaskCreate(Processing_Task, "Process", MOJ_STEK, NULL, PRI_PROCESSING, NULL);
    (void)xTaskCreate(Display_Task, "Display", MOJ_STEK, NULL, PRI_DISPLAY, NULL);
    (void)xTaskCreate(PC1Send_Task, "PC1TX", MOJ_STEK, NULL, PRI_PC1_SEND, NULL);
    (void)xTaskCreate(PC1Receive_Task, "PC1RX", MOJ_STEK, NULL, PRI_PC1_RX, NULL);
    (void)xTaskCreate(PC2Send_Task, "PC2TX", MOJ_STEK, NULL, PRI_PC2_SEND, NULL);
    (void)xTaskCreate(LED_Task, "LED", MOJ_STEK, NULL, PRI_LED, NULL);

    vTaskStartScheduler();

    /* Ovde se nikad ne stize ako je sve OK */
    while (LINT_WHILE != 0U) {}
}

/* =====================================================================
 *  TASK: SenzorSend_Task
 *  Salje triger karakter 'T' senzoru svakih 200 ms. UniCom kanal 0 je
 *  podesen da na 'T' automatski vrati string sa podacima senzora.
 * ===================================================================== */
static void SenzorSend_Task(void* pvParameters)
{
    (void)pvParameters;
    while (LINT_WHILE != 0U)
    {
        serial_send_char(SENZOR_CH, (uint8_t)'T');
        vTaskDelay(pdMS_TO_TICKS(200U));
    }
}

/* =====================================================================
 *  TASK: SenzorReceive_Task
 *  Prima string sa kanala 0, parsira ga i salje strukturu na SensorQueue.
 *  Ocekivani format (podesava se u UniCom-u kao odgovor na triger):
 *      $<vozac>,<suvozac>,<pritisak>#      npr.  $1,0,320#
 * ===================================================================== */
static void SenzorReceive_Task(void* pvParameters)
{
    (void)pvParameters;
    uint8_t cc = 0U;
    SenzorPojasevi data = { 0U, 0U, 0U };

    char vozacStr[8] = { '\0' };
    char suvozacStr[8] = { '\0' };
    char pritisStr[8] = { '\0' };

    while (LINT_WHILE != 0U)
    {
        (void)xSemaphoreTake(RXC0_Sem, portMAX_DELAY);

        /* Citamo SAMO ako je karakter zaista stigao na ovom kanalu */
        if (serial_get_char(SENZOR_CH, &cc) == 0)
        {
            if (cc == (uint8_t)'$')
            {
                sensor_rx_index = 0U;
            }
            else if (cc == (uint8_t)'#')
            {
                sensor_rx_buffer[sensor_rx_index] = '\0';

                /* MISRA dev: sscanf se koristi radi jednostavnosti parsiranja,
                 * nema pogodne alternative u okruzenju vezbi.                 */
                if (sscanf(sensor_rx_buffer, "%7[^,],%7[^,],%7s",
                    vozacStr, suvozacStr, pritisStr) == 3)
                {
                    data.vozac_pojas = (uint8_t)strtoul(vozacStr, NULL, 10);
                    data.suvozac_pojas = (uint8_t)strtoul(suvozacStr, NULL, 10);
                    data.suvozac_pritisak = (uint16_t)strtoul(pritisStr, NULL, 10);

                    (void)xQueueSend(SensorQueue, &data, 0U);
                }
                sensor_rx_index = 0U;
            }
            else
            {
                if (sensor_rx_index < (RX_BUFFER_SIZE - 1U))
                {
                    sensor_rx_buffer[sensor_rx_index] = (char)cc;
                    sensor_rx_index++;
                }
                else
                {
                    /* prelivanje bafera - odbaci */
                }
            }
        }
    }
}

/* =====================================================================
 *  TASK: Processing_Task
 *  Cita SensorQueue, azurira globalno stanje, salje displeju i generise
 *  status poruke za PC1 (na 2000 ms). Obradjuje SAMO kad je sistem ukljucen.
 * ===================================================================== */
static void Processing_Task(void* pvParameters)
{
    (void)pvParameters;
    SenzorPojasevi data = { 0U, 0U, 0U };
    char statusMsg[STATUS_MSG_LEN];
    static uint32_t lastPC1Send_ms = 0U;

    while (LINT_WHILE != 0U)
    {
        (void)xQueueReceive(SensorQueue, &data, portMAX_DELAY);

        if (sistemUkljucen != 0U)
        {
            uint8_t suvozacPrisutan;

            TrenutniPodaci = data;
            suvozacPrisutan = (data.suvozac_pritisak > pragPritiska) ? 1U : 0U;

            /* Status na PC1 svakih 2000 ms */
            {
                uint32_t now_ms = (uint32_t)xTaskGetTickCount() * (uint32_t)portTICK_PERIOD_MS;
                if ((now_ms - lastPC1Send_ms) >= 2000U)
                {
                    lastPC1Send_ms = now_ms;
                    /* MISRA dev: snprintf radi formatiranja izlazne poruke. */
                    (void)snprintf(statusMsg, sizeof(statusMsg),
                        "VOZAC: %s | SUVOZAC: %s | PRISUSTVO: %s\r\n",
                        (data.vozac_pojas != 0U) ? "VEZAN" : "NIJE VEZAN",
                        (data.suvozac_pojas != 0U) ? "VEZAN" : "NIJE VEZAN",
                        (suvozacPrisutan != 0U) ? "DA" : "NE");
                    (void)xQueueSend(PC1SendQueue, statusMsg, 0U);
                }
            }

            (void)xQueueSend(DisplayQueue, &data, 0U);
        }
        else
        {
            /* Sistem iskljucen -> status je NEPOZNATO (na 2000 ms) */
            uint32_t now_ms = (uint32_t)xTaskGetTickCount() * (uint32_t)portTICK_PERIOD_MS;
            if ((now_ms - lastPC1Send_ms) >= 2000U)
            {
                lastPC1Send_ms = now_ms;
                (void)snprintf(statusMsg, sizeof(statusMsg),
                    "VOZAC: NEPOZNATO | SUVOZAC: NEPOZNATO | PRISUSTVO: NEPOZNATO\r\n");
                (void)xQueueSend(PC1SendQueue, statusMsg, 0U);
            }
        }
    }
}

/* =====================================================================
 *  TASK: PC1Send_Task  - salje status poruke na kanal 1
 * ===================================================================== */
static void PC1Send_Task(void* pvParameters)
{
    (void)pvParameters;
    char msg[STATUS_MSG_LEN];

    while (LINT_WHILE != 0U)
    {
        (void)xQueueReceive(PC1SendQueue, msg, portMAX_DELAY);
        serial_send_string(PC_CH1, msg);
    }
}

/* =====================================================================
 *  TASK: PC1Receive_Task  - prima komande sa kanala 1 (zavrsava se CR=13)
 *  Komande: START, STOP, PRAG_<broj>
 * ===================================================================== */
static void PC1Receive_Task(void* pvParameters)
{
    (void)pvParameters;
    uint8_t cc = 0U;

    while (LINT_WHILE != 0U)
    {
        (void)xSemaphoreTake(RXC1_Sem, portMAX_DELAY);

        if (serial_get_char(PC_CH1, &cc) == 0)
        {
            if ((cc == (uint8_t)13U) || (cc == (uint8_t)10U))
            {
                if (pc1_rx_index > 0U)
                {
                    pc1_rx_buffer[pc1_rx_index] = '\0';

                    if (strcmp(pc1_rx_buffer, "START") == 0)
                    {
                        sistemUkljucen = 1U;
                        serial_send_string(PC_CH1, "OK: START\r\n");
                    }
                    else if (strcmp(pc1_rx_buffer, "STOP") == 0)
                    {
                        sistemUkljucen = 0U;
                        serial_send_string(PC_CH1, "OK: STOP\r\n");
                    }
                    else if (strncmp(pc1_rx_buffer, "PRAG_", 5U) == 0)
                    {
                        uint32_t noviPrag = strtoul(&pc1_rx_buffer[5], NULL, 10);
                        if (noviPrag > 1023U)
                        {
                            noviPrag = 1023U;
                        }
                        pragPritiska = (uint16_t)noviPrag;
                        serial_send_string(PC_CH1, "OK: PRAG\r\n");
                    }
                    else
                    {
                        serial_send_string(PC_CH1, "NEPOZNATA KOMANDA\r\n");
                    }

                    pc1_rx_index = 0U;
                }
            }
            else if ((cc >= 32U) && (cc <= 126U))   /* stampajuci karakteri */
            {
                if (pc1_rx_index < (RX_BUFFER_SIZE - 1U))
                {
                    if ((cc >= (uint8_t)'a') && (cc <= (uint8_t)'z'))
                    {
                        cc = (uint8_t)(cc - ((uint8_t)'a' - (uint8_t)'A'));
                    }
                    pc1_rx_buffer[pc1_rx_index] = (char)cc;
                    pc1_rx_index++;
                }
                else
                {
                    pc1_rx_index = 0U;
                    serial_send_string(PC_CH1, "GRESKA: KOMANDA PREDUGA\r\n");
                }
            }
            else
            {
                /* ignorisi ostale (npr. LF) */
            }
        }
    }
}

/* =====================================================================
 *  TASK: PC2Send_Task
 *  Na svakih 100 ms, ako je sistem ukljucen i neko nije vezan, salje
 *  upozorenje na kanal 2 (sa naznakom KO nije vezan).
 *  Sam generise poruku iz globalnog stanja (cadence tacno 100 ms).
 * ===================================================================== */
static void PC2Send_Task(void* pvParameters)
{
    (void)pvParameters;

    while (LINT_WHILE != 0U)
    {
        if (sistemUkljucen != 0U)
        {
            SenzorPojasevi snap = TrenutniPodaci;   /* snimak stanja */
            uint8_t suvozacPrisutan = (snap.suvozac_pritisak > pragPritiska) ? 1U : 0U;
            uint8_t vozacNV = (snap.vozac_pojas == 0U) ? 1U : 0U;
            uint8_t suvozacNV = ((suvozacPrisutan != 0U) && (snap.suvozac_pojas == 0U)) ? 1U : 0U;

            if ((vozacNV != 0U) && (suvozacNV != 0U))
            {
                serial_send_string(PC_CH2, "UPOZORENJE: VOZAC I SUVOZAC NISU VEZANI!\r\n");
            }
            else if (vozacNV != 0U)
            {
                serial_send_string(PC_CH2, "UPOZORENJE: VOZAC NIJE VEZAN!\r\n");
            }
            else if (suvozacNV != 0U)
            {
                serial_send_string(PC_CH2, "UPOZORENJE: SUVOZAC NIJE VEZAN!\r\n");
            }
            else
            {
                /* svi vezani - nema upozorenja */
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100U));
    }
}

/* =====================================================================
 *  TASK: Display_Task  - osvezava 7-seg displej svakih 100 ms
 *
 *  Raspored cifara (sa praznim segmentom izmedju podataka):
 *    [0] vozac pojas    : 'F' (vezan) / '-' (nije)
 *    [1] prazno
 *    [2] suvozac pojas  : 'F' / '-'
 *    [3] prazno
 *    [4] prisustvo      : 'P' (detektovan) / '0' (nije)
 *    [5..7] prazno
 *  Kad je sistem iskljucen -> sve crtice.
 * ===================================================================== */
static void Display_Task(void* pvParameters)
{
    (void)pvParameters;
    SenzorPojasevi dispData = { 0U, 0U, 0U };
    uint8_t segVozac;
    uint8_t segSuvozac;
    uint8_t segPrisutan;

    while (LINT_WHILE != 0U)
    {
        (void)xQueueReceive(DisplayQueue, &dispData, pdMS_TO_TICKS(IDLE_POLL_MS));

        if (sistemUkljucen != 0U)
        {
            uint8_t prisutan = (dispData.suvozac_pritisak > pragPritiska) ? 1U : 0U;
            segVozac = (dispData.vozac_pojas != 0U) ? SEG_F : SEG_DASH;
            segSuvozac = (dispData.suvozac_pojas != 0U) ? SEG_F : SEG_DASH;
            segPrisutan = (prisutan != 0U) ? SEG_P : hexnum[0];   /* '0' */
        }
        else
        {
            segVozac = SEG_DASH;
            segSuvozac = SEG_DASH;
            segPrisutan = SEG_DASH;
        }

        (void)select_7seg_digit(0U); (void)set_7seg_digit(segVozac);
        (void)select_7seg_digit(1U); (void)set_7seg_digit(SEG_BLANK);
        (void)select_7seg_digit(2U); (void)set_7seg_digit(segSuvozac);
        (void)select_7seg_digit(3U); (void)set_7seg_digit(SEG_BLANK);
        (void)select_7seg_digit(4U); (void)set_7seg_digit(segPrisutan);
        (void)select_7seg_digit(5U); (void)set_7seg_digit(SEG_BLANK);
        (void)select_7seg_digit(6U); (void)set_7seg_digit(SEG_BLANK);
        (void)select_7seg_digit(7U); (void)set_7seg_digit(SEG_BLANK);

        vTaskDelay(pdMS_TO_TICKS(IDLE_POLL_MS));
    }
}

/* =====================================================================
 *  TASK: LED_Task
 *    - stub 0 (ulaz): donja LED ukljucena -> sistem ukljucen (kontakt)
 *    - stub 1 (izlaz): donja LED kad je sistem ukljucen
 *    - stub 2 (izlaz): sve LED blinkaju kad neko nije vezan
 *        period 2 s ako je nevezan < 20 s, inace 400 ms
 *
 *  Sistem se moze paliti/gasiti i komandom START/STOP i ulaznim LED stubom.
 *  Da se ta dva ne bi tukla, ulazni stub se cita IVICAMA (edge): menja
 *  stanje samo kad se bit promeni, pa START/STOP ostaju na snazi izmedju
 *  promena na hardverskom prekidacu.
 * ===================================================================== */
static void LED_Task(void* pvParameters)
{
    (void)pvParameters;
    uint8_t  blinkState = 0U;
    uint32_t elapsed_ms = 0U;
    uint32_t delay_ms = IDLE_POLL_MS;
    uint8_t  prevInputBit;

    /* Inicijalizuj prethodno stanje ulaza bez izazivanja "promene" */
    {
        uint8_t startStub = 0U;
        (void)get_LED_BAR(0U, &startStub);
        prevInputBit = (uint8_t)(startStub & 0x01U);
        sistemUkljucen = prevInputBit;
    }

    while (LINT_WHILE != 0U)
    {
        uint8_t inputStub = 0U;
        uint8_t inputBit;

        (void)get_LED_BAR(0U, &inputStub);
        inputBit = (uint8_t)(inputStub & 0x01U);

        /* Promena na ulaznom stubu (kontakt) ima prednost na IVICI */
        if (inputBit != prevInputBit)
        {
            sistemUkljucen = inputBit;
            prevInputBit = inputBit;
            elapsed_ms = 0U;
            blinkState = 0U;
        }

        if (sistemUkljucen != 0U)
        {
            uint8_t suvozacPrisutan = (TrenutniPodaci.suvozac_pritisak > pragPritiska) ? 1U : 0U;
            uint8_t vozacNV = (TrenutniPodaci.vozac_pojas == 0U) ? 1U : 0U;
            uint8_t suvozacNV = ((suvozacPrisutan != 0U) && (TrenutniPodaci.suvozac_pojas == 0U)) ? 1U : 0U;
            uint8_t nijeVezan = ((vozacNV != 0U) || (suvozacNV != 0U)) ? 1U : 0U;

            (void)set_LED_BAR(1U, 0x01U);   /* sistem aktivan */

            if (nijeVezan != 0U)
            {
                uint32_t halfPeriod;

                /* izbor perioda blinkanja prema vremenu nevezanosti */
                if (elapsed_ms >= UNBELT_THRESHOLD_MS)
                {
                    halfPeriod = (uint32_t)BLINK_FAST_MS / 2U;     /* 200 ms */
                }
                else
                {
                    halfPeriod = (uint32_t)BLINK_NORMAL_MS / 2U;   /* 1000 ms */
                }

                blinkState = (blinkState == 0U) ? 1U : 0U;
                (void)set_LED_BAR(2U, (blinkState != 0U) ? 0xFFU : 0x00U);

                elapsed_ms += halfPeriod;
                delay_ms = halfPeriod;
            }
            else
            {
                (void)set_LED_BAR(2U, 0x00U);   /* svi vezani */
                elapsed_ms = 0U;
                blinkState = 0U;
                delay_ms = IDLE_POLL_MS;
            }
        }
        else
        {
            (void)set_LED_BAR(1U, 0x00U);
            (void)set_LED_BAR(2U, 0x00U);
            elapsed_ms = 0U;
            blinkState = 0U;
            delay_ms = IDLE_POLL_MS;
        }

        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}
