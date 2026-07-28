#ifndef TI_MSP_DL_CONFIG_H
#define TI_MSP_DL_CONFIG_H

#define CONFIG_MSPM0G350X

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

#define POWER_STARTUP_DELAY                         (16U)
#define CPUCLK_FREQ                                 (80000000U)

/* Board Type-C debug serial port: PA10 TX, PA11 RX, 115200-8-N-1. */
#define DEBUG_UART_INST                             UART0
#define DEBUG_UART_TX_IOMUX                         (IOMUX_PINCM21)
#define DEBUG_UART_RX_IOMUX                         (IOMUX_PINCM22)
#define DEBUG_UART_TX_FUNC                          IOMUX_PINCM21_PF_UART0_TX
#define DEBUG_UART_RX_FUNC                          IOMUX_PINCM22_PF_UART0_RX
#define DEBUG_UART_IBRD                             (21U)
#define DEBUG_UART_FBRD                             (45U)

/* Bluetooth serial port: PB6 TX, PB7 RX, 9600-8-N-1. */
#define BT_UART_INST                                UART1
#define BT_UART_TX_IOMUX                            (IOMUX_PINCM23)
#define BT_UART_RX_IOMUX                            (IOMUX_PINCM24)
#define BT_UART_TX_FUNC                             IOMUX_PINCM23_PF_UART1_TX
#define BT_UART_RX_FUNC                             IOMUX_PINCM24_PF_UART1_RX
#define BT_UART_IBRD                                (260U)
#define BT_UART_FBRD                                (27U)

/* C07A user LED is connected to PB9 and is active low. */
#define STATUS_LED_PORT                             GPIOB
#define STATUS_LED_PIN                              DL_GPIO_PIN_9
#define STATUS_LED_IOMUX                            (IOMUX_PINCM26)

void SYSCFG_DL_init(void);

#ifdef __cplusplus
}
#endif

#endif
