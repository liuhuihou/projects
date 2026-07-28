#include "ti_msp_dl_config.h"

static const DL_SYSCTL_SYSPLLConfig gSYSPLLConfig = {
    .inputFreq   = DL_SYSCTL_SYSPLL_INPUT_FREQ_16_32_MHZ,
    .rDivClk2x   = 1,
    .rDivClk1    = 0,
    .rDivClk0    = 0,
    .enableCLK2x = DL_SYSCTL_SYSPLL_CLK2X_ENABLE,
    .enableCLK1  = DL_SYSCTL_SYSPLL_CLK1_DISABLE,
    .enableCLK0  = DL_SYSCTL_SYSPLL_CLK0_DISABLE,
    .sysPLLMCLK  = DL_SYSCTL_SYSPLL_MCLK_CLK2X,
    .sysPLLRef   = DL_SYSCTL_SYSPLL_REF_SYSOSC,
    .qDiv        = 4,
    .pDiv        = DL_SYSCTL_SYSPLL_PDIV_2
};

static const DL_UART_Main_ClockConfig gUARTClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUARTConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

static void init_clock(void)
{
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
    DL_SYSCTL_setFlashWaitState(DL_SYSCTL_FLASH_WAIT_STATE_2);
    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_disableHFXT();
    DL_SYSCTL_disableSYSPLL();
    DL_SYSCTL_configSYSPLL((DL_SYSCTL_SYSPLLConfig *) &gSYSPLLConfig);
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_2);
    DL_SYSCTL_setMCLKSource(SYSOSC, HSCLK, DL_SYSCTL_HSCLK_SOURCE_SYSPLL);
}

static void init_uart(UART_Regs *uart, uint32_t integerDivisor,
    uint32_t fractionalDivisor)
{
    DL_UART_Main_setClockConfig(
        uart, (DL_UART_Main_ClockConfig *) &gUARTClockConfig);
    DL_UART_Main_init(uart, (DL_UART_Main_Config *) &gUARTConfig);
    DL_UART_Main_setOversampling(uart, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(
        uart, integerDivisor, fractionalDivisor);
    DL_UART_Main_enableFIFOs(uart);
    DL_UART_Main_setRXFIFOThreshold(uart, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_setTXFIFOThreshold(uart, DL_UART_TX_FIFO_LEVEL_1_2_EMPTY);
    DL_UART_Main_enable(uart);
}

void SYSCFG_DL_init(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_UART_Main_reset(DEBUG_UART_INST);
    DL_UART_Main_reset(BT_UART_INST);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_UART_Main_enablePower(DEBUG_UART_INST);
    DL_UART_Main_enablePower(BT_UART_INST);
    delay_cycles(POWER_STARTUP_DELAY);

    init_clock();

    DL_GPIO_initPeripheralOutputFunction(DEBUG_UART_TX_IOMUX,
        DEBUG_UART_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(DEBUG_UART_RX_IOMUX,
        DEBUG_UART_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(BT_UART_TX_IOMUX,
        BT_UART_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(BT_UART_RX_IOMUX,
        BT_UART_RX_FUNC);

    DL_GPIO_initDigitalOutput(STATUS_LED_IOMUX);
    DL_GPIO_setPins(STATUS_LED_PORT, STATUS_LED_PIN);
    DL_GPIO_enableOutput(STATUS_LED_PORT, STATUS_LED_PIN);

    init_uart(DEBUG_UART_INST, DEBUG_UART_IBRD, DEBUG_UART_FBRD);
    init_uart(BT_UART_INST, BT_UART_IBRD, BT_UART_FBRD);

    DL_SYSTICK_config(CPUCLK_FREQ / 1000U);
    DL_SYSTICK_enableInterrupt();
}
