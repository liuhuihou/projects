/*
 * Copyright (c) 2023, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

DL_TimerA_backupConfig gPWM_0Backup;

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_PWM_0_init();
    SYSCFG_DL_TIMER_0_init();
    SYSCFG_DL_I2C_MPU_init();
    SYSCFG_DL_USB_init();
    SYSCFG_DL_BLUETOOTH_init();
    SYSCFG_DL_K230_init();
    SYSCFG_DL_BATTERY_ADC_init();
    SYSCFG_DL_SYSTICK_init();
    /* Ensure backup structures have no valid state */
	gPWM_0Backup.backupRdy 	= false;



}
/*
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_saveConfiguration(PWM_0_INST, &gPWM_0Backup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_restoreConfiguration(PWM_0_INST, &gPWM_0Backup, false);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerA_reset(PWM_0_INST);
    DL_TimerG_reset(TIMER_0_INST);
    DL_I2C_reset(I2C_MPU_INST);
    DL_UART_Main_reset(USB_INST);
    DL_UART_Main_reset(BLUETOOTH_INST);
    DL_UART_Main_reset(K230_INST);
    DL_ADC12_reset(BATTERY_ADC_INST);


    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerA_enablePower(PWM_0_INST);
    DL_TimerG_enablePower(TIMER_0_INST);
    DL_I2C_enablePower(I2C_MPU_INST);
    DL_UART_Main_enablePower(USB_INST);
    DL_UART_Main_enablePower(BLUETOOTH_INST);
    DL_UART_Main_enablePower(K230_INST);
    DL_ADC12_enablePower(BATTERY_ADC_INST);

    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_0_C0_IOMUX,GPIO_PWM_0_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_0_C0_PORT, GPIO_PWM_0_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_0_C1_IOMUX,GPIO_PWM_0_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_0_C1_PORT, GPIO_PWM_0_C1_PIN);

    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_MPU_IOMUX_SDA,
        GPIO_I2C_MPU_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_MPU_IOMUX_SCL,
        GPIO_I2C_MPU_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_I2C_MPU_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_I2C_MPU_IOMUX_SCL);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_USB_IOMUX_TX, GPIO_USB_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_USB_IOMUX_RX, GPIO_USB_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_BLUETOOTH_IOMUX_TX, GPIO_BLUETOOTH_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_BLUETOOTH_IOMUX_RX, GPIO_BLUETOOTH_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_K230_IOMUX_TX, GPIO_K230_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_K230_IOMUX_RX, GPIO_K230_IOMUX_RX_FUNC);

    DL_GPIO_initDigitalInputFeatures(KEY_BLS_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(S28A_BUTTON_S28A_KEY_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(LED_led_IOMUX);

    DL_GPIO_initDigitalInput(MPU6050_INT_INT_IOMUX);

    DL_GPIO_initDigitalOutput(MOTOR_A_DIR_AIN1_IOMUX);

    DL_GPIO_initDigitalOutput(MOTOR_A_DIR_AIN2_IOMUX);

    DL_GPIO_initDigitalOutput(MOTOR_B_DIR_BIN1_IOMUX);

    DL_GPIO_initDigitalOutput(MOTOR_B_DIR_BIN2_IOMUX);

    DL_GPIO_initDigitalInput(ENCODERA_E1A_IOMUX);

    DL_GPIO_initDigitalInput(ENCODERA_E1B_IOMUX);

    DL_GPIO_initDigitalInput(ENCODERB_E2A_IOMUX);

    DL_GPIO_initDigitalInput(ENCODERB_E2B_IOMUX);

    DL_GPIO_initDigitalOutput(OLED_RST_IOMUX);

    DL_GPIO_initDigitalOutput(OLED_DC_IOMUX);

    DL_GPIO_initDigitalOutput(OLED_SCL_IOMUX);

    DL_GPIO_initDigitalOutput(OLED_SDA_IOMUX);

    DL_GPIO_initDigitalInput(IR_SENSORS_IR1_IOMUX);

    DL_GPIO_initDigitalInput(IR_SENSORS_IR2_IOMUX);

    DL_GPIO_initDigitalInput(IR_SENSORS_IR3_IOMUX);

    DL_GPIO_initDigitalInput(IR_SENSORS_IR4_IOMUX);

    DL_GPIO_initDigitalInput(IR_SENSORS_IR5_IOMUX);

    DL_GPIO_initDigitalInput(IR_SENSORS_IR6_IOMUX);

    DL_GPIO_initDigitalOutput(STEPPER_STEP_IOMUX);

    DL_GPIO_initDigitalOutput(STEPPER_DIR_IOMUX);

    DL_GPIO_initDigitalOutput(STEPPER_EN_IOMUX);

    DL_GPIO_clearPins(GPIOA, MOTOR_A_DIR_AIN1_PIN |
		MOTOR_A_DIR_AIN2_PIN |
		MOTOR_B_DIR_BIN1_PIN |
		MOTOR_B_DIR_BIN2_PIN |
		OLED_SCL_PIN |
		OLED_SDA_PIN);
    DL_GPIO_enableOutput(GPIOA, MOTOR_A_DIR_AIN1_PIN |
		MOTOR_A_DIR_AIN2_PIN |
		MOTOR_B_DIR_BIN1_PIN |
		MOTOR_B_DIR_BIN2_PIN |
		OLED_SCL_PIN |
		OLED_SDA_PIN);
    DL_GPIO_setUpperPinsPolarity(GPIOA, DL_GPIO_PIN_25_EDGE_RISE_FALL |
		DL_GPIO_PIN_26_EDGE_RISE_FALL);
    DL_GPIO_clearInterruptStatus(GPIOA, ENCODERA_E1A_PIN |
		ENCODERA_E1B_PIN);
    DL_GPIO_enableInterrupt(GPIOA, ENCODERA_E1A_PIN |
		ENCODERA_E1B_PIN);
    DL_GPIO_clearPins(GPIOB, LED_led_PIN |
		OLED_RST_PIN |
		OLED_DC_PIN |
		STEPPER_STEP_PIN |
		STEPPER_DIR_PIN |
		STEPPER_EN_PIN);
    DL_GPIO_enableOutput(GPIOB, LED_led_PIN |
		OLED_RST_PIN |
		OLED_DC_PIN |
		STEPPER_STEP_PIN |
		STEPPER_DIR_PIN |
		STEPPER_EN_PIN);
    DL_GPIO_setUpperPinsPolarity(GPIOB, DL_GPIO_PIN_20_EDGE_RISE_FALL |
		DL_GPIO_PIN_24_EDGE_RISE_FALL);
    DL_GPIO_clearInterruptStatus(GPIOB, ENCODERB_E2A_PIN |
		ENCODERB_E2B_PIN);
    DL_GPIO_enableInterrupt(GPIOB, ENCODERB_E2A_PIN |
		ENCODERB_E2B_PIN);

}


SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{

	//Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);

    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    /* Set default configuration */
    DL_SYSCTL_disableHFXT();
    DL_SYSCTL_disableSYSPLL();
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_1);
    DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);

}


/*
 * Timer clock configuration to be sourced by  / 1 (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   32000000 Hz = 32000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerA_ClockConfig gPWM_0ClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};

static const DL_TimerA_PWMConfig gPWM_0Config = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 8000,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_PWM_0_init(void) {

    DL_TimerA_setClockConfig(
        PWM_0_INST, (DL_TimerA_ClockConfig *) &gPWM_0ClockConfig);

    DL_TimerA_initPWMMode(
        PWM_0_INST, (DL_TimerA_PWMConfig *) &gPWM_0Config);

    DL_TimerA_setCaptureCompareOutCtl(PWM_0_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_0_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(PWM_0_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_0_INST, 8000, DL_TIMER_CC_0_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(PWM_0_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_1_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(PWM_0_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_0_INST, 8000, DL_TIMER_CC_1_INDEX);

    DL_TimerA_enableClock(PWM_0_INST);


    
    DL_TimerA_setCCPDirection(PWM_0_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT );


}



/*
 * Timer clock configuration to be sourced by BUSCLK /  (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   1000000 Hz = 32000000 Hz / (1 * (31 + 1))
 */
static const DL_TimerG_ClockConfig gTIMER_0ClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale    = 31U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * TIMER_0_INST_LOAD_VALUE = (10 ms * 1000000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gTIMER_0TimerConfig = {
    .period     = TIMER_0_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMER_0_init(void) {

    DL_TimerG_setClockConfig(TIMER_0_INST,
        (DL_TimerG_ClockConfig *) &gTIMER_0ClockConfig);

    DL_TimerG_initTimerMode(TIMER_0_INST,
        (DL_TimerG_TimerConfig *) &gTIMER_0TimerConfig);
    DL_TimerG_enableInterrupt(TIMER_0_INST , DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_TimerG_enableClock(TIMER_0_INST);





}


static const DL_I2C_ClockConfig gI2C_MPUClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

SYSCONFIG_WEAK void SYSCFG_DL_I2C_MPU_init(void) {

    DL_I2C_setClockConfig(I2C_MPU_INST,
        (DL_I2C_ClockConfig *) &gI2C_MPUClockConfig);
    DL_I2C_setAnalogGlitchFilterPulseWidth(I2C_MPU_INST,
        DL_I2C_ANALOG_GLITCH_FILTER_WIDTH_50NS);
    DL_I2C_enableAnalogGlitchFilter(I2C_MPU_INST);

    /* Configure Controller Mode */
    DL_I2C_resetControllerTransfer(I2C_MPU_INST);
    /* Set frequency to 100000 Hz*/
    DL_I2C_setTimerPeriod(I2C_MPU_INST, 31);
    DL_I2C_setControllerTXFIFOThreshold(I2C_MPU_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(I2C_MPU_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(I2C_MPU_INST);


    /* Enable module */
    DL_I2C_enableController(I2C_MPU_INST);


}


static const DL_UART_Main_ClockConfig gUSBClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUSBConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_USB_init(void)
{
    DL_UART_Main_setClockConfig(USB_INST, (DL_UART_Main_ClockConfig *) &gUSBClockConfig);

    DL_UART_Main_init(USB_INST, (DL_UART_Main_Config *) &gUSBConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115211.52
     */
    DL_UART_Main_setOversampling(USB_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(USB_INST, USB_IBRD_32_MHZ_115200_BAUD, USB_FBRD_32_MHZ_115200_BAUD);



    DL_UART_Main_enable(USB_INST);
}

static const DL_UART_Main_ClockConfig gBLUETOOTHClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gBLUETOOTHConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_BLUETOOTH_init(void)
{
    DL_UART_Main_setClockConfig(BLUETOOTH_INST, (DL_UART_Main_ClockConfig *) &gBLUETOOTHClockConfig);

    DL_UART_Main_init(BLUETOOTH_INST, (DL_UART_Main_Config *) &gBLUETOOTHConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 9600
     *  Actual baud rate: 9600.24
     */
    DL_UART_Main_setOversampling(BLUETOOTH_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(BLUETOOTH_INST, BLUETOOTH_IBRD_32_MHZ_9600_BAUD, BLUETOOTH_FBRD_32_MHZ_9600_BAUD);


    /* Configure FIFOs */
    DL_UART_Main_enableFIFOs(BLUETOOTH_INST);
    DL_UART_Main_setRXFIFOThreshold(BLUETOOTH_INST, DL_UART_RX_FIFO_LEVEL_1_2_FULL);
    DL_UART_Main_setTXFIFOThreshold(BLUETOOTH_INST, DL_UART_TX_FIFO_LEVEL_1_2_EMPTY);

    DL_UART_Main_enable(BLUETOOTH_INST);
}

static const DL_UART_Main_ClockConfig gK230ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gK230Config = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_K230_init(void)
{
    DL_UART_Main_setClockConfig(K230_INST, (DL_UART_Main_ClockConfig *) &gK230ClockConfig);

    DL_UART_Main_init(K230_INST, (DL_UART_Main_Config *) &gK230Config);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115211.52
     */
    DL_UART_Main_setOversampling(K230_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(K230_INST, K230_IBRD_32_MHZ_115200_BAUD, K230_FBRD_32_MHZ_115200_BAUD);


    /* Configure FIFOs */
    DL_UART_Main_enableFIFOs(K230_INST);
    DL_UART_Main_setRXFIFOThreshold(K230_INST, DL_UART_RX_FIFO_LEVEL_1_2_FULL);
    DL_UART_Main_setTXFIFOThreshold(K230_INST, DL_UART_TX_FIFO_LEVEL_1_2_EMPTY);

    DL_UART_Main_enable(K230_INST);
}

/* BATTERY_ADC Initialization */
static const DL_ADC12_ClockConfig gBATTERY_ADCClockConfig = {
    .clockSel       = DL_ADC12_CLOCK_ULPCLK,
    .divideRatio    = DL_ADC12_CLOCK_DIVIDE_8,
    .freqRange      = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
};
SYSCONFIG_WEAK void SYSCFG_DL_BATTERY_ADC_init(void)
{
    DL_ADC12_setClockConfig(BATTERY_ADC_INST, (DL_ADC12_ClockConfig *) &gBATTERY_ADCClockConfig);
    DL_ADC12_configConversionMem(BATTERY_ADC_INST, BATTERY_ADC_ADCMEM_0,
        DL_ADC12_INPUT_CHAN_0, DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_setSampleTime0(BATTERY_ADC_INST,500);
    DL_ADC12_enableConversions(BATTERY_ADC_INST);
}

SYSCONFIG_WEAK void SYSCFG_DL_SYSTICK_init(void)
{
}

