
#include <gd32vf103.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern "C" {
#include "lcd/lcd.h"
#include "pt/pt.h"
#include "pt/timer.h"

#define cmb() __asm__ __volatile__ ("" ::: "memory")
void * __dso_handle __weak_symbol;
int main(void);

}
#include "FastPID.h"

#define PA_OUT(n,s)                                         \
({  if (s%2)        gpio_bit_set(GPIOA, GPIO_PIN_##n);      \
    else            gpio_bit_reset(GPIOA, GPIO_PIN_##n);   })
#define PA_IN(n)    gpio_input_bit_get(GPIOA, GPIO_PIN_##n)

#define PB_OUT(n,s)                                         \
({  if (s%2)        gpio_bit_set(GPIOB, GPIO_PIN_##n);      \
    else            gpio_bit_reset(GPIOB, GPIO_PIN_##n);   })
#define PB_IN(n)    gpio_input_bit_get(GPIOB, GPIO_PIN_##n)

#define PC_OUT(n,s)                                         \
({  if (s%2)        gpio_bit_set(GPIOC, GPIO_PIN_##n);      \
    else            gpio_bit_reset(GPIOC, GPIO_PIN_##n);   })
#define PC_IN(n)    gpio_input_bit_get(GPIOC, GPIO_PIN_##n)

#define LEDR_TOG    gpio_bit_write(GPIOC, GPIO_PIN_13, (bit_status)(1-gpio_input_bit_get(GPIOC, GPIO_PIN_13)))
#define LEDR(s)     PC_OUT(13, s)
#define LEDG_TOG    gpio_bit_write(GPIOA, GPIO_PIN_1, (bit_status)(1-gpio_input_bit_get(GPIOA, GPIO_PIN_1)))
#define LEDG(s)     PA_OUT(1, s)
#define LEDB_TOG    gpio_bit_write(GPIOA, GPIO_PIN_2, (bit_status)(1-gpio_input_bit_get(GPIOA, GPIO_PIN_2)))
#define LEDB(s)     PA_OUT(2, s)

int16_t setVolt = -0.5979299363057324*4096/3.3;;
PT_THREAD(test_state(struct pt *pt))
{
    static struct timer tmr;

    PT_BEGIN(pt);

    timer_set(&tmr, 5 * 1000 * 1000);    //100ms
    while(1){
        LEDR(0);
        LCD_ShowString(0,  0, (u8*)"set: 1.0v", BLACK);
        setVolt = -0.5979299363057324*4096/3.3;
        PT_WAIT_UNTIL(pt, timer_expired(&tmr));
        timer_restart(&tmr);    //100ms
        LEDR(1);
        
        LEDG(0);
        LCD_ShowString(0,  0, (u8*)"set: 3.0v", BLACK);
        setVolt = -1.7937898089171975*4096/3.3;
        PT_WAIT_UNTIL(pt, timer_expired(&tmr));
        timer_restart(&tmr);    //100ms
        LEDG(1);
        
        LEDB(0);
        LCD_ShowString(0,  0, (u8*)"set: 2.5v", BLACK);
        setVolt = -1.494824840764331*4096/3.3;
        PT_WAIT_UNTIL(pt, timer_expired(&tmr));
        timer_restart(&tmr);    //100ms
        LEDB(1);
    }

    PT_END(pt);
}

FastPID myPID(0.004, 0.41, 0.00005, 100, 12, false);
PT_THREAD(scan_adc(struct pt *pt))
{
    static struct timer tmr;

    PT_BEGIN(pt);

    myPID.setOutputRange(0, 1.3*4096/3.3);
    timer_set(&tmr, 10 * 1000);    //10ms
    while(1){
        adc_software_trigger_enable(ADC0, ADC_INSERTED_CHANNEL);
        PT_WAIT_UNTIL(pt, timer_expired(&tmr));
        PT_WAIT_UNTIL(pt, adc_flag_get(ADC0, ADC_FLAG_EOC));
        adc_flag_clear(ADC0, ADC_FLAG_EOC);
        
        uint16_t    a0=ADC_IDATA0(ADC0);
                    // a1=ADC_IDATA1(ADC0),
                    // a2=ADC_IDATA2(ADC0);

        uint16_t pid_out = myPID.step(setVolt, -a0);

        LCD_ShowNum(0, (16*1)+1, a0, 5, RED);LCD_ShowNum1((5*8)+8, (16*1)+1, a0*(3.3/4096), 3, RED);
        LCD_ShowNum(0, (16*2)+1, pid_out, 5, BROWN);//LCD_ShowNum1((5*8)+8, (16*2)+1, a1*(3.3/4096), 3, BROWN);
        // LCD_ShowNum(0, (16*3)+1, a2, 5, BLUE);LCD_ShowNum1((5*8)+8, (16*3)+1, a2*(3.3/4096), 3, BLUE);
        
        dac_data_set(DAC0, DAC_ALIGN_12B_R, pid_out);
        dac_software_trigger_enable(DAC0);

        timer_restart(&tmr);    //10ms
    }

    PT_END(pt);
}

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    SystemInit();
    
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_DAC);
    rcu_periph_clock_enable(RCU_ADC0);
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV16);

    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1|GPIO_PIN_2);

    Lcd_Init();			// init OLED
    LCD_Clear(WHITE);
    BACK_COLOR=WHITE;

    LEDR(1);LEDG(1);LEDB(1);

    struct pt pt1,pt2;
    PT_INIT(&pt1);
    PT_INIT(&pt2);
    
    #pragma section DAC_Config
    #define DAC_GPIO_PORT   GPIOA
    #define DAC_GPIO_PIN    GPIO_PIN_4
     /* once enabled the DAC, the corresponding GPIO pin is connected to the DAC converter automatically */
    gpio_init(DAC_GPIO_PORT, GPIO_MODE_AIN, /*dont care for inputs*/0, DAC_GPIO_PIN);

    dac_deinit();
    dac_wave_mode_config(DAC0, DAC_WAVE_DISABLE);
    dac_trigger_source_config(DAC0, DAC_TRIGGER_SOFTWARE);
    dac_output_buffer_enable(DAC0);
    dac_trigger_enable(DAC0);
    dac_enable(DAC0);
    #pragma endsection DAC_Config

    #pragma section ADC_Config
    #define ADC_CH          0
    #define ADC_GPIO_PORT   GPIOA
    #define ADC_GPIO_PIN    GPIO_PIN_0

    //GPIO config
    gpio_init(ADC_GPIO_PORT, GPIO_MODE_AIN, /*dont care for inputs*/0, ADC_GPIO_PIN);

    //ADC config
    /* reset ADC */
    adc_deinit(ADC0);
    /* ADC mode config */
    adc_mode_config(ADC_MODE_FREE);
    /* ADC contineous function enable */
    adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
    /* ADC data alignment config */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);

    adc_tempsensor_vrefint_enable();

    /* ADC channel length config */
    adc_channel_length_config(ADC0, ADC_INSERTED_CHANNEL, 1);
 
    /* ADC regular channel config */
    adc_inserted_channel_config(ADC0, ADC_INSERTED_CHANNEL_0, ADC_CH, ADC_SAMPLETIME_239POINT5);
    // adc_inserted_channel_config(ADC0, ADC_INSERTED_CHANNEL_1, 16, ADC_SAMPLETIME_239POINT5);
    // adc_inserted_channel_config(ADC0, ADC_INSERTED_CHANNEL_2, 17, ADC_SAMPLETIME_239POINT5);
    /* ADC trigger config */
    adc_external_trigger_source_config(ADC0, ADC_INSERTED_CHANNEL, ADC0_1_EXTTRIG_INSERTED_NONE);
    adc_external_trigger_config(ADC0, ADC_INSERTED_CHANNEL, ENABLE);
    
    /* enable ADC interface */
    adc_enable(ADC0);
    delay_1ms(1);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC0);

    /* ADC software trigger enable */
    // adc_software_trigger_enable(ADC0, ADC_INSERTED_CHANNEL);
    #pragma endsection ADC_Config

    // LCD_ShowLogo();
    // delay_1ms(2000);

    if(myPID.err())
        LCD_ShowString(0,  0, (u8*)"pid: ok", BLACK);
    else
        LCD_ShowString(0,  0, (u8*)"pid: err", BLACK);

    while(PT_SCHEDULE(test_state(&pt1)) && PT_SCHEDULE(scan_adc(&pt2)));
}