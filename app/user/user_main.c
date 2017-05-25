/*
 * user_main.c
 *
 *  Created on: 2015年7月13日
 *      Author: Administrator
 */
//#include "driver/uart.h"
//#include "eagle_soc.h"
//#include "gpio.h"
//#include "ets_sys.h"
//#include "osapi.h"
//#include "hw_timer.h"
//#define GPIOM 0
//LOCAL void key_intr_handle(void *arg){
//	uint16 gpio_status=0;
//	gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);//判断IO口状态
//	gpio_pin_intr_state_set(GPIO_ID_PIN(GPIOM),GPIO_PIN_INTR_DISABLE);//关闭GPIO12中断
//	GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status);//清除中断标志
//	os_delay_us(10000);
//	if(GPIO_INPUT_GET(GPIOM)==0){
//		os_printf("trigger interrupt");
//	}
//
//	gpio_pin_intr_state_set(GPIO_ID_PIN(GPIOM),GPIO_PIN_INTR_NEGEDGE);//设置中断触发方式
//	ETS_GPIO_INTR_ENABLE();//打开中断
//
//
//
//}
//void ICACHE_FLASH_ATTR GPIO_intr_init(void){
//	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U,FUNC_GPIO0);//PIN脚选择GPIO12
//	ETS_GPIO_INTR_ATTACH(key_intr_handle,NULL);//设置中断回调函数
//	ETS_GPIO_INTR_DISABLE();//关闭所有GPIO中断
//	gpio_output_set(0,0,0,BIT0);//将GPIO12设置为输入
//	gpio_register_set(GPIO_PIN_ADDR(GPIOM),GPIO_PIN_INT_TYPE_SET(GPIO_PIN_INTR_DISABLE) | GPIO_PIN_PAD_DRIVER_GET(GPIO_PAD_DRIVER_DISABLE) | GPIO_PIN_SOURCE_SET(GPIO_AS_PIN_SOURCE));//设置GPIO12配置寄存器
//	gpio_pin_intr_state_set(GPIO_ID_PIN(GPIOM),GPIO_PIN_INTR_NEGEDGE);//设置中断触发方式
//	ETS_GPIO_INTR_ENABLE();//开启GPIO中断
//}
//void hw_test_timer_cb(void){
//	os_printf("time end");
//}

#include "ets_sys.h"
#include "osapi.h"
#include "user_interface.h"
#include "eagle_soc.h"
#include "gpio.h"
#include "hw_timer.h"

/** 开关输入相关 宏定义 */
#define SWITCH_Pin_NUM        4
#define SWITCH_Pin_FUNC       FUNC_GPIO4
#define SWITCH_Pin_MUX        PERIPHS_IO_MUX_GPIO4_U//PERIPHS_IO_MUX_MTDI_U//

#define SWITCH_Pin_Rd_Init()   GPIO_DIS_OUTPUT(SWITCH_Pin_NUM)
#define SWITCH_Pin_Wr_Init()   GPIO_OUTPUT_SET(SWITCH_Pin_NUM,0)
#define SWITCH_Pin_Set_High()  GPIO_OUTPUT_SET(SWITCH_Pin_NUM,1)
#define SWITCH_Pin_Set_Low()   GPIO_OUTPUT_SET(SWITCH_Pin_NUM,0)
#define SWITCH_Pin_State       ( GPIO_INPUT_GET(SWITCH_Pin_NUM) != 0 )
ETSTimer test_timer;
os_timer_t	timer;
/**
 *
*******************************************************************************
 * @brief       GPIO中断处理函数
 * @param       [in/out]  void
 * @return      void
 * @note        None
*******************************************************************************
*/

/** LED操作命令 */
void Led_Cmd( bool status )
{
    if ( status == true )
    {
    	os_printf("\nnwo set 0");
        gpio16_output_set( 0 );
    }
    else
    {
    	os_printf("\nnow set 1");
        gpio16_output_set( 1 );
    }
}

/** LED任务运行程序 */
void Led_Task_Run( void )
{
    static bool status = false;

    if ( status == true )
    {
        status = false;
    }
    else
    {
        status = true;
    }
    Led_Cmd( status );
}

static void GPIO_ISR_Handler( void )
{
    /** 读取GPIO中断状态 */
    u32 pin_status = GPIO_REG_READ( GPIO_STATUS_ADDRESS );
    os_printf("\nstatus:%d\n",pin_status);
    /** 关闭GPIO中断 */
    ETS_GPIO_INTR_DISABLE();

    /** 清除GPIO中断标志 */
    GPIO_REG_WRITE( GPIO_STATUS_W1TC_ADDRESS, pin_status );

    /** 检测是否已开关输入引脚中断 */
    if ( pin_status & BIT( SWITCH_Pin_NUM ) )
    {
        if( SWITCH_Pin_State )
        {
        	os_printf("\ntrigger up\n");
        	gpio16_output_set( 1 );

        }
        else
        {
        	gpio16_output_set(0 );
        	os_printf("\ntrigger down\n");
        	//Led_Task_Run(  );
        }
    }

    /** 开启GPIO中断 */
    ETS_GPIO_INTR_ENABLE();
}



/**
*******************************************************************************
 * @brief       输入初始化函数
 * @param       [in/out]  void
 * @return      void
 * @note        None
 *******************************************************************************
 */
static void drv_Input_Init( void )
{
    PIN_FUNC_SELECT( SWITCH_Pin_MUX, SWITCH_Pin_FUNC );

    SWITCH_Pin_Rd_Init();
    /** 关闭GPIO中断 */
    ETS_GPIO_INTR_DISABLE();

    ETS_GPIO_INTR_ATTACH( &GPIO_ISR_Handler, NULL );

    gpio_pin_intr_state_set( GPIO_ID_PIN( SWITCH_Pin_NUM ),GPIO_PIN_INTR_ANYEDGE );//GPIO_PIN_INTR_ANYEDGE

    /** 清除该引脚的GPIO中断标志 */
    GPIO_REG_WRITE( GPIO_STATUS_W1TC_ADDRESS, BIT(SWITCH_Pin_NUM) );

    ETS_GPIO_INTR_ENABLE();
}

/**
*******************************************************************************
 * @brief       用户初始化程序
 * @param       [in/out]  void
 * @return      void
 * @note        None
*******************************************************************************
*/
void hw_test_timer_cb(void){
	gpio_output_set(0, 0, 0, BIT4);
	uint16 status=GPIO_INPUT_GET(4);
	os_printf("time end:vcc=%d\n",status);
}




void	ICACHE_FLASH_ATTR	ADC_TEST()
{

	 wifi_set_opmode(NULL_MODE);
	// ets_intr_lock();		 //close	interrupt
	 uint16	adc_addr[10];
	 uint16	adc_num	=	10;
	 uint8	adc_clk_div	=	8;

	 uint16	res;

	 res= system_adc_read();


	   os_printf(" adc_v=%d\n",	res);
	// ets_intr_unlock();	 //open	interrupt
	 os_timer_disarm(&timer);

	 os_timer_setfn(&timer,	ADC_TEST,	NULL);

	 os_timer_arm(&timer,1000,1);

}
void ICACHE_FLASH_ATTR done_cb1(void){



	os_printf("Wifi connect success!");
	//ADC_TEST();
	 hw_timer_init(NMI_SOURCE, 1);
	 hw_timer_set_func(hw_test_timer_cb);
	 hw_timer_arm(1000000);


	// smartconfig_start(smartconfig_done);

}
void user_init( void )
{

    gpio16_output_conf();
    uart_init(115200,115200);
//    gpio16_output_conf();
    Led_Cmd( false );
    drv_Input_Init();
//
   system_init_done_cb(done_cb1);







//    hw_timer_init(FRC1_SOURCE, 1);
//    hw_timer_set_func(hw_test_timer_cb);
//    hw_timer_arm(1000000);

}


//void user_init(){
//uart_init(115200,115200);
//GPIO_intr_init();
//
//hw_timer_init(FRC1_SOURCE, 1);
//hw_timer_set_func(hw_test_timer_cb);
//hw_timer_arm(1000000);
//}
void user_rf_pre_init(){}

