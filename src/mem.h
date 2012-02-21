// --------------------------------------------------------------------------------------
// Module     : MEM
// Description: Memory mapped IO access
// Author     : R. Leslie (June 2011)
// --------------------------------------------------------------------------------------
// Entact Robotics inc. www.entactrobotics.com
// --------------------------------------------------------------------------------------

#ifndef MEM_H
#define MEM_H

// comment out if mutual exclusion is not required for access to mem.c
#define MEM_MUTEX

/*************************************************************
  Memory locations in /dev/mem
*************************************************************/

// FPGA (Encoder Registers)
#define QEI_BASE                    (0x2B000004UL)
#define CLR_QEI_BASE                (0x2B000044UL)

// OMAP3 GPIO registers
#define OMAP34XX_GPIO1_BASE         (0x48310000UL)
#define OMAP34XX_GPIO2_BASE         (0x49050000UL)
#define OMAP34XX_GPIO3_BASE         (0x49052000UL)
#define OMAP34XX_GPIO4_BASE         (0x49054000UL)
#define OMAP34XX_GPIO5_BASE         (0x49056000UL)
#define OMAP34XX_GPIO6_BASE         (0x49058000UL)

// GPIO offsets
#define OMAP24XX_GPIO_OE            (0x0034)
#define OMAP24XX_GPIO_DATAIN        (0x0038)
#define OMAP24XX_GPIO_DATAOUT       (0x003c)
#define OMAP24XX_GPIO_CLEARDATAOUT  (0x0090)
#define OMAP24XX_GPIO_SETDATAOUT    (0x0094)

/*************************************************************
  GPIO Indicies
*************************************************************/

#define GPIO_PIC1_RST               145
#define GPIO_PIC2_RST               144

#define GPIO_GPOUT_0                70
#define GPIO_GPOUT_1                71
#define GPIO_GPOUT_2                72
#define GPIO_GPOUT_3                73
#define GPIO_GPOUT_4                74
#define GPIO_GPOUT_5                75
#define GPIO_GPOUT_6                76
#define GPIO_GPOUT_7                77

int mem_init(void);
int mem_uninit(void);
int mem_encoder_read_all(int32_t *enc);
int mem_encoder_clear_all(int32_t *enc);
int mem_slave_disable(void);
int mem_slave_enable(void);
int mem_led_init(void);
int mem_led_set(int value);

#endif // MEM_H
