#ifndef __ACCEL_H__
#define __ACCEL_H__

#include <drivers/gpio.h>
#include <drivers/sensor.h>

int power_down_accelerometer(void);
int power_up_accelerometer(void);
int accel_init(void);
void startAccelSampling();
void stopAccelSampling();

#define LIS2DH_REG_WAI			0x0f
#define LIS2DH_CHIP_ID			0x33

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
#include <drivers/spi.h>
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
#include <drivers/i2c.h>
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c) */

#define LIS2DH_AUTOINCREMENT_ADDR	BIT(7)

#define LIS2DH_REG_CTRL0		0x1e
#define LIS2DH_SDO_PU_DISC_SHIFT	7
#define LIS2DH_SDO_PU_DISC_MASK		BIT(LIS2DH_SDO_PU_DISC_SHIFT)

#define LIS2DH_REG_CTRL1		0x20
#define LIS2DH_ACCEL_XYZ_SHIFT		0
#define LIS2DH_ACCEL_X_EN_BIT		BIT(0)
#define LIS2DH_ACCEL_Y_EN_BIT		BIT(1)
#define LIS2DH_ACCEL_Z_EN_BIT		BIT(2)
#define LIS2DH_ACCEL_EN_BITS		(LIS2DH_ACCEL_X_EN_BIT | \
					LIS2DH_ACCEL_Y_EN_BIT | \
					LIS2DH_ACCEL_Z_EN_BIT)
#define LIS2DH_ACCEL_XYZ_MASK		BIT_MASK(3)

#define LIS2DH_LP_EN_BIT_MASK		BIT(3)
#if defined(CONFIG_LIS2DH_OPER_MODE_LOW_POWER)
	#define LIS2DH_LP_EN_BIT	BIT(3)
#else
	#define LIS2DH_LP_EN_BIT	0
#endif

#define LIS2DH_ODR_1			1
#define LIS2DH_ODR_2			2
#define LIS2DH_ODR_3			3
#define LIS2DH_ODR_4			4
#define LIS2DH_ODR_5			5
#define LIS2DH_ODR_6			6
#define LIS2DH_ODR_7			7
#define LIS2DH_ODR_8			8
#define LIS2DH_ODR_9			9

#if defined(CONFIG_LIS2DH_ODR_1)
	#define LIS2DH_ODR_IDX		LIS2DH_ODR_1
#elif defined(CONFIG_LIS2DH_ODR_2)
	#define LIS2DH_ODR_IDX		LIS2DH_ODR_2
#elif defined(CONFIG_LIS2DH_ODR_3)
	#define LIS2DH_ODR_IDX		LIS2DH_ODR_3
#elif defined(CONFIG_LIS2DH_ODR_4) || defined(CONFIG_LIS2DH_ODR_RUNTIME)
	#define LIS2DH_ODR_IDX		LIS2DH_ODR_4
#elif defined(CONFIG_LIS2DH_ODR_5)
	#define LIS2DH_ODR_IDX		LIS2DH_ODR_5
#elif defined(CONFIG_LIS2DH_ODR_6)
	#define LIS2DH_ODR_IDX		LIS2DH_ODR_6
#elif defined(CONFIG_LIS2DH_ODR_7)
	#define LIS2DH_ODR_IDX		LIS2DH_ODR_7
#elif defined(CONFIG_LIS2DH_ODR_8)
	#define LIS2DH_ODR_IDX		LIS2DH_ODR_8
#elif defined(CONFIG_LIS2DH_ODR_9_NORMAL) || defined(CONFIG_LIS2DH_ODR_9_LOW)
	#define LIS2DH_ODR_IDX		LIS2DH_ODR_9
#endif

#define LIS2DH_ODR_SHIFT		4
#define LIS2DH_ODR_RATE(r)		((r) << LIS2DH_ODR_SHIFT)
#define LIS2DH_ODR_BITS			(LIS2DH_ODR_RATE(LIS2DH_ODR_IDX))
#define LIS2DH_ODR_MASK			(BIT_MASK(4) << LIS2DH_ODR_SHIFT)

#define LIS2DH_REG_CTRL2		0x21
#define LIS2DH_HPIS2_EN_BIT		BIT(1)
#define LIS2DH_FDS_EN_BIT		BIT(3)

#define LIS2DH_REG_CTRL3		0x22
#define LIS2DH_EN_DRDY1_INT1_SHIFT	4
#define LIS2DH_EN_DRDY1_INT1		BIT(LIS2DH_EN_DRDY1_INT1_SHIFT)

#define LIS2DH_REG_CTRL4		0x23
#define LIS2DH_FS_SHIFT			4
#define LIS2DH_FS_MASK			(BIT_MASK(2) << LIS2DH_FS_SHIFT)

#if defined(CONFIG_LIS2DH_ACCEL_RANGE_2G) ||\
	defined(CONFIG_LIS2DH_ACCEL_RANGE_RUNTIME)
	#define LIS2DH_FS_IDX		0
#elif defined(CONFIG_LIS2DH_ACCEL_RANGE_4G)
	#define LIS2DH_FS_IDX		1
#elif defined(CONFIG_LIS2DH_ACCEL_RANGE_8G)
	#define LIS2DH_FS_IDX		2
#elif defined(CONFIG_LIS2DH_ACCEL_RANGE_16G)
	#define LIS2DH_FS_IDX		3
#endif

#define LIS2DH_FS_SELECT(fs)		((fs) << LIS2DH_FS_SHIFT)
#define LIS2DH_FS_BITS			(LIS2DH_FS_SELECT(LIS2DH_FS_IDX))
#if defined(CONFIG_LIS2DH_OPER_MODE_HIGH_RES)
	#define LIS2DH_HR_BIT		BIT(3)
#else
	#define LIS2DH_HR_BIT		0
#endif

#define LIS2DH_REG_CTRL5		0x24
#define LIS2DH_LIR_INT2_SHIFT		1
#define LIS2DH_EN_LIR_INT2		BIT(LIS2DH_LIR_INT2_SHIFT)

#define LIS2DH_REG_CTRL6		0x25
#define LIS2DH_EN_INT2_INT2_SHIFT	5
#define LIS2DH_EN_INT2_INT2		BIT(LIS2DH_EN_INT2_INT2_SHIFT)

#define LIS2DH_REG_REFERENCE		0x26

#define LIS2DH_REG_STATUS		0x27
#define LIS2DH_STATUS_ZYZ_OVR		BIT(7)
#define LIS2DH_STATUS_Z_OVR		BIT(6)
#define LIS2DH_STATUS_Y_OVR		BIT(5)
#define LIS2DH_STATUS_X_OVR		BIT(4)
#define LIS2DH_STATUS_OVR_MASK		(BIT_MASK(4) << 4)
#define LIS2DH_STATUS_ZYX_DRDY		BIT(3)
#define LIS2DH_STATUS_Z_DRDY		BIT(2)
#define LIS2DH_STATUS_Y_DRDY		BIT(1)
#define LIS2DH_STATUS_X_DRDY		BIT(0)
#define LIS2DH_STATUS_DRDY_MASK		BIT_MASK(4)

#define LIS2DH_REG_ACCEL_X_LSB		0x28
#define LIS2DH_REG_ACCEL_Y_LSB		0x2A
#define LIS2DH_REG_ACCEL_Z_LSB		0x2C
#define LIS2DH_REG_ACCEL_X_MSB		0x29
#define LIS2DH_REG_ACCEL_Y_MSB		0x2B
#define LIS2DH_REG_ACCEL_Z_MSB		0x2D

#define LIS2DH_REG_FIFO_CTRL		0x2E
#define LIS2DH_REG_FIFO_SRC		    0x2F
#define LIS2DH_FIFO_SRC_OVR 		BIT(6)
#define LIS2DH_FIFO_SRC_FSS_MASK 	BIT_MASK(5)

#define LIS2DH_REG_INT1_CFG		0x30
#define LIS2DH_REG_INT2_CFG		0x34
#define LIS2DH_AOI_CFG			BIT(7)
#define LIS2DH_INT_CFG_ZHIE_ZUPE	BIT(5)
#define LIS2DH_INT_CFG_ZLIE_ZDOWNE	BIT(4)
#define LIS2DH_INT_CFG_YHIE_YUPE	BIT(3)
#define LIS2DH_INT_CFG_YLIE_YDOWNE	BIT(2)
#define LIS2DH_INT_CFG_XHIE_XUPE	BIT(1)
#define LIS2DH_INT_CFG_XLIE_XDOWNE	BIT(0)

#define LIS2DH_REG_INT2_SRC		0x35

#define LIS2DH_REG_INT2_THS		0x36

#define LIS2DH_REG_INT2_DUR		0x37

/* sample buffer size includes status register */
#define LIS2DH_BUF_SZ			7


union lis2dh_sample {
	uint8_t raw[LIS2DH_BUF_SZ];
	struct {
		uint8_t status;
		int16_t xyz[3];
	} __packed;
};

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
struct lis2dh_spi_cfg {
	struct spi_config spi_conf;
	const char *cs_gpios_label;
};
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */

union lis2dh_bus_cfg {
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
	uint16_t i2c_slv_addr;
#endif

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
	const struct lis2dh_spi_cfg *spi_cfg;
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */
};

struct lis2dh_config {
	const char *bus_name;
	int (*bus_init)(const struct device *dev);
	const union lis2dh_bus_cfg bus_cfg;
#ifdef CONFIG_LIS2DH_TRIGGER
	const char *irq1_dev_name;
	gpio_pin_t irq1_pin;
	gpio_dt_flags_t irq1_flags;
	const char *irq2_dev_name;
	gpio_pin_t irq2_pin;
	gpio_dt_flags_t irq2_flags;
#endif /* CONFIG_LIS2DH_TRIGGER */
	bool is_lsm303agr_dev;
	bool disc_pull_up;
};

struct lis2dh_transfer_function {
	int (*read_data)(const struct device *dev, uint8_t reg_addr,
			 uint8_t *value, uint8_t len);
	int (*write_data)(const struct device *dev, uint8_t reg_addr,
			  uint8_t *value, uint8_t len);
	int (*read_reg)(const struct device *dev, uint8_t reg_addr,
			uint8_t *value);
	int (*write_reg)(const struct device *dev, uint8_t reg_addr,
			uint8_t value);
	int (*update_reg)(const struct device *dev, uint8_t reg_addr,
			  uint8_t mask, uint8_t value);
};

struct lis2dh_data {
	const struct device *bus;
	const struct lis2dh_transfer_function *hw_tf;

	union lis2dh_sample sample;
	/* current scaling factor, in micro m/s^2 / lsb */
	uint32_t scale;

#ifdef CONFIG_LIS2DH_TRIGGER
	const struct device *dev;
	const struct device *gpio_int1;
	const struct device *gpio_int2;
	struct gpio_callback gpio_int1_cb;
	struct gpio_callback gpio_int2_cb;

	sensor_trigger_handler_t handler_drdy;
	sensor_trigger_handler_t handler_anymotion;
	atomic_t trig_flags;
	enum sensor_channel chan_drdy;

#if defined(CONFIG_LIS2DH_TRIGGER_OWN_THREAD)
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_LIS2DH_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
#elif defined(CONFIG_LIS2DH_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif

#endif /* CONFIG_LIS2DH_TRIGGER */

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
	struct spi_cs_control cs_ctrl;
#endif /* DT_SPI_DEV_HAS_CS_GPIOS(DT_INST(0, st_lis2mdl)) */
};

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
int lis2dh_spi_access(struct lis2dh_data *ctx, uint8_t cmd,
		      void *data, size_t length);
#endif

#define MAX_SAMPLE_CNT (2700) //2 second waveform = 1344Hz * 2 = 2688
#define DEFAULT_ACCEL_SAMPLING_INTERVAL 86400 //24 hours

typedef struct SAMPLE
{
    int16_t x;
    int16_t y;
    int16_t z;
} sample_t;

extern sample_t accelData[MAX_SAMPLE_CNT];


#endif
