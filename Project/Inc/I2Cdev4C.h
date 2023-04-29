#ifndef I2CDEV4C_H_
#define I2CDEV4C_H_

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

EXTERNC void DMA_I2C_TX_ISR(void);
EXTERNC void DMA_I2C_RX_ISR(void);
EXTERNC void DMA_I2C_TX_ISR_ERR(void);
EXTERNC void DMA_I2C_RX_ISR_ERR(void);

#endif /* I2CDEV4C_H_ */