#ifndef rf24_h
#define rf24_h

/* レジスタマップ */
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17

/* ビット演算マクロ */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0
#define AW          0
#define ARD         4
#define ARC         0
#define PLL_LOCK    4
#define RF_DR       3
#define RF_PWR      1
#define LNA_HCURR   0        
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0

/* SPIコマンド */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

/* GPIO設定 */
// CE->PC4 CSN->PA2 IRQ->PA1 SPI->SCK/MO/MI
#define NRF_CE  4
#define NRF_CSN 2
#define NRF_IRQ 1

#define mirf_CSN_hi     GPIOA->ODR |=  (1<<NRF_CSN);
#define mirf_CSN_lo     GPIOA->ODR &= ~(1<<NRF_CSN);
#define mirf_CE_hi       GPIOC->ODR |=  (1<<NRF_CE);
#define mirf_CE_lo       GPIOC->ODR &= ~(1<<NRF_CE);

/* チャンネル、ペイロード設定 */
#define mirf_CH         50
#define mirf_PAYLOAD    5

// 割り込み全て禁止
//#define mirf_CONFIG     ( (1<<MASK_RX_DR) | (1<<MASK_TX_DS) | (1<<MASK_MAX_RT) | (1<<EN_CRC) | (0<<CRCO) )
// 割り込み許可
#define mirf_CONFIG     ( (1<<EN_CRC) | (1<<CRCO) )
// Defines for setting the MiRF registers for transmitting or receiving mode
#define TX_POWERUP mirf_write_register(CONFIG, mirf_CONFIG | ( (1<<PWR_UP) | (0<<PRIM_RX) ) )
#define RX_POWERUP mirf_write_register(CONFIG, mirf_CONFIG | ( (1<<PWR_UP) | (1<<PRIM_RX) ) )
#define POWERDOWN mirf_write_register(CONFIG, mirf_CONFIG | ( (0<<PWR_UP) | (0<<PRIM_RX) ) )

#include "stm8s.h"

uint8_t spi_fast_shift(uint8_t value);
void spi_transmit_sync(uint8_t *value, uint8_t len);
void mirf_write_register(uint8_t reg, uint8_t value);
uint8_t mirf_read_register(uint8_t reg);
void mirf_address(uint8_t reg, uint8_t value);
void mirf_init(void);
void mirf_config(void);
void mirf_send(uint8_t *data);
void mirf_read(uint8_t *data);

#endif
