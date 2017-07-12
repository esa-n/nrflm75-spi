#include "rf24.h"

/* ソフトウェアSPI ピン設定 */
#define NRF_SCK     5
#define NRF_MO      6
#define NRF_MI      7
#define NRF_SCK_hi     GPIOC->ODR |=  (1<<NRF_SCK);
#define NRF_SCK_lo     GPIOC->ODR &= ~(1<<NRF_SCK);
#define NRF_MO_hi     GPIOC->ODR |=  (1<<NRF_MO);
#define NRF_MO_lo     GPIOC->ODR &= ~(1<<NRF_MO);

uint8_t spi_fast_shift(uint8_t value)
/* ソフトウェアSPI 送受信 */
{
	uint8_t ret, i;  // ret初期化していないのでワーニングでる

	for (i=0 ; i<8 ;i++) {
        NRF_SCK_lo;
        if (value & 0x80) {  // SPIはMSBファースト
            NRF_MO_hi;
        } else {
            NRF_MO_lo;
        }
        NRF_SCK_hi;
        value = value<<1;

        ret = ret<<1;
        if (GPIOC->IDR & (1<<NRF_MI)) ret += 1;
    }
    NRF_SCK_lo;
    return ret;
}

void spi_transmit_sync(uint8_t *value, uint8_t len)
/* SPI 連続通信 */
{
	uint8_t i;

	for (i=0 ; i<len ; i++) {
		spi_fast_shift(value[i]);
	}
}

void mirf_write_register(uint8_t reg, uint8_t value)
/* MiRF レジスタライト */
{
	mirf_CSN_lo;
	spi_fast_shift(W_REGISTER | (REGISTER_MASK & reg));
	spi_fast_shift(value);
	mirf_CSN_hi;
}

uint8_t mirf_read_register(uint8_t reg)
/* MiRF レジスタリード */
{
	uint8_t ret;

	mirf_CSN_lo;
	spi_fast_shift(R_REGISTER | (REGISTER_MASK & reg));
	ret = spi_fast_shift(0);
	mirf_CSN_hi;

	return ret;
}

void mirf_address(uint8_t reg, uint8_t value)
/* MiRF アドレス設定 */
{
	uint8_t addr[] = { 0xF0, 0xFA, 0x12, 0x88, 0x64 };
	
	mirf_CSN_lo;
	spi_fast_shift(W_REGISTER | (REGISTER_MASK & reg));
	spi_transmit_sync(value + addr,5);
	mirf_CSN_hi;
}

void mirf_init(void)
/* MiRF 端子初期化 */
{
	//CE->PC4 CSN->PD2 IRQ->PD3 SPI->SCK/MO/MI
    //GPIO Setting
    GPIOC->DDR |= (1<<NRF_CE);
    GPIOA->DDR |= (1<<NRF_CSN);
    GPIOA->DDR &= ~(1<<NRF_IRQ);
    //Software SPI GPIO Setting
    GPIOC->DDR |= ( (1<<NRF_SCK) | (1<<NRF_MO) );
    GPIOC->DDR &= ~(1<<NRF_MI);
    //IO Init
	mirf_CE_lo;
	mirf_CSN_hi;
}

void mirf_config(void)
/* デバイス初期設定 */
{
	// 自動ACK
	mirf_write_register(EN_AA,0x3f);
	// 送信アドレス
	mirf_address(TX_ADDR,0);
	// 受信パイプ0アドレス
	mirf_address(RX_ADDR_P0,0);
	// チャンネル設定
	mirf_write_register(RF_CH,mirf_CH);
	// RFセットアップ 250kbps 0dbm
	mirf_write_register(RF_SETUP,0x26);
    // リトライ回数設定 250us 15回
    mirf_write_register(SETUP_RETR,0x2f);
	// パイプ０のペイロード長設定 
	mirf_write_register(RX_PW_P0, mirf_PAYLOAD);
	// PWR_UP:1/PRIM_RX:1 受信モードでパワーアップ
	//RX_POWERUP;
	//受信動作開始
	//mirf_CE_hi;
}

void mirf_send(uint8_t *data) 
/* データ送信 */
{
	mirf_CE_lo;
	TX_POWERUP;  // TXパワーアップ

	mirf_CSN_lo;  // CSロー
	spi_fast_shift( FLUSH_TX );  // SPIコマンド FLUSH_TX (TX FIFO 送信待ち一括消去)
	mirf_CSN_hi;  // CSハイ

	mirf_CSN_lo;  // CSロー
	spi_fast_shift( W_TX_PAYLOAD );  // SPIコマンド W_TX_PAYLOAD (TXペイロード書き込み)
	spi_transmit_sync(data, mirf_PAYLOAD);  // ペイロード書き込み
	mirf_CSN_hi;  // CSハイ

	mirf_CE_hi;  // 送信開始パルス
	__asm__ ("nop");
	__asm__ ("nop");
	mirf_CE_lo;
}

void mirf_read(uint8_t *data) 
/* データ受信 */
{
	uint8_t i;

	mirf_CSN_lo;  // CS ロー
	spi_fast_shift( R_RX_PAYLOAD );  // SPIコマンド R_RX_PAYLOAD (RXペイロード読み出し)
	for (i=0 ; i<mirf_PAYLOAD ; i++) {
		data[i] = spi_fast_shift(0x00);  // ペイロード読み込み
	}
	mirf_CSN_hi;  // CS ハイ
	
	mirf_write_register(STATUS,(1<<RX_DR));   // Reset status register
}
