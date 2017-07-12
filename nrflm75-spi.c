#include "stm8s.h"
#include "rf24.h"

/* ./stm8flash -c stlinkv2 -p stm8s003f3 -w nrflm75-spi.ihx */

//ノードID
#define Node_ID 2

// I2Cパラメータ
#define I2C_MAX_STANDARD_FREQ ((uint32_t)100000)
#define I2C_MAX_FAST_FREQ     ((uint32_t)400000)
#define I2C_MAX_INPUT_FREQ     ((uint8_t)16)
#define CLOCK_SPEED 100000
#define SLAVE_ADDRESS 0x30<<1
#define INPUT_CLOCK 2

// LM75
#define LM75_ADDRESS 0x48
#define LM75_TEMP_REGISTER 0
#define LM75_CONF_REGISTER 1
#define LM75_THYST_REGISTER 2
#define LM75_TOS_REGISTER 3
#define LM75_CONF_SHUTDOWN  0
#define LM75_CONF_OS_COMP_INT 1
#define LM75_CONF_OS_POL 2
#define LM75_CONF_OS_F_QUE 3

// PA3 NRF電源
#define NRF_POWER_pin 3
#define NRF_ON    GPIOA->ODR |= (1<<NRF_POWER_pin)
#define NRF_OFF  GPIOA->ODR &= ~(1<<NRF_POWER_pin)

// PD4 基準電圧LED
#define LEDa_pin 4
#define LEDa_hi    GPIOD->ODR |= (1<<LEDa_pin)
#define LEDa_lo    GPIOD->ODR &= ~(1<<LEDa_pin)

void TIM2_SetCounter(uint16_t Counter)
{
  /* カウンタセット */
  TIM2->CNTRH = (uint8_t)(Counter >> 8);
  TIM2->CNTRL = (uint8_t)(Counter);
}

void TIM2_SetAutoreload(uint16_t Autoreload)
{
  /* ARRレジスタセット */
  TIM2->ARRH = (uint8_t)(Autoreload >> 8);
  TIM2->ARRL = (uint8_t)(Autoreload);
}

/* タイマー2 wait利用で初期化 */
void TIM2_Wait_Init(void)
{
    /* 更新割り込み無効 */
    TIM2->IER &= ~TIM2_IER_UIE;
	/* カウンタストップ */
	TIM2->CR1 &= ~TIM2_CR1_CEN;
    /* ワンパルスモード有効 */
    TIM2->CR1 |= TIM2_CR1_OPM;
}

/* マイクロ秒wait タイマー2 */
void wait(uint16_t t)
{
    if ( t < 100 ) return;
    t = ((t - 100) / 10) * 20;  // 10usec*t (t*2で10usec)
    TIM2_SetCounter(0);
    TIM2_SetAutoreload(t);
    /* set the CEN Bit */
    TIM2->CR1 |= (uint8_t)TIM2_CR1_CEN;
    while(TIM2->CR1 & TIM2_CR1_CEN) ;  // CENがクリアされるまでループ
}

/* ミリ秒wait タイマー2 */
void wait_ms(uint16_t t)
{
    uint16_t i;
    for (i=0 ; i<t ; i++){
        TIM2_SetCounter(0);
        TIM2_SetAutoreload(2000);
        /* set the CEN Bit */
        TIM2->CR1 |= (uint8_t)TIM2_CR1_CEN;
        while(TIM2->CR1 & TIM2_CR1_CEN) ;  // CENがクリアされるまでループ
    }
}

/* I2C初期化関数はライブラリを使用(Ackは内蔵して余計なのは消した) */
void I2C_Init(uint32_t OutputClockFrequencyHz, uint16_t OwnAddress, 
              I2C_DutyCycle_TypeDef I2C_DutyCycle, I2C_Ack_TypeDef Ack, 
              I2C_AddMode_TypeDef AddMode, uint8_t InputClockFrequencyMHz )
{
  uint16_t result = 0x0004;
  uint16_t tmpval = 0;
  uint8_t tmpccrh = 0;

  /*------------------------- I2C FREQ Configuration ------------------------*/
  /* Clear frequency bits */
  I2C->FREQR &= (uint8_t)(~I2C_FREQR_FREQ);
  /* Write new value */
  I2C->FREQR |= InputClockFrequencyMHz;

  /*--------------------------- I2C CCR Configuration ------------------------*/
  /* Disable I2C to configure TRISER */
  I2C->CR1 &= (uint8_t)(~I2C_CR1_PE);

  /* Clear CCRH & CCRL */
  I2C->CCRH &= (uint8_t)(~(I2C_CCRH_FS | I2C_CCRH_DUTY | I2C_CCRH_CCR));
  I2C->CCRL &= (uint8_t)(~I2C_CCRL_CCR);

  /* Detect Fast or Standard mode depending on the Output clock frequency selected */
  if (OutputClockFrequencyHz > I2C_MAX_STANDARD_FREQ) /* FAST MODE */
  {
    /* Set F/S bit for fast mode */
    tmpccrh = I2C_CCRH_FS;

    if (I2C_DutyCycle == I2C_DUTYCYCLE_2)
    {
      /* Fast mode speed calculate: Tlow/Thigh = 2 */
      result = (uint16_t) ((InputClockFrequencyMHz * 1000000) / (OutputClockFrequencyHz * 3));
    }
    else /* I2C_DUTYCYCLE_16_9 */
    {
      /* Fast mode speed calculate: Tlow/Thigh = 16/9 */
      result = (uint16_t) ((InputClockFrequencyMHz * 1000000) / (OutputClockFrequencyHz * 25));
      /* Set DUTY bit */
      tmpccrh |= I2C_CCRH_DUTY;
    }

    /* Verify and correct CCR value if below minimum value */
    if (result < (uint16_t)0x01)
    {
      /* Set the minimum allowed value */
      result = (uint16_t)0x0001;
    }

    /* Set Maximum Rise Time: 300ns max in Fast Mode
    = [300ns/(1/InputClockFrequencyMHz.10e6)]+1
    = [(InputClockFrequencyMHz * 3)/10]+1 */
    tmpval = ((InputClockFrequencyMHz * 3) / 10) + 1;
    I2C->TRISER = (uint8_t)tmpval;

  }
  else /* STANDARD MODE */
  {

    /* Calculate standard mode speed */
    result = (uint16_t)((InputClockFrequencyMHz * 1000000) / (OutputClockFrequencyHz << (uint8_t)1));

    /* Verify and correct CCR value if below minimum value */
    if (result < (uint16_t)0x0004)
    {
      /* Set the minimum allowed value */
      result = (uint16_t)0x0004;
    }

    /* Set Maximum Rise Time: 1000ns max in Standard Mode
    = [1000ns/(1/InputClockFrequencyMHz.10e6)]+1
    = InputClockFrequencyMHz+1 */
    I2C->TRISER = (uint8_t)(InputClockFrequencyMHz + (uint8_t)1);

  }

  /* Write CCR with new calculated value */
  I2C->CCRL = (uint8_t)result;
  I2C->CCRH = (uint8_t)((uint8_t)((uint8_t)(result >> 8) & I2C_CCRH_CCR) | tmpccrh);

  /* Enable I2C */
  I2C->CR1 |= I2C_CR1_PE;

  /* Configure I2C acknowledgement */
  if (Ack == I2C_ACK_NONE)
  {
    /* Disable the acknowledgement */
    I2C->CR2 &= (uint8_t)(~I2C_CR2_ACK);
  }
  else
  {
    /* Enable the acknowledgement */
    I2C->CR2 |= I2C_CR2_ACK;

    if (Ack == I2C_ACK_CURR)
    {
      /* Configure (N)ACK on current byte */
      I2C->CR2 &= (uint8_t)(~I2C_CR2_POS);
    }
    else
    {
      /* Configure (N)ACK on next byte */
      I2C->CR2 |= I2C_CR2_POS;
    }
  }

  /*--------------------------- I2C OAR Configuration ------------------------*/
  I2C->OARL = (uint8_t)(OwnAddress);
  I2C->OARH = (uint8_t)((uint8_t)(AddMode | I2C_OARH_ADDCONF) |
                   (uint8_t)((OwnAddress & (uint16_t)0x0300) >> (uint8_t)7));
}

/* タイムアウトタイマー タイマー2 */
void timeout_timer (uint16_t t)
{
	TIM2_SetCounter(0);
	TIM2_SetAutoreload(t);
	/* set the CEN Bit */
	TIM2->CR1 |= (uint8_t)TIM2_CR1_CEN;  // タイマースタート
}

/* タイムアウト付きレジスタセットまち*/
uint8_t i2c_wait (uint8_t reg, uint8_t val)
{
	timeout_timer(600);  // タイマースタート 300usec (300x2)
	while (!(*(volatile uint8_t *)(0x5210 + reg) & val))  // レジスタセットまち
	{
		if ((TIM2->CR1 & TIM2_CR1_CEN) == 0)  // タイムアウト
			goto err;
	}
	return 0;
err:
	return 1;
}

/* addr書き込んでレジスタ２回読み(16bit) */
uint16_t lm75_register16 (uint8_t addr_reg)
{
	uint8_t null;
	uint16_t ret;
	I2C->CR2 |= I2C_CR2_START;  // START bit
	if (i2c_wait(0x07, I2C_SR1_SB)) goto err;  // SBセットまち
	I2C->DR = (LM75_ADDRESS<<1) + 0;  // Slabe ADDR + Write
	if (i2c_wait(0x07, I2C_SR1_ADDR)) goto err;  // ADDRセットまち
	null = I2C->SR3;
	if (i2c_wait(0x07, I2C_SR1_TXE)) goto err;  // TXEセットまち
	I2C->DR = addr_reg;
	if (i2c_wait(0x07, I2C_SR1_BTF)) goto err;  // BTFまち
	I2C->CR2 |= I2C_CR2_STOP;  // STOP bit
	
	I2C->CR2 |= I2C_CR2_START;  // START bit
	if (i2c_wait(0x07, I2C_SR1_SB)) goto err;  // SBセットまち
	I2C->DR = (LM75_ADDRESS<<1) + 1;  // Slabe ADDR + Read
	if (i2c_wait(0x07, I2C_SR1_ADDR)) goto err;  // ADDRセットまち
	null = I2C->SR3;
	/* BTFセットまち BTFはDR(N-1)とシフトレジスタ(N)が一杯になるとセットされる N-1とNのデータは合計２個のタイミング*/
	if (i2c_wait(0x07, I2C_SR1_BTF)) goto err;
	I2C->CR2 |= I2C_CR2_STOP;  // STOP bit
	/* DRを続けて読むが、この時エラッタではSTOPとN-1の間は割り込みを全てマスクしないとデータ化けるとある */
	ret = (uint16_t)(I2C->DR)<<8;  // N-1
	ret += (uint16_t)(I2C->DR);  // N

	i2c_wait(0x09, I2C_SR3_BUSY);  //BUSY 通信終了まち
	return ret;
err:
	I2C->CR2 |= I2C_CR2_STOP;  // STOP bit
	return 0;
}

/* AWU割り込み */
void  AWU_IRQHandler (void) __interrupt (1)
{
	uint8_t null;
	__asm__ ("nop");
	AWU->TBR = 0;
	null = AWU->CSR;  //フラグクリア
}

/* アクティブHALT */
void awu_halt (void)
{
	/* パワーダウン */
	mirf_CE_lo;
	POWERDOWN;
	/* SPI */
	mirf_CSN_lo;  // リーク電流対策
	NRF_OFF;  // lm75 電源オフ

	/* AWU設定 */
	AWU->APR = 0x3e;
	AWU->TBR = 0x0f;
	AWU->CSR = 0x10;
	halt();
}

void ack (void)
{
	uint8_t ret;
	uint16_t timeout = 0;

	timeout_timer(2000);  // タイマースタート 1msec
	while(1)  //  ACK受信ループ
	{
		if ( (GPIOA->IDR & (1<<NRF_IRQ) ) == 0 ) {  //NRF割り込み発生
			mirf_CSN_lo;
			ret = spi_fast_shift(NOP);  //ステータス読込
			mirf_CSN_hi;

			if ( (ret  & (1<<TX_DS) ) != 0 ) {  //送信完了フラグ
				mirf_write_register(STATUS,(1<<TX_DS));
				//LEDa_hi;  // 通信完了表示
				//wait_ms(50);
				//LEDa_lo;
				break;
			}
			if ( (ret  & (1<<MAX_RT) ) != 0 ) {  //MAX_RTフラグ
				mirf_write_register(STATUS,(1<<MAX_RT));
				LEDa_lo;  // 通信エラー表示
				wait_ms(100);
				LEDa_hi;
				wait_ms(50);
				LEDa_lo;
				break;
			}
		}

		/* タイムアウト処理 */
		if ((TIM2->CR1 & TIM2_CR1_CEN) == 0)  //  タイマー停止
		{
			timeout++;
			timeout_timer(2000);  // タイマー再スタート 1msec
		}
		if (timeout > 2000)  // 500msec
		{
			// タイムアウト
			//mirf_config();  // 通信初期化
			break;
		}
	}
}

void main(void)
{
	uint8_t data[10], temph, templ, pid;
	uint16_t ret;  //  LM75用16bit

	/* GPIO Setting */
    GPIOA->DDR |= (1<<NRF_POWER_pin);
    GPIOA->CR1 |= (1<<NRF_POWER_pin);
    GPIOD->DDR |= (1<<LEDa_pin);
    GPIOD->CR1 |= (1<<LEDa_pin);
    // 未使用ピンの処理(出力に設定)
    // これをしないと未使用入力ポートが暴れてHALT中電流増加
    GPIOC->DDR |= (1<<3);
    GPIOD->DDR |= ( (1<<2) | (1<<5) | (1<<6) );

	/* waitタイマー nrf初期化 */
	TIM2_Wait_Init();
	mirf_init();
	
	/* HALT設定 */
	CLK->ICKR |= CLK_ICKR_SWUAH;  // REGAH HALT中メインレギュレータオフ
	FLASH->CR1 |= FLASH_CR1_AHALT;  // AHELT中フラッシュパワーダウン

     /* I2C初期設定 割り込み許可 */
	I2C_Init(CLOCK_SPEED, SLAVE_ADDRESS, I2C_DUTYCYCLE_2, I2C_ACK_CURR, I2C_ADDMODE_7BIT, INPUT_CLOCK);

	pid = 0;
    // メインループ
    while(1){
		/* 初期設定 */
		LEDa_hi;  // 基準電圧兼用LED点灯
		ADC1->CSR |=  4;  // ADC入力チャンネル
		ADC1->CR1 |= ADC1_CR1_ADON;  // ADCオン
		ADC1->CR1 |= ADC1_CR1_ADON;  // 変換開始
		NRF_ON;  // LM75電源オン
		wait_ms(50);
		mirf_CSN_hi;  // nrf24 CSN
		mirf_config();
		TX_POWERUP;  // 早めに動かさないとパワーダウンから起きない
		
		/* 温度取得 */
		wait_ms(100);  //センサーが起きるの非常に遅い
		ret = lm75_register16(LM75_TEMP_REGISTER);
		/* バッテリー電圧取得 */
		ADC1->CSR &= ~ADC1_CSR_EOC;  //  フラグクリア
		temph = ADC1->DRH;
		templ = ADC1->DRL;
		ADC1->CR1 &= ~ADC1_CR1_ADON;  // ADC停止

		/* データ格納 送信 */
		if (pid > 0xfe) pid = 0;  // 同一パケット送信対策
		data[0] = Node_ID;  //  ノードID
		data[1] = (uint8_t)(ret>>8);
		data[2] = (uint8_t)ret;
		data[3] = temph;
		data[4] = pid++;
		mirf_send(data);
		ack();

		LEDa_lo;  //  基準電圧兼用LEDオフ
		awu_halt();
	}
}
