// OpenRB-150 高速 USB <-> DYNAMIXEL 桥接固件（替代原厂 usb_to_dynamixel 示例）
//
// 和原厂示例的区别：
//   1. DXL 接收用 DMA 进环形缓冲，不再逐字节中断。SAMD21 48 MHz 下逐字节中断在 2 M（5 µs/字节）就开始丢字节
//      （实测每 7–10 个字节丢 1 个），3 M、4 M 更不可能。
//   2. 波特率 >= 2 Mbps 时 UART 用 8 倍过采样（原厂固定 16 倍，2 M 以上分频不整）。
//      2 M: 48e6/(8*2e6) = 3；3 M: 2；4 M: 1.5（分数模式）。
//   3. DXL 总线供电开关（BDPIN_DXL_PWR_EN）显式打开。
// 波特率跟随 USB 虚拟串口的设置：PC 端 open 成多少，这里就把 DXL 总线设成多少。
// 刷回原厂：Arduino IDE -> File -> Examples -> OpenRB-150 -> usb_to_dynamixel。
// 进 bootloader：双击 RESET（LED 慢闪）。

#include <Arduino.h>
#include "wiring_private.h"

#define USB     Serial
#define DXL     Serial1
#define RXBUF   1024                      // DMA 环形缓冲（2 的幂）
#define DMA_CH  0

static uint8_t  rxbuf[RXBUF] __attribute__((aligned(16)));
static DmacDescriptor dma_desc[1]  __attribute__((aligned(16)));   // 通道 0 的描述符
static DmacDescriptor dma_wrb[1]   __attribute__((aligned(16)));   // 写回区
static uint32_t rd_pos = 0;               // 已转发到的位置
static uint8_t  txbuf[256];
static uint32_t cur_baud = 0;
static uint32_t led_t = 0;

// ---- DMA：SERCOM2 RX -> rxbuf 循环 ----
static void dma_rx_init() {
  PM->AHBMASK.reg  |= PM_AHBMASK_DMAC;
  PM->APBBMASK.reg |= PM_APBBMASK_DMAC;
  DMAC->CTRL.bit.DMAENABLE = 0;
  DMAC->CTRL.bit.SWRST = 1;
  while (DMAC->CTRL.bit.SWRST) {}
  DMAC->BASEADDR.reg = (uint32_t)dma_desc;
  DMAC->WRBADDR.reg  = (uint32_t)dma_wrb;
  DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xF);

  dma_desc[0].BTCTRL.reg  = DMAC_BTCTRL_VALID | DMAC_BTCTRL_BEATSIZE_BYTE | DMAC_BTCTRL_DSTINC |
                            DMAC_BTCTRL_BLOCKACT_NOACT;
  dma_desc[0].BTCNT.reg   = RXBUF;
  dma_desc[0].SRCADDR.reg = (uint32_t)&SERCOM2->USART.DATA.reg;
  dma_desc[0].DSTADDR.reg = (uint32_t)rxbuf + RXBUF;          // 递增时写"结束地址"
  dma_desc[0].DESCADDR.reg = (uint32_t)&dma_desc[0];          // 指回自己 = 循环

  DMAC->CHID.reg = DMAC_CHID_ID(DMA_CH);
  DMAC->CHCTRLA.bit.ENABLE = 0;
  DMAC->CHCTRLA.bit.SWRST = 1;
  while (DMAC->CHCTRLA.bit.SWRST) {}
  DMAC->CHCTRLB.reg = DMAC_CHCTRLB_LVL(0) | DMAC_CHCTRLB_TRIGSRC(SERCOM2_DMAC_ID_RX) | DMAC_CHCTRLB_TRIGACT_BEAT;
  DMAC->CHCTRLA.bit.ENABLE = 1;
  rd_pos = 0;
}

// 现在 DMA 写到了哪：挂起通道让它把 BTCNT 写回，读完立刻恢复（串口有 2 字节缓冲，挂起 1 µs 不会丢）
static inline uint32_t dma_wpos() {
  DMAC->CHID.reg = DMAC_CHID_ID(DMA_CH);
  DMAC->CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_SUSPEND_Val;
  while (!DMAC->CHINTFLAG.bit.SUSP) {}
  uint32_t remaining = dma_wrb[0].BTCNT.reg;
  DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_SUSP;
  DMAC->CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_RESUME_Val;
  // 写回描述符还没被取过（刚启动）时 BTCNT 是 0：当作没数据
  if (remaining == 0 || remaining > RXBUF) return rd_pos;
  return (RXBUF - remaining) & (RXBUF - 1);
}

static void dxl_begin(uint32_t baud) {
  DXL.end();
  DXL.begin(baud);                                   // 引脚、NVIC、16 倍过采样
  if (baud >= 2000000) {                             // 高波特率改成 8 倍过采样
    sercom2.initUART(UART_INT_CLOCK, SAMPLE_RATE_x8, baud);
    sercom2.initFrame(UART_CHAR_SIZE_8_BITS, LSB_FIRST, SERCOM_NO_PARITY, SERCOM_STOP_BIT_1);
    sercom2.initPads(PAD_SERIAL1_TX, PAD_SERIAL1_RX);
    sercom2.enableUART();
  }
  SERCOM2->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_RXC;   // 接收交给 DMA，核心的中断别来读
  NVIC_SetPriority(SERCOM2_IRQn, 0);                 // 发送还是中断驱动，优先级提高
  NVIC_SetPriority(USB_IRQn, 2);
  cur_baud = baud;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);              // DXL 总线供电
  USB.begin(1000000);
  dxl_begin(1000000);
  dma_rx_init();
}

void loop() {
  uint32_t b = USB.baud();
  if (b != cur_baud && b >= 9600) {
    dxl_begin(b);
  }
  // USB -> DXL
  int n = USB.available();
  if (n > 0) {
    if (n > (int)sizeof(txbuf)) n = sizeof(txbuf);
    n = USB.readBytes((char *)txbuf, n);
    DXL.write(txbuf, n);
    led_t = millis();
  }
  // DXL(DMA 环) -> USB
  uint32_t w = dma_wpos();
  if (w != rd_pos) {
    if (w > rd_pos) {
      USB.write(rxbuf + rd_pos, w - rd_pos);
    } else {                                         // 绕回
      USB.write(rxbuf + rd_pos, RXBUF - rd_pos);
      if (w) USB.write(rxbuf, w);
    }
    rd_pos = w;
  }
  digitalWrite(LED_BUILTIN, (millis() - led_t) < 100);
}
