
#pragma once
#include <hardware/uart.h>

constexpr uint8_t kRxBuffLengthPow = 5;
constexpr uint8_t kTxBuffLengthPow = 5;
constexpr uint16_t kRxBuffLength = 1 << (kRxBuffLengthPow);
constexpr uint16_t kTxBuffLength = 1 << (kTxBuffLengthPow);
constexpr int kUartRxChannel = 0;
constexpr int kUartTxChannel = 1;
constexpr int kUartTxPin = 0;
constexpr int kUartRxPin = 1;

class DmaUart {
 private:
  uart_inst_t* uart_;

  __attribute__((aligned(kRxBuffLength)))
  uint8_t rx_buffer_[kRxBuffLength];
  uint16_t rx_user_index_; // next index to read
  uint16_t rx_dma_index_;  // next index dma will write

  __attribute__((aligned(kTxBuffLength)))
  uint8_t tx_buffer_[kTxBuffLength];
  uint16_t tx_user_index_;  // next index to write
  uint16_t tx_dma_index_;  // next index dma will read 

 public:
  DmaUart(uart_inst_t* uart, uint baudrate);
  uint16_t write(const uint8_t* data, uint16_t length);
  void flush();
  void write_and_flush(const uint8_t* data, uint16_t length);
  uint16_t read_byte(uint8_t* data);
  uint16_t read(uint8_t* data, uint16_t length);
  uint16_t read_all(uint8_t* data);
  
 private:
  void init_uart(uart_inst_t* uart, uint baudrate);
  void init_dma();
  
};
