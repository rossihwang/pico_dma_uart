
#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/uart.h>
#include <string.h>

#include <dma_uart.hpp>

DmaUart::DmaUart(uart_inst_t* uart, uint baudrate)
    : uart_(uart),
      rx_user_index_(0),
      rx_dma_index_(0),
      tx_user_index_(0),
      tx_dma_index_(0) {
  init_uart(uart_, baudrate);
  init_dma();
}

void DmaUart::init_uart(uart_inst_t* uart, uint baudrate) {
  gpio_set_function(kUartTxPin, GPIO_FUNC_UART);
  gpio_set_function(kUartRxPin, GPIO_FUNC_UART);
  uart_init(uart, baudrate);
}

static void dma_irq_handler() {
  dma_hw->ints0 = 1u << kUartRxChannel;
  dma_channel_set_trans_count(kUartRxChannel, kRxBuffLength, true);
}

void DmaUart::init_dma() {
  /// DMA uart read
  dma_channel_config rx_config = dma_channel_get_default_config(kUartRxChannel);
  channel_config_set_transfer_data_size(&rx_config, DMA_SIZE_8);
  channel_config_set_read_increment(&rx_config, false);
  channel_config_set_write_increment(&rx_config, true);
  channel_config_set_ring(&rx_config, true, kRxBuffLengthPow);
  channel_config_set_dreq(&rx_config, DREQ_UART0_RX);
  channel_config_set_enable(&rx_config, true);
  dma_channel_configure(kUartRxChannel, &rx_config, rx_buffer_, &uart0_hw->dr,
                        kRxBuffLength, true);
  dma_channel_set_irq0_enabled(kUartRxChannel, true);

  irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
  irq_set_enabled(DMA_IRQ_0, true);

  /// DMA uart write
  dma_channel_config tx_config = dma_channel_get_default_config(kUartTxChannel);
  channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_8);
  channel_config_set_read_increment(&tx_config, true);
  channel_config_set_write_increment(&tx_config, false);
  channel_config_set_ring(&tx_config, false, kTxBuffLengthPow);
  channel_config_set_dreq(&tx_config, DREQ_UART0_TX);
  dma_channel_set_config(kUartTxChannel, &tx_config, false);
  dma_channel_set_write_addr(kUartTxChannel, &uart0_hw->dr, false);
}

uint16_t DmaUart::write(const uint8_t* data, uint16_t length) {
  if (length == 0) {
    return 0;
  }
  uint16_t available =
      (tx_dma_index_ <= tx_user_index_)
          ? (kTxBuffLength - 1 + tx_dma_index_ - tx_user_index_)
          : (tx_dma_index_ - tx_user_index_);
  if (length <= available) {
    if (tx_dma_index_ < tx_user_index_) {
      if ((kTxBuffLength - 1) < tx_user_index_ + length) {
        memcpy(&tx_buffer_[tx_user_index_], data,
               kTxBuffLength - tx_user_index_);
        memcpy(tx_buffer_, &data[kTxBuffLength - tx_user_index_],
               length - (kTxBuffLength - tx_user_index_));
      } else {
        memcpy(&tx_buffer_[tx_user_index_], data, length);
      }

    } else {
      if ((kTxBuffLength - 1) < tx_user_index_ + length) {
        memcpy(&tx_buffer_[tx_user_index_], data,
               kTxBuffLength - tx_user_index_);
        memcpy(tx_buffer_, &data[kTxBuffLength - tx_user_index_],
               length - (kTxBuffLength - tx_user_index_));
      } else {
        memcpy(&tx_buffer_[tx_user_index_], data, length);
      }
    }
    tx_user_index_ = (tx_user_index_ + length) & (kTxBuffLength - 1);
  } else {  // no enougth space to write
    // TODO: write as much data as possible
    return 0;
  }

  return length;
}

void DmaUart::flush() {
  uint size = (tx_dma_index_ <= tx_user_index_)
                  ? (tx_user_index_ - tx_dma_index_)
                  : (kTxBuffLength + tx_user_index_ - tx_dma_index_);
  // Size check
  if (size == 0) {
    return;
  }
  // Busy check
  while (dma_channel_is_busy(kUartTxChannel))
    ;

  uint8_t* start = &tx_buffer_[tx_dma_index_];
  dma_channel_transfer_from_buffer_now(kUartTxChannel, start, size);
  tx_dma_index_ = (tx_dma_index_ + size) & (kTxBuffLength - 1);
}

void DmaUart::write_and_flush(const uint8_t* data, uint16_t length) {
  write(data, length);
  flush();
}

uint16_t DmaUart::read_byte(uint8_t* data) {
  // Update dma index
  rx_dma_index_ =
      kRxBuffLength - dma_channel_hw_addr(kUartRxChannel)->transfer_count;

  if (rx_dma_index_ == rx_user_index_) {
    return -1;
  }
  *data = rx_buffer_[rx_user_index_];
  rx_user_index_ = (rx_user_index_ + 1) & (kRxBuffLength - 1);
  return 0;
}

uint16_t DmaUart::read(uint8_t* data, uint16_t length) {
  // Update DMA index
  rx_dma_index_ =
      kRxBuffLength - dma_channel_hw_addr(kUartRxChannel)->transfer_count;

  uint16_t available;
  available = (rx_user_index_ <= rx_dma_index_)
                  ? (rx_dma_index_ - rx_user_index_)
                  : (kRxBuffLength + rx_dma_index_ - rx_user_index_);
  
  if (available < length) {
    // read as much as we have
    length = available;
  }
  
  if (length == 0) {
    return 0;
  }

  if (rx_user_index_ < rx_dma_index_) {
    memcpy(data, &rx_buffer_[rx_user_index_], length);
  } else {
    uint16_t left = kRxBuffLength - rx_user_index_;

    if (length < left) {
      // limit to target buffer size!
      left = length;
    }

    memcpy(data, &rx_buffer_[rx_user_index_], left);

    if (left < length) {
      memcpy(&data[left], rx_buffer_, length - left);
    }
  }
  rx_user_index_ = (rx_user_index_ + length) & (kRxBuffLength - 1);

  return length;
}

uint16_t DmaUart::read_all(uint8_t* data) {
  // Update DMA index
  rx_dma_index_ =
      kRxBuffLength - dma_channel_hw_addr(kUartRxChannel)->transfer_count;

  uint16_t available;
  available = (rx_user_index_ <= rx_dma_index_)
                  ? (rx_dma_index_ - rx_user_index_)
                  : (kRxBuffLength + rx_dma_index_ - rx_user_index_);

  if (0 < available) {
    if (rx_user_index_ < rx_dma_index_) {
      memcpy(data, &rx_buffer_[rx_user_index_], available);
    } else {
      uint16_t left = kRxBuffLength - rx_user_index_;
      memcpy(data, &rx_buffer_[rx_user_index_], left);
      memcpy(&data[left], rx_buffer_, available - left);
    }
    rx_user_index_ = (rx_user_index_ + available) & (kRxBuffLength - 1);
    return available;
  }
  return 0;
}
