
#include <dma_uart.hpp>
#include <pico/time.h>
#include <hardware/uart.h>
#include <iostream>

int main() {

  DmaUart u(uart0, 115200);
  int i = 0;

  uint8_t user_data[64];

  for (;;) {
     
    std::string s1 = "hello(" + std::to_string(i++) + ")\n\r";
    u.write_and_flush(reinterpret_cast<const uint8_t*>(s1.c_str()), s1.size());
    uint16_t read_size = u.read_all(user_data);
    std::string s2 = "loopback: ";
    std::string s3 = "\n\r";
    if (0 < read_size) {
      u.write(reinterpret_cast<const uint8_t*>(s2.c_str()), s2.size());
      u.write(user_data, read_size);
      u.write_and_flush(reinterpret_cast<const uint8_t*>(s3.c_str()), s3.size());
    }
    
    sleep_ms(2000);
  }
}