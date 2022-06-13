//
//  uart.h
//

void SetupUART(void);
void putch(int ch);     // Write 8-bit value to UART
void putwd(long wd);    // Write 16-bit value to UART
void putswab(long wd);  // Write 16-bit byte swappped to UART
