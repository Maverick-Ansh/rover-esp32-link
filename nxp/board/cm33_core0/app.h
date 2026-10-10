/* ESP32-CAM link test on the FRDM-MCXN947: every pin and peripheral choice in one place.
 *
 *   signal               MCU pin   header    function
 *   link RX  <- ESP32    P0_12     J8[9]     FC0_P0 = LPUART0 receive   (Alt3)
 *   link TX  -> ESP32    P0_13     J8[12]    FC0_P1 = LPUART0 transmit  (Alt3)
 *   GND                  GND       any GND pin on the board, tied to the ESP32-CAM's GND
 *
 * Why these pins: LPUART0 on P0_12 / P0_13 is the pair the SDK's own LPUART examples use on
 * this board (examples/_boards/frdmmcxn947/driver_examples/lpuart/..., readme "J8-9 / J8-12").
 * J8 is the parallel-LCD header, which the rover does not use. None of them clash with the
 * blind-rover firmware's pins (encoders P0_28/29/31/19, L298N P3_18/19 + P4_2/3, LEDs, SW2/SW3),
 * so the link can later live in the same firmware. The Arduino-header UART (FC2 on P4_2 / P4_3)
 * was NOT used because the rover drives the L298N direction inputs from those two pins.
 *
 * Both chips run 3.3 V logic, so the wires go straight across with no level shifter.
 */
#ifndef _APP_H_
#define _APP_H_

#include "pin_mux.h"

/* Must equal LINK_BAUD in esp32cam/LinkTest/LinkTest.ino. From the 12 MHz clock the LPUART gets
 * 923077 baud for 921600 (0.16% off, well inside the 3% a UART tolerates). */
#define LINK_BAUD          921600U
#define LINK_UART          LPUART0
#define LINK_UART_CLK_FREQ CLOCK_GetLPFlexCommClkFreq(0U)
#define LINK_IRQ           LP_FLEXCOMM0_IRQn
#define LINK_IRQ_HANDLER   LP_FLEXCOMM0_IRQHandler

void BOARD_InitHardware(void);

#endif /* _APP_H_ */
