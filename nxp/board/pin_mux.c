/* Pin muxing for the ESP32-CAM link test (see cm33_core0/app.h for the wiring table).
 *
 * Written by hand, in the style of the blind-rover firmware. Mux numbers come from the SDK's own
 * FRDM-MCXN947 pin_mux files: FC4_P0 / FC4_P1 (debug UART) are Alt2, FC0_P0 / FC0_P1 on
 * P0_12 / P0_13 are Alt3, GPIO is Alt0. On a LP_FLEXCOMM in UART mode, P0 is RXD and P1 is TXD
 * (the SDK labels P1_8 = FC4_P0 as DEBUG_UART_RX). The input buffer must be enabled on every pin
 * that is read.
 *
 * The link RX pin gets a pull-up: a UART line idles high, so with the ESP32 unplugged the pin
 * reads "idle" instead of floating and inventing bytes.
 */
#include "fsl_common.h"
#include "fsl_port.h"
#include "pin_mux.h"

static void pin(PORT_Type *port, uint32_t n, port_mux_t mux, uint16_t pull)
{
    const port_pin_config_t c = {pull,
                                 kPORT_LowPullResistor,
                                 kPORT_FastSlewRate,
                                 kPORT_PassiveFilterDisable,
                                 kPORT_OpenDrainDisable,
                                 kPORT_LowDriveStrength,
                                 mux,
                                 kPORT_InputBufferEnable,
                                 kPORT_InputNormal,
                                 kPORT_UnlockRegister};
    PORT_SetPinConfig(port, n, &c);
}

void BOARD_InitBootPins(void)
{
    BOARD_InitPins();
}

void BOARD_InitPins(void)
{
    CLOCK_EnableClock(kCLOCK_Port0);
    CLOCK_EnableClock(kCLOCK_Port1);

    pin(PORT1, 8U, kPORT_MuxAlt2, kPORT_PullDisable);    /* FC4_P0  debug UART RX */
    pin(PORT1, 9U, kPORT_MuxAlt2, kPORT_PullDisable);    /* FC4_P1  debug UART TX */

    pin(PORT0, 12U, kPORT_MuxAlt3, kPORT_PullUp);        /* FC0_P0  link RX <- ESP32 IO14  J8[9]  */
    pin(PORT0, 13U, kPORT_MuxAlt3, kPORT_PullDisable);   /* FC0_P1  link TX -> ESP32 IO13  J8[12] */

    pin(PORT0, 10U, kPORT_MuxAlt0, kPORT_PullDisable);   /* LED red   (active low) */
    pin(PORT0, 27U, kPORT_MuxAlt0, kPORT_PullDisable);   /* LED green (active low) */
    pin(PORT1, 2U, kPORT_MuxAlt0, kPORT_PullDisable);    /* LED blue  (active low) */
}
