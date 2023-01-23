#include "WindowCoveringImpl.h"

#include <app/clusters/window-covering-server/window-covering-server.h>
#include <app-common/zap-generated/attributes/Accessors.h>
#include <app-common/zap-generated/cluster-objects.h>

#include <matter_config.h>
#include <DeviceInfoProviderImpl.h>

using namespace chip;
using namespace chip::app::Clusters;
using namespace chip::app::Clusters::WindowCovering;
using namespace ::chip::DeviceLayer;

#include "stdbool.h"
#include "em_usart.h"
#include "cmsis_os2.h"
#include "sl_cmsis_os2_common.h"
#include "em_gpio.h"
#include "em_cmu.h"

#include "efr32_utils.h"
#include "sl_sleeptimer.h"

#define LIN_START_DELAY_MS 500
__ALIGNED(8) static uint8_t inst_task_stack[4096];
__ALIGNED(4) static uint8_t inst_thread_cb[osThreadCbSize];
osThreadId_t gLinTid = 0;
osSemaphoreId_t gLinSemaphoreId;

volatile uint16_t target_height = 0;
volatile uint16_t current_height = 0;
volatile uint16_t current_target_height = 0;
volatile bool is_moving = false;
volatile bool movement_requested = false;
volatile bool movement_blocked = false;
static uint32_t timeout_ticks;

volatile uint8_t txbuf[8] = {0};
volatile size_t tx_len = 0;
volatile uint8_t rxbuf[10] = {0};
volatile size_t rx_len = 0;
volatile bool has_action = false;

typedef struct {
  uint8_t content[10]; // LIN: PID + max 8 data bytes + checksum
  uint8_t valid;
} lin_register_entry_t;

volatile lin_register_entry_t gLinState[64];

/*
 * Basic target design:
 * TX:
 *    A DMA channel is set up to pipe 'responses' to the USART since the USART's
 *    FIFO is not large enough to contain the longer responses. When a PID is
 *    detected that this device should respond on, the DMA descriptor is updated
 *    with the TX buffer and length containing the response, and kicked off.
 *    During this transmission (right before DMA start and until TXIDLE) RX is
 *    blocked to avoid taking a bunch of needless interrupts.
 * RX:
 *    Incoming messages are logged to a global array. For certain PIDs (which
 *    the application cares about), the ISR generates a message queue event with
 *    the received PID.
 *    Practically: we take an RX interrupt on the frame error event. If the RX
 *    byte is 0x00 + frame error, then this is a break condition. We take
 *    another interrupt on the next received byte (the PID, since LINAK skips
 *    sending the SYNC byte). Depending on the PID, we start the TX logic or
 *    start a DMA reception into the PID-offset into the global array.
 *    On the next break event, any running RX DMA channel is aborted and a MSGQ
 *    event is generated.
 */

static uint8_t calc_lin_checksum(volatile uint8_t* buf, size_t buf_len)
{
  uint16_t checksum = 0;
  for (size_t i = 0; i < buf_len; i++) {
    checksum += buf[i];
    if (checksum > 0xFF) {
      checksum = (checksum & 0xFF) + 1;
    }
  }
  return ~((uint8_t)checksum);
}

static void lin_task(void *arg)
{
  (void) arg;

  uint32_t last_tick;
  uint16_t last_height;

  // Setup semaphore to unblock task from ISR
  gLinSemaphoreId = osSemaphoreNew(1U, 0U, 0);

  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_USART0, true);

  // Configure the USART TX pin to the board controller as an output
  GPIO_PinModeSet(gpioPortC, 2, gpioModePushPull, 0);

  // Configure the USART RX pin to the board controller as an input
  GPIO_PinModeSet(gpioPortC, 3, gpioModeInput, 0);

  EFM_ASSERT(osDelay(((uint64_t)osKernelGetTickFreq() * LIN_START_DELAY_MS) / 1000) == osOK);

  // Setup USART and pins
  // Default asynchronous initializer (115.2 Kbps, 8N1, no flow control)
  USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;
  init.baudrate = 19200;
  init.enable = usartDisable;

  // Route USART0 TX and RX to the board controller TX and RX pins
  GPIO->USARTROUTE[0].TXROUTE = (gpioPortC << _GPIO_USART_TXROUTE_PORT_SHIFT)
      | (2 << _GPIO_USART_TXROUTE_PIN_SHIFT);
  GPIO->USARTROUTE[0].RXROUTE = (gpioPortC << _GPIO_USART_RXROUTE_PORT_SHIFT)
      | (3 << _GPIO_USART_RXROUTE_PIN_SHIFT);

  // Enable RX and TX signals now that they have been routed
  GPIO->USARTROUTE[0].ROUTEEN = GPIO_USART_ROUTEEN_RXPEN | GPIO_USART_ROUTEEN_TXPEN;

  // Configure USART0
  USART_InitAsync(USART0, &init);

  // Invert TX since it's driving an N-channel MOSFET gate
  USART0->CTRL_SET = USART_CTRL_TXINV;

  // Set interrupt source to RXDATAV
  USART0->IEN = USART_IEN_RXDATAV;

  // Enable USART0
  USART_Enable(USART0, usartEnable);

  // Enable NVIC USART sources
  NVIC_ClearPendingIRQ(USART0_RX_IRQn);
  NVIC_EnableIRQ(USART0_RX_IRQn);
  NVIC_ClearPendingIRQ(USART0_TX_IRQn);
  NVIC_EnableIRQ(USART0_TX_IRQn);

  while (1) {
    osSemaphoreAcquire(gLinSemaphoreId, osWaitForever);

    // Got poked by ISR
    if (rxbuf[0] == 0x80) {
      uint16_t pos = rxbuf[1] + (rxbuf[2] << 8);
      if (pos != current_height) {
        if (current_height == 0) {
            // We've rebooted, update attribute
            chip::DeviceLayer::PlatformMgr().LockChipStack();
            NPercent100ths percent100ths;
            percent100ths.SetNonNull(LiftToPercent100ths(1, pos));
            LiftPositionSet(1, percent100ths);
            chip::DeviceLayer::PlatformMgr().UnlockChipStack();
        }

        current_height = pos;

        if (is_moving) {
            if (last_height != current_height) {
                last_height = current_height;
                last_tick = sl_sleeptimer_get_tick_count();
            } else if (sl_sleeptimer_get_tick_count() >= last_tick + timeout_ticks) {
                movement_blocked = true;
            }
        }
      }

      // We process the next action to take upon reception of the motor position as
      // a form of rate limiting on the LIN thread.
      if (movement_requested) {
        if (!is_moving) {
            if (current_height != target_height) {
                current_target_height = target_height;
                txbuf[0] = 0xCA;
                txbuf[1] = current_target_height & 0xFF;
                txbuf[2] = (current_target_height >> 8) & 0xFF;
                txbuf[3] = calc_lin_checksum(txbuf, 3);
                tx_len = 4;
                last_tick = sl_sleeptimer_get_tick_count();
                last_height = current_height;
                movement_blocked = false;
                is_moving = true;
            }
        } else if (movement_blocked || (current_height == current_target_height)) {
            // Made it to target, or motor stopped by itself
            tx_len = 0;
            is_moving = false;
            movement_requested = false;
            movement_blocked = false;

            // Update attribute
            chip::DeviceLayer::PlatformMgr().LockChipStack();
            NPercent100ths percent100ths;
            percent100ths.SetNonNull(LiftToPercent100ths(1, current_height));
            LiftPositionSet(1, percent100ths);
            chip::DeviceLayer::PlatformMgr().UnlockChipStack();
        } else if (target_height != current_target_height) {
            // Target was updated while in motion, need to handle this by allowing motor to stop and restart movement
            // TODO
        }
      } else {
        if (is_moving) {
            // We were moving but got aborted. Report current state and reset.
            tx_len = 0;
            is_moving = false;
            movement_requested = false;
            movement_blocked = false;

            // Update attribute
            chip::DeviceLayer::PlatformMgr().LockChipStack();
            NPercent100ths percent100ths;
            percent100ths.SetNonNull(LiftToPercent100ths(1, current_height));
            LiftPositionSet(1, percent100ths);
        }
      }
    }
  }
}

void USART0_TX_IRQHandler(void)
{
  // TXIDLE (after starting DMA op) means transaction is done and we can restart
  // the RX chain.

}

// PRBS (safety sequence) responses
static const uint8_t prbs_sequence[9][2] = {
    {0x3F, 0xD8},
    {0xDF, 0x38},
    {0xCF, 0x48},
    {0xD7, 0x40},
    {0xC3, 0x54},
    {0xDD, 0x3A},
    {0xCC, 0x4B},
    {0x55, 0xC2},
    {0x80, 0x97},
};

void USART0_RX_IRQHandler(void)
{
  static int current_pid = -1;
  static size_t rxd_len = 0;
  static size_t prbs_seq = 0;

  while(USART0->STATUS & USART_STATUS_RXDATAV) {
    uint32_t received = USART0->RXDATAX;

    if (((received & 0xFF) == 0) && (received & USART_RXDATAX_FERR)) {
      // Break received, poke thread if needed and reset reception
      if (current_pid >= 0) {
        rxbuf[0] = current_pid;
        rx_len = rxd_len;
        osSemaphoreRelease(gLinSemaphoreId);
      }

      rxd_len = 0;
      current_pid = -1;
    } else if ((received & USART_RXDATAX_FERR) == 0) {
      uint8_t byte = received & 0xFFUL;
      // Non-break character received, figure out what to do
      if (rxd_len == 0) {
        // This is the PID. It determines what action we can take
        switch (byte) {
          case 0x80:
            // Ref1 position PID -> motor will answer
            current_pid = 0x80;
            break;
          case 0x64:
            // Request power -> always respond
            // Write TX data directly since we can (TX FIFO can contain two bytes)
            USART0->TXDATA = 0x9A;
            USART0->TXDATA = 0x01;
            break;
          case 0xE7:
            // Safety sequence -> respond in sequence for as long as there is an active command
            current_pid = -1; // Discard - do not need to make thread aware of PRBS

            if (tx_len > 0) {
              // Write TX data directly since we can (TX FIFO can contain two bytes)
              USART0->TXDATA = prbs_sequence[prbs_seq][0];
              USART0->TXDATA = prbs_sequence[prbs_seq][1];

              // Progress with next PRBS sequence (unless it's updated again by HS2)
              prbs_seq += 1;
              if (prbs_seq >= 9) prbs_seq = 0;
            }
            break;
          case 0xCA:
            // Ref1 input -> respond with command if actual
            if (tx_len == 4 && txbuf[0] == 0xCA) {
              USART0->TXDATA = txbuf[1];
              USART0->TXDATA = txbuf[2];
              USART0->TXDATA = txbuf[3];
            }
            break;
          default:
            current_pid = -1; // Set PID to discard this message
            break;
        }
      } else if (rxd_len < sizeof(rxbuf)) {
        if (current_pid >= 0) {
          rxbuf[rxd_len] = byte;
        }
      }
      rxd_len += 1;
    }
  }
}

void WindowCoveringImpl::Init(void)
{
    sl_sleeptimer_init();
    // Precalculate tick value
    uint32_t ms = 500;
    sl_status_t result = sl_sleeptimer_ms32_to_tick(ms, &timeout_ticks);
    while (result != SL_STATUS_OK) {
      ms /= 2;
      result = sl_sleeptimer_ms32_to_tick(ms, &timeout_ticks);
    }

    // Start thread for LIN bus handling
    osThreadAttr_t attr;

    attr.name = "LIN";
    attr.priority = osPriorityHigh;
    attr.stack_mem = inst_task_stack;  // Will allocate dynamically if set to NULL
    attr.stack_size = 4096;
    attr.cb_mem = inst_thread_cb; // Will allocate dynamically if set to NULL
    attr.cb_size = osThreadCbSize;
    attr.attr_bits = 0u;
    attr.tz_module = 0u;

    gLinTid = osThreadNew(&lin_task, 0, &attr);
    EFM_ASSERT(gLinTid != 0);
}

/**
 * @brief
 *   This method adjusts window covering position so the physical lift/slide and tilt is at the target
 *   open/up position set before calling this method. This will happen as fast as possible.
 *
 *   @param[in]  type            window covering type.
 *
 *   @return CHIP_NO_ERROR On success.
 *   @return Other Value indicating it failed to adjust window covering position.
 */
CHIP_ERROR WindowCoveringImpl::HandleMovement(WindowCoveringType type)
{
    // Need to respond to the server setting Attributes::TargetPositionLiftPercent100ths
    emberAfWindowCoveringClusterPrint("Handle callback to move covering of type %d", (uint8_t)type);
    if (type == WindowCoveringType::Lift) {
        NPercent100ths target;
        Attributes::TargetPositionLiftPercent100ths::Get(1, target);
        uint16_t current_height_pctpct = LiftToPercent100ths(1, is_moving ? current_target_height : current_height);
        if (target.Value() != current_height_pctpct) {
            if (is_moving) {
                // TODO: cancel current movement and start new
                return CHIP_ERROR_INCORRECT_STATE;
            } else {
                target_height = Percent100thsToLift(1, target.Value());
                emberAfWindowCoveringClusterPrint("Set requested height to %d", target_height);
                movement_requested = true;
            }
        }
    }
    return CHIP_NO_ERROR;
}

/**
 * @brief
 *   This method stops any adjusting to the physical tilt and lift/slide that is currently occurring.
 *
 *   @return CHIP_NO_ERROR On success.
 *   @return Other Value indicating it failed to stop any adjusting to the physical tilt and lift/slide that is currently
 * occurring..
 */
CHIP_ERROR WindowCoveringImpl::HandleStopMotion()
{
    movement_requested = false;
    return CHIP_NO_ERROR;
}