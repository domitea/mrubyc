/*! @file
  @brief
  Hardware abstraction layer
        for Zephyr RTOS

  <pre>
  Copyright (C) 2015- Kyushu Institute of Technology.
  Copyright (C) 2015- Shimane IT Open-Innovation Center.
  Copyright (C) 2025 Dominik M.

  This file is distributed under BSD 3-Clause License.
  </pre>
*/

/***** Feature test switches ************************************************/
/***** System headers *******************************************************/
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <string.h>


/***** Local headers ********************************************************/
#include "hal.h"


/***** Constant values ******************************************************/
/***** Macros ***************************************************************/
/***** Typedefs *************************************************************/
/***** Function prototypes **************************************************/
/***** Local variables ******************************************************/
#ifndef MRBC_NO_TIMER
static struct k_timer mrbc_timer;
static unsigned int irq_lock_key;
static int irq_nested = 0;
#endif


/***** Global variables *****************************************************/
/***** Signal catching functions ********************************************/
/***** Local functions ******************************************************/
#ifndef MRBC_NO_TIMER
//================================================================
/*!@brief
  Timer callback function

  Called by Zephyr kernel when timer expires
*/
static void timer_callback(struct k_timer *timer_id)
{
  mrbc_tick();
}


#endif


/***** Global functions *****************************************************/
#ifndef MRBC_NO_TIMER

//================================================================
/*!@brief
  Initialize HAL

  Sets up the periodic timer for mruby/c scheduler
*/
void hal_init(void)
{
  // Initialize and start periodic timer
  k_timer_init(&mrbc_timer, timer_callback, NULL);
  k_timer_start(&mrbc_timer, K_MSEC(MRBC_TICK_UNIT), K_MSEC(MRBC_TICK_UNIT));
}


//================================================================
/*!@brief
  Enable interrupts (exit critical section)

  Note: Zephyr's irq_lock() returns a key that must be passed to irq_unlock()
  We use a simple nesting counter to handle multiple disable/enable pairs
*/
void hal_enable_irq(void)
{
  if (irq_nested > 0) {
    irq_nested--;
    if (irq_nested == 0) {
      irq_unlock(irq_lock_key);
    }
  }
}


//================================================================
/*!@brief
  Disable interrupts (enter critical section)

  Note: Zephyr's irq_lock() returns a key that must be passed to irq_unlock()
  We use a simple nesting counter to handle multiple disable/enable pairs
*/
void hal_disable_irq(void)
{
  if (irq_nested == 0) {
    irq_lock_key = irq_lock();
  }
  irq_nested++;
}


#endif /* ifndef MRBC_NO_TIMER */


//================================================================
/*!@brief
  Abort program

  Called on fatal errors (e.g., out of memory)

  @param s	additional message.
*/
void hal_abort(const char *s)
{
  if (s) {
    printk("%s", s);
  }

  // In Zephyr, you can use k_panic() or k_oops() or just hang
  k_panic();
}
