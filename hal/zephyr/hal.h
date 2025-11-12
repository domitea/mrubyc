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

#ifndef MRBC_SRC_HAL_H_
#define MRBC_SRC_HAL_H_

/***** Feature test switches ************************************************/
/***** System headers *******************************************************/
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>


/***** Local headers ********************************************************/
/***** Constant values ******************************************************/
/***** Macros ***************************************************************/
#ifndef MRBC_SCHEDULER_EXIT
#define MRBC_SCHEDULER_EXIT 1
#endif

#if !defined(MRBC_TICK_UNIT)
#define MRBC_TICK_UNIT_1_MS   1
#define MRBC_TICK_UNIT_2_MS   2
#define MRBC_TICK_UNIT_4_MS   4
#define MRBC_TICK_UNIT_10_MS 10
// Default tick unit for Zephyr (can be adjusted based on CONFIG_SYS_CLOCK_TICKS_PER_SEC)
#define MRBC_TICK_UNIT MRBC_TICK_UNIT_10_MS
// Substantial timeslice value (millisecond) will be
// MRBC_TICK_UNIT * MRBC_TIMESLICE_TICK_COUNT (+ Jitter).
// MRBC_TIMESLICE_TICK_COUNT must be natural number
// (recommended value is from 1 to 10).
#define MRBC_TIMESLICE_TICK_COUNT 1
#endif


/***** Typedefs *************************************************************/
/***** Global variables *****************************************************/
/***** Function prototypes **************************************************/
#ifdef __cplusplus
extern "C" {
#endif

void mrbc_tick(void);

#if !defined(MRBC_NO_TIMER)
void hal_init(void);
void hal_enable_irq(void);
void hal_disable_irq(void);
#define hal_idle_cpu()    k_sleep(K_MSEC(MRBC_TICK_UNIT))

#else // MRBC_NO_TIMER
#define hal_init()        ((void)0)
#define hal_enable_irq()  ((void)0)
#define hal_disable_irq() ((void)0)
#define hal_idle_cpu()    (k_sleep(K_MSEC(MRBC_TICK_UNIT)), mrbc_tick())

#endif

void hal_abort(const char *s);


/***** Inline functions *****************************************************/

//================================================================
/*!@brief
  Write

  @param  fd    file descriptor (1=stdout, 2=stderr)
  @param  buf   pointer of buffer.
  @param  nbytes        output byte length.
*/
inline static int hal_write(int fd, const void *buf, int nbytes)
{
  // Use Zephyr's printk for output
  // Note: printk doesn't return number of bytes written, so we return nbytes
  const char *cbuf = (const char *)buf;
  for (int i = 0; i < nbytes; i++) {
    printk("%c", cbuf[i]);
  }
  return nbytes;
}

//================================================================
/*!@brief
  Flush write buffer

  @param  fd    file descriptor
*/
inline static int hal_flush(int fd)
{
  // printk is typically unbuffered in Zephyr, so no flush needed
  return 0;
}


#ifdef __cplusplus
}
#endif
#endif // ifndef MRBC_HAL_H_
