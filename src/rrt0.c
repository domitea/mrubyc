/*! @file
  @brief
  Simplified realtime task monitor for mruby/c

  This scheduler runs entirely in a single OS thread and provides
  cooperative green threads, sleep, mutexes, and a small soft-IRQ
  mechanism. It keeps the public API and data structures defined in
  rrt0.h but uses a minimal, easy-to-follow internal layout.
*/

/***** Feature test switches ************************************************/
/***** System headers *******************************************************/
//@cond
#include "vm_config.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
//@endcond

/***** Local headers ********************************************************/
#include "mrubyc.h"

/***** Macros ***************************************************************/
#define VM2TCB(p) ((mrbc_tcb *)((uint8_t *)(p) - offsetof(mrbc_tcb, vm)))

#ifndef MRBC_MAX_IRQ_LINES
#define MRBC_MAX_IRQ_LINES 8
#endif

/***** Typedefs *************************************************************/
struct irq_slot {
  volatile uint8_t pending;
  mrbc_irq_callback_t cb;
  void *user_data;
};

// Snapshot of in-VM scheduling state. Only mrbc_tick and mrbc_irq_raise
// touch it from IRQ context; everything else runs in the VM thread.
struct scheduler_state {
  mrbc_tcb *ready;
  mrbc_tcb *waiting;
  mrbc_tcb *suspended;
  mrbc_tcb *dormant;
  mrbc_tcb *current;
  volatile uint32_t tick;
};

/***** Local variables ******************************************************/
static struct scheduler_state sched_;
static struct irq_slot irq_table_[MRBC_MAX_IRQ_LINES];

/***** Forward declarations for Ruby bindings ******************************/
static void c_sleep(mrbc_vm *vm, mrbc_value v[], int argc);
static void c_sleep_ms(mrbc_vm *vm, mrbc_value v[], int argc);
static void c_task_get(mrbc_vm *vm, mrbc_value v[], int argc);
static void c_task_list(mrbc_vm *vm, mrbc_value v[], int argc);
static void c_task_name_list(mrbc_vm *vm, mrbc_value v[], int argc);
static void c_task_set_name(mrbc_vm *vm, mrbc_value v[], int argc);
static void c_task_name(mrbc_vm *vm, mrbc_value v[], int argc);
static void c_task_set_priority(mrbc_vm *vm, mrbc_value v[], int argc);
static void c_task_priority(mrbc_vm *vm, mrbc_value v[], int argc);
static void c_task_status(mrbc_vm *vm, mrbc_value v[], int argc);
static void c_task_suspend(mrbc_vm *vm, mrbc_value v[], int argc);
static void c_task_resume(mrbc_vm *vm, mrbc_value v[], int argc);
static void c_task_terminate(mrbc_vm *vm, mrbc_value v[], int argc);
static void c_task_raise(mrbc_vm *vm, mrbc_value v[], int argc);
static void c_task_join(mrbc_vm *vm, mrbc_value v[], int argc);
static void c_task_value(mrbc_vm *vm, mrbc_value v[], int argc);
static void c_task_pass(mrbc_vm *vm, mrbc_value v[], int argc);
static void c_task_create(mrbc_vm *vm, mrbc_value v[], int argc);
static void c_task_run(mrbc_vm *vm, mrbc_value v[], int argc);
static void c_task_rewind(mrbc_vm *vm, mrbc_value v[], int argc);
static void c_mutex_new(mrbc_vm *vm, mrbc_value v[], int argc);
static void c_mutex_lock(mrbc_vm *vm, mrbc_value v[], int argc);
static void c_mutex_unlock(mrbc_vm *vm, mrbc_value v[], int argc);
static void c_mutex_trylock(mrbc_vm *vm, mrbc_value v[], int argc);
static void c_mutex_locked(mrbc_vm *vm, mrbc_value v[], int argc);
static void c_mutex_owned(mrbc_vm *vm, mrbc_value v[], int argc);
static void c_vm_tick(mrbc_vm *vm, mrbc_value v[], int argc);

/***** Static helpers *******************************************************/
// Enqueue into a priority-sorted READY list (lower value = higher priority).
// Tasks with equal priority are appended after peers to provide simple
// round-robin ordering within the same priority.
static void enqueue_ready(mrbc_tcb *tcb)
{
  mrbc_tcb **pp = &sched_.ready;
  while( *pp && (*pp)->priority_preemption <= tcb->priority_preemption ) {
    pp = &(*pp)->next;
  }
  tcb->next = *pp;
  *pp = tcb;
}


// Enqueue at the head of the specified queue.
static void enqueue_simple(mrbc_tcb **queue, mrbc_tcb *tcb)
{
  tcb->next = *queue;
  *queue = tcb;
}


// Remove a task from a singly linked list if present.
static void remove_from_queue(mrbc_tcb **queue, mrbc_tcb *tcb)
{
  while( *queue && *queue != tcb ) {
    queue = &(*queue)->next;
  }
  if( *queue == tcb ) {
    *queue = tcb->next;
    tcb->next = NULL;
  }
}


// Wake tasks whose sleep has expired.
static void wake_sleeping_tasks(void)
{
  mrbc_tcb *prev = NULL;
  mrbc_tcb *t = sched_.waiting;
  while( t ) {
    int32_t diff = (int32_t)(t->wakeup_tick - sched_.tick);
    if( t->reason == TASKREASON_SLEEP && diff <= 0 ) {
      mrbc_tcb *waking = t;
      t = t->next;
      if( prev ) {
        prev->next = t;
      } else {
        sched_.waiting = t;
      }
      waking->state = TASKSTATE_READY;
      waking->reason = 0;
      enqueue_ready(waking);
      continue;
    }
    prev = t;
    t = t->next;
  }
}


// Notify tasks waiting for a join on the finished TCB.
static void wake_joiners(const mrbc_tcb *finished)
{
  mrbc_tcb *prev = NULL;
  mrbc_tcb *t = sched_.waiting;
  while( t ) {
    if( t->reason == TASKREASON_JOIN && t->tcb_join == finished ) {
      mrbc_tcb *joining = t;
      t = t->next;
      if( prev ) prev->next = t; else sched_.waiting = t;
      joining->state = TASKSTATE_READY;
      joining->reason = 0;
      enqueue_ready(joining);
      continue;
    }
    prev = t;
    t = t->next;
  }

  for( t = sched_.suspended; t; t = t->next ) {
    if( t->reason == TASKREASON_JOIN && t->tcb_join == finished ) {
      t->reason = 0;
    }
  }
}


// Place a task in the appropriate queue based on its state.
static void enqueue_by_state(mrbc_tcb *tcb)
{
  switch( tcb->state ) {
  case TASKSTATE_READY:
    enqueue_ready(tcb);
    break;
  case TASKSTATE_WAITING:
    enqueue_simple(&sched_.waiting, tcb);
    break;
  case TASKSTATE_SUSPENDED:
    enqueue_simple(&sched_.suspended, tcb);
    break;
  case TASKSTATE_DORMANT:
  default:
    enqueue_simple(&sched_.dormant, tcb);
    break;
  }
}


// Clear scheduler state and IRQ table.
static void scheduler_reset(void)
{
  memset(&sched_, 0, sizeof(sched_));
  memset(&irq_table_, 0, sizeof(irq_table_));
}


/***** Soft IRQ implementation *********************************************/
int mrbc_irq_register(int line, mrbc_irq_callback_t cb, void *user_data)
{
  if( line < 0 || line >= MRBC_MAX_IRQ_LINES ) return -1;
  irq_table_[line].cb = cb;
  irq_table_[line].user_data = user_data;
  irq_table_[line].pending = 0;
  return 0;
}


void mrbc_irq_raise(int line)
{
  if( line < 0 || line >= MRBC_MAX_IRQ_LINES ) return;
  irq_table_[line].pending = 1;
}


void mrbc_irq_poll(void)
{
  for( int i = 0; i < MRBC_MAX_IRQ_LINES; i++ ) {
    struct irq_slot *slot = &irq_table_[i];
    if( slot->pending && slot->cb ) {
      slot->pending = 0;
      slot->cb(i, slot->user_data);
    }
  }
}


/***** Public API ***********************************************************/
// Advance VM time from a hardware timer IRQ and wake sleeping tasks.
void mrbc_tick(void)
{
  hal_disable_irq();
  sched_.tick++;
  wake_sleeping_tasks();
  hal_enable_irq();
}


mrbc_tcb * mrbc_tcb_new(int regs_size, enum MrbcTaskState task_state, int priority)
{
  unsigned int size = sizeof(mrbc_tcb) + sizeof(mrbc_value) * regs_size;
  mrbc_tcb *tcb = mrbc_raw_alloc(size);
  if( !tcb ) return NULL;

  memset(tcb, 0, size);
#if defined(MRBC_DEBUG)
  memcpy(tcb->obj_mark_, "TCB", 4);
#endif
  tcb->priority = priority;
  tcb->priority_preemption = priority;
  tcb->state = task_state;
  tcb->vm.regs_size = regs_size;
  return tcb;
}


mrbc_tcb * mrbc_create_task(const void *byte_code, mrbc_tcb *tcb)
{
  if( tcb == NULL ) {
    tcb = mrbc_tcb_new(MAX_REGS_SIZE, MRBC_TASK_DEFAULT_STATE,
                       MRBC_TASK_DEFAULT_PRIORITY);
  }
  if( tcb == NULL ) return NULL;

  if( mrbc_vm_open(&tcb->vm) == NULL ) {
    return NULL;
  }
  if( mrbc_load_mrb(&tcb->vm, byte_code) != 0 ) {
    mrbc_print_vm_exception(&tcb->vm);
    mrbc_vm_close(&tcb->vm);
    return NULL;
  }
  mrbc_vm_begin(&tcb->vm);

  hal_disable_irq();
  enqueue_by_state(tcb);
  hal_enable_irq();
  return tcb;
}


int mrbc_delete_task(mrbc_tcb *tcb)
{
  if( tcb->state != TASKSTATE_DORMANT ) return -1;

  hal_disable_irq();
  remove_from_queue(&sched_.dormant, tcb);
  hal_enable_irq();

  mrbc_vm_close(&tcb->vm);
  return 0;
}


void mrbc_set_task_name(mrbc_tcb *tcb, const char *name)
{
  for( int i = 0; i < MRBC_TASK_NAME_LEN; i++ ) {
    if( (tcb->name[i] = *name++) == 0 ) break;
  }
}


mrbc_tcb * mrbc_find_task(const char *name)
{
  mrbc_tcb *tcb;
  hal_disable_irq();

  for( tcb = sched_.ready; tcb; tcb = tcb->next ) {
    if( strcmp(tcb->name, name) == 0 ) goto FOUND;
  }
  for( tcb = sched_.waiting; tcb; tcb = tcb->next ) {
    if( strcmp(tcb->name, name) == 0 ) goto FOUND;
  }
  for( tcb = sched_.suspended; tcb; tcb = tcb->next ) {
    if( strcmp(tcb->name, name) == 0 ) goto FOUND;
  }
  for( tcb = sched_.dormant; tcb; tcb = tcb->next ) {
    if( strcmp(tcb->name, name) == 0 ) goto FOUND;
  }
  tcb = NULL;

FOUND:
  hal_enable_irq();
  return tcb;
}


int mrbc_start_task(mrbc_tcb *tcb)
{
  if( tcb->state != TASKSTATE_DORMANT ) return -1;

  hal_disable_irq();
  remove_from_queue(&sched_.dormant, tcb);
  tcb->state = TASKSTATE_READY;
  tcb->reason = 0;
  tcb->priority_preemption = tcb->priority;
  enqueue_ready(tcb);
  hal_enable_irq();
  return 0;
}


int mrbc_run(void)
{
  mrbc_irq_poll();
  hal_disable_irq();
  wake_sleeping_tasks();

  // Pick next ready task in priority order.
  mrbc_tcb *tcb = sched_.ready;
  if( tcb == NULL ) {
    hal_enable_irq();
    return 0;
  }
  sched_.ready = tcb->next;
  tcb->next = NULL;
  sched_.current = tcb;
  tcb->state = TASKSTATE_RUNNING;
  tcb->timeslice = MRBC_TIMESLICE_TICK_COUNT;
  hal_enable_irq();

  // Run one VM quantum cooperatively. VM sets flag_preemption when
  // Ruby code yields (sleep/pass) so we simply requeue afterward.
  int vm_ret = mrbc_vm_run(&tcb->vm);
  tcb->vm.flag_preemption = 0;

  hal_disable_irq();
  sched_.current = NULL;

  if( vm_ret != 0 ) {
    // Task finished or crashed; move to dormant and wake joiners.
    tcb->state = TASKSTATE_DORMANT;
    tcb->reason = 0;
    enqueue_simple(&sched_.dormant, tcb);
    wake_joiners(tcb);
    hal_enable_irq();
    if( !tcb->vm.flag_permanence ) mrbc_vm_end(&tcb->vm);
    return vm_ret;
  }

  switch( tcb->state ) {
  case TASKSTATE_RUNNING:
    // Normal cooperative yield: return to ready queue tail for its priority.
    tcb->state = TASKSTATE_READY;
    enqueue_ready(tcb);
    break;
  case TASKSTATE_WAITING:
    enqueue_simple(&sched_.waiting, tcb);
    break;
  case TASKSTATE_SUSPENDED:
    enqueue_simple(&sched_.suspended, tcb);
    break;
  case TASKSTATE_DORMANT:
    enqueue_simple(&sched_.dormant, tcb);
    break;
  default:
    break;
  }

  hal_enable_irq();
  return 0;
}


void mrbc_sleep_ms(mrbc_tcb *tcb, uint32_t ms)
{
  uint32_t ticks = ms / MRBC_TICK_UNIT + ((ms % MRBC_TICK_UNIT) ? 1 : 0);

  hal_disable_irq();
  // NEŠAHEJ na fronty, tcb je právě běžící task, ne v queue.
  tcb->state       = TASKSTATE_WAITING;
  tcb->reason      = TASKREASON_SLEEP;
  tcb->wakeup_tick = sched_.tick + ticks;
  hal_enable_irq();

  // Přinutíme VM k preempci/yieldu.
  tcb->vm.flag_preemption = 1;
}


// Wake a sleeping task immediately.
void mrbc_wakeup_task(mrbc_tcb *tcb)
{
  hal_disable_irq();

  if( tcb->state == TASKSTATE_WAITING && tcb->reason == TASKREASON_SLEEP ) {
    remove_from_queue(&sched_.waiting, tcb);
    tcb->state = TASKSTATE_READY;
    tcb->reason = 0;
    enqueue_ready(tcb);
  } else if( tcb->state == TASKSTATE_SUSPENDED && tcb->reason == TASKREASON_SLEEP ) {
    tcb->reason = 0;
  }

  hal_enable_irq();
}


void mrbc_relinquish(mrbc_tcb *tcb)
{
  tcb->timeslice = 0;
  tcb->vm.flag_preemption = 1;
}


void mrbc_change_priority(mrbc_tcb *tcb, int priority)
{
  hal_disable_irq();
  tcb->priority = priority;
  tcb->priority_preemption = priority;

  if( tcb->state == TASKSTATE_READY ) {
    remove_from_queue(&sched_.ready, tcb);
    enqueue_ready(tcb);
  }
  hal_enable_irq();
}


void mrbc_suspend_task(mrbc_tcb *tcb)
{
  if( tcb->state == TASKSTATE_SUSPENDED ) return;

  hal_disable_irq();
  remove_from_queue(&sched_.ready, tcb);
  remove_from_queue(&sched_.waiting, tcb);
  tcb->state = TASKSTATE_SUSPENDED;
  enqueue_simple(&sched_.suspended, tcb);
  hal_enable_irq();

  tcb->vm.flag_preemption = 1;
}


void mrbc_resume_task(mrbc_tcb *tcb)
{
  if( tcb->state != TASKSTATE_SUSPENDED ) return;

  hal_disable_irq();
  remove_from_queue(&sched_.suspended, tcb);
  tcb->state = (tcb->reason == 0) ? TASKSTATE_READY : TASKSTATE_WAITING;
  enqueue_by_state(tcb);
  hal_enable_irq();

  if( tcb->reason == TASKREASON_SLEEP ) {
    // keep wakeup_tick as-is
  }
}


void mrbc_terminate_task(mrbc_tcb *tcb)
{
  if( tcb->state == TASKSTATE_DORMANT ) return;

  hal_disable_irq();
  remove_from_queue(&sched_.ready, tcb);
  remove_from_queue(&sched_.waiting, tcb);
  remove_from_queue(&sched_.suspended, tcb);
  tcb->state = TASKSTATE_DORMANT;
  tcb->reason = 0;
  enqueue_simple(&sched_.dormant, tcb);
  hal_enable_irq();

  tcb->vm.flag_preemption = 1;
}


void mrbc_join_task(mrbc_tcb *tcb, const mrbc_tcb *tcb_join)
{
  if( tcb->state == TASKSTATE_DORMANT ) return;
  if( tcb_join->state == TASKSTATE_DORMANT ) return;

  hal_disable_irq();
  remove_from_queue(&sched_.ready, tcb);
  tcb->state = TASKSTATE_WAITING;
  tcb->reason = TASKREASON_JOIN;
  tcb->tcb_join = tcb_join;
  enqueue_simple(&sched_.waiting, tcb);
  hal_enable_irq();

  tcb->vm.flag_preemption = 1;
}


mrbc_mutex * mrbc_mutex_init(mrbc_mutex *mutex)
{
  if( mutex == NULL ) {
    mutex = mrbc_raw_alloc(sizeof(mrbc_mutex));
    if( mutex == NULL ) return NULL;
  }
  mutex->lock = 0;
  mutex->tcb = NULL;
  return mutex;
}


int mrbc_mutex_lock(mrbc_mutex *mutex, mrbc_tcb *tcb)
{
  int ret = 0;
  hal_disable_irq();

  if( mutex->lock == 0 ) {
    mutex->lock = 1;
    mutex->tcb = tcb;
    ret = 0;
  } else if( mutex->tcb == tcb ) {
    ret = 1;   // recursive lock attempt
  } else {
    // Park in WAITING until the owner releases the lock.
    remove_from_queue(&sched_.ready, tcb);
    tcb->state = TASKSTATE_WAITING;
    tcb->reason = TASKREASON_MUTEX;
    tcb->mutex = mutex;
    enqueue_simple(&sched_.waiting, tcb);
    tcb->vm.flag_preemption = 1;
    ret = 0;
  }

  hal_enable_irq();
  return ret;
}


int mrbc_mutex_unlock(mrbc_mutex *mutex, mrbc_tcb *tcb)
{
  if( mutex->lock == 0 ) return 1;
  if( mutex->tcb != tcb ) return 2;

  hal_disable_irq();
  mrbc_tcb *best = NULL;
  mrbc_tcb *iter = sched_.waiting;

  // Find the highest-priority waiter for this mutex, if any.
  while( iter ) {
    if( iter->reason == TASKREASON_MUTEX && iter->mutex == mutex ) {
      if( best == NULL || iter->priority_preemption < best->priority_preemption ) {
        best = iter;
      }
    }
    iter = iter->next;
  }

  // find predecessor pointer for removal if best exists
  if( best ) {
    mrbc_tcb **pp = &sched_.waiting;
    while( *pp && *pp != best ) pp = &(*pp)->next;
    if( *pp == best ) *pp = best->next;

    // Transfer ownership and wake the waiter.
    best->state = TASKSTATE_READY;
    best->reason = 0;
    mutex->tcb = best;
    enqueue_ready(best);
  } else {
    mutex->lock = 0;
    mutex->tcb = NULL;
  }

  hal_enable_irq();
  return 0;
}


int mrbc_mutex_trylock(mrbc_mutex *mutex, mrbc_tcb *tcb)
{
  int ret;
  hal_disable_irq();
  if( mutex->lock == 0 ) {
    mutex->lock = 1;
    mutex->tcb = tcb;
    ret = 0;
  } else {
    ret = 1;
  }
  hal_enable_irq();
  return ret;
}


void mrbc_cleanup(void)
{
  mrbc_cleanup_alloc();
  mrbc_cleanup_vm();
  mrbc_cleanup_symbol();
  scheduler_reset();
}


void mrbc_init(void *heap_ptr, unsigned int size)
{
  static uint8_t hal_initialized;
  if( !hal_initialized ) {
    hal_init();
    hal_initialized = 1;
  }

  scheduler_reset();
  mrbc_init_alloc(heap_ptr, size);
  mrbc_init_global();
  mrbc_init_class();

  mrbc_define_method(0, 0, "sleep", c_sleep);
  mrbc_define_method(0, 0, "sleep_ms", c_sleep_ms);
}


/***** Ruby binding functions **********************************************/
static void c_sleep(mrbc_vm *vm, mrbc_value v[], int argc)
{
  uint32_t ms = 0;

  if (argc >= 1) {
    // podle tvojí verze mruby/c – jestli nemáš mrbc_int, klidně přes int
    if (mrbc_type(v[1]) == MRBC_TT_INTEGER) {
      ms = (uint32_t)mrbc_integer(v[1]);
    }
  }

  mrbc_tcb *tcb = VM2TCB(vm);   // aktuální task

  // zavoláme tu PŮVODNÍ schedulerovou funkci
  mrbc_sleep_ms(tcb, ms);

  SET_NIL_RETURN();
}


static void c_sleep_ms(mrbc_vm *vm, mrbc_value v[], int argc)
{
  mrbc_tcb *tcb = VM2TCB(vm);
  mrbc_int_t ms = mrbc_integer(v[1]);
  SET_INT_RETURN(ms);
  mrbc_sleep_ms(tcb, ms);
}


// Task.get / Task.current: return current task or lookup by name.
static void c_task_get(mrbc_vm *vm, mrbc_value v[], int argc)
{
  mrbc_tcb *tcb = NULL;

  if( mrbc_type(v[0]) != MRBC_TT_CLASS ) goto RETURN_NIL;

  if( argc == 0 ) {
    tcb = VM2TCB(vm);
  } else if( mrbc_type(v[1]) == MRBC_TT_STRING ) {
    tcb = mrbc_find_task(mrbc_string_cstr(&v[1]));
  }

  if( tcb ) {
    mrbc_value ret = mrbc_instance_new(vm, v->cls, sizeof(mrbc_tcb *));
    *(mrbc_tcb **)ret.instance->data = tcb;
    SET_RETURN(ret);
    return;
  }

RETURN_NIL:
  SET_NIL_RETURN();
}


// Task.list: build an Array of all tasks in every queue.
static void c_task_list(mrbc_vm *vm, mrbc_value v[], int argc)
{
  mrbc_value ret = mrbc_array_new(vm, 1);
  hal_disable_irq();

  for( mrbc_tcb *t = sched_.ready; t; t = t->next ) {
    mrbc_value task = mrbc_instance_new(vm, v->cls, sizeof(mrbc_tcb *));
    *(mrbc_tcb **)task.instance->data = t;
    mrbc_array_push(&ret, &task);
  }
  for( mrbc_tcb *t = sched_.waiting; t; t = t->next ) {
    mrbc_value task = mrbc_instance_new(vm, v->cls, sizeof(mrbc_tcb *));
    *(mrbc_tcb **)task.instance->data = t;
    mrbc_array_push(&ret, &task);
  }
  for( mrbc_tcb *t = sched_.suspended; t; t = t->next ) {
    mrbc_value task = mrbc_instance_new(vm, v->cls, sizeof(mrbc_tcb *));
    *(mrbc_tcb **)task.instance->data = t;
    mrbc_array_push(&ret, &task);
  }
  for( mrbc_tcb *t = sched_.dormant; t; t = t->next ) {
    mrbc_value task = mrbc_instance_new(vm, v->cls, sizeof(mrbc_tcb *));
    *(mrbc_tcb **)task.instance->data = t;
    mrbc_array_push(&ret, &task);
  }

  hal_enable_irq();
  SET_RETURN(ret);
}


// Task.name_list: Array of task names (strings) for all queues.
static void c_task_name_list(mrbc_vm *vm, mrbc_value v[], int argc)
{
  mrbc_value ret = mrbc_array_new(vm, 1);
  hal_disable_irq();

  for( mrbc_tcb *t = sched_.ready; t; t = t->next ) {
    mrbc_value s = mrbc_string_new_cstr(vm, t->name);
    mrbc_array_push(&ret, &s);
  }
  for( mrbc_tcb *t = sched_.waiting; t; t = t->next ) {
    mrbc_value s = mrbc_string_new_cstr(vm, t->name);
    mrbc_array_push(&ret, &s);
  }
  for( mrbc_tcb *t = sched_.suspended; t; t = t->next ) {
    mrbc_value s = mrbc_string_new_cstr(vm, t->name);
    mrbc_array_push(&ret, &s);
  }
  for( mrbc_tcb *t = sched_.dormant; t; t = t->next ) {
    mrbc_value s = mrbc_string_new_cstr(vm, t->name);
    mrbc_array_push(&ret, &s);
  }

  hal_enable_irq();
  SET_RETURN(ret);
}


// Task.name=: set a task name (current or specified instance).
static void c_task_set_name(mrbc_vm *vm, mrbc_value v[], int argc)
{
  if( mrbc_type(v[1]) != MRBC_TT_STRING ) {
    mrbc_raise(vm, MRBC_CLASS(ArgumentError), 0);
    return;
  }

  mrbc_tcb *tcb = (mrbc_type(v[0]) == MRBC_TT_CLASS) ?
    VM2TCB(vm) : *(mrbc_tcb **)v[0].instance->data;

  mrbc_set_task_name(tcb, mrbc_string_cstr(&v[1]));
  mrbc_incref(&v[1]);
  SET_RETURN(v[1]);
}


// Task.name: get the task name for current or specified instance.
static void c_task_name(mrbc_vm *vm, mrbc_value v[], int argc)
{
  mrbc_value ret;
  if( mrbc_type(v[0]) == MRBC_TT_CLASS ) {
    ret = mrbc_string_new_cstr(vm, VM2TCB(vm)->name);
  } else {
    mrbc_tcb *tcb = *(mrbc_tcb **)v[0].instance->data;
    ret = mrbc_string_new_cstr(vm, tcb->name);
  }
  SET_RETURN(ret);
}


// Task.priority=: set numeric priority (0 = highest).
static void c_task_set_priority(mrbc_vm *vm, mrbc_value v[], int argc)
{
  if( mrbc_type(v[1]) != MRBC_TT_INTEGER ) {
    mrbc_raise(vm, MRBC_CLASS(ArgumentError), 0);
    return;
  }
  int n = mrbc_integer(v[1]);
  if( n < 0 || n > 255 ) {
    mrbc_raise(vm, MRBC_CLASS(ArgumentError), 0);
    return;
  }

  mrbc_tcb *tcb = (mrbc_type(v[0]) == MRBC_TT_CLASS) ?
    VM2TCB(vm) : *(mrbc_tcb **)v[0].instance->data;
  mrbc_change_priority(tcb, n);
  SET_RETURN(v[1]);
}


// Task.priority: return numeric priority for current/instance.
static void c_task_priority(mrbc_vm *vm, mrbc_value v[], int argc)
{
  mrbc_tcb *tcb = (mrbc_type(v[0]) == MRBC_TT_CLASS) ?
    VM2TCB(vm) : *(mrbc_tcb **)v[0].instance->data;
  SET_INT_RETURN(tcb->priority);
}


// Task.status: string status with optional wait reason suffix.
static void c_task_status(mrbc_vm *vm, mrbc_value v[], int argc)
{
  static const char *status_name[] = { "DORMANT", "READY", "WAITING ", "", "SUSPENDED" };
  static const char *reason_name[] = { "", "SLEEP", "MUTEX", "", "JOIN" };

  if( mrbc_type(v[0]) == MRBC_TT_CLASS ) return;

  const mrbc_tcb *tcb = *(mrbc_tcb **)v[0].instance->data;
  mrbc_value ret = mrbc_string_new_cstr(vm, status_name[tcb->state / 2]);

  if( tcb->state == TASKSTATE_WAITING ) {
    mrbc_string_append_cstr(&ret, reason_name[tcb->reason]);
  }
  SET_RETURN(ret);
}


// Task.suspend / task.suspend: move task to suspended queue.
static void c_task_suspend(mrbc_vm *vm, mrbc_value v[], int argc)
{
  mrbc_tcb *tcb = (mrbc_type(v[0]) == MRBC_TT_CLASS) ?
    VM2TCB(vm) : *(mrbc_tcb **)v[0].instance->data;
  mrbc_suspend_task(tcb);
}


// task.resume: resume a suspended task.
static void c_task_resume(mrbc_vm *vm, mrbc_value v[], int argc)
{
  if( mrbc_type(v[0]) == MRBC_TT_CLASS ) return;
  mrbc_tcb *tcb = *(mrbc_tcb **)v[0].instance->data;
  mrbc_resume_task(tcb);
}


// Task.terminate / task.terminate: end task and set DORMANT.
static void c_task_terminate(mrbc_vm *vm, mrbc_value v[], int argc)
{
  mrbc_tcb *tcb = (mrbc_type(v[0]) == MRBC_TT_CLASS) ?
    VM2TCB(vm) : *(mrbc_tcb **)v[0].instance->data;
  mrbc_terminate_task(tcb);
}


// task.raise: inject exception into another task.
static void c_task_raise(mrbc_vm *vm, mrbc_value v[], int argc)
{
  if( mrbc_type(v[0]) == MRBC_TT_CLASS ) return;

  mrbc_tcb *tcb = *(mrbc_tcb **)v[0].instance->data;
  mrbc_vm *vm1 = &tcb->vm;
  mrbc_value exc;

  if( argc == 0 ) {
    exc = mrbc_exception_new(vm1, MRBC_CLASS(RuntimeError), 0, 0);
  } else if( mrbc_type(v[1]) == MRBC_TT_EXCEPTION ) {
    exc = v[1];
    mrbc_incref(&exc);
  } else {
    mrbc_raise(vm, MRBC_CLASS(ArgumentError), 0);
    return;
  }

  mrbc_decref(&vm1->exception);
  vm1->exception = exc;
  vm1->flag_preemption = 2;

  if( tcb->state == TASKSTATE_WAITING && tcb->reason == TASKREASON_SLEEP ) {
    mrbc_wakeup_task(tcb);
  }
}


// task.join: block current task until another completes.
static void c_task_join(mrbc_vm *vm, mrbc_value v[], int argc)
{
  if( mrbc_type(v[0]) == MRBC_TT_CLASS ) return;
  mrbc_tcb *tcb_me = VM2TCB(vm);
  mrbc_tcb *tcb_join = *(mrbc_tcb **)v[0].instance->data;
  mrbc_join_task(tcb_me, tcb_join);
}


// task.value: return result of a completed (DORMANT) task.
static void c_task_value(mrbc_vm *vm, mrbc_value v[], int argc)
{
  if( mrbc_type(v[0]) == MRBC_TT_CLASS ) return;
  mrbc_tcb *tcb = *(mrbc_tcb **)v[0].instance->data;

  if( tcb->state != TASKSTATE_DORMANT ) {
    mrbc_raise(vm, 0, "task must be end");
    return;
  }

  mrbc_incref(&tcb->vm.regs[0]);
  SET_RETURN(tcb->vm.regs[0]);
}


// Task.pass: cooperative yield from current task.
static void c_task_pass(mrbc_vm *vm, mrbc_value v[], int argc)
{
  if( mrbc_type(v[0]) != MRBC_TT_CLASS ) return;
  mrbc_tcb *tcb = VM2TCB(vm);
  mrbc_relinquish(tcb);
}


// Task.create: allocate a new task (dormant) with given bytecode.
static void c_task_create(mrbc_vm *vm, mrbc_value v[], int argc)
{
  const char *byte_code;
  int regs_size = MAX_REGS_SIZE;

  if( mrbc_type(v[0]) != MRBC_TT_CLASS ) {
    mrbc_raise(vm, MRBC_CLASS(ArgumentError), 0);
    return;
  }

  if( argc >= 1 && mrbc_type(v[1]) != MRBC_TT_STRING ) goto ARG_ERROR;
  mrbc_incref(&v[1]);
  byte_code = mrbc_string_cstr(&v[1]);

  if( argc >= 2 ) {
    if( mrbc_type(v[2]) != MRBC_TT_INTEGER ) goto ARG_ERROR;
    regs_size = mrbc_integer(v[2]);
  }

  mrbc_tcb *tcb = mrbc_tcb_new(regs_size, TASKSTATE_DORMANT, MRBC_TASK_DEFAULT_PRIORITY);
  if( !tcb ) {
    mrbc_raise(vm, MRBC_CLASS(NoMemoryError), 0);
    return;
  }
  tcb->vm.flag_permanence = 1;
  if( !mrbc_create_task(byte_code, tcb) ) return;

  mrbc_value ret = mrbc_instance_new(vm, v->cls, sizeof(mrbc_tcb *));
  *(mrbc_tcb **)ret.instance->data = tcb;
  SET_RETURN(ret);
  return;

ARG_ERROR:
  mrbc_raise(vm, MRBC_CLASS(ArgumentError), 0);
}


// task.run: move a dormant task to READY state.
static void c_task_run(mrbc_vm *vm, mrbc_value v[], int argc)
{
  if( mrbc_type(v[0]) == MRBC_TT_CLASS ) return;
  mrbc_tcb *tcb = *(mrbc_tcb **)v[0].instance->data;
  if( tcb->state != TASKSTATE_DORMANT ) return;
  mrbc_start_task(tcb);
}


// task.rewind: reset VM state for a dormant task.
static void c_task_rewind(mrbc_vm *vm, mrbc_value v[], int argc)
{
  if( mrbc_type(v[0]) == MRBC_TT_CLASS ) return;
  mrbc_tcb *tcb = *(mrbc_tcb **)v[0].instance->data;
  if( tcb->state != TASKSTATE_DORMANT ) return;
  mrbc_vm_begin(&tcb->vm);
}


// Mutex.new: allocate a mutex instance backing storage.
static void c_mutex_new(mrbc_vm *vm, mrbc_value v[], int argc)
{
  *v = mrbc_instance_new(vm, v->cls, sizeof(mrbc_mutex));
  if( !v->instance ) return;
  mrbc_mutex_init((mrbc_mutex *)(v->instance->data));
}


// Mutex#lock: blockingly acquire mutex or raise on recursion.
static void c_mutex_lock(mrbc_vm *vm, mrbc_value v[], int argc)
{
  int r = mrbc_mutex_lock((mrbc_mutex *)v->instance->data, VM2TCB(vm));
  if( r == 0 ) return;
  assert(!"Mutex recursive lock.");
}


// Mutex#unlock: release mutex, handing to next waiter if any.
static void c_mutex_unlock(mrbc_vm *vm, mrbc_value v[], int argc)
{
  int r = mrbc_mutex_unlock((mrbc_mutex *)v->instance->data, VM2TCB(vm));
  if( r == 0 ) return;
  assert(!"Mutex unlock error.");
}


// Mutex#try_lock: attempt non-blocking lock, returns boolean.
static void c_mutex_trylock(mrbc_vm *vm, mrbc_value v[], int argc)
{
  int r = mrbc_mutex_trylock((mrbc_mutex *)v->instance->data, VM2TCB(vm));
  SET_BOOL_RETURN(r == 0);
}


// Mutex#locked?: true if any owner holds the lock.
static void c_mutex_locked(mrbc_vm *vm, mrbc_value v[], int argc)
{
  mrbc_mutex *mutex = (mrbc_mutex *)v->instance->data;
  SET_BOOL_RETURN(mutex->lock != 0);
}


// Mutex#owned?: true if current task owns the lock.
static void c_mutex_owned(mrbc_vm *vm, mrbc_value v[], int argc)
{
  mrbc_mutex *mutex = (mrbc_mutex *)v->instance->data;
  SET_BOOL_RETURN(mutex->lock != 0 && mutex->tcb == VM2TCB(vm));
}


// VM.tick: expose scheduler tick counter to Ruby.
static void c_vm_tick(mrbc_vm *vm, mrbc_value v[], int argc)
{
  SET_INT_RETURN(sched_.tick);
}


#include "_autogen_class_rrt0.h"


/***** Debug helpers *******************************************************/
#ifdef MRBC_DEBUG
void pq(const mrbc_tcb *p_tcb)
{
  if( p_tcb == NULL ) return;
  for( const mrbc_tcb *t = p_tcb; t; t = t->next ) {
    mrbc_printf("%d:%08x %-8.8s ", t->vm.vm_id, MRBC_PTR_TO_UINT32(t),
                t->name[0] ? t->name : "(noname)");
  }
  mrbc_printf("\n");
}


void pqall(void)
{
  hal_disable_irq();
  mrbc_printf("<< tick = %d >>\n", sched_.tick);
  mrbc_printf("<<<<< READY >>>>>\n");     pq(sched_.ready);
  mrbc_printf("<<<<< WAITING >>>>>\n");   pq(sched_.waiting);
  mrbc_printf("<<<<< SUSPENDED >>>>>\n"); pq(sched_.suspended);
  mrbc_printf("<<<<< DORMANT >>>>>\n");   pq(sched_.dormant);
  hal_enable_irq();
}
#endif
