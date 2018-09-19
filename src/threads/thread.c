#include "threads/thread.h"
#include <debug.h>
#include <stddef.h>
#include <random.h>
#include <stdio.h>
#include <string.h>
#include "threads/flags.h"
#include "threads/interrupt.h"
#include "threads/intr-stubs.h"
#include "threads/palloc.h"
#include "threads/switch.h"
#include "threads/synch.h"
#include "threads/vaddr.h"
#include "threads/malloc.h"
#include "devices/timer.h"
#include "threads/fixedpoint.h"
#ifdef USERPROG
#include "userprog/process.h"
#endif

/* Random value for struct thread's `magic' member.
   Used to detect stack overflow.  See the big comment at the top
   of thread.h for details. */
#define THREAD_MAGIC 0xcd6abf4b

/* List of all processes.  Processes are added to this list
   when they are first scheduled and removed when they exit. */
static struct list all_list;

/* Idle thread. */
static struct thread *idle_thread;

/* Initial thread, the thread running init.c:main(). */
static struct thread *initial_thread;

/* Lock used by allocate_tid(). */
static struct lock tid_lock;

/* Stack frame for kernel_thread(). */
struct kernel_thread_frame 
  {
    void *eip;                  /* Return address. */
    thread_func *function;      /* Function to call. */
    void *aux;                  /* Auxiliary data for function. */
  };

/* Function pointers for generic scheduling algorithms */
struct thread_funtion_pointers
{
    void (*thread_init)(void);
    int (*thread_get_priority)(void);
    void (*thread_set_priority)(int new_priority);
    void (*thread_ready_queue_push)(struct thread *t);
    struct thread *(*thread_next_to_run)(void);
    void (*thread_block)(void *s, struct thread *blocking_t);
    void (*thread_unblock)(struct thread *t);
    void (*thread_preempt)(void);
};

static struct thread_funtion_pointers tfp;

/* Statistics. */
static long long idle_ticks;    /* # of timer ticks spent idle. */
static long long kernel_ticks;  /* # of timer ticks in kernel threads. */
static long long user_ticks;    /* # of timer ticks in user programs. */

/* Scheduling. */
#define TIME_SLICE 4            /* # of timer ticks to give each thread. */
static unsigned thread_ticks;   /* # of timer ticks since last yield. */

/* If false (default), use round-robin scheduler.
   If true, use multi-level feedback queue scheduler.
   Controlled by kernel command-line option "-o mlfqs". */
bool thread_mlfqs;

static void kernel_thread (thread_func *, void *aux);

static void idle (void *aux UNUSED);
static struct thread *running_thread (void);
static struct thread *next_thread_to_run (void);
static void init_thread (struct thread *, const char *name, int priority);
static bool is_thread (struct thread *) UNUSED;
static void *alloc_frame (struct thread *, size_t size);
static void schedule (void);
void thread_schedule_tail (struct thread *prev);
static tid_t allocate_tid (void);

/* stub functions */
static void thread_stub_v_t(struct thread *t);
static void thread_stub_v_vt(void *s, struct thread *t);

/* priority scheduler function prototypes and data structures */
static void thread_init_prio(void);
static int thread_get_priority_prio(void);
static void thread_set_priority_prio(int priority);
static void thread_ready_queue_push_prio(struct thread *t);
static struct thread *thread_next_to_run_prio(void);
static void thread_block_prio(void *sema, struct thread *t_blocking);
static void thread_unblock_prio(struct thread *t);
static void thread_preempt_prio(void);
static void init_pq(void);
static void pq_switch_priority_slot(struct thread *t,
                                    int old_priority,
                                    int new_priority);
static void push_next_available_pq_slot(struct thread *t, int priority);
static struct thread **get_next_available_pq_slot(int priority);
static struct thread *get_pq_front(void);
static struct thread *pop_pq_front(void);
static bool pq_remove(struct thread *t, int priority);
static void priority_donation_init(void);
static struct priority_donation_elem *get_pde_fresh(void);
static void priority_donation_list_remove(struct thread *t);
static int get_priority_virtual(struct thread *t);
static struct priority_donation_elem *get_pde(struct thread *t);
static struct priority_donation_elem *get_pde_highest_priority(
                                            struct thread *t);
static bool pde_has_children(struct priority_donation_elem *pde);
static void pde_update_virtual_priority(struct priority_donation_elem *pde);
static void pde_replace_parent(struct priority_donation_elem *pde_parent, 
                               struct priority_donation_elem *pde_child);

#define PQA_SIZE    (PRI_MAX - PRI_MIN + 1)
#define SLOTS_PER_QUEUE 128
#define THREAD_INVALID  (struct thread *)0xffff
/* array of priority queue ring buffers */
static struct thread *priority_queue_array[PQA_SIZE][SLOTS_PER_QUEUE];
/* array of indecies to priority_queue_array ring buffer next thread to run */
static int pq_next_index[PQA_SIZE];

/* struct for priority donation forest */
struct priority_donation_elem
{
    struct priority_donation_elem *parent;
    struct thread *t;
    void *sema;
    int priority_virtual;
};

#define NUM_DONATION_ELEMENTS   128
/* Pool of priority_donation_elem objects to be used for the forest */
static struct priority_donation_elem pde_pool[NUM_DONATION_ELEMENTS];

/* advanced scheduler function prototypes and data structures */

static struct list mlfqs_ready_list;
static struct thread *mlfqs_next_to_run(void);
static void mlfq_thread_init(void);
static void mlfq_ready_queue_push(struct thread *t);
//static void mlfq_ready_queue_pop(struct thread *t);
static struct thread *mlfqs_get_highest_priority_thread(void);
static int mlfq_get_priority(void);
static void mlfq_set_priority(int priority UNUSED);
static void mlfq_set_priority_generic(struct thread *t);
//static void mlfq_set_priority_wrapper(int priority UNUSED, void *aux UNUSED);
static void mlfq_set_priority_all(void);
//static void mlfq_set_recent_cpu(struct thread *t);
static void mlfq_set_recent_cpu_all(void);
//static void mlfq_set_recent_cpu_wrapper(struct thread *t, void *aux UNUSED);
static void mlfq_set_load_average(void);
static int mlfq_thread_get_priority_generic(struct thread *t);
static void mlfq_thread_preempt(void);
static int mlfq_thread_count(void);

fixed_point load_average = 0;

/* Initializes the threading system by transforming the code
   that's currently running into a thread.  This can't work in
   general and it is possible in this case only because loader.S
   was careful to put the bottom of the stack at a page boundary.

   Also initializes the run queue and the tid lock.

   After calling this function, be sure to initialize the page
   allocator before trying to create any threads with
   thread_create().

   It is not safe to call thread_current() until this function
   finishes. */
void thread_init (void)
{

  ASSERT (intr_get_level () == INTR_OFF);

  lock_init (&tid_lock);
  list_init (&all_list);

  /* Set up a thread structure for the running thread. */
  initial_thread = running_thread ();
  init_thread (initial_thread, "main", PRI_DEFAULT);
  initial_thread->status = THREAD_RUNNING;
  initial_thread->tid = allocate_tid ();

  if(thread_mlfqs){
      /* BSD Advanced Scheduler */
      tfp.thread_init = mlfq_thread_init;
      tfp.thread_get_priority = mlfq_get_priority;
      tfp.thread_set_priority = mlfq_set_priority;
      tfp.thread_ready_queue_push = mlfq_ready_queue_push;
      tfp.thread_next_to_run = mlfqs_next_to_run;
      tfp.thread_block = thread_stub_v_vt;
      tfp.thread_unblock = thread_stub_v_t;
      tfp.thread_preempt = mlfq_thread_preempt;
      thread_sort_func = mlfq_thread_get_priority_generic;
  }
  else{
      /* Priority Scheduler */
      tfp.thread_init = thread_init_prio;
      tfp.thread_get_priority = thread_get_priority_prio;
      tfp.thread_set_priority = thread_set_priority_prio;
      tfp.thread_ready_queue_push = thread_ready_queue_push_prio;
      tfp.thread_next_to_run = thread_next_to_run_prio;
      tfp.thread_block = thread_block_prio;
      tfp.thread_unblock = thread_unblock_prio;
      tfp.thread_preempt = thread_preempt_prio;
      thread_sort_func = get_priority_virtual;
  }

  tfp.thread_init();
}

/* Starts preemptive thread scheduling by enabling interrupts.
   Also creates the idle thread. */
void thread_start (void){
  /* Create the idle thread. */
  struct semaphore idle_started;
  sema_init (&idle_started, 0);
  thread_create ("idle", PRI_MIN, idle, &idle_started);

  /* Start preemptive thread scheduling. */
  intr_enable ();

  /* Wait for the idle thread to initialize idle_thread. */
  sema_down (&idle_started);
}

/* Called by the timer interrupt handler at each timer tick.
   Thus, this function runs in an external interrupt context. */
void thread_tick (void) 
{
  struct thread *t = thread_current ();
  int64_t ticks_curr;

  /* Update statistics. */
  if (t == idle_thread)
    idle_ticks++;
#ifdef USERPROG
  else if (t->pagedir != NULL)
    user_ticks++;
#endif
  else
    kernel_ticks++;

  /****************************************/
  // mlfqs updates
  /****************************************/

    if (thread_mlfqs){

        if (t != idle_thread){
            static fixed_point one = 0;
            if(one == 0)
                one = fp(1);
            t->recent_cpu = fp_add(t->recent_cpu, one);
        }
        ticks_curr = timer_ticks();

        // every 4 ticks, recalculate priority
        if (ticks_curr % TIME_SLICE == 0){
            mlfq_set_priority_all();
        }
        
        // every 1 second recalculate load_average & recent_cpu
        if (ticks_curr % TIMER_FREQ == 0){
            mlfq_set_load_average();
            mlfq_set_recent_cpu_all();
        }
    }

  /* Enforce preemption. */
  if (++thread_ticks >= TIME_SLICE)
    intr_yield_on_return ();
}

/* Prints thread statistics. */
void thread_print_stats (void)  
{
  printf ("Thread: %lld idle ticks, %lld kernel ticks, %lld user ticks\n",
          idle_ticks, kernel_ticks, user_ticks);
}

/* Creates a new kernel thread named NAME with the given initial
   PRIORITY, which executes FUNCTION passing AUX as the argument,
   and adds it to the ready queue.  Returns the thread identifier
   for the new thread, or TID_ERROR if creation fails.

   If thread_start() has been called, then the new thread may be
   scheduled before thread_create() returns.  It could even exit
   before thread_create() returns.  Contrariwise, the original
   thread may run for any amount of time before the new thread is
   scheduled.  Use a semaphore or some other form of
   synchronization if you need to ensure ordering.

   The code provided sets the new thread's `priority' member to
   PRIORITY, but no actual priority scheduling is implemented.
   Priority scheduling is the goal of Problem 1-3. */
tid_t thread_create (const char *name, int priority,
               thread_func *function, void *aux) 
{
  struct thread *t;
  struct kernel_thread_frame *kf;
  struct switch_entry_frame *ef;
  struct switch_threads_frame *sf;
  tid_t tid;

  ASSERT (function != NULL);

  /* Allocate thread. */
  t = palloc_get_page (PAL_ZERO);
  if (t == NULL) {
    return TID_ERROR;
  }

  /* Initialize thread. */
  init_thread (t, name, priority);
  tid = t->tid = allocate_tid ();

  /* Stack frame for kernel_thread(). */
  kf = alloc_frame (t, sizeof *kf);
  kf->eip = NULL;
  kf->function = function;
  kf->aux = aux;

  /* Stack frame for switch_entry(). */
  ef = alloc_frame (t, sizeof *ef);
  ef->eip = (void (*) (void)) kernel_thread;

  /* Stack frame for switch_threads(). */
  sf = alloc_frame (t, sizeof *sf);
  sf->eip = switch_entry;
  sf->ebp = 0;

  /* Add to run queue. */
  thread_unblock (t);

  return tid;
}

/* Puts the current thread to sleep.  It will not be scheduled
   again until awoken by thread_unblock().

   This function must be called with interrupts turned off.  It
   is usually a better idea to use one of the synchronization
   primitives in synch.h. */
void thread_block (void *sema, struct thread *blocking_t) 
{
  ASSERT (!intr_context ());
  ASSERT (intr_get_level () == INTR_OFF);

  if(is_thread(blocking_t))
      tfp.thread_block(sema, blocking_t);
  thread_current ()->status = THREAD_BLOCKED;
  schedule ();
}

/* Transitions a blocked thread T to the ready-to-run state.
   This is an error if T is not blocked.  (Use thread_yield() to
   make the running thread ready.)

   This function does not preempt the running thread.  This can
   be important: if the caller had disabled interrupts itself,
   it may expect that it can atomically unblock a thread and
   update other data. */
void thread_unblock (struct thread *t)
{
  enum intr_level old_level;

  ASSERT (is_thread (t));

  old_level = intr_disable ();
  ASSERT (t->status == THREAD_BLOCKED);
  tfp.thread_unblock(t);
  tfp.thread_ready_queue_push(t);
  t->status = THREAD_READY;
  if(thread_current() != idle_thread)
      tfp.thread_preempt();
  intr_set_level (old_level);
}

/* Returns the name of the running thread. */
const char * thread_name (void) 
{
  return thread_current ()->name;
}

/* Returns the running thread.
   This is running_thread() plus a couple of sanity checks.
   See the big comment at the top of thread.h for details. */
struct thread * thread_current (void) 
{
  struct thread *t = running_thread ();
  
  /* Make sure T is really a thread.
     If either of these assertions fire, then your thread may
     have overflowed its stack.  Each thread has less than 4 kB
     of stack, so a few big automatic arrays or moderate
     recursion can cause stack overflow. */
  ASSERT (is_thread (t));
  ASSERT (t->status == THREAD_RUNNING);

  return t;
}

/* Returns the running thread's tid. */
tid_t thread_tid (void) 
{
  return thread_current ()->tid;
}

/* Deschedules the current thread and destroys it.  Never
   returns to the caller. */
void thread_exit (void) 
{
  ASSERT (!intr_context ());

#ifdef USERPROG
  process_exit ();
#endif

  /* Remove thread from all threads list, set our status to dying,
     and schedule another process.  That process will destroy us
     when it calls thread_schedule_tail(). */
  intr_disable ();
  list_remove (&thread_current()->allelem);
  thread_current ()->status = THREAD_DYING;
  schedule ();
  NOT_REACHED ();
}

/* Yields the CPU.  The current thread is not put to sleep and
   may be scheduled again immediately at the scheduler's whim. */
void thread_yield (void) 
{
  struct thread *cur = thread_current ();
  enum intr_level old_level;

  ASSERT (!intr_context ());

  old_level = intr_disable ();
  if (cur != idle_thread) 
      tfp.thread_ready_queue_push(cur);
  cur->status = THREAD_READY;
  schedule ();
  intr_set_level (old_level);
}

/* Invoke function 'func' on all threads, passing along 'aux'.
   This function must be called with interrupts off. */
void thread_foreach (thread_action_func *func, void *aux) 
{
  
  struct list_elem *e;
  ASSERT (intr_get_level () == INTR_OFF);

  for (e = list_begin (&all_list); e != list_end (&all_list);
       e = list_next (e)){
      struct thread *t = list_entry (e, struct thread, allelem);
      func (t, aux);
    }
}

/* Sets the current thread's priority to NEW_PRIORITY. */
void thread_set_priority (int new_priority) 
{
    tfp.thread_set_priority(new_priority);
}

/* Returns the current thread's priority. */
int thread_get_priority (void) 
{
    return tfp.thread_get_priority();
}

/* Sets the current thread's nice value to NICE. */
void thread_set_nice (int nice)
{
    enum intr_level old_level = intr_disable();
    thread_current()->nice = fp(nice);
    intr_set_level(old_level);
}

/* Returns the current thread's nice value. */
int thread_get_nice (void) 
{
    return fp_to_int(thread_current()->nice);
}

/* Returns 100 times the system load average. */
int thread_get_load_avg (void) 
{
    return fp_to_int_round(mixed_mul(load_average, 100));
}

/* Returns 100 times the current thread's recent_cpu value. */
int thread_get_recent_cpu (void) 
{
    return fp_to_int(thread_current()->recent_cpu) * 100;
}

/* Idle thread.  Executes when no other thread is ready to run.

   The idle thread is initially put on the ready list by
   thread_start().  It will be scheduled once initially, at which
   point it initializes idle_thread, "up"s the semaphore passed
   to it to enable thread_start() to continue, and immediately
   blocks.  After that, the idle thread never appears in the
   ready list.  It is returned by next_thread_to_run() as a
   special case when the ready list is empty. */

static void idle (void *idle_started_ UNUSED) 
{
  struct semaphore *idle_started = idle_started_;
  idle_thread = thread_current ();
  sema_up (idle_started);

  for (;;) 
    {
      /* Let someone else run. */
      intr_disable ();
      thread_block (NULL, NULL);

      /* Re-enable interrupts and wait for the next one.

         The `sti' instruction disables interrupts until the
         completion of the next instruction, so these two
         instructions are executed atomically.  This atomicity is
         important; otherwise, an interrupt could be handled
         between re-enabling interrupts and waiting for the next
         one to occur, wasting as much as one clock tick worth of
         time.

         See [IA32-v2a] "HLT", [IA32-v2b] "STI", and [IA32-v3a]
         7.11.1 "HLT Instruction". */
      asm volatile ("sti; hlt" : : : "memory");
    }
}

/* Function used as the basis for a kernel thread. */
static void kernel_thread (thread_func *function, void *aux) 
{
  ASSERT (function != NULL);

  intr_enable ();       /* The scheduler runs with interrupts off. */
  function (aux);       /* Execute the thread function. */
  thread_exit ();       /* If function() returns, kill the thread. */
}

/* Returns the running thread. */
struct thread * running_thread (void) 
{
  uint32_t *esp;

  /* Copy the CPU's stack pointer into `esp', and then round that
     down to the start of a page.  Because `struct thread' is
     always at the beginning of a page and the stack pointer is
     somewhere in the middle, this locates the curent thread. */
  asm ("mov %%esp, %0" : "=g" (esp));
  return pg_round_down (esp);
}

/* Returns true if T appears to point to a valid thread. */
static bool is_thread (struct thread *t)
{
  return t != NULL && t->magic == THREAD_MAGIC;
}

/* Does basic initialization of T as a blocked thread named
   NAME. */
static void init_thread (struct thread *t, const char *name, int priority)
{
  enum intr_level old_level;

  ASSERT (t != NULL);
  ASSERT (PRI_MIN <= priority && priority <= PRI_MAX);
  ASSERT (name != NULL);

  memset (t, 0, sizeof *t);
  t->status = THREAD_BLOCKED;
  strlcpy (t->name, name, sizeof t->name);
  t->stack = (uint8_t *) t + PGSIZE;
  t->priority = priority;
  t->magic = THREAD_MAGIC;

  //******************************************************************************
  // for mlfqs
  //******************************************************************************

  if (t == initial_thread){
      t->nice = fp(0);
      t->recent_cpu = fp(0);
  }
  else{
      struct thread *t_curr = thread_current();
      t->nice = t_curr->nice;
      t->recent_cpu = t_curr->recent_cpu;
  }

  //******************************************************************************

  old_level = intr_disable ();
  list_push_back (&all_list, &t->allelem);
  intr_set_level (old_level);
}

/* Allocates a SIZE-byte frame at the top of thread T's stack and
   returns a pointer to the frame's base. */
static void * alloc_frame (struct thread *t, size_t size) 
{
  /* Stack data is always allocated in word-size units. */
  ASSERT (is_thread (t));
  ASSERT (size % sizeof (uint32_t) == 0);

  t->stack -= size;
  return t->stack;
}

/* Chooses and returns the next thread to be scheduled.  Should
   return a thread from the run queue, unless the run queue is
   empty.  (If the running thread can continue running, then it
   will be in the run queue.)  If the run queue is empty, return
   idle_thread. */
static struct thread * next_thread_to_run (void) 
{
    return tfp.thread_next_to_run();
}

/* Completes a thread switch by activating the new thread's page
   tables, and, if the previous thread is dying, destroying it.

   At this function's invocation, we just switched from thread
   PREV, the new thread is already running, and interrupts are
   still disabled.  This function is normally invoked by
   thread_schedule() as its final action before returning, but
   the first time a thread is scheduled it is called by
   switch_entry() (see switch.S).

   It's not safe to call printf() until the thread switch is
   complete.  In practice that means that printf()s should be
   added at the end of the function.

   After this function and its caller returns, the thread switch
   is complete. */
void thread_schedule_tail (struct thread *prev)
{
  struct thread *cur = running_thread ();
  
  ASSERT (intr_get_level () == INTR_OFF);

  /* Mark us as running. */
  cur->status = THREAD_RUNNING;

  /* Start new time slice. */
  thread_ticks = 0;

#ifdef USERPROG
  /* Activate the new address space. */
  process_activate ();
#endif

  /* If the thread we switched from is dying, destroy its struct
     thread.  This must happen late so that thread_exit() doesn't
     pull out the rug under itself.  (We don't free
     initial_thread because its memory was not obtained via
     palloc().) */
  if (prev != NULL && prev->status == THREAD_DYING && prev != initial_thread) 
    {
      ASSERT (prev != cur);
      palloc_free_page (prev);
    }
}

/* Schedules a new process.  At entry, interrupts must be off and
   the running process's state must have been changed from
   running to some other state.  This function finds another
   thread to run and switches to it.

   It's not safe to call printf() until thread_schedule_tail()
   has completed. */
static void schedule (void) 
{
  struct thread *cur = running_thread ();
  struct thread *next = next_thread_to_run ();
  struct thread *prev = NULL;

  ASSERT (intr_get_level () == INTR_OFF);
  ASSERT (cur->status != THREAD_RUNNING);
  ASSERT (is_thread (next));

  if (cur != next)
  {
    prev = switch_threads (cur, next);
  }
  thread_schedule_tail (prev);
}

/* Returns a tid to use for a new thread. */
static tid_t
allocate_tid (void) 
{
  static tid_t next_tid = 1;
  tid_t tid;

  lock_acquire (&tid_lock);
  tid = next_tid++;
  lock_release (&tid_lock);

  return tid;
}

static void thread_stub_v_t(struct thread *t UNUSED){return;}
static void thread_stub_v_vt(void *s UNUSED, struct thread *t UNUSED){return;}


/* Offset of `stack' member within `struct thread'.
   Used by switch.S, which can't figure it out on its own. */
uint32_t thread_stack_ofs = offsetof (struct thread, stack);


/*******************************************************************************
 * PRIORITY SCHEDULER
 ******************************************************************************/
static void thread_init_prio(void)
{
    init_pq();
    priority_donation_init();
}

static int thread_get_priority_prio(void)
{
    struct thread *t_curr = thread_current();
    return get_priority_virtual(t_curr);
}

static void thread_set_priority_prio(int priority)
{
    struct thread *t_curr = thread_current();
    int priority_prev = t_curr->priority;
    
    ASSERT(priority >= PRI_MIN);
    ASSERT(priority <= PRI_MAX);
    
    t_curr->priority = priority;
    
    if(priority_prev > priority)
    {
        thread_preempt_prio();
    }
}

static void thread_ready_queue_push_prio(struct thread *t)
{
    int priority;
    enum intr_level old_level;

    if(t == idle_thread)
        return;

    ASSERT(is_thread(t));
    old_level = intr_disable();
    priority = get_priority_virtual(t);
    push_next_available_pq_slot(t, priority);
    intr_set_level(old_level);
}

int id = 0;
int ma = 0;
static struct thread *thread_next_to_run_prio(void)
{
    struct thread *t_next = pop_pq_front();

    if(!t_next)
    {
        t_next = idle_thread;
    }

    return t_next;
}

static void thread_block_prio(void *sema_blocking,
                              struct thread *t_blocking)
{
    struct priority_donation_elem *pde;
    struct priority_donation_elem *pde_child;
    enum intr_level old_level;
    int priority_old;
    
    if(!t_blocking || !sema_blocking)
    {
        return;
    }

    old_level = intr_disable();
    pde_child = get_pde_fresh();
    pde_child->t = thread_current();
    pde_child->sema = sema_blocking;
    pde_child->priority_virtual = pde_child->t->priority;
    
    pde = get_pde(t_blocking);
    if(!pde)
    {
        pde = get_pde_fresh();
        pde->parent = NULL;
        pde->t = t_blocking;
        pde->sema = NULL;
        pde->priority_virtual = t_blocking->priority;
    }
    
    pde_child->parent = pde;
    
    /* follow the tree up and update the priorities */
    while(pde)
    {
        if(pde->priority_virtual < pde_child->priority_virtual)
        {
            priority_old = get_priority_virtual(pde->t);
            pde->priority_virtual = pde_child->priority_virtual;
            pq_switch_priority_slot(pde->t,
                                    priority_old,
                                    get_priority_virtual(pde->t));
        }
        else
        {
            /* all other nodes above this will be higher priority */
            break;
        }
        pde_child = pde;
        pde = pde->parent;
    }
    intr_set_level(old_level);
}

static void thread_unblock_prio(struct thread *t)
{
    /* a thread coming unblocked means it's parent has released the lock */
    struct priority_donation_elem *pde;
    struct priority_donation_elem *pde_parent;
    enum intr_level old_level;

    old_level = intr_disable();
    
    pde = get_pde(t);
    /* having the sema field set denotes the thread is blocked 
     * by a lock which has donated priority. */
    if(!pde || !pde->sema)
    {
        intr_set_level(old_level);
        return;
    }
    
    pde_parent = pde->parent;
    pde->parent = NULL;

    if(pde_parent)
    {
        pde_replace_parent(pde_parent, pde);

        if(!pde_has_children(pde_parent) && !pde->sema)
        {
            priority_donation_list_remove(pde_parent->t);
        }
        else
        {
            pde_update_virtual_priority(pde_parent);
        }
    }
    
    pde->sema = NULL;
    if(pde_has_children(pde))
    {
        pde_update_virtual_priority(pde);
    }
    else
    {
        /* check the priority donation list and remove if this thread is present */
        priority_donation_list_remove(pde->t);
    }
    intr_set_level(old_level);
}

static void thread_preempt_prio(void)
{
    if(intr_context())
    {
        intr_yield_on_return();
    }
    else if(get_priority_virtual(thread_current()) 
             < get_priority_virtual(get_pq_front()))
    {
        thread_yield();
    }
}

/* PRIORITY QUEUE ACCESSORS */
static void init_pq(void)
{
    int i0, i1;
    
    for(i0 = 0; i0 < PQA_SIZE; ++i0)
    {
        for(i1 = 0; i1 < SLOTS_PER_QUEUE; ++i1)
        {
            priority_queue_array[i0][i1] = NULL;
        }
    }
    
    for(i0 = 0; i0 < PQA_SIZE; ++i0)
    {
        pq_next_index[i0] = 0;
    }
}

static void pq_switch_priority_slot(struct thread *t,
                                    int old_priority,
                                    int new_priority)
{
    if(old_priority != new_priority)
    {
        /* remove the thread from the old priority's ready queue */
        if(pq_remove(t, old_priority))
            /* and add it to the new priority's ready queue */
            push_next_available_pq_slot(t, new_priority);
    }
}

static void push_next_available_pq_slot(struct thread *t, int priority)
{
    enum intr_level old_level;
    struct thread **t_slot;
    
    old_level = intr_disable();
    t_slot = get_next_available_pq_slot(priority);
    ASSERT(t_slot); // this will only happen if we've run out of slots
    ASSERT(*t_slot == NULL || *t_slot == THREAD_INVALID);
    *t_slot = t;
    intr_set_level(old_level);
}

static struct thread **get_next_available_pq_slot(int priority)
{
    int i;
    struct thread *t;
    enum intr_level old_level;
    int next_index = pq_next_index[priority];
    
    /* start at next + 1 because the next is next to run */
    old_level = intr_disable();
    for(i = 0; 
        i < SLOTS_PER_QUEUE; 
        ++i)
    {
        t = priority_queue_array[priority][(i + next_index) % SLOTS_PER_QUEUE];
        if(t == NULL || t == THREAD_INVALID)
        {
            break;
        }
    }
    intr_set_level(old_level);
    
    return &priority_queue_array[priority][(i + next_index) % SLOTS_PER_QUEUE];
}

static struct thread *get_pq_front(void)
{
    struct thread *t_next = NULL;
    enum intr_level old_level;
    int priority;

    old_level = intr_disable();
    for(priority = PRI_MAX; priority >= PRI_MIN; --priority)
    {
        t_next = priority_queue_array[priority][pq_next_index[priority]];
        if(t_next && t_next != THREAD_INVALID)
        {
            break;
        }
        else
        {
            t_next = NULL;
        }
    }
    intr_set_level(old_level);

    return t_next;
}

static struct thread *pop_pq_front(void)
{
    enum intr_level old_level;
    int priority;
    struct thread *t_next = NULL;

    old_level = intr_disable();
    for(priority = PRI_MAX; priority >= PRI_MIN; --priority)
    {
        if (priority_queue_array[priority][pq_next_index[priority]]
         && priority_queue_array[priority][pq_next_index[priority]]
                                           != THREAD_INVALID)
        {
            t_next = priority_queue_array[priority][pq_next_index[priority]];
            ASSERT(is_thread(t_next));
            do
            {
                priority_queue_array[priority][pq_next_index[priority]] = NULL;
                pq_next_index[priority] = (pq_next_index[priority] + 1) 
                                                 % SLOTS_PER_QUEUE;
            } while(priority_queue_array[priority][pq_next_index[priority]] == 
                    THREAD_INVALID);
            break;
        }
    }
    intr_set_level(old_level);
    
    return t_next;
}

static bool pq_remove(struct thread *t, int priority)
{
    enum intr_level old_level;
    int i;
    bool removed = false;

    old_level = intr_disable();
    /* just do a dumb search */
    for(i = 0; i < SLOTS_PER_QUEUE; ++i)
    {
        if(priority_queue_array[priority][i] == t)
        {
            /* use thread invalid instead of NULL because 
             * this could be in the middle of the list */
            priority_queue_array[priority][i] = THREAD_INVALID;
            removed = true;
            break;
        }
    }
    intr_set_level(old_level);

    return removed;
}

/* PRIORITY DONATION ACCESSORS */
static void priority_donation_init(void)
{
    int i;
    
    for(i = 0; i < NUM_DONATION_ELEMENTS; ++i)
    {
        pde_pool[i].parent = NULL;
        pde_pool[i].t = NULL;
        pde_pool[i].sema = NULL;
        pde_pool[i].priority_virtual = 0;
    }
}

static struct priority_donation_elem *get_pde_fresh(void)
{
    return get_pde(NULL);
}

static void priority_donation_list_remove(struct thread *t)
{
    int i;
    struct priority_donation_elem *this = NULL;
    enum intr_level old_level;
    struct thread *t_removed;
    int priority_removed;
    
    old_level = intr_disable();
    /* find the pde containing t and NULL it out */
    for(i = 0; i < NUM_DONATION_ELEMENTS; ++i)
    {
        if(pde_pool[i].t == t)
        {
            t_removed = pde_pool[i].t;
            priority_removed = get_priority_virtual(pde_pool[i].t);
            pde_pool[i].parent = NULL;
            pde_pool[i].t = NULL;
            pde_pool[i].sema = NULL;
            pde_pool[i].priority_virtual = 0;
            this = &pde_pool[i];
            pq_switch_priority_slot(t_removed,
                                    priority_removed,
                                    get_priority_virtual(pde_pool[i].t));
            break;
        }
    }
    
    /* find all the entries that had this as a parent 
     * and set their parents to NULL */
    for(i = 0; i < NUM_DONATION_ELEMENTS; ++i)
    {
        if(pde_pool[i].parent == this)
        {
            pde_pool[i].parent = NULL;
        }
    }
    intr_set_level(old_level);
}

static int get_priority_virtual(struct thread *t)
{
    struct priority_donation_elem *pde;
    int priority;
    
    if(!t || !is_thread(t))
    {
        return -1;
    }
    
    pde = get_pde_highest_priority(t);
    
    if(pde)
    {
        priority = pde->priority_virtual;
    }
    else
    {
        priority = t->priority;
    }
    
    return priority;
}

static struct priority_donation_elem *get_pde(struct thread *t)
{
    struct priority_donation_elem *pde = NULL;
    enum intr_level old_level;
    int i;
    
    old_level = intr_disable();
    for(i = 0; i < NUM_DONATION_ELEMENTS; ++i)
    {
        if(pde_pool[i].t == t)
        {
            pde = &pde_pool[i];
            break;
        }
    }
    intr_set_level(old_level);
    
    return pde;
}

static struct priority_donation_elem *get_pde_highest_priority(struct thread *t)
{
    struct priority_donation_elem *pde = NULL;
    enum intr_level old_level;
    int i;
    int priority = 0;

    old_level = intr_disable();
    for(i = 0; i < NUM_DONATION_ELEMENTS; ++i)
    {
        if(pde_pool[i].t == t && pde_pool[i].priority_virtual > priority)
        {
            pde = &pde_pool[i];
            priority = pde->priority_virtual;
        }
    }
    intr_set_level(old_level);

    return pde;
}

static bool pde_has_children(struct priority_donation_elem *pde)
{
    enum intr_level old_level;
    int i;
    bool ret = false;

    old_level = intr_disable();
    for(i = 0; i < NUM_DONATION_ELEMENTS; ++i)
    {
        if(pde_pool[i].parent == pde)
        {
            ret = true;
            break;
        }
    }
    intr_set_level(old_level);
    
    return ret;
}

static void pde_update_virtual_priority(struct priority_donation_elem *pde)
{
    enum intr_level old_level;
    int i;
    int priority_virtual_new = pde->t->priority;
    int priority_virtual_old;

    old_level = intr_disable();
    for(i = 0; i < NUM_DONATION_ELEMENTS; ++i)
    {
        if( pde_pool[i].parent == pde 
         && priority_virtual_new < pde_pool[i].priority_virtual)
        {
            priority_virtual_new = pde_pool[i].priority_virtual;
        }
    }

    priority_virtual_old = get_priority_virtual(pde->t);
    pde->priority_virtual = priority_virtual_new;
    pq_switch_priority_slot(pde->t,
                            priority_virtual_old,
                            get_priority_virtual(pde->t));

    intr_set_level(old_level);
}

static void pde_replace_parent(struct priority_donation_elem *pde_parent, 
                               struct priority_donation_elem *pde_child)
{
    enum intr_level old_level;
    int i;

    old_level = intr_disable();
    for(i = 0; i < NUM_DONATION_ELEMENTS; ++i)
    {
        if( pde_pool[i].parent == pde_parent
         && pde_pool[i].sema == pde_child->sema)
        {
            pde_pool[i].parent = pde_child;
        }
    }
    intr_set_level(old_level);
}

/*******************************************************************************
 * ADVANCED SCHEDULER
 ******************************************************************************/

static void mlfq_thread_init(void){

    ASSERT(thread_mlfqs);
    ASSERT(intr_get_level () == INTR_OFF);

    list_init(&mlfqs_ready_list);
}

static void mlfq_ready_queue_push(struct thread *t)
{
    ASSERT(thread_mlfqs);
    enum intr_level old_level;

    if(t == idle_thread)
        return;

    old_level = intr_disable();
    list_push_back(&mlfqs_ready_list, &t->elem);
    intr_set_level(old_level);
}

static struct thread *mlfqs_next_to_run(void)
{
    struct thread *ret_thread = idle_thread;
    enum intr_level old_level;
    struct list_elem *e;
    struct list_elem *e_highest = NULL;

    old_level = intr_disable();
    if(!list_empty(&mlfqs_ready_list)){
        for (e = list_begin(&mlfqs_ready_list);
             e != list_end(&mlfqs_ready_list);
             e = list_next (e))
        {
            if( e_highest == NULL
             || list_entry(e, struct thread, elem)->priority
              > list_entry(e_highest, struct thread, elem)->priority)
            {
                e_highest = e;
            }
        }

        list_remove(e_highest);
        ret_thread = list_entry(e_highest, struct thread, elem);
    }
    intr_set_level(old_level);
    return ret_thread;
}

static struct thread *mlfqs_get_highest_priority_thread(void)
{
    struct  thread *ret_thread = idle_thread;
    enum    intr_level old_level;
    struct  list_elem *e;
    struct  list_elem *e_highest = NULL;

    old_level = intr_disable();
    if(!list_empty(&mlfqs_ready_list)){
        for (e  = list_begin(&mlfqs_ready_list);
             e != list_end(&mlfqs_ready_list);
             e  = list_next (e))
        {
            if( e_highest == NULL
             || list_entry(e, struct thread, elem)->priority
              > list_entry(e_highest, struct thread, elem)->priority)
            {
                e_highest = e;
            }
        }

        ret_thread = list_entry(e_highest, struct thread, elem);
    }
    intr_set_level(old_level);
    return ret_thread;
}

static int mlfq_get_priority(void)s
{
    ASSERT(thread_mlfqs);
    struct thread *t = thread_current();
    return mlfq_thread_get_priority_generic(t);
}

/* priority is recalculated every 4 ticks */
static void mlfq_set_priority(int priority UNUSED)
{
    ASSERT(thread_mlfqs);
    struct thread *t = thread_current();
    mlfq_set_priority_generic(t);
}

static void mlfq_set_priority_generic(struct thread *t)
{
    // priority = PRI_MAX - (recent_cpu / 4) - (nice * 2)
    fixed_point recent_cpu_fp = t->recent_cpu;
    int new_priority_int;
    fixed_point a1 = mixed_div(recent_cpu_fp, 4);
    fixed_point a2 = mixed_mul(t->nice, 2);
    fixed_point new_priority;
    fixed_point pri_max_fp = 0;

    if(pri_max_fp == 0)
        pri_max_fp = fp(PRI_MAX);

    new_priority = fp_sub(pri_max_fp, a1);
    new_priority = fp_sub(new_priority, a2);
    new_priority_int = fp_to_int(new_priority);

    if(new_priority_int > PRI_MAX){ t->priority = PRI_MAX; }
    else if(new_priority_int < PRI_MIN){ t->priority = PRI_MIN; }
    else{ t->priority = new_priority_int; }
}

static void mlfq_set_priority_all(void)
{
    struct thread *t;
    enum intr_level old_level;
    struct list_elem *e;

    old_level = intr_disable();
    if(!list_empty(&all_list)){
        for (e = list_begin(&all_list);
             e != list_end(&all_list);
             e = list_next (e))
        {
            t = list_entry(e, struct thread, allelem);
            if(t != idle_thread)
                mlfq_set_priority_generic(t);
        }
    }
    intr_set_level(old_level);
}

static void mlfq_set_recent_cpu_all(void)
{
    /* recent_cpu = (2*load_avg)/(2*load_avg + 1) * recent_cpu + nice */
    ASSERT(thread_mlfqs);
    struct list_elem *e;
    struct thread *t = thread_current();
    fixed_point recent_cpu_fp;
    fixed_point a1 = mixed_mul(load_average, 2);
    fixed_point a2;
    fixed_point a3;
    static fixed_point one = 0;

    if(one == 0)
        one = fp(1);
    a2 = fp_add(a1, one);
    a3 = fp_div(a1, a2);

    if(!list_empty(&all_list)){
        for (e = list_begin(&all_list);
             e != list_end(&all_list);
             e = list_next (e))
        {
            t = list_entry(e, struct thread, allelem);
            if(t != idle_thread)
            {
                recent_cpu_fp = t->recent_cpu;
                recent_cpu_fp = fp_mul(recent_cpu_fp, a3);
                recent_cpu_fp = fp_add(recent_cpu_fp, t->nice);
                t->recent_cpu = recent_cpu_fp;
            }
        }
    }
}

static void mlfq_set_load_average(void)
{   // load_avg = (59/60)*load_avg + (1/60)*ready_threads

    ASSERT(thread_mlfqs);
    static fixed_point fiftynine_div_sixty = 0;
    static fixed_point one_div_sixty = 0;

    int ready_threads = mlfq_thread_count();

    if(fiftynine_div_sixty == 0)
        fiftynine_div_sixty = mixed_div(fp(59), 60);
    if(one_div_sixty == 0)
        one_div_sixty = mixed_div(fp(1), 60);

    if(thread_current() != idle_thread)
        ready_threads += 1;

    load_average = fp_add(fp_mul(fiftynine_div_sixty, load_average),
                          mixed_mul(one_div_sixty, ready_threads));
}

static int mlfq_thread_get_priority_generic(struct thread *t)
{
    fixed_point recent_cpu_fp = t->recent_cpu;
    fixed_point a1 = mixed_div(recent_cpu_fp, 4);
    fixed_point a2 = mixed_mul(t->nice, 2);
    fixed_point new_priority = fp(PRI_MAX);
    new_priority = fp_sub(new_priority, a1);
    new_priority = mixed_sub(new_priority, a2);

    if(fp_to_int(new_priority) > PRI_MAX){ return PRI_MAX; }
    else if(fp_to_int(new_priority) < PRI_MIN){ return PRI_MIN; }
    else { return fp_to_int(new_priority); }
}

static void mlfq_thread_preempt(void)
{
    if(intr_context())
    {
        intr_yield_on_return();
    }
    else if(mlfq_get_priority()
        < mlfq_thread_get_priority_generic(mlfqs_get_highest_priority_thread()))
    {
        thread_yield();
    }
}

static int mlfq_thread_count(void)
{
	  int ct=0;
    if (!list_empty(&mlfqs_ready_list)){
        ct = list_size(&mlfqs_ready_list);
    }
	  return ct;
}