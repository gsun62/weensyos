#include "kernel.hh"
#include "k-apic.hh"
#include "k-vmiter.hh"
#include <atomic>

// kernel.cc
//
//    This is the kernel.


// INITIAL PHYSICAL MEMORY LAYOUT
//
//  +-------------- Base Memory --------------+
//  v                                         v
// +-----+--------------------+----------------+--------------------+---------/
// |     | Kernel      Kernel |       :    I/O | App 1        App 1 | App 2
// |     | Code + Data  Stack |  ...  : Memory | Code + Data  Stack | Code ...
// +-----+--------------------+----------------+--------------------+---------/
// 0  0x40000              0x80000 0xA0000 0x100000             0x140000
//                                             ^
//                                             | \___ PROC_SIZE ___/
//                                      PROC_START_ADDR

#define PROC_SIZE 0x40000       // initial state only

proc ptable[NPROC];             // array of process descriptors
                                // Note that `ptable[0]` is never used.
proc* current;                  // pointer to currently executing proc

#define HZ 100                  // timer interrupt frequency (interrupts/sec)
static std::atomic<unsigned long> ticks; // # timer interrupts so far


// Memory state - see `kernel.hh`
physpageinfo physpages[NPAGES];


[[noreturn]] void schedule();
[[noreturn]] void run(proc* p);
void exception(regstate* regs);
uintptr_t syscall(regstate* regs);
void memshow();


// kernel_start(command)
//    Initialize the hardware and processes and start running. The `command`
//    string is an optional string passed from the boot loader.

static void process_setup(pid_t pid, const char* program_name);

void kernel_start(const char* command) {
    // initialize hardware
    init_hardware();
    log_printf("Starting WeensyOS\n");

    ticks = 1;
    init_timer(HZ);

    // clear screen
    console_clear();

    // (re-)initialize kernel page table
    for (vmiter it(kernel_pagetable);
         it.va() < MEMSIZE_PHYSICAL;
         it += PAGESIZE) {
        if (it.va() == CONSOLE_ADDR || it.va() >= PROC_START_ADDR) {
            it.map(it.va(), PTE_P | PTE_W | PTE_U); // user access to console and processes
        }
        else if (it.va() != 0) { 
            it.map(it.va(), PTE_P | PTE_W); // no user access to kernel mappings
        } else {
            // nullptr is inaccessible even to the kernel
            it.map(it.va(), 0);
        }
    }

    // set up process descriptors
    for (pid_t i = 0; i < NPROC; i++) {
        ptable[i].pid = i;
        ptable[i].state = P_FREE;
    }
    if (command && !program_image(command).empty()) {
        process_setup(1, command);
    } else {
        process_setup(1, "allocator");
        process_setup(2, "allocator2");
        process_setup(3, "allocator3");
        process_setup(4, "allocator4");
    }

    // Switch to the first process using run()
    run(&ptable[1]);
}


// kalloc(sz)
//    Kernel physical memory allocator. Allocates at least `sz` contiguous bytes
//    and returns a pointer to the allocated memory, or `nullptr` on failure.
//    The returned pointer’s address is a valid physical address, but since the
//    WeensyOS kernel uses an identity mapping for virtual memory, it is also
//    a valid virtual address that the kernel can access or modify.
//
//    The allocator selects from physical pages that can be allocated for
//    process use (so not reserved pages or kernel data), and from physical
//    pages that are currently unused (so `physpages[I].refcount == 0`).
//
//    On WeensyOS, `kalloc` is a page-based allocator: if `sz > PAGESIZE`
//    the allocation fails; if `sz < PAGESIZE` it allocates a whole page
//    anyway.
//
//    The handout code returns the next allocatable free page it can find.
//    It checks all pages. (You could maybe make this faster!)
//
//    The returned memory is initially filled with 0xCC, which corresponds to
//    the x86 instruction `int3`. This may help you debug.

void* kalloc(size_t sz) {
    if (sz > PAGESIZE) {
        return nullptr;
    }

    for (uintptr_t pa = 0; pa != MEMSIZE_PHYSICAL; pa += PAGESIZE) {
        if (allocatable_physical_address(pa)
            && physpages[pa / PAGESIZE].refcount == 0) {
            ++physpages[pa / PAGESIZE].refcount;
            memset((void*) pa, 0xCC, PAGESIZE);
            return (void*) pa;
        }
    }
    return nullptr;
}


// kfree(kptr)
//    Free `kptr`, which must have been previously returned by `kalloc`.
//    If `kptr == nullptr` does nothing.

void kfree(void* kptr) {
    (void) kptr;

    // we check refcount since we can only free pages that have been allocated
    if (kptr != nullptr && physpages[(uintptr_t) kptr / PAGESIZE].refcount != 0) {
        physpages[(uintptr_t) kptr / PAGESIZE].refcount--;
    }
}


// process_setup(pid, program_name)
//    Load application program `program_name` as process number `pid`.
//    This loads the application's code and data into memory, sets its
//    %rip and %rsp, gives it a stack page, and marks it as runnable.

void process_setup(pid_t pid, const char* program_name) {
    init_process(&ptable[pid], 0);

    ptable[pid].pagetable = kalloc_pagetable();

    // allows us to iterate through kernel and process pagetables
    vmiter kit(kernel_pagetable, 0);
    vmiter pit(ptable[pid].pagetable, 0);
    for (; kit.va() < PROC_START_ADDR; kit += PAGESIZE, pit += PAGESIZE) {
        pit.map(kit.pa(), kit.perm()); // maps the kernel pages & perms to process pagetable
    }

    // obtain reference to the program image
    program_image pgm(program_name);

    // allocate and map global memory required by loadable segments
    for (auto seg = pgm.begin(); seg != pgm.end(); ++seg) {
        for (uintptr_t a = round_down(seg.va(), PAGESIZE);  
             a < seg.va() + seg.size();
             a += PAGESIZE) {
            void* p = kalloc(PAGESIZE);
            // check if a segment can use read-only memory and map perms accordingly
            if (seg.writable()) {
                 vmiter(ptable[pid].pagetable, a).map(p, PTE_PWU);
            } else {
                 vmiter(ptable[pid].pagetable, a).map(p, PTE_P | PTE_U);
            }
        }
    }

    // initialize data in loadable segments
    for (auto seg = pgm.begin(); seg != pgm.end(); ++seg) {

        memset((void*) vmiter(ptable[pid].pagetable, seg.va()).pa(), 0, seg.size());
        memcpy((void*) vmiter(ptable[pid].pagetable, seg.va()).pa(), seg.data(), seg.data_size());

    }

    // mark entry point
    ptable[pid].regs.reg_rip = pgm.entry();

    // allocate and map stack segment
    // Compute process virtual address for stack page
    uintptr_t stack_addr = MEMSIZE_VIRTUAL - PAGESIZE;
    void* stack_page = kalloc(PAGESIZE);
    // map the address for the stack page of the current process's pagetable
    vmiter(ptable[pid].pagetable, stack_addr).map((uintptr_t) stack_page, PTE_PWU);
    ptable[pid].regs.reg_rsp = stack_addr + PAGESIZE;

    // mark process as runnable
    ptable[pid].state = P_RUNNABLE;
}


// exception(regs)
//    Exception handler (for interrupts, traps, and faults).
//
//    The register values from exception time are stored in `regs`.
//    The processor responds to an exception by saving application state on
//    the kernel's stack, then jumping to kernel assembly code (in
//    k-exception.S). That code saves more registers on the kernel's stack,
//    then calls exception().
//
//    Note that hardware interrupts are disabled when the kernel is running.

void exception(regstate* regs) {
    // Copy the saved registers into the `current` process descriptor.
    current->regs = *regs;
    regs = &current->regs;

    // It can be useful to log events using `log_printf`.
    // Events logged this way are stored in the host's `log.txt` file.
    /* log_printf("proc %d: exception %d at rip %p\n",
                current->pid, regs->reg_intno, regs->reg_rip); */

    // Show the current cursor location and memory state
    // (unless this is a kernel fault).
    console_show_cursor(cursorpos);
    if (regs->reg_intno != INT_PF || (regs->reg_errcode & PTE_U)) {
        memshow();
    }

    // If Control-C was typed, exit the virtual machine.
    check_keyboard();


    // Actually handle the exception.
    switch (regs->reg_intno) {

    case INT_IRQ + IRQ_TIMER:
        ++ticks;
        lapicstate::get().ack();
        schedule();
        break;                  /* will not be reached */

    case INT_PF: {
        // Analyze faulting address and access type.
        uintptr_t addr = rdcr2();
        const char* operation = regs->reg_errcode & PTE_W
                ? "write" : "read";
        const char* problem = regs->reg_errcode & PTE_P
                ? "protection problem" : "missing page";

        if (!(regs->reg_errcode & PTE_U)) {
            panic("Kernel page fault on %p (%s %s)!\n",
                  addr, operation, problem);
        }
        console_printf(CPOS(24, 0), 0x0C00,
                       "Process %d page fault on %p (%s %s, rip=%p)!\n",
                       current->pid, addr, operation, problem, regs->reg_rip);
        current->state = P_FAULTED;
        break;
    }

    default:
        panic("Unexpected exception %d!\n", regs->reg_intno);

    }

    // Return to the current process (or run something else).
    if (current->state == P_RUNNABLE) {
        run(current);
    } else {
        schedule();
    }
}


// syscall(regs)
//    System call handler.
//
//    The register values from system call time are stored in `regs`.
//    The return value, if any, is returned to the user process in `%rax`.
//
//    Note that hardware interrupts are disabled when the kernel is running.

int syscall_page_alloc(uintptr_t addr);
pid_t syscall_fork();
void syscall_exit();
void free_everything(x86_64_pagetable* pt);

uintptr_t syscall(regstate* regs) {
    // Copy the saved registers into the `current` process descriptor.
    current->regs = *regs;
    regs = &current->regs;

    // Show the current cursor location and memory state.
    console_show_cursor(cursorpos);
    memshow();

    // If Control-C was typed, exit the virtual machine.
    check_keyboard();


    // Actually handle the exception.
    switch (regs->reg_rax) {

    case SYSCALL_PANIC:
        user_panic(current);    // does not return

    case SYSCALL_GETPID:
        return current->pid;

    case SYSCALL_YIELD:
        current->regs.reg_rax = 0;
        schedule();             // does not return

    case SYSCALL_PAGE_ALLOC:
        return syscall_page_alloc(current->regs.reg_rdi);

    case SYSCALL_FORK:
        return syscall_fork();

    case SYSCALL_EXIT:
        syscall_exit();
        break; // accounts for fall through

    default:
        panic("Unexpected system call %ld!\n", regs->reg_rax);

    }

    panic("Should not get here!\n");
}


// syscall_page_alloc(addr)
//    Handles the SYSCALL_PAGE_ALLOC system call. This function
//    should implement the specification for `sys_page_alloc`
//    in `u-lib.hh` (but in the handout code, it does not).


int syscall_page_alloc(uintptr_t addr) {
    // ensure that kernel isolation is preserved
    if (addr % PAGESIZE != 0 || addr < PROC_START_ADDR || addr >= MEMSIZE_VIRTUAL) {
        return -1;
    }
    void* p = kalloc(PAGESIZE);

    // case for if there is no more physical memory available
    if (p == nullptr) {
        return -1;
    }
    if (vmiter(current, addr).try_map((uintptr_t) p, PTE_PWU) < 0) {
        kfree(p);
        return -1;
    }
    memset((void*) vmiter(current, addr).pa(), 0, PAGESIZE);

    return 0;
}


// schedule
//    Pick the next process to run and then run it.
//    If there are no runnable processes, spins forever.

void schedule() {
    pid_t pid = current->pid;
    for (unsigned spins = 1; true; ++spins) {
        pid = (pid + 1) % NPROC;
        if (ptable[pid].state == P_RUNNABLE) {
            run(&ptable[pid]);
        }

        // If Control-C was typed, exit the virtual machine.
        check_keyboard();

        // If spinning forever, show the memviewer.
        if (spins % (1 << 12) == 0) {
            memshow();
            log_printf("%u\n", spins);
        }
    }
}


// run(p)
//    Run process `p`. This involves setting `current = p` and calling
//    `exception_return` to restore its page table and registers.

void run(proc* p) {
    assert(p->state == P_RUNNABLE);
    current = p;

    // Check the process's current pagetable.
    check_pagetable(p->pagetable);

    // This function is defined in k-exception.S. It restores the process's
    // registers then jumps back to user mode.
    exception_return(p);

    // should never get here
    while (true) {
    }
}


// syscall_fork()
//    Starts a new process as a copy of an existing process.

pid_t syscall_fork() {

    // iterate through processes, starting at 1
    for (int pid = 1; pid < NPROC; pid++) {
        if (ptable[pid].state == P_FREE) {
            ptable[pid].pagetable = kalloc_pagetable(); // kalloc a new pagetable for the child
            if (ptable[pid].pagetable == nullptr) { // checks if there is still physical memory available
                return -1;
            }

            // initialize vmiters to iterate through parent and child pagetables
            vmiter pait(current->pagetable, 0);
            vmiter chit(ptable[pid].pagetable, 0);

            // identical mappings for child and parent below PROC_START_ADDR
            for (; pait.va() < PROC_START_ADDR; pait += PAGESIZE, chit += PAGESIZE) {
                if (chit.try_map(pait.pa(), pait.perm()) < 0) {
                    free_everything(ptable[pid].pagetable);
                    ptable[pid].state = P_FREE;
                    return -1;
                }
            }
            for (; pait.va() < MEMSIZE_VIRTUAL; pait += PAGESIZE, chit += PAGESIZE) {
                if (pait.user() && pait.writable()) { // case for readable memory 
                    void* page = kalloc(PAGESIZE);
                    if (page == nullptr) { // checks if there is there is no more physical memory available
                        free_everything(ptable[pid].pagetable);
                        ptable[pid].state = P_FREE;
                        return -1;
                    } else if (chit.try_map(page, pait.perm()) < 0) { // maps different page for child
                        free_everything(ptable[pid].pagetable);
                        ptable[pid].state = P_FREE;
                        kfree(page); // address potential memory leak
                        return -1;
                    }
                    memcpy(page, pait.kptr(), PAGESIZE); // copy parent data into child page
                } else if (pait.user() && !pait.writable()) { // case for read-only memory
                    if (chit.try_map(pait.pa(), pait.perm()) < 0) { // share addresses
                        free_everything(ptable[pid].pagetable);
                        ptable[pid].state = P_FREE;
                        return -1;
                    }

                    // increase the number of allocations, since memory is being shared
                    physpages[pait.pa() / PAGESIZE].refcount++; 
                }
            }
            ptable[pid].regs = current->regs; // copy parent process registers
            ptable[pid].regs.reg_rax = 0; // return 0 to the child process
            ptable[pid].state = P_RUNNABLE;

            return pid;
        }
    }
    return -1; // executes if no free process slot is ever found
}


// free_everything(pt)
//    Frees all process memory.

void free_everything(x86_64_pagetable* pt) {
    
    // first free all memory accessible via unprivileged mappings
    for (vmiter it(pt, 0); it.va() < MEMSIZE_VIRTUAL; it += PAGESIZE) {
        if (it.user() && it.va() != CONSOLE_ADDR) {
            kfree(it.kptr());
        }
    }

    // free all pagetable pages
    for (ptiter it(pt); it.va() < MEMSIZE_VIRTUAL; it.next()) {
        kfree(it.kptr());
    }
    kfree(pt); // free the pagetable itself
}


// syscall_exit()
//    Exits the current process.
void syscall_exit() {
    free_everything(current->pagetable); // frees all process memory
    current->state = P_FREE; // marks the process as free
    schedule();

}


// memshow()
//    Draw a picture of memory (physical and virtual) on the CGA console.
//    Switches to a new process's virtual memory map every 0.25 sec.
//    Uses `console_memviewer()`, a function defined in `k-memviewer.cc`.

void memshow() {
    static unsigned last_ticks = 0;
    static int showing = 0;

    // switch to a new process every 0.25 sec
    if (last_ticks == 0 || ticks - last_ticks >= HZ / 2) {
        last_ticks = ticks;
        showing = (showing + 1) % NPROC;
    }

    proc* p = nullptr;
    for (int search = 0; !p && search < NPROC; ++search) {
        if (ptable[showing].state != P_FREE
            && ptable[showing].pagetable) {
            p = &ptable[showing];
        } else {
            showing = (showing + 1) % NPROC;
        }
    }

    console_memviewer(p);
    if (!p) {
        console_printf(CPOS(10, 29), 0x0F00, "VIRTUAL ADDRESS SPACE\n"
            "                          [All processes have exited]\n"
            "\n\n\n\n\n\n\n\n\n\n\n");
    }
}