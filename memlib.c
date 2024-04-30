/*
 * memlib.c - a module that simulates the memory system.  Needed because it
 *            allows us to interleave calls from the student's malloc package
 *            with the system's malloc package in libc.
 */
#include "memlib.h"
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include "config.h"

/*
  ---------------------------
  @ mem_start_brk -> heap의 시작
  @ mem_brk       -> VM 힙공간안에 할당받은 heap의 꼭대기
  @ mem_max_addr  -> VM에서 Heap의 최대 크기 (현재 코드에서 var MAX_HEAP)
  ---------------------------
*/
/* private variables */
static char *mem_start_brk; /* points to first byte of heap */
static char *mem_brk;       /* points to last byte of heap */
static char *mem_max_addr;  /* largest legal heap address */

/*
 * mem_init - initialize the memory system model
 */
void mem_init(void) {
  /* allocate the storage we will use to model the available VM */

  // @ VM 내에서의 사용가능한 HEAP을 할당받음
  if ((mem_start_brk = (char *)malloc(MAX_HEAP)) == NULL) {
    fprintf(stderr, "mem_init_vm: malloc error\n");
    exit(1);
  }

  mem_max_addr = mem_start_brk + MAX_HEAP; /* max legal heap address */
  mem_brk = mem_start_brk;                 /* heap is empty initially */
}

/*
 * mem_deinit - free the storage used by the memory system model
 */
void mem_deinit(void) { free(mem_start_brk); }

/*
 * mem_reset_brk - reset the simulated brk pointer to make an empty heap
 */
void mem_reset_brk() { mem_brk = mem_start_brk; }

/*
 * mem_sbrk - simple model of the sbrk function. Extends the heap
 *    by incr bytes and returns the start address of the new area. In
 *    this model, the heap cannot be shrunk.
 */
void *mem_sbrk(int incr) {
  char *old_brk = mem_brk;

  // @ VM 내의 Heap상한선(MAX_HEAP) 보다 커지면 더이상 할당 불가
  if ((incr < 0) || ((mem_brk + incr) > mem_max_addr)) {
    errno = ENOMEM;
    fprintf(stderr, "ERROR: mem_sbrk failed. Ran out of memory...\n");
    return (void *)-1;
  }
  mem_brk += incr;
  return (void *)old_brk;
}

/*
 * mem_heap_lo - return address of the first heap byte
 * @ mem_start_brk가 static 변수기에 외부에서 접근할때 함수를 통해서 접근할 수
 * @ 있도록 한다.
 * @ (for 캡슐화)
 */
void *mem_heap_lo() { return (void *)mem_start_brk; }

/*
 * mem_heap_hi - return address of last heap byte
 */
void *mem_heap_hi() { return (void *)(mem_brk - 1); }

/*
 * mem_heapsize() - returns the heap size in bytes
 */
size_t mem_heapsize() { return (size_t)(mem_brk - mem_start_brk); }

/*
 * mem_pagesize() - returns the page size of the system
 */
size_t mem_pagesize() { return (size_t)getpagesize(); }
