/*
 * mm-naive.c - The fastest, least memory-efficient malloc package.
 *
 * In this naive approach, a block is allocated by simply incrementing
 * the brk pointer.  A block is pure payload. There are no headers or
 * footers.  Blocks are never coalesced or reused. Realloc is
 * implemented directly using mm_malloc and mm_free.
 *
 * NOTE TO STUDENTS: Replace this header comment with your own header
 * comment that gives a high level description of your solution.
 */
#include "mm.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "memlib.h"

/*********************************************************
 * NOTE TO STUDENTS: Before you do anything else, please
 * provide your team information in the following struct.
 ********************************************************/
team_t team = {
    /* Team name */
    "ateam",
    /* First member's full name */
    "Harry Bovik",
    /* First member's email address */
    "bovik@cs.cmu.edu",
    /* Second member's full name (leave blank if none) */
    "",
    /* Second member's email address (leave blank if none) */
    ""};

/*
  ------------------- 매크로 설명 -------------------
  @ void *bp -> 블록포인터 즉, malloc으로 할당받은 블록의 시작주소를 의미
  @ CHUNKSIZE (1 << 12) -> 힙을 늘릴때 늘려줄 크기

  @ ALIGNMENT -> 8바이트(DOUBLE WORD) 정렬을 위해
  @ ALIGN(size) -> size를 8바이트 정렬로 변경
  @ SIZE_T_SIZE -> size_t의 ALIGN macro를 사용해 크기를 8바이트로 변경
  @ PACK(size, alloc) -> size와 alloc을 하나로 합침 (size : 29bit, alloc : 1bit)
    ex) size = 0x0000000c, alloc = 0x1 -> 0x0000000d

  @ GET(p) -> p(주소)의 값을 읽어옴
  @ PUT(p, val) -> p(주소)에 val을 저장

  @ GET_SIZE(p) -> p(주소)의 size를 읽어옴
    ex) 0x0000000d(할당된 블록) -> 0x0000000c
        1을 빼준다고 생각하면 됨
  @ GET_ALLOC(p) -> p(주소)의 할당 여부를 읽어옴
    ex) GET(p) = 0x0000000d (p의 값이 0x0000000d라면)
                 0x00000001 (0x1)
                 FFFFFFFFFT (둘다 1일때 T 즉, T는 1로 생각)
                 &연산은 둘다 1일때 1을 반환하므로 0x1 반환

  @ HDRP(bp) -> *bp - WSIZE (bp의 주소에서 1WORD(4byte)만큼 뒤로 이동)
  @ FTRP(bp) -> *bp + GET_SIZE(HDRP(bp)) - DSIZE(footer 크기)
                (포인터 연산 : bp의 주소에서 size만큼 뒤로 이동)
      +-----------------+-----------------+-----------------+-----------------+
      |  Header (WSIZE) |  Payload        |  Padding        |  Footer (WSIZE) |
      +-----------------+-----------------+-----------------+-----------------+
                        ⌃(bp가 가르키는 위치)
          즉, Header만큼 더 이동하기떄문에 Footer를 포함한 DSIZE 빼이함

  @ *** 괄호조심 !!! ***
  @ NEXT_BLKP(bp) -> *bp + GET_SIZE(((char*)bp - WSIZE))
        bp의 주소에서 WSIZE 만큼 뒤로 움직이면 HEADER를 가르키기때문에 빼줌
        즉, 현재 블록의 Header로 옮기고 Header의 size를 읽어와서 더해줌
  @ PREV_BLKP(bp) -> *bp - GET_SIZE(((char*)bp - DSIZE))
        GET_SIZE(((char *)(bp)-DSIZE) : 이전 블록의 Footer에서 size를 읽어옴
          DSIZE를 빼주는 이유 bp가 HEADER에서 시작하고 2칸 뒤로가야 FOOTER를
          가르키기 떄문
      +-----------------+-----------------+-----------------+-----------------+
      |   PREV FOOTER   |   NEXT HEADER   |  Padding(option)|  Footer (WSIZE) |
      +-----------------+-----------------+-----------------+-----------------+
      ⌃(bp - DSIZE)         <---          ⌃(bp가 가르키는 위치)


  -------------------------------------------------
*/
/* single word (4) or double word (8) alignment */
#define ALIGNMENT 8

/* rounds up to the nearest multiple of ALIGNMENT */
#define ALIGN(size) (((size) + (ALIGNMENT - 1)) & ~0x7)
#define SIZE_T_SIZE (ALIGN(sizeof(size_t)))

// ---------------- Basic constants and macros -----------------
#define WSIZE 4             /* Word and header/footer size (bytes) */
#define DSIZE 8             /* Double word size (bytes) */
#define CHUNKSIZE (1 << 12) /* Extend heap by this amount (bytes) */

#define MAX(x, y) ((x) > (y) ? (x) : (y))

/* Pack a size and allocated bit into a word */
#define PACK(size, alloc) ((size) | (alloc))

/* Read and write a word at address p */
#define GET(p) (*(unsigned int *)(p))
#define PUT(p, val) (*(unsigned int *)(p) = (val))

/* Read the size and allocated fields from address p */
#define GET_SIZE(headerPtr) (GET(headerPtr) & ~0x7)
#define GET_ALLOC(headerPtr) (GET(headerPtr) & 0x1)

/* Given block ptr bp, compute address of its header and footer */
#define HDRP(bp) ((char *)(bp)-WSIZE)
#define FTRP(bp) ((char *)(bp) + GET_SIZE(HDRP(bp)) - DSIZE)

/* Given block ptr bp, compute address of next and previous blocks */
#define NEXT_BLKP(bp) ((char *)(bp) + GET_SIZE(((char *)(bp)-WSIZE)))
#define PREV_BLKP(bp) ((char *)(bp)-GET_SIZE(((char *)(bp)-DSIZE)))
// --------------------------------------------------------------

// --------------------- global variable ---------------------
/*
  @ heap_listp - 힙의 시작점을 가리키는 포인터
*/
static char *heap_listp;
// -----------------------------------------------------------

/*
 * mm_init - initialize the malloc package.
 * @ 초기 힙 생성
 */
int mm_init(void) { return 0; }

/*
 * mm_malloc - Allocate a block by incrementing the brk pointer.
 *     Always allocate a block whose size is a multiple of the alignment.
 */
void *mm_malloc(size_t size) {
  int newsize = ALIGN(size + SIZE_T_SIZE);
  void *p = mem_sbrk(newsize);
  if (p == (void *)-1)
    return NULL;
  else {
    *(size_t *)p = size;
    return (void *)((char *)p + SIZE_T_SIZE);
  }
}

/*
 * mm_free - Freeing a block does nothing.
 */
void mm_free(void *ptr) {}

/*
 * mm_realloc - Implemented simply in terms of mm_malloc and mm_free
 */
void *mm_realloc(void *ptr, size_t size) {
  void *oldptr = ptr;
  void *newptr;
  size_t copySize;

  newptr = mm_malloc(size);
  if (newptr == NULL) return NULL;
  copySize = *(size_t *)((char *)oldptr - SIZE_T_SIZE);
  if (size < copySize) copySize = size;
  memcpy(newptr, oldptr, copySize);
  mm_free(oldptr);
  return newptr;
}
