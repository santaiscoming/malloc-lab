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
  @ ALIGNMENT -> 8바이트(DOUBLE WORD) 정렬을 위해
  @ ALIGN(size) -> size를 8바이트 정렬로 변경
  @ SIZE_T_SIZE -> size_t의 크기를 8바이트로 변경
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

// --------------------- prototype ---------------------
static void place(void *bp, size_t asize);
static void *extend_heap(size_t words);
static void *coalesce(void *bp);
static void *find_fit(size_t asize, int fitNum);
static void *first_fit(size_t asize);
static void *next_fit(size_t asize);
static void *best_fit(size_t asize);
// -----------------------------------------------------

// --------------------- global variable ---------------------
/*
  @ heap_listp - 힙의 시작점을 가리키는 포인터
*/
static char *heap_listp;
static char *prev_bp;
// -----------------------------------------------------------

/*
 * mm_init - initialize the malloc package.
 * prologue - 헤더와 푸터로만 구성된 8바이트(DSIZE) 블록
 * epilogue - 헤더와 푸터로만 구성 (사이즈 0)
 */
int mm_init(void) {
  /*Create the initial empty heap */
  if ((heap_listp = mem_sbrk(4 * WSIZE)) == (void *)-1) return -1;

  PUT(heap_listp, 0);                            /* Alignment padding */
  PUT(heap_listp + (1 * WSIZE), PACK(DSIZE, 1)); /* Prologue */
  PUT(heap_listp + (2 * WSIZE), PACK(DSIZE, 1)); /* Prologue */
  PUT(heap_listp + (3 * WSIZE), PACK(0, 1));     /* Epilogue */
  heap_listp += (2 * WSIZE);
  /* Extend the empty heap with a free block of CHUNKSIZE bytes */
  if (extend_heap(CHUNKSIZE / WSIZE) == NULL) return 1;

  return 0;
}
/*
 * mm_malloc - Allocate a block by incrementing the brk pointer.
 *     Always allocate a block whose size is a multiple of the alignment.
 */
void *mm_malloc(size_t size) {
  if (size == 0) return NULL;
  /*
    @ asize : 할당될 사이즈
    @ extendsize : 현재 블록보다 클 경우 늘려줄 사이즈 (맞는 fit이 없음)
    @ bp : 블록의 시작 주소
  */
  size_t asize;
  size_t extendsize;
  char *bp;

  /* 정렬을 위한 블록 할당 */
  if (size <= DSIZE) {
    asize = 2 * DSIZE;
  } else {  // 더블워드의 배수로 설정하기 위함
    asize = ALIGN(size + SIZE_T_SIZE);
  }

  /* fit에 맞는 가용 블록 찾기 */
  bp = find_fit(asize, 2);
  if (bp) {
    place(bp, asize);
    return bp;
  }

  extendsize = MAX(asize, CHUNKSIZE); /* 맞는 fit이 없을떄 더 큰 메모리를
                                         요청하고 거기에 블록을 할당 */
  bp = extend_heap(extendsize / WSIZE);
  if (!bp) return NULL; /* 더이상 블록 할당이 불가 */
  place(bp, asize);

  return bp;
}

/*
 * mm_free - Freeing a block does nothing.
 */
void mm_free(void *bp) {
  size_t size = GET_SIZE(HDRP(bp));

  PUT(HDRP(bp), PACK(size, 0));
  PUT(FTRP(bp), PACK(size, 0));
  coalesce(bp);
}

/*
 * mm_realloc - Implemented simply in terms of mm_malloc and mm_free
 */
void *mm_realloc(void *ptr, size_t size) {
  void *oldptr = ptr;
  void *newptr;
  size_t copySize;

  newptr = mm_malloc(size);
  if (newptr == NULL) return NULL;

  copySize = GET_SIZE(HDRP(oldptr));
  if (size < copySize) copySize = size;
  // oldptr에서 newptr로 copySize 바이트를 복사
  memcpy(newptr, oldptr, copySize);
  mm_free(oldptr);

  return newptr;
}

static void place(void *bp, size_t asize) {
  size_t curr_size = GET_SIZE(HDRP(bp));

  if ((curr_size - asize) >= (2 * DSIZE)) {
    PUT(HDRP(bp), PACK(asize, 1));
    PUT(FTRP(bp), PACK(asize, 1));

    bp = NEXT_BLKP(bp); /* 새로운 블록 포인터 */

    PUT(HDRP(bp), PACK((curr_size - asize), 0));
    PUT(FTRP(bp), PACK((curr_size - asize), 0));
  } else {
    PUT(HDRP(bp), PACK(curr_size, 1));
    PUT(FTRP(bp), PACK(curr_size, 1));
  }
}

static void *extend_heap(size_t words) {
  char *bp;
  size_t size;
  char *next_bp;

  /* Allocate an even number of words to maintain alignment */
  size = (words % 2) ? (words + 1) * WSIZE : words * WSIZE;

  if ((long)(bp = mem_sbrk(size)) == -1) return NULL;
  /* Initialize free block header/footer and the epilogue header */
  PUT(HDRP(bp), PACK(size, 0));         /* Free block header */
  PUT(FTRP(bp), PACK(size, 0));         /* Free block footer */
  PUT(HDRP(NEXT_BLKP(bp)), PACK(0, 1)); /* New epilogue header */
  /* Coalesce if the previous block was free */
  return coalesce(bp);
}

static void *coalesce(void *bp) {
  size_t prev_alloc = GET_ALLOC(FTRP(PREV_BLKP(bp)));
  size_t next_alloc = GET_ALLOC(HDRP(NEXT_BLKP(bp)));
  size_t size = GET_SIZE(HDRP(bp));

  /* Case 1 */
  if (prev_alloc && next_alloc) {
    return bp;
  }

  /* Case 2 */
  else if (prev_alloc && !next_alloc) {
    size += GET_SIZE(HDRP(NEXT_BLKP(bp)));
    PUT(HDRP(bp), PACK(size, 0));
    PUT(FTRP(bp), PACK(size, 0));
  }

  /* Case 3 */
  else if (!prev_alloc && next_alloc) {
    size += GET_SIZE(HDRP(PREV_BLKP(bp)));
    PUT(FTRP(bp), PACK(size, 0));
    PUT(HDRP(PREV_BLKP(bp)), PACK(size, 0));
    bp = PREV_BLKP(bp);
  }

  /* Case 4 */
  else {
    size += GET_SIZE(HDRP(PREV_BLKP(bp))) + GET_SIZE(FTRP(NEXT_BLKP(bp)));
    PUT(HDRP(PREV_BLKP(bp)), PACK(size, 0));
    PUT(FTRP(NEXT_BLKP(bp)), PACK(size, 0));
    bp = PREV_BLKP(bp);
  }

  return bp;
}

static void *find_fit(size_t asize, int fitNum) {
  void *bp = NULL;

  switch (fitNum) {
    case 1:
      bp = first_fit(asize);
      break;
    case 2:
      bp = best_fit(asize);
      break;
    case 3:
      bp = next_fit(asize);
      break;
    default:
      bp = first_fit(asize);
      break;
  }

  return bp;
}

/* First-fit */
static void *first_fit(size_t asize) {
  if (!mem_heap_lo()) return NULL;

  size_t cur_block_size;
  void *cur_bp = mem_heap_lo() + DSIZE;  // 힙의 prologue 이후부터 탐색한다

  while (1) {
    if (GET_SIZE(HDRP(cur_bp)) == 0) break;  //

    cur_block_size = GET_SIZE(HDRP(cur_bp));
    // 할당되지 않음 && 사이즈가 낭낭함 -> 해당 bp return
    if (!GET_ALLOC(HDRP(cur_bp)) && (asize <= cur_block_size)) return cur_bp;

    cur_bp = NEXT_BLKP(cur_bp);
  }

  return NULL; /* No fit */
}

/* Best-fit */
static void *best_fit(size_t asize) {
  void *bp;
  void *best_bp = NULL;
  size_t min_size = 0;

  for (bp = heap_listp; GET_SIZE(HDRP(bp)) > 0; bp = NEXT_BLKP(bp)) {
    if (!GET_ALLOC(HDRP(bp)) && (asize <= GET_SIZE(HDRP(bp)))) {
      if (min_size == 0 || GET_SIZE(HDRP(bp)) < min_size) {
        min_size = GET_SIZE(HDRP(bp));
        best_bp = bp;
      }
    }
  }

  return best_bp;
}

/* Next-fit */
static void *next_fit(size_t size) {
  char *bp = prev_bp;

  for (; GET_SIZE(HDRP(bp)) > 0; bp = NEXT_BLKP(bp)) {
    if (!GET_ALLOC(HDRP(bp)) && (size <= GET_SIZE(HDRP(bp)))) {
      prev_bp = bp;
      return bp;
    }
  }

  for (bp = heap_listp; bp < prev_bp; bp = NEXT_BLKP(bp)) {
    if (!GET_ALLOC(HDRP(bp)) && (size <= GET_SIZE(HDRP(bp)))) {
      prev_bp = bp;
      return bp;
    }
  }

  return NULL;
}
