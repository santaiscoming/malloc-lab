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

/*
  For explicit-list
  @ GET_SUCC(bp) -> bp의 "다음 가용 블록"을 가르키는 포인터
  @ GET_PRED(bp) -> bp의 "이전 가용 블록"을 가르키는 포인터
*/
#define GET_PRED(bp) (*(char **)(bp))
#define GET_SUCC(bp) (*(char **)(bp + WSIZE))

// --------------------------------------------------------------

// --------------------- function prototype ---------------------
static void place(void *bp, size_t asize);
static void *extend_heap(size_t words);
static void *coalesce(void *bp);
static void *find_fit(size_t asize);
static void remove_free_block(void *bp);
static void add_free_block(void *bp);
// --------------------------------------------------------------

// ---------------------- global variable -----------------------
/*
  @ free_list_p -> 가용블록을 추적하기 위한 포인터
*/
static char *free_list_p;
// --------------------------------------------------------------

/*********************************************************
 *              explicit-list (using LIFO)
 ********************************************************/

/*
 * mm_init - initialize the malloc package.
 * @ 초기 힙 생성
 */
int mm_init(void) {
  /*
    8워드 크기의 힙 생성
    implicit과 다르게 가용블록만 추적
    ps.
        가용블록을 추적하는 이유는 가용블록을 찾기위해 모든 블록을 순회할 필요가
        없기때문 반대로 말하면 implicit는 모든 블록을 순회해야함
  */
  if ((free_list_p = mem_sbrk(8 * WSIZE)) == (void *)-1) return -1;

  PUT(free_list_p, 0);                                /* Alignment padding */
  PUT(free_list_p + (1 * WSIZE), PACK(DSIZE, 1));     /* Prologue */
  PUT(free_list_p + (2 * WSIZE), PACK(DSIZE, 1));     /* Prologue */
  PUT(free_list_p + (3 * WSIZE), PACK(4 * WSIZE, 0)); /* 첫 가용 블록 Header */
  PUT(free_list_p + (4 * WSIZE), NULL);               /* pred */
  PUT(free_list_p + (5 * WSIZE), NULL);               /* succ */
  PUT(free_list_p + (6 * WSIZE), PACK(4 * WSIZE, 0)); /* 첫 가용 블록 Footer */
  PUT(free_list_p + (7 * WSIZE), PACK(0, 1));         /* Epilogue */

  /* 가용 블록을 가르키는 포인터을 에필로그만큼 옮겨줌 */
  free_list_p += (4 * WSIZE);

  /* 힙을 CHUNKSIZE bytes로 확장 */
  if (extend_heap((CHUNKSIZE / WSIZE) - 2 * DSIZE) == NULL) return -1;

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

  /*
    @ 정렬을 위한 블록 할당
      if -> 16바이트 이하일 경우 16바이트로 할당
      else -> 더블워드의 배수로 설정하기 위함
  */
  if (size <= DSIZE) {
    asize = 2 * DSIZE;
  } else {  // 더블워드의 배수로 설정하기 위함
    asize = ALIGN(size + SIZE_T_SIZE);
  }

  /* fit에 맞는 가용 블록 찾기 */
  bp = find_fit(asize);
  if (bp) {
    place(bp, asize);
    return bp;
  }

  /* 맞는 fit이 없을떄 더 큰 메모리를 요청하고 거기에 블록을 할당 */
  extendsize = MAX(asize, CHUNKSIZE);
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

/*
  heap 영역을 늘림

*/
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

/* 가용 블록 리스트 연결 */
static void *coalesce(void *bp) {
  size_t prev_alloc = GET_ALLOC(FTRP(PREV_BLKP(bp)));  // 이전 블록 할당 상태
  size_t next_alloc = GET_ALLOC(HDRP(NEXT_BLKP(bp)));  // 다음 블록 할당 상태
  size_t size = GET_SIZE(HDRP(bp));  // 현재 블록 사이즈

  /*
    [case 1] : 앞 뒤 모두 할당된 경우
      free_list에 추가후 넘겨받은 bp return
  */
  if (prev_alloc && next_alloc) {
    add_free_block(bp);

    return bp;
  }

  /*
    [case 2] : 다음 블록만 빈 경우

  */
  else if (prev_alloc && !next_alloc) {
    splice_free_block(NEXT_BLKP(bp));  // 가용 블록을 free_list에서 제거
    size += GET_SIZE(HDRP(NEXT_BLKP(bp)));
    PUT(HDRP(bp), PACK(size, 0));  // 현재 블록 헤더 재설정
    // 다음 블록 푸터 재설정
    //(위에서 헤더를 재설정했으므로, FTRP(bp)는 합쳐질 다음 블록의 푸터가 됨)
    PUT(FTRP(bp), PACK(size, 0));
  }

  // 이전 블록만 빈 경우
  else if (!prev_alloc && next_alloc) {
    splice_free_block(PREV_BLKP(bp));  // 가용 블록을 free_list에서 제거
    size += GET_SIZE(HDRP(PREV_BLKP(bp)));
    PUT(HDRP(PREV_BLKP(bp)), PACK(size, 0));  // 이전 블록 헤더 재설정
    PUT(FTRP(bp), PACK(size, 0));             // 현재 블록 푸터 재설정
    bp = PREV_BLKP(bp);  // 이전 블록의 시작점으로 포인터 변경
  }

  // 이전 블록과 다음 블록 모두 빈 경우
  else {
    splice_free_block(PREV_BLKP(bp));  // 이전 가용 블록을 free_list에서 제거
    splice_free_block(NEXT_BLKP(bp));  // 다음 가용 블록을 free_list에서 제거
    size += GET_SIZE(HDRP(PREV_BLKP(bp))) + GET_SIZE(FTRP(NEXT_BLKP(bp)));
    PUT(HDRP(PREV_BLKP(bp)), PACK(size, 0));  // 이전 블록 헤더 재설정
    PUT(FTRP(NEXT_BLKP(bp)), PACK(size, 0));  // 다음 블록 푸터 재설정
    bp = PREV_BLKP(bp);  // 이전 블록의 시작점으로 포인터 변경
  }
  add_free_block(bp);  // 현재 병합한 가용 블록을 free_list에 추가
  return bp;           // 병합한 가용 블록의 포인터 반환
}

/*
  @ 가용 블록에 할당하는 함수
    first-fit으로 찾은 가용 블록에 할당
*/
static void place(void *bp, size_t asize) {
  remove_free_block(bp); /* Free-Block_List 찾은 가용 블록 제거 */

  size_t cur_block_size = GET_SIZE(HDRP(bp));  // 현재 블록의 크기

  // 차이가 최소 블록 크기 16보다 같거나 크면 분할
  if ((cur_block_size - asize) >= (2 * DSIZE)) {
    // 현재 블록에는 필요한 만큼만 할당
    PUT(HDRP(bp), PACK(asize, 1));
    PUT(FTRP(bp), PACK(asize, 1));
    // 남은 크기를 다음 블록에 할당(가용 블록)
    PUT(HDRP(NEXT_BLKP(bp)), PACK((cur_block_size - asize), 0));
    PUT(FTRP(NEXT_BLKP(bp)), PACK((cur_block_size - asize), 0));
    add_free_block(NEXT_BLKP(bp));  // 남은 블록을 free_list에 추가
  } else {
    PUT(HDRP(bp), PACK(cur_block_size, 1));  // 해당 블록 전부 사용
    PUT(FTRP(bp), PACK(cur_block_size, 1));
  }
}

/* First-fit */
static void *find_fit(size_t asize) {
  if (asize == 0 || !free_list_p) return NULL;

  /* @ bp -> 가용 블록의 첫번째 */
  void *bp = free_list_p;

  /*
    while : 다음 가용 블록이 있는동안
      if : 적합한 사이즈의 블록을 찾으면 해당 블록 포인터 반환
      else : 다음 가용 블록(succ)으로 이동
  */
  while (bp) {
    if ((asize <= GET_SIZE(HDRP(bp)))) return bp;

    bp = GET_SUCC(bp);
  }
  return NULL;
}

// 가용 리스트에서 bp에 해당하는 블록을 제거하는 함수
static void remove_free_block(void *bp) {
  // 분리하려는 블록이 free_list 맨 앞에 있는 블록이면
  // (이전 블록이 없음)
  if (bp == free_list_p) {
    free_list_p = GET_SUCC(free_list_p);  // 다음 블록을 루트로 변경
    return;
  }
  // 이전 블록의 SUCC을 다음 가용 블록으로 연결
  GET_SUCC(GET_PRED(bp)) = GET_SUCC(bp);
  // 다음 블록의 PRED를 이전 블록으로 변경
  // 다음 가용 블록이 있을 경우만
  if (GET_SUCC(bp) != NULL) GET_PRED(GET_SUCC(bp)) = GET_PRED(bp);
}

// 가용 리스트의 맨 앞에 현재 블록을 추가하는 함수
static void add_free_block(void *bp) {
  GET_SUCC(bp) = free_list_p;  // bp의 SUCC은 루트가 가리키던 블록

  // free list에 블록이 존재했을 경우만
  // 루트였던 블록의 PRED를 추가된 블록으로 연결
  if (free_list_p != NULL) GET_PRED(free_list_p) = bp;

  free_list_p = bp;  // 루트를 현재 블록으로 변경
}