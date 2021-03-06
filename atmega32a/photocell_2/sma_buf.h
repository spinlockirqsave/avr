/*
 * @brief   SMA buffer. Calculates simple moving average on the go.
 * @author  Piotr Gregor <piotrek.gregor gmail.com>
 */


#ifndef __SMA_BUF_H__
#define __SMA_BUF_H__


#include <stdio.h>
#include <stdlib.h>
#ifndef _MSC_VER
#include <stdint.h>
#endif
#include <string.h>


struct sma_buf{
    size_t len;
    BUFF_TYPE *data;
    BUFF_TYPE sma;
    size_t pos;
    size_t lpos;
};

#define INIT_SMA_BUF(b, l) \
    { \
	(void)memset((b), 0, sizeof(struct sma_buf)); \
	(b)->len = (l); \
	(b)->data = (BUFF_TYPE *) malloc(sizeof(BUFF_TYPE) * (l)); \
	(b)->sma = 0; \
	(b)->pos = 0; \
	(b)->lpos = 0; \
    }

#define GET_SMA_SAMPLE(b, p) ((b)->data[(p) % (b)->len])
#define SET_SMA_SAMPLE(b, p, v) ((b)->data[(p) % (b)->len] = (v))
#define GET_CURRENT_SMA_POS(b) ((b)->pos)
#define GET_CURRENT_SMA_LPOS(b) ((b)->lpos)

#define INC_SMA_POS(b) \
{ \
    (b)->lpos++; \
    (b)->pos = (b)->lpos % (b)->len; \
}

#define APPEND_SMA_VAL(b, v) \
{ \
    (b)->sma -= ((b)->data[(b)->pos] / (BUFF_TYPE)(b)->len); \
    (b)->data[(b)->pos] = (v); \
    (((b)->lpos) >= ((b)->len)) ? ((b)->sma += ((b)->data[(b)->pos] / (BUFF_TYPE)(b)->len)) : \
        ((b)->sma = ((((b)->sma)*((b)->pos)) + ((b)->data[(b)->pos])) / ((BUFF_TYPE)(((b)->pos) + 1)))  ; \
    INC_SMA_POS(b); \
}

#define RESET_SMA_BUF(b) \
{ \
    (b)->sma = 0; \
    (void)memset((b)->data, 0, sizeof(BUFF_TYPE) * (b)->len); \
    (b)->pos = 0; \
    (b)->lpos = 0; \
}

#define DESTROY_SMA_BUF(b) \
    do{ \
        free((b)->data); \
    } while(0);


#endif /* __SMA_BUF_H__ */


/* Unit test.

int main(void)
{
    int i;
    sma_buffer_t b;

    INIT_SMA_BUFFER(&b, 100);

    for(i = 0; i < 20; i++){
	APPEND_SMA_VAL(&b, 100.0);
	printf("SMA = %lf\n", b.sma);
    }

    DESTROY_SMA_BUFFER(&b);

    return EXIT_SUCCESS;
}

*/

