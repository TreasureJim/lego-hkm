#ifndef MYALLOC_H
#define MYALLOC_H

void* myalloc(size_t size);
void** mymatrixalloc(size_t rows, size_t cols);

void myfree(void* ptr);
void mymatrixfree(void** ptr);

#endif
