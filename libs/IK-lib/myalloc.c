#include <stdlib.h>

void* myalloc(size_t size) {
    return malloc(size);
}

void** mymatrixalloc(size_t rows, size_t cols) {
    void** ptr;
    // ptr = malloc((rows * cols + rows * sizeof(double*)) * sizeof(double));
    ptr = calloc(rows * cols + rows * sizeof(double*), sizeof(double));
    double* data = (double*) ptr + rows;
    for (size_t i = 0; i < rows; i++) {
        ptr[i] = data + i * cols;
    }
    return ptr;
}

void myfree(void* ptr) {
    return free(ptr);
}
