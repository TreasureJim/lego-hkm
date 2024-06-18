/*
 * Copyright (C) 2016, 2018 Cognibotics
 *
 * Use or copying of this file or any part of it, without explicit
 * permission from Cognibotics is not allowed in any way.
 */
#ifndef DEBUG_H
#define DEBUG_H

#define D2R(a) (a * M_PI / 180.0)

#define PRINTM(mtx, m, n) do {                  \
                puts("");                       \
                puts(#mtx);                     \
                printm((double *) mtx, m, n);   \
        } while(0)


static inline void printm(double *mtx, size_t m, size_t n)
{
        for (size_t ii = 0; ii < m; ii++) {
                for (size_t jj = 0; jj < n; jj++) {
                        printf("%e\t", *(mtx + n * ii + jj));
                }
                puts("");
        }
}

#endif
