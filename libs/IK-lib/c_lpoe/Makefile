CFLAGS:=-std=gnu99 -Wall -Wextra -Werror -Wno-missing-braces -Wno-missing-field-initializers -Wno-unused-parameter -O3 -g
LDFLAGS:=-L.
LDLIBS:=-llpoe -lm
# Add runtime path to test binaries to let them find lib
RUNPATH:=-Wl,-rpath,$(shell pwd) -Wl,-rpath,.

VALGRIND:=valgrind --track-origins=yes --error-exitcode=1

TEST_BIN:=test_lpoe test_matrix_math

UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
	MACOS_FIX=-undefined dynamic_lookup
endif

liblpoe.so: lpoe.c matrix_math.c
	$(CC) $(CFLAGS) -shared $(MACOS_FIX) -o $@ -fpic $^

test_lpoe: test_lpoe.c liblpoe.so
	$(CC) $(CFLAGS) $(LDFLAGS) $(RUNPATH) -o $@ $< $(LDLIBS)

test_matrix_math: test_matrix_math.c liblpoe.so
	$(CC) $(CFLAGS) $(LDFLAGS) $(RUNPATH) -o $@ $< $(LDLIBS)

.PHONY: test
test: c_test

.PHONY: c_test
c_test: $(TEST_BIN)
	for test in $^ ; do \
		$(VALGRIND) ./$$test || exit 1 ; \
	done

.PHONY: clean
clean:
	$(RM) *.so
	$(RM) -r html latex
	$(RM) *.o
	$(RM) test_lpoe
	$(RM) test_matrix_math

.PHONY: doc
doc: html latex

html latex: Doxyfile lpoe.c lpoe.h README.md
	doxygen $<
