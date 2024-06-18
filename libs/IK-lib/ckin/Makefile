LPOE_PATH:=./c_lpoe

CFLAGS:=-std=gnu99 -Wall -Wextra -Werror -Wno-missing-braces -Wno-missing-field-initializers -g -O3 -I $(LPOE_PATH)
LDLIBS:=-lm

VALGRIND:=LD_LIBRARY_PATH=. valgrind --track-origins=yes --error-exitcode=1
PYTHON:=PYTHONPATH=$(LPOE_PATH):$$PYTHONPATH python

GEN:=gen

TEST_BIN:=test_dh test_common test_lpoe_invkin test_lin_alg

UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
	MACOS_FIX=-undefined dynamic_lookup
endif

.PHONY: all
all: deps libkin.so

.PHONY: deps
deps:
	make -C $(LPOE_PATH)

libkin.so: dh.c lpoe_invkin.c common.c kin_skel.c lin_alg.c $(LPOE_PATH)/lpoe.c $(LPOE_PATH)/matrix_math.c
	$(CC) $(CFLAGS) -shared $(MACOS_FIX) -o $@ -fpic $^

test_dh: test_dh.o libkin.so

test_lpoe_invkin: test_lpoe_invkin.o example_params.o libkin.so

test_common: test_common.o libkin.so

test_lin_alg: test_lin_alg.o libkin.so

.PHONY: test
test: c_test

.PHONY: c_test
c_test: $(TEST_BIN)
	for test in $^ ; do \
		$(VALGRIND) ./$$test || exit 1 ; \
	done

.PHONY: clean
clean:
	$(RM) *.o
	$(RM) *.so
	$(RM) *.pyc
	$(RM) -r html latex
	$(RM) -r $(TEST_BIN)

# Extra tests that requires cblib
# TODO: Move to cblib/itest_deps

.PHONY: test_extra
test_extra: py_test run_test_inv_vg run_test_inv

.PHONY: py_test
py_test: libkin.so
	$(PYTHON) ./kin_wrapper.py
	$(PYTHON) ./test_kin_skel.py

test_time: test_time.c $(GEN)/lpoe_IRB140.c libkin.so
	$(CC) $(CFLAGS) -Wno-unused-variable -Wno-unused-parameter \
	        $(LDFLAGS) -L . $(filter %c %.o,$^) -o $@ -lkin $(LDLIBS)

test_inv: test_inv.c $(GEN)/lpoe_IRB140.c libkin.so
	$(CC) $(CFLAGS) -Wno-unused-variable -Wno-unused-parameter \
	        $(LDFLAGS) -L . $(filter %c %.o,$^) -o $@ -lkin $(LDLIBS)

.PHONY: lpoe
lpoe: $(GEN)/lpoe_IRB140.c

$(GEN)/lpoe_IRB140.c:
	$(PYTHON) -m cb.codegen.ne rob_def --robot-name IRB140 --name lpoe_IRB140

.PHONY: run_test_inv_vg
run_test_inv_vg: test_inv
	LD_LIBRARY_PATH=./$(GEN):$(CKIN_PATH):$(LPOE_PATH) valgrind ./test_inv && echo "$@ ok\n"

.PHONY: run_test_inv
run_test_inv: test_inv
	LD_LIBRARY_PATH=./$(GEN):$(CKIN_PATH):$(LPOE_PATH) ./test_inv test && echo "$@ ok\n"

.PHONY: doc
doc: html latex

html latex: Doxyfile kin_skel.h kin_skel.c common.h common.c dh.h dh.c lpoe_invkin.h lpoe_invkin.c README.md
	doxygen $<
