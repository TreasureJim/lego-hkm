CFLAGS=-Wall -Wconversion -g -fstack-usage
CFLAGS+=-DEXTRA_CALC
CFLAGS+=-O0
LDLIBS=-lm
DEPS=kinematics.o ax4_kinematics.o c_lpoe/liblpoe.so ckin/lin_alg.o ckin/common.o myalloc.o
CALTESTS=test_calib_vel_acc_calc_mark2_0 test_calib_kin_mark2_0 test_ax3_stiffness_mark2_0

VALGRIND:=LD_LIBRARY_PATH=c_lpoe valgrind --track-origins=yes --error-exitcode=1

test: test.o dyn.o kin_model.o $(DEPS)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@

test_long: test_long.o kin_model_long.o $(DEPS)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@

test_mark1_5: test_mark1_5.o mark1_5.o $(DEPS)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@

test_mark2_0: test_mark2_0.o mark2_0.o $(DEPS)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@

test_calib_kin_mark2_0: test_calib_kin_mark2_0.o dyn.o mark2_0.o mark2_0_perturbed.o mark2_0_calib.o mark2_0.o $(DEPS)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@

test_calib_vel_acc_calc_mark2_0: test_calib_vel_acc_calc_mark2_0.o dyn.o mark2_0.o mark2_0.o mark2_0_perturbed.o mark2_0_lpoe.o $(DEPS)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@

test_ax3_stiffness_mark2_0: test_ax3_stiffness_mark2_0.o mark2_0.o mark2_0.o mark2_0_lpoe.o mark2_0_calib.o dyn.o $(DEPS)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@

test_dyn: test_dyn.c dyn.o concept_prototype.o dyn.o $(DEPS) kin_model.o
	$(CC) $(CFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@

test_dyn_long: test_dyn_long.c dyn.o dyn.o $(DEPS) kin_model_long.o HKM1800.o
	$(CC) $(CFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@

test_dyn_mark1_5: test_dyn_mark1_5.c dyn.o $(DEPS) mark1_5.o mark1_5_lpoe.o
	$(CC) $(CFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@

test_dyn_mark2_0: test_dyn_mark2_0.c dyn.o $(DEPS) mark2_0.o mark2_0_lpoe.o
	$(CC) $(CFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@

test_vel_acc_calc: test_vel_acc_calc.c dyn.o kin_model.o $(DEPS)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@

test_vel_acc_calc_long: test_vel_acc_calc_long.c dyn.o kin_model_long.o $(DEPS)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@

test_vel_acc_calc_mark1_5: test_vel_acc_calc_mark1_5.c mark1_5.o $(DEPS)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@

test_vel_acc_calc_mark2_0: test_vel_acc_calc_mark2_0.c mark2_0.o $(DEPS)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@

test_new_joint_space: test_new_joint_space.c dyn.o $(DEPS) kin_model_long.o mark1_5.o mark2_0.o
	$(CC) $(CFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@

bench_dyn: bench_dyn.c dyn.o concept_prototype.o kin_model.o $(DEPS) mark1_5.o mark1_5_lpoe.o mark2_0.o mark2_0_lpoe.o
	$(CC) $(CFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@

bench_vel_acc_calc: bench_vel_acc_calc.c dyn.o kin_model.o $(DEPS) mark1_5.o mark2_0.o
	$(CC) $(CFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@

.PHONY: run_test
run_test: test test_long test_mark1_5 test_mark2_0
	$(VALGRIND) ./test || exit 1 ; \
	$(VALGRIND) ./test_long || exit 1 ; \
	$(VALGRIND) ./test_mark1_5 || exit 1 ; \
	$(VALGRIND) ./test_mark2_0 || exit 1 ; \

.PHONY: run_test_dyn
run_test_dyn: test_dyn test_dyn_long test_dyn_mark1_5 test_dyn_mark2_0
	$(VALGRIND) ./test_dyn || exit 1 ; \
	$(VALGRIND) ./test_dyn_long || exit 1 ; \
	$(VALGRIND) ./test_dyn_mark1_5 || exit 1 ; \
	$(VALGRIND) ./test_dyn_mark2_0 || exit 1 ; \

.PHONY: run_test_vel_acc_calc
run_test_vel_acc_calc: test_vel_acc_calc test_vel_acc_calc_long test_vel_acc_calc_mark1_5 test_vel_acc_calc_mark2_0
	$(VALGRIND) ./test_vel_acc_calc || exit 1 ; \
	$(VALGRIND) ./test_vel_acc_calc_long || exit 1 ; \
	$(VALGRIND) ./test_vel_acc_calc_mark1_5 || exit 1 ; \
	$(VALGRIND) ./test_vel_acc_calc_mark2_0 || exit 1 ; \

.PHONY: run_test_new_joint_space
run_test_new_joint_space: test_new_joint_space
	$(VALGRIND) ./test_new_joint_space || exit 1 ; \

.PHONY: run_test_calib
run_test_calib: test_calib_kin_mark2_0 test_calib_vel_acc_calc_mark2_0 test_ax3_stiffness_mark2_0
	$(VALGRIND) ./test_calib_kin_mark2_0 || exit 1 ; \
	$(VALGRIND) ./test_calib_vel_acc_calc_mark2_0 || exit 1 ; \
	$(VALGRIND) ./test_ax3_stiffness_mark2_0 || exit 1 ; \

.PHONY: run_test_all
run_test_all: run_test run_test_dyn run_test_vel_acc_calc

.PHONY: run_bench_dyn
run_bench_dyn: bench_dyn
	./bench_dyn

.PHONY: run_bench_vel_acc_calc
run_bench_vel_acc_calc: bench_vel_acc_calc
	./bench_vel_acc_calc

libagile_all.so: dyn.c kinematics.c $(DEPS)
	$(CC) $(CFLAGS) -shared -o $@ -fpic -Wno-unused-function $^

.PHONY: run_wrapper
run_wrapper: libagile_all.so
	PYTHONPATH=../py:$$PYTHONPATH LD_LIBRARY_PATH=. python agile_wrapper.py

clean:
	$(RM) *.o test test_mark1_5 test_mark2_0 test_dyn test_dyn_mark1_5 test_dyn_mark2_0 libagile_all.so test_new_joint_space test_long test_dyn_long bench_dyn bench_vel_acc_calc *.su test_vel_acc_calc test_vel_acc_calc_long test_vel_acc_calc_mark1_5 test_vel_acc_calc_mark2_0 $(CALTESTS)
